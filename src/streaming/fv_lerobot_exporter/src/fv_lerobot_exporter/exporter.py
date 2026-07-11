"""LeRobot v3 dataset export for FluentVision recorder episodes."""

from __future__ import annotations

import bisect
import json
import math
import os
import shutil
import stat
from concurrent.futures import FIRST_COMPLETED, Future, ProcessPoolExecutor, wait
from dataclasses import dataclass, replace
from datetime import datetime, timezone
from multiprocessing import get_context, set_forkserver_preload
from pathlib import Path
from threading import Lock
from typing import Literal, Mapping, TypeVar

import numpy as np
import pyarrow.parquet as pq
import pandas as pd
from datasets import Dataset
from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata
from lerobot.datasets.utils import (
    get_hf_features_from_features,
    write_episodes,
    write_info,
    write_stats,
    write_tasks,
)
from lerobot.datasets.video_utils import get_video_info
from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    ValidationError,
    field_validator,
    model_validator,
)
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from typing_extensions import Self

from .timing import (
    MAX_VIDEO_TIMESTAMP_FRAME_DISTANCE,
    max_video_timestamp_tolerance_s,
)


JsonValue = str | int | float | bool | None | list["JsonValue"] | dict[str, "JsonValue"]
VIDEO_QUERY_TIMESTAMP_SOURCE_KEY = "video_query_timestamp_source"
VIDEO_QUERY_TIMESTAMP_SOURCE_FRAME_INDEX = "frame_index_over_fps"
RECORDER_VIDEO_TIME_BASE_DENOMINATOR = 90_000
StampSource = Literal["message_header", "rosbag_recv", "system"]
JointUnit = Literal["degrees", "percent"]
StatValue = list[float] | list[int] | list[list[list[float]]]
LerobotDataValue = int | float | list[float]
LerobotEpisodeValue = int | float | str | list[str]


class LerobotDatasetExportError(RuntimeError):
    """Raised when a selected episode cannot be exported without changing semantics."""

    def __init__(self, code: str, detail: str):
        super().__init__(detail)
        self.code = code
        self.detail = detail

    def __reduce__(self) -> tuple[type["LerobotDatasetExportError"], tuple[str, str]]:
        return type(self), (self.code, self.detail)


class LerobotDatasetExportRequest(BaseModel):
    dataset_id: str = Field(..., min_length=1, max_length=256)
    dataset_name: str = Field(..., min_length=1, max_length=256)
    episode_ids: list[str] = Field(..., min_length=1)
    fps: int = Field(default=30, ge=1, le=120)
    max_alignment_error_s: float = Field(default=0.1, gt=0.0, le=0.1)
    max_mux_status_age_s: float = Field(default=2.5, gt=0.0, le=10.0)

    @field_validator("dataset_id")
    @classmethod
    def validate_dataset_id(cls, value: str) -> str:
        parts = value.split("/")
        if any(part in {"", ".", ".."} for part in parts):
            raise ValueError("dataset_id contains an invalid path segment")
        if value.startswith("/") or "\\" in value:
            raise ValueError("dataset_id must be a relative POSIX path")
        return value

    @field_validator("episode_ids")
    @classmethod
    def validate_episode_ids(cls, value: list[str]) -> list[str]:
        if len(set(value)) != len(value):
            raise ValueError("episode_ids must not contain duplicates")
        return value

    @model_validator(mode="after")
    def validate_alignment_error(self) -> Self:
        limit_s = max_video_timestamp_tolerance_s(self.fps)
        if "max_alignment_error_s" not in self.model_fields_set:
            self.max_alignment_error_s = limit_s
        elif self.max_alignment_error_s > limit_s:
            raise ValueError(
                f"max_alignment_error_s must be at most {limit_s:g} for fps={self.fps}"
            )
        return self


class LerobotDatasetExportResponse(BaseModel):
    dataset_id: str
    dataset_name: str
    dataset_path: str
    episode_count: int
    frame_count: int
    exported_at: str
    provenance_path: str


def _export_worker_count(item_count: int) -> int:
    if item_count <= 1:
        return 1
    raw = os.environ.get("FV_LEROBOT_EXPORT_WORKERS", "").strip()
    if raw:
        try:
            requested = int(raw)
        except ValueError as exc:
            raise LerobotDatasetExportError(
                "export_workers_invalid",
                "FV_LEROBOT_EXPORT_WORKERS must be an integer from 1 to 32",
            ) from exc
        if requested < 1 or requested > 32:
            raise LerobotDatasetExportError(
                "export_workers_invalid",
                "FV_LEROBOT_EXPORT_WORKERS must be an integer from 1 to 32",
            )
        return min(item_count, requested)
    return min(item_count, max(1, min(os.cpu_count() or 4, 8)))


_ALIGNMENT_EXECUTOR: ProcessPoolExecutor | None = None
_ALIGNMENT_EXECUTOR_LOCK = Lock()


def _alignment_executor() -> ProcessPoolExecutor:
    global _ALIGNMENT_EXECUTOR
    with _ALIGNMENT_EXECUTOR_LOCK:
        if _ALIGNMENT_EXECUTOR is None:
            set_forkserver_preload(["fv_lerobot_exporter.exporter"])
            _ALIGNMENT_EXECUTOR = ProcessPoolExecutor(
                max_workers=_export_worker_count(32),
                mp_context=get_context("forkserver"),
            )
        return _ALIGNMENT_EXECUTOR


class TopicSpec(BaseModel):
    topic: str = Field(..., min_length=1)
    type: str | None = None


class JointCommandSpec(TopicSpec):
    leader: TopicSpec | None = None
    vr: TopicSpec | None = None
    ai: TopicSpec | None = None

    def topic_for_source(self, source: str) -> str:
        source_topic = {
            "leader": self.leader,
            "vr": self.vr,
            "ai": self.ai,
        }.get(source)
        if source not in {"leader", "vr", "ai"}:
            raise LerobotDatasetExportError(
                "action_source_unsupported",
                f"mux source '{source}' is not supported for dataset export",
            )
        if source_topic is None:
            raise LerobotDatasetExportError(
                "action_topic_not_configured",
                f"profile.lerobot has no explicit action topic for mux source '{source}'",
            )
        return source_topic.topic


class ArmRxSpec(BaseModel):
    joint_state: TopicSpec
    joint_command: JointCommandSpec


class ArmStreamSpec(BaseModel):
    key: str
    namespace: str | None = None
    side: str | None = None
    joints: list[str] = Field(..., min_length=1)
    joint_units: dict[str, JointUnit] | None = None
    rx: ArmRxSpec

    @model_validator(mode="after")
    def validate_joint_units(self) -> Self:
        if self.joint_units is None:
            return self
        unknown = set(self.joint_units) - set(self.joints)
        if unknown:
            raise ValueError(f"joint_units keys not in joints: {sorted(unknown)}")
        return self

    def unit_for(self, joint_name: str) -> JointUnit:
        if self.joint_units is None:
            return "degrees"
        return self.joint_units.get(joint_name, "degrees")


class CameraSpec(BaseModel):
    name: str
    topic: str
    source: str | None = None
    enabled: bool = Field(default=True, strict=True)


class LerobotSpec(BaseModel):
    arm_streams: list[ArmStreamSpec] = Field(..., min_length=1)
    cameras: list[CameraSpec] = Field(default_factory=list)


class ProfileExportSpec(BaseModel):
    model_config = ConfigDict(extra="ignore")

    name: str | None = None
    lerobot: LerobotSpec


class RecorderCameraSegment(BaseModel):
    file: str = Field(..., min_length=1)


class RecorderCameraMeta(BaseModel):
    model_config = ConfigDict(extra="ignore")

    name: str = Field(..., min_length=1)
    topic: str = Field(..., min_length=1)
    width: int | None = None
    height: int | None = None
    segments: list[RecorderCameraSegment] = Field(default_factory=list)


class RecordedTopicMeta(BaseModel):
    model_config = ConfigDict(extra="ignore")

    topic: str = Field(..., min_length=1)
    stamp_source: StampSource | None = None


class SourceEpisodeMeta(BaseModel):
    model_config = ConfigDict(extra="ignore")

    schema_version: Literal[2]
    episode_id: str = Field(..., min_length=1)
    state: str
    task_description: str = ""
    profile: str = Field(..., min_length=1)
    started_at: str
    stopped_at: str | None = None
    duration_s: float | None = Field(default=None, ge=0.0)
    outcome: str | None = None
    tags: list[str] = Field(default_factory=list)
    cameras: list[RecorderCameraMeta] = Field(default_factory=list)
    recorded_topics: list[RecordedTopicMeta] = Field(default_factory=list)
    bag_path: str = "bag/"

def _read_current_source_episode(
    raw_meta: JsonValue,
    meta_path: Path,
) -> SourceEpisodeMeta:
    if not isinstance(raw_meta, dict):
        raise LerobotDatasetExportError(
            "episode_schema_version_missing",
            f"episode meta must be a JSON object: {meta_path}",
        )
    schema_version = raw_meta.get("schema_version")
    if type(schema_version) is not int or schema_version <= 0:
        raise LerobotDatasetExportError(
            "episode_schema_version_missing",
            f"episode meta must declare a positive integer schema_version: {meta_path}",
        )
    if schema_version != 2:
        raise LerobotDatasetExportError(
            "episode_schema_version_unsupported",
            f"episode schema_version {schema_version} is not current; "
            f"expected 2: {meta_path}",
        )
    try:
        return SourceEpisodeMeta.model_validate(raw_meta)
    except ValidationError as exc:
        raise LerobotDatasetExportError(
            "episode_meta_invalid",
            f"episode meta is invalid: {meta_path}",
        ) from exc


class FrameSidecarRow(BaseModel):
    frame_index: int
    segment_file: str
    segment_local_frame: int
    video_pts: int = Field(..., ge=0)
    ros_stamp_ns: int
    recv_stamp_ns: int
    source_seq: int
    dropped_before: int
    keyframe: bool


class SourceEvent(BaseModel):
    stamp_ns: int
    source: str


class JointSample(BaseModel):
    stamp_ns: int
    names: list[str]
    positions: list[float]


class SourceEpisodeProvenance(BaseModel):
    episode_id: str
    task_description: str
    started_at: str
    stopped_at: str | None
    duration_s: float | None
    source_tags: list[str]


class LerobotDatasetExportProvenance(BaseModel):
    dataset_id: str
    dataset_name: str
    exported_at: str
    exporter: str = "fv_lerobot_exporter"
    source_episodes: list[SourceEpisodeProvenance]


@dataclass(frozen=True)
class EpisodeForExport:
    meta: SourceEpisodeMeta
    episode_dir: Path


@dataclass(frozen=True)
class CameraExportMapping:
    feature_key: str
    lerobot_name: str
    recorder_name: str
    topic: str
    width: int
    height: int
    source_video: Path
    frame_pts_seconds: list[float]
    video_origin_ros_ns: int
    from_timestamp_s: float
    sidecar_rows: list[FrameSidecarRow]


@dataclass(frozen=True)
class EpisodeExportData:
    rows: list[dict[str, LerobotDataValue]]
    videos: dict[str, "VideoExportData"]
    task_description: str
    started_at: str
    stopped_at: str | None
    duration_s: float | None
    source_tags: list[str]


@dataclass(frozen=True)
class VideoExportData:
    source_video: Path
    width: int
    height: int
    from_timestamp_s: float


@dataclass(frozen=True)
class BagTopicData:
    joint_samples_by_topic: dict[str, list[JointSample]]
    source_events: list[SourceEvent]


@dataclass(frozen=True)
class JointSampleIndex:
    samples: list[JointSample]
    stamps: list[int]


@dataclass(frozen=True)
class SourceEventIndex:
    events: list[SourceEvent]
    stamps: list[int]


@dataclass(frozen=True)
class CameraFrameIndex:
    rows: list[FrameSidecarRow]
    pts_seconds: list[float]


@dataclass(frozen=True)
class AlignedJointSample:
    sample: JointSample
    origin_grid_index: int


@dataclass(frozen=True)
class AlignedCameraFrame:
    frame: FrameSidecarRow
    pts_s: float
    origin_grid_index: int


CarryValueT = TypeVar("CarryValueT", AlignedJointSample, AlignedCameraFrame)


def export_lerobot_dataset(
    *,
    request: LerobotDatasetExportRequest,
    profile_payload: Mapping[str, JsonValue],
    datasets_dir: Path,
) -> LerobotDatasetExportResponse:
    root = datasets_dir.resolve()
    episodes = _load_episodes_for_export(root, request.episode_ids)
    profile_name = episodes[0].meta.profile
    if any(ep.meta.profile != profile_name for ep in episodes):
        raise LerobotDatasetExportError(
            "profile_mismatch", "all selected episodes must use the same profile"
        )

    try:
        profile = ProfileExportSpec.model_validate(profile_payload)
    except ValidationError as exc:
        raise LerobotDatasetExportError(
            "profile_invalid",
            f"profile.lerobot export contract is invalid: {exc}",
        ) from exc
    dataset_root = _safe_dataset_root(root, request.dataset_id)
    tmp_root = _safe_tmp_root(root, request.dataset_id)
    if dataset_root.exists():
        raise LerobotDatasetExportError(
            "dataset_exists", f"dataset already exists: {dataset_root}"
        )
    if tmp_root.exists():
        raise LerobotDatasetExportError(
            "export_tmp_exists", f"stale export tmp exists: {tmp_root}"
        )

    try:
        episode_exports = _convert_episodes(
            episodes=episodes,
            profile=profile,
            fps=request.fps,
            max_alignment_error_s=request.max_alignment_error_s,
            max_mux_status_age_s=request.max_mux_status_age_s,
        )
        _validate_video_feature_sets(episode_exports)
        _write_lerobot_v3(
            tmp_root=tmp_root,
            dataset_id=request.dataset_id,
            dataset_name=request.dataset_name,
            fps=request.fps,
            video_timestamp_tolerance_s=request.max_alignment_error_s,
            profile=profile,
            episode_exports=episode_exports,
        )
        exported_at = _utc_now_iso()
        provenance = LerobotDatasetExportProvenance(
            dataset_id=request.dataset_id,
            dataset_name=request.dataset_name,
            exported_at=exported_at,
            source_episodes=[
                SourceEpisodeProvenance(
                    episode_id=episode.meta.episode_id,
                    task_description=data.task_description,
                    started_at=data.started_at,
                    stopped_at=data.stopped_at,
                    duration_s=data.duration_s,
                    source_tags=data.source_tags,
                )
                for episode, data in zip(episodes, episode_exports, strict=True)
            ],
        )
        provenance_path = tmp_root / "meta" / "fv_episode_export.json"
        provenance_path.write_text(
            provenance.model_dump_json(indent=2), encoding="utf-8"
        )

        dataset_root.parent.mkdir(parents=True, exist_ok=True)
        tmp_root.rename(dataset_root)
    except Exception:
        if tmp_root.exists():
            shutil.rmtree(tmp_root, ignore_errors=True)
        raise

    return LerobotDatasetExportResponse(
        dataset_id=request.dataset_id,
        dataset_name=request.dataset_name,
        dataset_path=str(dataset_root),
        episode_count=len(episode_exports),
        frame_count=sum(len(data.rows) for data in episode_exports),
        exported_at=exported_at,
        provenance_path=str(dataset_root / "meta" / "fv_episode_export.json"),
    )


def _load_episodes_for_export(
    root: Path, episode_ids: list[str]
) -> list[EpisodeForExport]:
    episodes_root = root / "episodes"
    if not episodes_root.exists():
        raise LerobotDatasetExportError(
            "episode_store_missing", f"episode store missing: {episodes_root}"
        )
    requested_ids = set(episode_ids)
    matches: dict[str, dict[Path, EpisodeForExport]] = {
        episode_id: {} for episode_id in episode_ids
    }
    for meta_path in episodes_root.glob("*/*/*/meta.json"):
        try:
            raw_meta = json.loads(meta_path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            raise LerobotDatasetExportError(
                "episode_meta_invalid",
                f"episode meta is invalid: {meta_path}",
            ) from exc
        if not isinstance(raw_meta, dict):
            raise LerobotDatasetExportError(
                "episode_schema_version_missing",
                f"episode meta must be a JSON object: {meta_path}",
            )
        raw_episode_id = raw_meta.get("episode_id")
        if raw_episode_id not in requested_ids:
            continue
        meta = _read_current_source_episode(raw_meta, meta_path)
        episode_dir = meta_path.parent.resolve()
        matches[meta.episode_id][episode_dir] = EpisodeForExport(
            meta=meta,
            episode_dir=episode_dir,
        )

    episodes: list[EpisodeForExport] = []
    for episode_id in episode_ids:
        episode_matches = matches[episode_id]
        if len(episode_matches) > 1:
            raise LerobotDatasetExportError(
                "episode_id_ambiguous",
                f"episode_id {episode_id} maps to multiple directories: "
                f"{sorted(str(path) for path in episode_matches)}",
            )
        if not episode_matches:
            raise LerobotDatasetExportError(
                "episode_not_found", f"episode not found: {episode_id}"
            )
        episode = next(iter(episode_matches.values()))
        if episode.meta.state != "finished":
            raise LerobotDatasetExportError(
                "episode_not_finished",
                f"episode {episode_id} state is '{episode.meta.state}', not 'finished'",
            )
        if episode.meta.outcome != "success":
            raise LerobotDatasetExportError(
                "episode_not_success",
                f"episode {episode_id} outcome is '{episode.meta.outcome}', not 'success'",
            )
        episodes.append(episode)
    return episodes


def _convert_episodes(
    *,
    episodes: list[EpisodeForExport],
    profile: ProfileExportSpec,
    fps: int,
    max_alignment_error_s: float,
    max_mux_status_age_s: float,
) -> list[EpisodeExportData]:
    episode_total = len(episodes)
    if episode_total == 1:
        episode = episodes[0]
        try:
            converted_episode = _convert_episode(
                episode=episode,
                profile=profile,
                fps=fps,
                max_alignment_error_s=max_alignment_error_s,
                max_mux_status_age_s=max_mux_status_age_s,
            )
        except LerobotDatasetExportError as exc:
            raise LerobotDatasetExportError(
                exc.code,
                f"episode {episode.meta.episode_id}: {exc.detail}",
            ) from exc
        return [converted_episode]

    converted: list[EpisodeExportData | None] = [None] * episode_total
    executor = _alignment_executor()
    futures: dict[Future[EpisodeExportData], tuple[int, EpisodeForExport]] = {}
    next_episode_index = 0

    def submit_next_episode() -> None:
        nonlocal next_episode_index
        if next_episode_index >= episode_total:
            return
        episode = episodes[next_episode_index]
        future = executor.submit(
            _convert_episode,
            episode=episode,
            profile=profile,
            fps=fps,
            max_alignment_error_s=max_alignment_error_s,
            max_mux_status_age_s=max_mux_status_age_s,
        )
        futures[future] = (next_episode_index, episode)
        next_episode_index += 1

    def cancel_and_drain_outstanding() -> None:
        if not futures:
            return
        for outstanding in futures:
            outstanding.cancel()
        wait(tuple(futures))

    try:
        for _ in range(_export_worker_count(episode_total)):
            submit_next_episode()

        while futures:
            done, _ = wait(tuple(futures), return_when=FIRST_COMPLETED)
            for future in done:
                episode_index, episode = futures.pop(future)
                try:
                    converted_episode = future.result()
                except LerobotDatasetExportError as exc:
                    raise LerobotDatasetExportError(
                        exc.code,
                        f"episode {episode.meta.episode_id}: {exc.detail}",
                    ) from exc
                converted[episode_index] = converted_episode
                submit_next_episode()
    except BaseException as exc:
        try:
            cancel_and_drain_outstanding()
        except BaseException as cleanup_exc:
            raise exc from cleanup_exc
        raise

    result: list[EpisodeExportData] = []
    for episode in converted:
        if episode is None:
            raise RuntimeError("episode conversion worker returned no result")
        result.append(episode)
    return result


def _safe_dataset_root(root: Path, dataset_id: str) -> Path:
    target = (root / dataset_id).resolve()
    if root not in target.parents:
        raise LerobotDatasetExportError(
            "invalid_dataset_id", "dataset_id escapes datasets directory"
        )
    return target


def _safe_tmp_root(root: Path, dataset_id: str) -> Path:
    target = (root / ".lerobot_exports" / dataset_id).resolve()
    if root not in target.parents:
        raise LerobotDatasetExportError(
            "invalid_dataset_id", "dataset_id escapes datasets directory"
        )
    return target


def _convert_episode(
    *,
    episode: EpisodeForExport,
    profile: ProfileExportSpec,
    fps: int,
    max_alignment_error_s: float,
    max_mux_status_age_s: float,
) -> EpisodeExportData:
    camera_mappings = _resolve_camera_mappings(episode, profile)
    stamp_sources = _recorded_topic_stamp_sources(episode.meta)
    required_bag_topics = _required_bag_topics(profile)
    optional_action_topics = _configured_action_bag_topics(profile)
    bag = _read_bag_topics(
        episode.episode_dir / episode.meta.bag_path,
        required_bag_topics,
        optional_action_topics,
        stamp_sources,
    )
    if not bag.source_events:
        raise LerobotDatasetExportError(
            "mux_status_missing",
            f"episode {episode.meta.episode_id} has no mux status samples",
        )
    frame_stamps = _fixed_fps_timeline_from_recorded_data(
        camera_mappings=camera_mappings,
        bag=bag,
        profile=profile,
        fps=fps,
    )
    grid_start_ros_ns = frame_stamps[0]
    camera_mappings = [
        replace(
            mapping,
            from_timestamp_s=(
                grid_start_ros_ns - mapping.video_origin_ros_ns
            )
            / 1_000_000_000.0,
        )
        for mapping in camera_mappings
    ]
    source_events = SourceEventIndex(
        events=bag.source_events,
        stamps=[event.stamp_ns for event in bag.source_events],
    )
    joint_samples_by_topic = {
        topic: JointSampleIndex(
            samples=samples,
            stamps=[sample.stamp_ns for sample in samples],
        )
        for topic, samples in bag.joint_samples_by_topic.items()
    }
    camera_frames_by_feature = {
        mapping.feature_key: CameraFrameIndex(
            rows=mapping.sidecar_rows,
            pts_seconds=mapping.frame_pts_seconds,
        )
        for mapping in camera_mappings
    }

    arm_streams = profile.lerobot.arm_streams
    state_names = _joint_feature_names(arm_streams)
    last_state_samples: dict[str, AlignedJointSample] = {}
    last_action_samples: dict[str, AlignedJointSample] = {}
    last_camera_frames: dict[str, AlignedCameraFrame] = {}
    rows: list[dict[str, LerobotDataValue]] = []
    for grid_index, stamp_ns in enumerate(frame_stamps):
        source = _source_at(source_events, stamp_ns, max_mux_status_age_s)
        state = _combined_joint_values(
            arm_streams=arm_streams,
            samples_by_topic=joint_samples_by_topic,
            stamp_ns=stamp_ns,
            grid_index=grid_index,
            kind="state",
            source=source,
            max_alignment_error_s=max_alignment_error_s,
            last_samples_by_topic=last_state_samples,
        )
        action = _combined_joint_values(
            arm_streams=arm_streams,
            samples_by_topic=joint_samples_by_topic,
            stamp_ns=stamp_ns,
            grid_index=grid_index,
            kind="action",
            source=source,
            max_alignment_error_s=max_alignment_error_s,
            last_samples_by_topic=last_action_samples,
        )
        for mapping in camera_mappings:
            _camera_frame_with_carry(
                frame_index=camera_frames_by_feature[mapping.feature_key],
                query_timestamp_s=mapping.from_timestamp_s + grid_index / fps,
                grid_index=grid_index,
                max_error_s=max_alignment_error_s,
                feature_key=mapping.feature_key,
                last_frames_by_feature=last_camera_frames,
            )
        if len(state) != len(state_names) or len(action) != len(state_names):
            raise LerobotDatasetExportError(
                "joint_vector_mismatch",
                f"episode {episode.meta.episode_id} produced invalid joint vector length",
            )
        rows.append(
            {
                "observation.state": state,
                "action": action,
                "timestamp": grid_index / fps,
                "frame_index": grid_index,
            }
        )

    videos: dict[str, VideoExportData] = {}
    for mapping in camera_mappings:
        videos[mapping.feature_key] = VideoExportData(
            source_video=mapping.source_video,
            width=mapping.width,
            height=mapping.height,
            from_timestamp_s=mapping.from_timestamp_s,
        )
    return EpisodeExportData(
        rows=rows,
        videos=videos,
        task_description=episode.meta.task_description,
        started_at=episode.meta.started_at,
        stopped_at=episode.meta.stopped_at,
        duration_s=episode.meta.duration_s,
        source_tags=episode.meta.tags,
    )


def _fixed_fps_timeline_from_recorded_data(
    *,
    camera_mappings: list[CameraExportMapping],
    bag: BagTopicData,
    profile: ProfileExportSpec,
    fps: int,
) -> list[int]:
    camera_first_stamps = [
        mapping.sidecar_rows[0].ros_stamp_ns for mapping in camera_mappings
    ]
    last_stamps = [mapping.sidecar_rows[-1].ros_stamp_ns for mapping in camera_mappings]
    for stream in profile.lerobot.arm_streams:
        topic = stream.rx.joint_state.topic
        samples = bag.joint_samples_by_topic.get(topic)
        if not samples:
            raise LerobotDatasetExportError(
                "joint_samples_missing",
                f"no samples for required joint state topic {topic}",
            )
        last_stamps.append(samples[-1].stamp_ns)

    # Hardlinked videos start at local PTS zero, so the dataset timeline cannot
    # query before any camera origin. This boundary selection does not make a
    # camera the master clock; the fixed-FPS grid remains anchored to ROS time zero.
    available_start_ros_ns = max(camera_first_stamps)
    available_end_ros_ns = max(last_stamps)
    return _fixed_fps_timeline_for_interval(
        available_start_ros_ns=available_start_ros_ns,
        available_end_ros_ns=available_end_ros_ns,
        fps=fps,
    )


def _fixed_fps_timeline_for_interval(
    *, available_start_ros_ns: int, available_end_ros_ns: int, fps: int
) -> list[int]:
    if available_end_ros_ns < available_start_ros_ns:
        raise LerobotDatasetExportError(
            "episode_timeline_empty",
            "camera and required joint streams have no common ROS timestamp interval",
        )

    first_grid_index = (
        available_start_ros_ns * fps + 1_000_000_000 - 1
    ) // 1_000_000_000
    last_grid_index = available_end_ros_ns * fps // 1_000_000_000
    stamps = [
        (grid_index * 1_000_000_000 + fps // 2) // fps
        for grid_index in range(first_grid_index, last_grid_index + 1)
    ]
    if not stamps:
        raise LerobotDatasetExportError(
            "episode_timeline_empty",
            "camera and required joint streams share no fixed-FPS grid timestamp",
        )
    return stamps


def _required_bag_topics(profile: ProfileExportSpec) -> set[str]:
    topics = {
        stream.rx.joint_state.topic for stream in profile.lerobot.arm_streams
    }
    topics.add(_mux_status_topic(profile))
    return topics


def _configured_action_bag_topics(profile: ProfileExportSpec) -> set[str]:
    topics: set[str] = set()
    for stream in profile.lerobot.arm_streams:
        if stream.rx.joint_command.leader is not None:
            topics.add(stream.rx.joint_command.leader.topic)
        if stream.rx.joint_command.vr is not None:
            topics.add(stream.rx.joint_command.vr.topic)
        if stream.rx.joint_command.ai is not None:
            topics.add(stream.rx.joint_command.ai.topic)
    return topics


def _mux_status_topic(profile: ProfileExportSpec) -> str:
    for stream in profile.lerobot.arm_streams:
        if stream.namespace:
            return f"/{stream.namespace.strip('/')}/teleop_mux/status"
    raise LerobotDatasetExportError(
        "mux_topic_not_configured", "profile.lerobot has no arm namespace"
    )


def _recorded_topic_stamp_sources(meta: SourceEpisodeMeta) -> dict[str, StampSource]:
    stamp_sources: dict[str, StampSource] = {}
    for entry in meta.recorded_topics:
        if entry.stamp_source is not None:
            stamp_sources[entry.topic] = entry.stamp_source
    return stamp_sources


def _read_bag_topics(
    bag_dir: Path,
    required_topics: set[str],
    optional_action_topics: set[str],
    stamp_sources: Mapping[str, StampSource],
) -> BagTopicData:
    if not bag_dir.exists():
        raise LerobotDatasetExportError(
            "bag_missing", f"bag directory missing: {bag_dir}"
        )

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([bag_dir], default_typestore=typestore) as reader:
        type_by_topic = {
            connection.topic: connection.msgtype for connection in reader.connections
        }
        read_topics = required_topics | optional_action_topics
        connections = [
            connection
            for connection in reader.connections
            if connection.topic in read_topics
        ]

        missing = sorted(
            topic for topic in required_topics if topic not in type_by_topic
        )
        if missing:
            raise LerobotDatasetExportError(
                "bag_topic_missing", f"required topics missing from bag: {missing}"
            )

        mux_topic = next(
            topic for topic in required_topics if topic.endswith("/teleop_mux/status")
        )
        joint_samples: dict[str, list[JointSample]] = {
            topic: []
            for topic in read_topics
            if topic != mux_topic and topic in type_by_topic
        }
        source_events: list[SourceEvent] = []

        for connection, bag_stamp_ns, raw_data in reader.messages(
            connections=connections
        ):
            topic = connection.topic
            msg_type = connection.msgtype
            if topic == mux_topic:
                if msg_type != "std_msgs/msg/String":
                    raise LerobotDatasetExportError(
                        "mux_status_type_mismatch",
                        f"{topic} type is {msg_type}, expected std_msgs/msg/String",
                    )
                msg = typestore.deserialize_cdr(raw_data, msg_type)
                source_events.append(
                    _parse_source_event(str(msg.data), int(bag_stamp_ns))
                )
                continue

            if msg_type != "sensor_msgs/msg/JointState":
                raise LerobotDatasetExportError(
                    "joint_topic_type_mismatch",
                    f"{topic} type is {msg_type}, expected sensor_msgs/msg/JointState",
                )
            msg = typestore.deserialize_cdr(raw_data, msg_type)
            stamp_ns = _joint_stamp_ns(
                topic=topic,
                sec=int(msg.header.stamp.sec),
                nanosec=int(msg.header.stamp.nanosec),
                bag_stamp_ns=int(bag_stamp_ns),
                stamp_sources=stamp_sources,
            )
            names = [str(name) for name in msg.name]
            positions = [float(value) for value in msg.position]
            if not names:
                raise LerobotDatasetExportError(
                    "joint_names_missing", f"{topic} JointState has no joint names"
                )
            if len(names) > len(positions):
                raise LerobotDatasetExportError(
                    "joint_position_missing", f"{topic} has fewer positions than names"
                )
            joint_samples[topic].append(
                JointSample(stamp_ns=stamp_ns, names=names, positions=positions)
            )

    for samples in joint_samples.values():
        samples.sort(key=lambda sample: sample.stamp_ns)
    source_events.sort(key=lambda event: event.stamp_ns)
    return BagTopicData(
        joint_samples_by_topic=joint_samples, source_events=source_events
    )


def _joint_stamp_ns(
    *,
    topic: str,
    sec: int,
    nanosec: int,
    bag_stamp_ns: int,
    stamp_sources: Mapping[str, StampSource],
) -> int:
    source = stamp_sources.get(topic)
    if source == "rosbag_recv":
        return int(bag_stamp_ns)
    if source == "message_header":
        stamp_ns = sec * 1_000_000_000 + nanosec
        if stamp_ns <= 0:
            raise LerobotDatasetExportError(
                "message_header_stamp_missing",
                f"{topic} declares message_header stamp_source but header stamp is zero",
            )
        return stamp_ns
    raise LerobotDatasetExportError(
        "stamp_source_missing",
        f"{topic} has no recorded_topics stamp_source; export refuses implicit timestamp semantics",
    )


def _parse_source_event(raw_json: str, bag_stamp_ns: int) -> SourceEvent:
    try:
        payload = json.loads(raw_json)
    except json.JSONDecodeError as exc:
        raise LerobotDatasetExportError(
            "mux_status_invalid", "mux status is not valid JSON"
        ) from exc
    if not isinstance(payload, dict):
        raise LerobotDatasetExportError(
            "mux_status_invalid", "mux status JSON is not an object"
        )
    source = payload.get("source")
    if not isinstance(source, str) or not source:
        raise LerobotDatasetExportError(
            "mux_status_invalid", "mux status JSON has no source"
        )
    return SourceEvent(stamp_ns=int(bag_stamp_ns), source=source)


def _source_at(source_events: SourceEventIndex, stamp_ns: int, max_age_s: float) -> str:
    index = bisect.bisect_right(source_events.stamps, stamp_ns) - 1
    if index < 0:
        event = source_events.events[0]
        # teleop_mux/status is latched; rosbag may receive the first status
        # just after the first camera frame. Bound that startup lookahead.
        age_s = (event.stamp_ns - stamp_ns) / 1_000_000_000.0
        if age_s > max_age_s:
            raise LerobotDatasetExportError(
                "mux_status_before_frame_missing", "no mux status before frame"
            )
    else:
        event = source_events.events[index]
        age_s = (stamp_ns - event.stamp_ns) / 1_000_000_000.0
    if age_s > max_age_s:
        raise LerobotDatasetExportError(
            "mux_status_stale",
            f"mux source sample is stale: age={age_s:.3f}s max={max_age_s:.3f}s",
        )
    if event.source == "stop":
        raise LerobotDatasetExportError(
            "mux_source_stop", "cannot export action while mux source is stop"
        )
    return event.source


def _combined_joint_values(
    *,
    arm_streams: list[ArmStreamSpec],
    samples_by_topic: Mapping[str, JointSampleIndex],
    stamp_ns: int,
    grid_index: int,
    kind: Literal["state", "action"],
    source: str,
    max_alignment_error_s: float,
    last_samples_by_topic: dict[str, AlignedJointSample],
) -> list[float]:
    values: list[float] = []
    for stream in arm_streams:
        topic = (
            stream.rx.joint_state.topic
            if kind == "state"
            else stream.rx.joint_command.topic_for_source(source)
        )
        sample_index = samples_by_topic.get(topic)
        sample = _joint_sample_with_carry(
            sample_index=sample_index,
            stamp_ns=stamp_ns,
            grid_index=grid_index,
            max_error_s=max_alignment_error_s,
            kind=kind,
            topic=topic,
            last_samples_by_topic=last_samples_by_topic,
        )
        position_by_name = {
            name: sample.positions[index] for index, name in enumerate(sample.names)
        }
        missing_joints = [
            joint for joint in stream.joints if joint not in position_by_name
        ]
        if missing_joints:
            raise LerobotDatasetExportError(
                "joint_name_mismatch",
                f"{topic} is missing profile joints: {missing_joints}",
            )
        for joint in stream.joints:
            wire_value = float(position_by_name[joint])
            values.append(
                math.degrees(wire_value)
                if stream.unit_for(joint) == "degrees"
                else wire_value * 100.0
            )
    return values


def _joint_sample_with_carry(
    *,
    sample_index: JointSampleIndex | None,
    stamp_ns: int,
    grid_index: int,
    max_error_s: float,
    kind: Literal["state", "action"],
    topic: str,
    last_samples_by_topic: dict[str, AlignedJointSample],
) -> JointSample:
    previous = last_samples_by_topic.get(topic)
    try:
        if sample_index is None or not sample_index.samples:
            raise LerobotDatasetExportError(
                "samples_missing", f"no {kind} samples for {topic}"
            )
        sample = _nearest_sample(sample_index, stamp_ns, max_error_s)
    except LerobotDatasetExportError as exc:
        if exc.code not in {
            "sample_alignment_error",
            "sample_missing",
            "samples_missing",
        }:
            raise
        return _carry_previous_value(
            previous=previous,
            age_s=(
                (stamp_ns - previous.sample.stamp_ns) / 1_000_000_000.0
                if previous is not None
                else None
            ),
            grid_index=grid_index,
            max_error_s=max_error_s,
            stream=f"{kind} topic {topic}",
            cause=exc,
            allow_future=False,
        ).sample
    if previous is not None and sample is previous.sample:
        return _carry_previous_value(
            previous=previous,
            age_s=(stamp_ns - sample.stamp_ns) / 1_000_000_000.0,
            grid_index=grid_index,
            max_error_s=max_error_s,
            stream=f"{kind} topic {topic}",
            cause=None,
            allow_future=True,
        ).sample
    last_samples_by_topic[topic] = AlignedJointSample(
        sample=sample, origin_grid_index=grid_index
    )
    return sample


def _nearest_sample(
    sample_index: JointSampleIndex, stamp_ns: int, max_error_s: float
) -> JointSample:
    index = bisect.bisect_left(sample_index.stamps, stamp_ns)
    candidates: list[JointSample] = []
    if index < len(sample_index.samples):
        candidates.append(sample_index.samples[index])
    if index > 0:
        candidates.append(sample_index.samples[index - 1])
    if not candidates:
        raise LerobotDatasetExportError("sample_missing", "no sample candidates")
    nearest = min(candidates, key=lambda sample: abs(sample.stamp_ns - stamp_ns))
    error_s = abs(nearest.stamp_ns - stamp_ns) / 1_000_000_000.0
    if error_s > max_error_s:
        raise LerobotDatasetExportError(
            "sample_alignment_error",
            f"nearest sample is {error_s:.3f}s away, max={max_error_s:.3f}s",
        )
    return nearest


def _nearest_camera_frame(
    frame_index: CameraFrameIndex,
    query_timestamp_s: float,
    max_error_s: float,
) -> tuple[FrameSidecarRow, float]:
    if not frame_index.rows:
        raise LerobotDatasetExportError(
            "camera_frame_missing", "no camera frame candidates"
        )
    index = bisect.bisect_left(frame_index.pts_seconds, query_timestamp_s)
    candidate_indices: list[int] = []
    if index > 0:
        candidate_indices.append(index - 1)
    if index < len(frame_index.rows):
        candidate_indices.append(index)
    if not candidate_indices:
        raise LerobotDatasetExportError(
            "camera_frame_missing", "no camera frame candidates"
        )
    nearest_index = min(
        candidate_indices,
        key=lambda candidate_index: abs(
            frame_index.pts_seconds[candidate_index] - query_timestamp_s
        ),
    )
    nearest = frame_index.rows[nearest_index]
    nearest_pts_s = frame_index.pts_seconds[nearest_index]
    error_s = abs(nearest_pts_s - query_timestamp_s)
    if error_s > max_error_s:
        raise LerobotDatasetExportError(
            "camera_alignment_error",
            f"nearest camera frame is {error_s:.3f}s away, max={max_error_s:.3f}s",
        )
    return nearest, nearest_pts_s


def _camera_frame_with_carry(
    *,
    frame_index: CameraFrameIndex,
    query_timestamp_s: float,
    grid_index: int,
    max_error_s: float,
    feature_key: str,
    last_frames_by_feature: dict[str, AlignedCameraFrame],
) -> FrameSidecarRow:
    previous = last_frames_by_feature.get(feature_key)
    try:
        frame, frame_pts_s = _nearest_camera_frame(
            frame_index,
            query_timestamp_s,
            max_error_s,
        )
    except LerobotDatasetExportError as exc:
        if exc.code not in {"camera_alignment_error", "camera_frame_missing"}:
            raise
        return _carry_previous_value(
            previous=previous,
            age_s=(
                query_timestamp_s - previous.pts_s if previous is not None else None
            ),
            grid_index=grid_index,
            max_error_s=max_error_s,
            stream=f"camera {feature_key}",
            cause=exc,
            allow_future=False,
        ).frame
    if previous is not None and (
        frame.frame_index == previous.frame.frame_index
        and frame.ros_stamp_ns == previous.frame.ros_stamp_ns
    ):
        return _carry_previous_value(
            previous=previous,
            age_s=query_timestamp_s - frame_pts_s,
            grid_index=grid_index,
            max_error_s=max_error_s,
            stream=f"camera {feature_key}",
            cause=None,
            allow_future=True,
        ).frame
    last_frames_by_feature[feature_key] = AlignedCameraFrame(
        frame=frame,
        pts_s=frame_pts_s,
        origin_grid_index=grid_index,
    )
    return frame


def _carry_previous_value(
    *,
    previous: CarryValueT | None,
    age_s: float | None,
    grid_index: int,
    max_error_s: float,
    stream: str,
    cause: LerobotDatasetExportError | None,
    allow_future: bool,
) -> CarryValueT:
    if previous is None or age_s is None:
        detail = cause.detail if cause is not None else "no previously aligned value"
        raise LerobotDatasetExportError(
            "episode_alignment_gap",
            f"{stream} is missing at the first grid frame: {detail}",
        ) from cause
    carry_frames = grid_index - previous.origin_grid_index
    # Reuse is safe only while both agreed bounds hold. Fallback carry never
    # adopts a future value; direct nearest selection may reuse one within tolerance.
    if (
        carry_frames > MAX_VIDEO_TIMESTAMP_FRAME_DISTANCE
        or (age_s < 0.0 and not allow_future)
        or abs(age_s) > max_error_s
    ):
        detail = f"; {cause.detail}" if cause is not None else ""
        raise LerobotDatasetExportError(
            "episode_alignment_gap",
            f"{stream} cannot carry to grid frame {grid_index}: "
            f"carry_frames={carry_frames} age={age_s:.3f}s "
            f"max_frames={MAX_VIDEO_TIMESTAMP_FRAME_DISTANCE} "
            f"max_age={max_error_s:.3f}s{detail}",
        ) from cause
    return previous


def _resolve_camera_mappings(
    episode: EpisodeForExport,
    profile: ProfileExportSpec,
) -> list[CameraExportMapping]:
    episode_camera_by_topic = {camera.topic: camera for camera in episode.meta.cameras}
    mappings: list[CameraExportMapping] = []
    for camera in profile.lerobot.cameras:
        if not camera.enabled:
            continue
        episode_camera = episode_camera_by_topic.get(camera.topic)
        if episode_camera is None:
            raise LerobotDatasetExportError(
                "camera_missing",
                f"required profile camera {camera.name} topic {camera.topic} missing from episode",
            )
        if len(episode_camera.segments) != 1:
            raise LerobotDatasetExportError(
                "camera_segments_unsupported",
                f"camera {episode_camera.name} must have exactly one mp4 segment",
            )
        segment = episode_camera.segments[0]
        source_video = (
            episode.episode_dir / "videos" / episode_camera.name / segment.file
        )
        if not source_video.exists():
            raise LerobotDatasetExportError(
                "camera_video_missing", f"video missing: {source_video}"
            )
        _require_read_only_source(source_video, "video_source_mutable")
        sidecar_path = (
            episode.episode_dir / "videos" / episode_camera.name / "frames.parquet"
        )
        if sidecar_path.exists():
            _require_read_only_source(sidecar_path, "frames_sidecar_mutable")
        sidecar_rows = _read_sidecar_rows(sidecar_path)
        if not sidecar_rows:
            raise LerobotDatasetExportError(
                "camera_sidecar_empty",
                f"camera {episode_camera.name} frames sidecar is empty",
            )
        if any(
            row.segment_file != segment.file or row.segment_local_frame != index
            for index, row in enumerate(sidecar_rows)
        ):
            raise LerobotDatasetExportError(
                "camera_sidecar_video_mismatch",
                f"camera {episode_camera.name} sidecar does not map one-to-one to {segment.file}",
            )
        video_origin_ros_ns = sidecar_rows[0].ros_stamp_ns
        expected_video_pts = [
            round(
                (row.ros_stamp_ns - video_origin_ros_ns)
                * RECORDER_VIDEO_TIME_BASE_DENOMINATOR
                / 1_000_000_000
            )
            for row in sidecar_rows
        ]
        if any(
            row.video_pts != expected
            for row, expected in zip(sidecar_rows, expected_video_pts)
        ):
            raise LerobotDatasetExportError(
                "camera_video_pts_ros_stamp_mismatch",
                f"camera {episode_camera.name} sidecar PTS does not match ROS timestamps",
            )
        frame_pts_seconds = [
            row.video_pts / RECORDER_VIDEO_TIME_BASE_DENOMINATOR
            for row in sidecar_rows
        ]
        if frame_pts_seconds[0] != 0.0 or any(
            right <= left
            for left, right in zip(frame_pts_seconds, frame_pts_seconds[1:])
        ):
            raise LerobotDatasetExportError(
                "camera_video_pts_invalid",
                f"camera {episode_camera.name} video PTS must start at zero and increase strictly",
            )
        width = int(episode_camera.width or 0)
        height = int(episode_camera.height or 0)
        if width <= 0 or height <= 0:
            width, height = _probe_video_size(source_video)
        mappings.append(
            CameraExportMapping(
                feature_key=f"observation.images.{camera.name}",
                lerobot_name=camera.name,
                recorder_name=episode_camera.name,
                topic=camera.topic,
                width=width,
                height=height,
                source_video=source_video,
                frame_pts_seconds=frame_pts_seconds,
                video_origin_ros_ns=video_origin_ros_ns,
                from_timestamp_s=0.0,
                sidecar_rows=sidecar_rows,
            )
        )
    if not mappings:
        raise LerobotDatasetExportError(
            "camera_mapping_empty", "no profile.lerobot cameras matched episode cameras"
        )
    return mappings


def _read_sidecar_rows(path: Path) -> list[FrameSidecarRow]:
    if not path.exists():
        raise LerobotDatasetExportError(
            "frames_sidecar_missing", f"frames sidecar missing: {path}"
        )
    table = pq.read_table(path)
    rows = [FrameSidecarRow.model_validate(row) for row in table.to_pylist()]
    if any(
        right.ros_stamp_ns <= left.ros_stamp_ns for left, right in zip(rows, rows[1:])
    ):
        raise LerobotDatasetExportError(
            "camera_timestamps_invalid",
            f"camera frame timestamps are not strictly increasing: {path}",
        )
    return rows


def _probe_video_size(path: Path) -> tuple[int, int]:
    try:
        info = get_video_info(path)
    except Exception as exc:
        raise LerobotDatasetExportError(
            "video_probe_failed", f"cannot probe video size: {path}"
        ) from exc
    width = int(info.get("video.width") or 0)
    height = int(info.get("video.height") or 0)
    if width <= 0 or height <= 0:
        raise LerobotDatasetExportError(
            "video_probe_failed", f"cannot probe video size: {path}"
        )
    return width, height


def _hardlink_video(source: Path, destination: Path) -> None:
    _require_read_only_source(source, "video_source_mutable")
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.unlink(missing_ok=True)
    try:
        destination.hardlink_to(source)
    except OSError as exc:
        destination.unlink(missing_ok=True)
        raise LerobotDatasetExportError(
            "video_hardlink_failed",
            f"cannot hardlink immutable episode video {source} to {destination}: {exc}",
        ) from exc


def _require_read_only_source(path: Path, error_code: str) -> None:
    mode = stat.S_IMODE(path.stat().st_mode)
    write_bits = stat.S_IWUSR | stat.S_IWGRP | stat.S_IWOTH
    if mode & write_bits:
        raise LerobotDatasetExportError(
            error_code,
            f"finished episode source must be read-only before export: {path} mode={mode:o}",
        )


def _validate_video_feature_sets(episode_exports: list[EpisodeExportData]) -> None:
    expected_videos = episode_exports[0].videos
    expected = set(expected_videos)
    for episode in episode_exports[1:]:
        current = set(episode.videos)
        if current != expected:
            raise LerobotDatasetExportError(
                "camera_feature_mismatch",
                f"selected episodes have different camera features: expected={sorted(expected)} actual={sorted(current)}",
            )
        for feature_key, expected_video in expected_videos.items():
            video = episode.videos[feature_key]
            if (video.width, video.height) != (
                expected_video.width,
                expected_video.height,
            ):
                raise LerobotDatasetExportError(
                    "camera_resolution_mismatch",
                    f"{feature_key} resolution differs between episodes: "
                    f"expected={expected_video.width}x{expected_video.height} "
                    f"actual={video.width}x{video.height}",
                )


def _write_lerobot_v3(
    *,
    tmp_root: Path,
    dataset_id: str,
    dataset_name: str,
    fps: int,
    video_timestamp_tolerance_s: float,
    profile: ProfileExportSpec,
    episode_exports: list[EpisodeExportData],
) -> None:
    features = _features(profile, episode_exports)
    meta = LeRobotDatasetMetadata.create(
        repo_id=dataset_id,
        fps=fps,
        features=features,
        robot_type="vlabor",
        root=tmp_root,
        use_videos=True,
        video_files_size_in_mb=500000,
    )

    all_rows: list[dict[str, LerobotDataValue]] = []
    episode_rows: list[dict[str, LerobotEpisodeValue]] = []
    global_index = 0
    task_index_by_name: dict[str, int] = {}

    for episode_index, episode_data in enumerate(episode_exports):
        task_index = task_index_by_name.setdefault(
            episode_data.task_description, len(task_index_by_name)
        )
        start_index = global_index
        for frame in episode_data.rows:
            row = dict(frame)
            row["episode_index"] = episode_index
            row["index"] = global_index
            row["task_index"] = task_index
            all_rows.append(row)
            global_index += 1
        episode_row: dict[str, LerobotEpisodeValue] = {
            "episode_index": episode_index,
            "tasks": [episode_data.task_description],
            "length": len(episode_data.rows),
            "data/chunk_index": 0,
            "data/file_index": 0,
            "dataset_from_index": start_index,
            "dataset_to_index": global_index,
        }
        for feature_key in episode_data.videos:
            video = episode_data.videos[feature_key]
            episode_row[f"videos/{feature_key}/chunk_index"] = 0
            episode_row[f"videos/{feature_key}/file_index"] = episode_index
            episode_row[f"videos/{feature_key}/from_timestamp"] = video.from_timestamp_s
            episode_row[f"videos/{feature_key}/to_timestamp"] = (
                video.from_timestamp_s + len(episode_data.rows) / fps
            )
        episode_rows.append(episode_row)

    written_video_paths_by_feature = _write_dataset_videos(
        root=meta.root,
        video_path_template=meta.video_path,
        episode_exports=episode_exports,
    )
    for feature_key, video_paths in written_video_paths_by_feature.items():
        meta.info["features"][feature_key]["info"] = get_video_info(video_paths[0])
        meta.info["features"][feature_key]["info"]["video.fps"] = fps
    meta.info["dataset_name"] = dataset_name
    meta.info["video_timestamp_tolerance_s"] = video_timestamp_tolerance_s
    meta.info[VIDEO_QUERY_TIMESTAMP_SOURCE_KEY] = (
        VIDEO_QUERY_TIMESTAMP_SOURCE_FRAME_INDEX
    )
    meta.info["repo_id"] = dataset_id
    meta.info["total_episodes"] = len(episode_exports)
    meta.info["total_frames"] = len(all_rows)
    meta.info["total_tasks"] = len(task_index_by_name)
    meta.info["splits"] = {"train": f"0:{len(episode_exports)}"}
    write_info(meta.info, meta.root)
    _write_data_parquet(meta, all_rows)
    _write_episode_parquet(meta.root, episode_rows)
    _write_tasks_parquet(meta.root, task_index_by_name)
    _write_stats(meta.root, all_rows)


def _features(
    profile: ProfileExportSpec,
    episode_exports: list[EpisodeExportData],
) -> dict[str, dict[str, JsonValue]]:
    names = _joint_feature_names(profile.lerobot.arm_streams)
    features: dict[str, dict[str, JsonValue]] = {
        "observation.state": {
            "dtype": "float32",
            "shape": [len(names)],
            "names": names,
        },
        "action": {"dtype": "float32", "shape": [len(names)], "names": names},
    }
    for feature_key, video in episode_exports[0].videos.items():
        features[feature_key] = {
            "dtype": "video",
            "shape": [video.height, video.width, 3],
            "names": ["height", "width", "channel"],
        }
    return features


def _joint_feature_names(arm_streams: list[ArmStreamSpec]) -> list[str]:
    names: list[str] = []
    for stream in arm_streams:
        prefix = stream.namespace.strip("/") if stream.namespace else stream.key
        names.extend([f"{prefix}_{joint}" for joint in stream.joints])
    return names


def _write_dataset_videos(
    *,
    root: Path,
    video_path_template: str | None,
    episode_exports: list[EpisodeExportData],
) -> dict[str, list[Path]]:
    if video_path_template is None:
        raise LerobotDatasetExportError(
            "video_path_missing", "LeRobot metadata has no video_path template"
        )
    feature_keys = list(episode_exports[0].videos)
    written_by_feature: dict[str, list[Path]] = {
        feature_key: [] for feature_key in feature_keys
    }
    for feature_key in feature_keys:
        for episode_index, episode_data in enumerate(episode_exports):
            video = episode_data.videos[feature_key]
            destination = root / video_path_template.format(
                video_key=feature_key,
                chunk_index=0,
                file_index=episode_index,
            )
            _hardlink_video(video.source_video, destination)
            written_by_feature[feature_key].append(destination)
    return written_by_feature


def _write_data_parquet(
    meta: LeRobotDatasetMetadata,
    rows: list[dict[str, LerobotDataValue]],
) -> None:
    path = meta.root / meta.data_path.format(chunk_index=0, file_index=0)
    path.parent.mkdir(parents=True, exist_ok=True)
    Dataset.from_list(
        rows, features=get_hf_features_from_features(meta.features)
    ).to_parquet(path)


def _write_episode_parquet(
    root: Path,
    rows: list[dict[str, LerobotEpisodeValue]],
) -> None:
    write_episodes(Dataset.from_list(rows), root)


def _write_tasks_parquet(root: Path, task_index_by_name: Mapping[str, int]) -> None:
    ordered = sorted(task_index_by_name.items(), key=lambda item: item[1])
    tasks = pd.DataFrame(
        {"task_index": [task_index for _, task_index in ordered]},
        index=[task for task, _ in ordered],
    )
    write_tasks(tasks, root)


def _write_stats(
    root: Path,
    rows: list[dict[str, LerobotDataValue]],
) -> None:
    numeric = {
        "observation.state": np.asarray(
            [row["observation.state"] for row in rows], dtype=np.float32
        ),
        "action": np.asarray([row["action"] for row in rows], dtype=np.float32),
    }
    stats: dict[str, dict[str, StatValue]] = {
        key: _numeric_stats(values) for key, values in numeric.items()
    }
    write_stats(stats, root)


def _numeric_stats(values: np.ndarray) -> dict[str, StatValue]:
    quantiles = np.quantile(values, [0.01, 0.10, 0.50, 0.90, 0.99], axis=0)
    return {
        "min": values.min(axis=0).astype(float).tolist(),
        "max": values.max(axis=0).astype(float).tolist(),
        "mean": values.mean(axis=0).astype(float).tolist(),
        "std": values.std(axis=0).astype(float).tolist(),
        "count": [int(values.shape[0])],
        "q01": quantiles[0].astype(float).tolist(),
        "q10": quantiles[1].astype(float).tolist(),
        "q50": quantiles[2].astype(float).tolist(),
        "q90": quantiles[3].astype(float).tolist(),
        "q99": quantiles[4].astype(float).tolist(),
    }


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")
