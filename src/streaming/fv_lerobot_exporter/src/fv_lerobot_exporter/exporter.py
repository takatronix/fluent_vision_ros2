"""LeRobot v3 dataset export for FluentVision recorder episodes."""

from __future__ import annotations

import bisect
import json
import os
import shutil
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from datetime import datetime, timezone
from functools import lru_cache
from multiprocessing import get_context, set_forkserver_preload
from pathlib import Path
from typing import Callable, Literal, Mapping, Self

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

from .timing import max_video_timestamp_tolerance_s


JsonValue = str | int | float | bool | None | list["JsonValue"] | dict[str, "JsonValue"]
StampSource = Literal["message_header", "rosbag_recv", "system"]
StatValue = list[float] | list[int] | list[list[list[float]]]
LerobotDataValue = int | float | list[float]
LerobotEpisodeValue = int | float | str | list[str]
LerobotDatasetExportProgressPhase = Literal[
    "preparing",
    "aligning",
    "writing_rows",
    "writing_videos",
    "writing_metadata",
    "writing_stats",
    "validating",
    "finalizing",
]


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


class LerobotDatasetExportProgress(BaseModel):
    phase: LerobotDatasetExportProgressPhase
    progress_percent: float = Field(..., ge=0.0, le=100.0)
    message: str = Field(..., min_length=1)
    detail: str | None = None


LerobotDatasetExportProgressReporter = Callable[[LerobotDatasetExportProgress], None]


def _report_progress(
    progress: LerobotDatasetExportProgressReporter | None,
    *,
    phase: LerobotDatasetExportProgressPhase,
    progress_percent: float,
    message: str,
    detail: str | None = None,
) -> None:
    if progress is None:
        return
    progress(
        LerobotDatasetExportProgress(
            phase=phase,
            progress_percent=progress_percent,
            message=message,
            detail=detail,
        )
    )


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


@lru_cache(maxsize=1)
def _alignment_executor() -> ProcessPoolExecutor:
    set_forkserver_preload(["fv_lerobot_exporter.exporter"])
    return ProcessPoolExecutor(
        max_workers=_export_worker_count(32),
        mp_context=get_context("forkserver"),
    )


class TopicSpec(BaseModel):
    topic: str = Field(..., min_length=1)
    type: str | None = None


class JointCommandSpec(TopicSpec):
    leader: TopicSpec | None = None
    vr: TopicSpec | None = None
    ai: TopicSpec | None = None

    def topic_for_source(self, source: str) -> str:
        if source == "leader" and self.leader is not None:
            return self.leader.topic
        if source == "vr" and self.vr is not None:
            return self.vr.topic
        if source == "ai" and self.ai is not None:
            return self.ai.topic
        if self.topic:
            return self.topic
        raise LerobotDatasetExportError(
            "action_topic_not_configured",
            f"profile.lerobot has no action topic for mux source '{source}'",
        )


class ArmRxSpec(BaseModel):
    joint_state: TopicSpec
    joint_command: JointCommandSpec


class ArmStreamSpec(BaseModel):
    key: str
    namespace: str | None = None
    side: str | None = None
    joints: list[str] = Field(..., min_length=1)
    rx: ArmRxSpec


class CameraSpec(BaseModel):
    name: str
    topic: str
    source: str | None = None
    enabled: bool | str | None = None


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
    video_timing_mode: str | None = None
    video_pts_origin_ros_ns: int | None = None
    segments: list[RecorderCameraSegment] = Field(default_factory=list)


class RecordedTopicMeta(BaseModel):
    model_config = ConfigDict(extra="ignore")

    topic: str = Field(..., min_length=1)
    stamp_source: StampSource | None = None


class SourceEpisodeMeta(BaseModel):
    model_config = ConfigDict(extra="ignore")

    episode_id: str = Field(..., min_length=1)
    state: str
    task_description: str = ""
    profile: str = Field(..., min_length=1)
    started_at: str
    stopped_at: str | None = None
    duration_s: float | None = None
    outcome: str | None = None
    tags: list[str] = Field(default_factory=list)
    cameras: list[RecorderCameraMeta] = Field(default_factory=list)
    recorded_topics: list[RecordedTopicMeta] = Field(default_factory=list)
    bag_path: str = "bag/"


class FrameSidecarRow(BaseModel):
    frame_index: int
    segment_file: str
    segment_local_frame: int
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
    stopped_at: str
    duration_s: float
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
    video_pts_origin_ros_ns: int
    sidecar_rows: list[FrameSidecarRow]


@dataclass(frozen=True)
class EpisodeExportData:
    rows: list[dict[str, LerobotDataValue]]
    videos: dict[str, "VideoExportData"]
    task_description: str
    started_at: str
    stopped_at: str
    duration_s: float
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
    stamps: list[int]


def export_lerobot_dataset(
    *,
    request: LerobotDatasetExportRequest,
    profile_payload: Mapping[str, JsonValue],
    datasets_dir: Path,
    progress: LerobotDatasetExportProgressReporter | None = None,
) -> LerobotDatasetExportResponse:
    root = datasets_dir.resolve()
    _report_progress(
        progress,
        phase="preparing",
        progress_percent=6.0,
        message="エピソード候補を確認しています。",
    )
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
    _report_progress(
        progress,
        phase="preparing",
        progress_percent=8.0,
        message="LeRobot変換設定を確認しています。",
        detail=f"profile: {profile_name}",
    )
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
            progress=progress,
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
            progress=progress,
        )
        _report_progress(
            progress,
            phase="validating",
            progress_percent=94.0,
            message="LeRobotデータセットの必須ファイルを検証しています。",
        )
        _validate_lerobot_export_files(tmp_root)
        exported_at = _utc_now_iso()
        _report_progress(
            progress,
            phase="finalizing",
            progress_percent=95.0,
            message="作成したデータセットを配置しています。",
        )
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
    episodes: list[EpisodeForExport] = []
    for episode_id in episode_ids:
        episode = _find_episode(root, episode_id)
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
    progress: LerobotDatasetExportProgressReporter | None,
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
        _report_progress(
            progress,
            phase="aligning",
            progress_percent=35.0,
            message="エピソードのフレームと関節データを時刻合わせしています。",
            detail=f"1/1: {len(converted_episode.rows)} frames",
        )
        return [converted_episode]

    converted: list[EpisodeExportData | None] = [None] * episode_total
    futures = {
        _alignment_executor().submit(
            _convert_episode,
            episode=episode,
            profile=profile,
            fps=fps,
            max_alignment_error_s=max_alignment_error_s,
            max_mux_status_age_s=max_mux_status_age_s,
        ): (episode_index, episode)
        for episode_index, episode in enumerate(episodes)
    }
    completed = 0
    for future in as_completed(futures):
        episode_index, episode = futures[future]
        try:
            converted_episode = future.result()
        except LerobotDatasetExportError as exc:
            raise LerobotDatasetExportError(
                exc.code,
                f"episode {episode.meta.episode_id}: {exc.detail}",
            ) from exc
        converted[episode_index] = converted_episode
        completed += 1
        _report_progress(
            progress,
            phase="aligning",
            progress_percent=10.0 + (25.0 * (completed / episode_total)),
            message="エピソードのフレームと関節データを時刻合わせしています。",
            detail=f"{completed}/{episode_total}: {len(converted_episode.rows)} frames",
        )

    result: list[EpisodeExportData] = []
    for episode in converted:
        if episode is None:
            raise RuntimeError("episode conversion worker returned no result")
        result.append(episode)
    return result


def _find_episode(root: Path, episode_id: str) -> EpisodeForExport:
    episodes_root = root / "episodes"
    if not episodes_root.exists():
        raise LerobotDatasetExportError(
            "episode_store_missing", f"episode store missing: {episodes_root}"
        )
    suffix = episode_id[-8:] if len(episode_id) >= 8 else episode_id
    for pattern in (f"*/*/*{suffix}/meta.json", f"*/*/{episode_id}/meta.json"):
        for meta_path in episodes_root.glob(pattern):
            try:
                meta = SourceEpisodeMeta.model_validate(
                    json.loads(meta_path.read_text(encoding="utf-8"))
                )
            except (OSError, ValueError, ValidationError) as exc:
                raise LerobotDatasetExportError(
                    "episode_meta_invalid",
                    f"episode meta is invalid: {meta_path}",
                ) from exc
            if meta.episode_id == episode_id:
                return EpisodeForExport(meta=meta, episode_dir=meta_path.parent)
    raise LerobotDatasetExportError(
        "episode_not_found", f"episode not found: {episode_id}"
    )


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
    primary_camera = camera_mappings[0]
    primary_frame_stamps = [row.ros_stamp_ns for row in primary_camera.sidecar_rows]
    if not primary_frame_stamps:
        raise LerobotDatasetExportError(
            "camera_sidecar_empty",
            f"episode {episode.meta.episode_id} has no frames for {primary_camera.recorder_name}",
        )
    frame_stamps = _fixed_fps_timeline(
        first_stamp_ns=primary_frame_stamps[0],
        last_stamp_ns=primary_frame_stamps[-1],
        fps=fps,
    )

    stamp_sources = _recorded_topic_stamp_sources(episode.meta)
    bag_topics = _required_bag_topics(profile)
    bag = _read_bag_topics(
        episode.episode_dir / episode.meta.bag_path, bag_topics, stamp_sources
    )
    if not bag.source_events:
        raise LerobotDatasetExportError(
            "mux_status_missing",
            f"episode {episode.meta.episode_id} has no mux status samples",
        )
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
            stamps=[row.ros_stamp_ns for row in mapping.sidecar_rows],
        )
        for mapping in camera_mappings
    }

    arm_streams = profile.lerobot.arm_streams
    state_names = _joint_feature_names(arm_streams)
    aligned_rows: list[tuple[int, list[float], list[float]] | None] = []
    alignment_errors: list[LerobotDatasetExportError | None] = []
    for stamp_ns in frame_stamps:
        source = _source_at(source_events, stamp_ns, max_mux_status_age_s)
        try:
            state = _combined_joint_values(
                arm_streams=arm_streams,
                samples_by_topic=joint_samples_by_topic,
                stamp_ns=stamp_ns,
                kind="state",
                source=source,
                max_alignment_error_s=max_alignment_error_s,
            )
            action = _combined_joint_values(
                arm_streams=arm_streams,
                samples_by_topic=joint_samples_by_topic,
                stamp_ns=stamp_ns,
                kind="action",
                source=source,
                max_alignment_error_s=max_alignment_error_s,
            )
            for mapping in camera_mappings:
                _nearest_camera_frame(
                    camera_frames_by_feature[mapping.feature_key],
                    stamp_ns,
                    max_alignment_error_s,
                )
        except LerobotDatasetExportError as exc:
            # Camera recording can start/stop slightly outside joint streams.
            # Trim only unaligned boundaries; an interior hole cannot be represented
            # by LeRobot's fixed-fps row timestamps without changing time semantics.
            if exc.code in {
                "sample_alignment_error",
                "sample_missing",
                "samples_missing",
                "camera_alignment_error",
                "camera_frame_missing",
            }:
                aligned_rows.append(None)
                alignment_errors.append(exc)
                continue
            raise
        if len(state) != len(state_names) or len(action) != len(state_names):
            raise LerobotDatasetExportError(
                "joint_vector_mismatch",
                f"episode {episode.meta.episode_id} produced invalid joint vector length",
            )
        aligned_rows.append((stamp_ns, state, action))
        alignment_errors.append(None)

    valid_indices = [index for index, row in enumerate(aligned_rows) if row is not None]
    if not valid_indices:
        raise LerobotDatasetExportError(
            "episode_joint_alignment_empty",
            f"episode {episode.meta.episode_id} has no frames aligned to joint samples",
        )
    first_valid = valid_indices[0]
    last_valid = valid_indices[-1]
    for index in range(first_valid, last_valid + 1):
        if aligned_rows[index] is None:
            cause = alignment_errors[index]
            detail = cause.detail if cause is not None else "unknown alignment error"
            raise LerobotDatasetExportError(
                "episode_alignment_gap",
                f"fixed-fps timeline has an unaligned interior frame at index {index}: {detail}",
            )

    pending_rows = [
        row for row in aligned_rows[first_valid : last_valid + 1] if row is not None
    ]
    first_output_stamp_ns = pending_rows[0][0]

    rows: list[dict[str, LerobotDataValue]] = []
    for output_frame_index, (_stamp_ns, state, action) in enumerate(pending_rows):
        rows.append(
            {
                "observation.state": state,
                "action": action,
                "timestamp": output_frame_index / fps,
                "frame_index": output_frame_index,
            }
        )

    videos: dict[str, VideoExportData] = {}
    for mapping in camera_mappings:
        from_timestamp_s = (
            first_output_stamp_ns - mapping.video_pts_origin_ros_ns
        ) / 1_000_000_000.0
        if from_timestamp_s < 0:
            raise LerobotDatasetExportError(
                "camera_video_query_before_origin",
                f"{mapping.feature_key} starts after the common aligned timeline",
            )
        videos[mapping.feature_key] = VideoExportData(
            source_video=mapping.source_video,
            width=mapping.width,
            height=mapping.height,
            from_timestamp_s=from_timestamp_s,
        )
    return EpisodeExportData(
        rows=rows,
        videos=videos,
        task_description=episode.meta.task_description,
        started_at=episode.meta.started_at,
        stopped_at=episode.meta.stopped_at or "",
        duration_s=episode.meta.duration_s or 0.0,
        source_tags=episode.meta.tags,
    )


def _fixed_fps_timeline(
    *, first_stamp_ns: int, last_stamp_ns: int, fps: int
) -> list[int]:
    if last_stamp_ns < first_stamp_ns:
        raise LerobotDatasetExportError(
            "camera_timestamps_invalid", "primary camera timestamps are not monotonic"
        )
    frame_count = ((last_stamp_ns - first_stamp_ns) * fps) // 1_000_000_000 + 1
    return [
        first_stamp_ns + round(frame_index * 1_000_000_000 / fps)
        for frame_index in range(frame_count)
    ]


def _required_bag_topics(profile: ProfileExportSpec) -> set[str]:
    topics: set[str] = set()
    for stream in profile.lerobot.arm_streams:
        topics.add(stream.rx.joint_state.topic)
        topics.add(stream.rx.joint_command.topic)
        if stream.rx.joint_command.leader is not None:
            topics.add(stream.rx.joint_command.leader.topic)
        if stream.rx.joint_command.vr is not None:
            topics.add(stream.rx.joint_command.vr.topic)
        if stream.rx.joint_command.ai is not None:
            topics.add(stream.rx.joint_command.ai.topic)
    topics.add(_mux_status_topic(profile))
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
        connections = [
            connection
            for connection in reader.connections
            if connection.topic in required_topics
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
            topic: [] for topic in required_topics if topic != mux_topic
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
    kind: Literal["state", "action"],
    source: str,
    max_alignment_error_s: float,
) -> list[float]:
    values: list[float] = []
    for stream in arm_streams:
        topic = (
            stream.rx.joint_state.topic
            if kind == "state"
            else stream.rx.joint_command.topic_for_source(source)
        )
        sample_index = samples_by_topic.get(topic)
        if sample_index is None or not sample_index.samples:
            raise LerobotDatasetExportError(
                "samples_missing", f"no {kind} samples for {topic}"
            )
        sample = _nearest_sample(sample_index, stamp_ns, max_alignment_error_s)
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
        values.extend(float(position_by_name[joint]) for joint in stream.joints)
    return values


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
    frame_index: CameraFrameIndex, stamp_ns: int, max_error_s: float
) -> FrameSidecarRow:
    if not frame_index.rows:
        raise LerobotDatasetExportError(
            "camera_frame_missing", "no camera frame candidates"
        )
    if stamp_ns < frame_index.stamps[0] or stamp_ns > frame_index.stamps[-1]:
        raise LerobotDatasetExportError(
            "camera_alignment_error",
            "query timestamp is outside the camera video timestamp range",
        )
    index = bisect.bisect_left(frame_index.stamps, stamp_ns)
    candidates: list[FrameSidecarRow] = []
    if index < len(frame_index.rows):
        candidates.append(frame_index.rows[index])
    if index > 0:
        candidates.append(frame_index.rows[index - 1])
    if not candidates:
        raise LerobotDatasetExportError(
            "camera_frame_missing", "no camera frame candidates"
        )
    nearest = min(candidates, key=lambda row: abs(row.ros_stamp_ns - stamp_ns))
    error_s = abs(nearest.ros_stamp_ns - stamp_ns) / 1_000_000_000.0
    if error_s > max_error_s:
        raise LerobotDatasetExportError(
            "camera_alignment_error",
            f"nearest camera frame is {error_s:.3f}s away, max={max_error_s:.3f}s",
        )
    return nearest


def _resolve_camera_mappings(
    episode: EpisodeForExport,
    profile: ProfileExportSpec,
) -> list[CameraExportMapping]:
    episode_camera_by_topic = {camera.topic: camera for camera in episode.meta.cameras}
    mappings: list[CameraExportMapping] = []
    for camera in profile.lerobot.cameras:
        episode_camera = episode_camera_by_topic.get(camera.topic)
        if episode_camera is None:
            if camera.enabled is True:
                raise LerobotDatasetExportError(
                    "camera_missing",
                    f"required profile camera {camera.name} topic {camera.topic} missing from episode",
                )
            continue
        if len(episode_camera.segments) != 1:
            raise LerobotDatasetExportError(
                "camera_segments_unsupported",
                f"camera {episode_camera.name} must have exactly one mp4 segment",
            )
        if (
            episode_camera.video_timing_mode != "ros_header_stamp_to_pts"
            or not episode_camera.video_pts_origin_ros_ns
        ):
            raise LerobotDatasetExportError(
                "camera_video_timing_contract_missing",
                f"camera {episode_camera.name} does not declare ros_header_stamp_to_pts video timing",
            )
        segment = episode_camera.segments[0]
        source_video = (
            episode.episode_dir / "videos" / episode_camera.name / segment.file
        )
        if not source_video.exists():
            raise LerobotDatasetExportError(
                "camera_video_missing", f"video missing: {source_video}"
            )
        sidecar_rows = _read_sidecar_rows(
            episode.episode_dir / "videos" / episode_camera.name / "frames.parquet"
        )
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
        if sidecar_rows[0].ros_stamp_ns != episode_camera.video_pts_origin_ros_ns:
            raise LerobotDatasetExportError(
                "camera_video_pts_origin_mismatch",
                f"camera {episode_camera.name} video origin does not match its first sidecar frame",
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
                video_pts_origin_ros_ns=episode_camera.video_pts_origin_ros_ns,
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
    progress: LerobotDatasetExportProgressReporter | None = None,
) -> None:
    _report_progress(
        progress,
        phase="writing_rows",
        progress_percent=38.0,
        message="LeRobotのフレーム表を作成しています。",
    )
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
        progress=progress,
    )
    for feature_key, video_paths in written_video_paths_by_feature.items():
        meta.info["features"][feature_key]["info"] = get_video_info(video_paths[0])
        meta.info["features"][feature_key]["info"]["video.fps"] = fps
    meta.info["dataset_name"] = dataset_name
    meta.info["video_timestamp_tolerance_s"] = video_timestamp_tolerance_s
    meta.info["repo_id"] = dataset_id
    meta.info["total_episodes"] = len(episode_exports)
    meta.info["total_frames"] = len(all_rows)
    meta.info["total_tasks"] = len(task_index_by_name)
    meta.info["splits"] = {"train": f"0:{len(episode_exports)}"}
    _report_progress(
        progress,
        phase="writing_metadata",
        progress_percent=88.0,
        message="LeRobotメタデータを書き出しています。",
    )
    write_info(meta.info, meta.root)
    _write_data_parquet(meta, all_rows)
    _write_episode_parquet(meta.root, episode_rows)
    _write_tasks_parquet(meta.root, task_index_by_name)
    _write_stats(meta.root, all_rows, progress=progress)


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
    progress: LerobotDatasetExportProgressReporter | None = None,
) -> dict[str, list[Path]]:
    if video_path_template is None:
        raise LerobotDatasetExportError(
            "video_path_missing", "LeRobot metadata has no video_path template"
        )
    feature_keys = list(episode_exports[0].videos)
    written_by_feature: dict[str, list[Path]] = {
        feature_key: [] for feature_key in feature_keys
    }
    total_videos = len(feature_keys) * len(episode_exports)
    completed = 0
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
            completed += 1
            _report_progress(
                progress,
                phase="writing_videos",
                progress_percent=40.0 + (45.0 * (completed / total_videos)),
                message="録画動画をデータセットへ関連付けています。",
                detail=(
                    f"{completed}/{total_videos}: "
                    f"{feature_key} "
                    f"{episode_index + 1}/{len(episode_exports)}"
                ),
            )
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


def _validate_lerobot_export_files(root: Path) -> None:
    required = [
        root / "meta" / "info.json",
        root / "meta" / "tasks.parquet",
        root / "meta" / "episodes" / "chunk-000" / "file-000.parquet",
        root / "data" / "chunk-000" / "file-000.parquet",
        root / "meta" / "stats.json",
    ]
    missing = [str(path) for path in required if not path.exists()]
    if missing:
        raise LerobotDatasetExportError(
            "lerobot_export_incomplete",
            f"LeRobot export did not produce required files: {missing}",
        )


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
    *,
    progress: LerobotDatasetExportProgressReporter | None = None,
) -> None:
    _report_progress(
        progress,
        phase="writing_stats",
        progress_percent=90.0,
        message="数値データの統計を計算しています。",
    )
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
