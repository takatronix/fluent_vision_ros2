from __future__ import annotations

import json
import math
import shutil
import time
from concurrent.futures import Future, ThreadPoolExecutor
from fractions import Fraction
from multiprocessing.context import BaseContext
from pathlib import Path
from typing import Callable

import av
import cv2
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
import pytest
import lerobot.datasets.video_utils as lerobot_video_utils
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

import fv_lerobot_exporter.exporter as exporter
from fv_lerobot_exporter import (
    LerobotDatasetExportError,
    LerobotDatasetExportRequest,
    JsonValue,
    export_lerobot_dataset,
)


EPISODE_ID = "episode-12345678"
STATE_TOPIC = "/follower_arm/joint_states"
ACTION_TOPIC = "/follower_arm/joint_ctrl"
IDLE_ACTION_TOPIC = "/follower_arm/joint_cmd"
MUX_TOPIC = "/follower_arm/teleop_mux/status"
CAMERA_TOPIC = "/camera/color/image_raw"
SECOND_CAMERA_TOPIC = "/camera/top/image_raw"
LOCAL_VFR_READER_AVAILABLE = hasattr(
    lerobot_video_utils,
    "VIDEO_QUERY_TIMESTAMP_SOURCE_KEY",
)


class _NotifyingFuture(Future[exporter.EpisodeExportData]):
    def cancel(self) -> bool:
        cancelled = super().cancel()
        if cancelled:
            self.set_running_or_notify_cancel()
        return cancelled


def _alignment_test_episodes(tmp_path: Path) -> list[exporter.EpisodeForExport]:
    return [
        exporter.EpisodeForExport(
            meta=exporter.SourceEpisodeMeta(
                schema_version=2,
                episode_id=f"episode-{index}",
                state="finished",
                task_description="task",
                profile="profile",
                started_at="2026-07-10T00:00:00+00:00",
                outcome="success",
            ),
            episode_dir=tmp_path / f"episode-{index}",
        )
        for index in range(5)
    ]


def _alignment_test_profile() -> exporter.ProfileExportSpec:
    return exporter.ProfileExportSpec.model_validate(
        {
            "lerobot": {
                "arm_streams": [
                    {
                        "key": "arm",
                        "joints": ["joint1"],
                        "rx": {
                            "joint_state": {"topic": "/joint_states"},
                            "joint_command": {"topic": "/joint_ctrl"},
                        },
                    }
                ]
            }
        }
    )


def test_export_request_bounds_alignment_to_three_frames() -> None:
    request = LerobotDatasetExportRequest(
        dataset_id="dataset-1",
        dataset_name="dataset_1",
        episode_ids=[EPISODE_ID],
        fps=60,
    )

    assert request.max_alignment_error_s == pytest.approx(0.05)
    with pytest.raises(ValueError, match="at most 0.05"):
        LerobotDatasetExportRequest(
            dataset_id="dataset-1",
            dataset_name="dataset_1",
            episode_ids=[EPISODE_ID],
            fps=60,
            max_alignment_error_s=0.051,
        )


def test_export_request_refuses_duplicate_episode_ids() -> None:
    with pytest.raises(ValueError, match="episode_ids must not contain duplicates"):
        LerobotDatasetExportRequest(
            dataset_id="dataset-1",
            dataset_name="dataset_1",
            episode_ids=[EPISODE_ID, EPISODE_ID],
        )


def test_alignment_failure_cancels_bounded_outstanding_work(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    episodes = _alignment_test_episodes(tmp_path)
    profile = _alignment_test_profile()

    class FailingExecutor:
        def __init__(self) -> None:
            self.submitted_episode_ids: list[str] = []
            self.pending_future: Future[exporter.EpisodeExportData] | None = None

        def submit(
            self,
            function: Callable[..., exporter.EpisodeExportData],
            *,
            episode: exporter.EpisodeForExport,
            profile: exporter.ProfileExportSpec,
            fps: int,
            max_alignment_error_s: float,
            max_mux_status_age_s: float,
        ) -> Future[exporter.EpisodeExportData]:
            self.submitted_episode_ids.append(episode.meta.episode_id)
            future: Future[exporter.EpisodeExportData]
            if len(self.submitted_episode_ids) == 1:
                future = Future()
                future.set_exception(
                    LerobotDatasetExportError("alignment_failed", "broken")
                )
            else:
                future = _NotifyingFuture()
                self.pending_future = future
            return future

    executor = FailingExecutor()
    monkeypatch.setattr(exporter, "_alignment_executor", lambda: executor)
    monkeypatch.setattr(exporter, "_export_worker_count", lambda _count: 2)

    with pytest.raises(LerobotDatasetExportError, match="episode episode-0: broken"):
        exporter._convert_episodes(
            episodes=episodes,
            profile=profile,
            fps=30,
            max_alignment_error_s=0.1,
            max_mux_status_age_s=2.5,
        )

    assert executor.submitted_episode_ids == ["episode-0", "episode-1"]
    assert executor.pending_future is not None
    assert executor.pending_future.cancelled()


def test_alignment_submit_failure_cancels_already_submitted_work(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    class SubmitFailingExecutor:
        def __init__(self) -> None:
            self.submit_count = 0
            self.pending_future = _NotifyingFuture()

        def submit(
            self,
            function: Callable[..., exporter.EpisodeExportData],
            *,
            episode: exporter.EpisodeForExport,
            profile: exporter.ProfileExportSpec,
            fps: int,
            max_alignment_error_s: float,
            max_mux_status_age_s: float,
        ) -> Future[exporter.EpisodeExportData]:
            self.submit_count += 1
            if self.submit_count == 2:
                raise RuntimeError("submit failed")
            return self.pending_future

    executor = SubmitFailingExecutor()
    monkeypatch.setattr(exporter, "_alignment_executor", lambda: executor)
    monkeypatch.setattr(exporter, "_export_worker_count", lambda _count: 2)

    with pytest.raises(RuntimeError, match="submit failed"):
        exporter._convert_episodes(
            episodes=_alignment_test_episodes(tmp_path),
            profile=_alignment_test_profile(),
            fps=30,
            max_alignment_error_s=0.1,
            max_mux_status_age_s=2.5,
        )

    assert executor.pending_future.cancelled()


def test_alignment_executor_is_singleton_during_concurrent_first_use(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class StubAlignmentExecutor:
        pass

    created: list[StubAlignmentExecutor] = []

    def create_executor(
        *, max_workers: int, mp_context: BaseContext
    ) -> StubAlignmentExecutor:
        assert max_workers > 0
        assert mp_context is not None
        time.sleep(0.05)
        executor = StubAlignmentExecutor()
        created.append(executor)
        return executor

    monkeypatch.setattr(exporter, "_ALIGNMENT_EXECUTOR", None)
    monkeypatch.setattr(exporter, "ProcessPoolExecutor", create_executor)

    with ThreadPoolExecutor(max_workers=2) as callers:
        results = list(callers.map(lambda _index: exporter._alignment_executor(), range(2)))

    assert len(created) == 1
    assert results[0] is results[1] is created[0]


def test_fixed_fps_timeline_uses_absolute_ros_clock_grid() -> None:
    assert exporter._fixed_fps_timeline_for_interval(
        available_start_ros_ns=1_010_000_000,
        available_end_ros_ns=1_110_000_000,
        fps=30,
    ) == [1_033_333_333, 1_066_666_667, 1_100_000_000]


def test_nearest_camera_frame_prefers_previous_on_midpoint_tie() -> None:
    previous = exporter.FrameSidecarRow(
        frame_index=0,
        segment_file="segment-000.mp4",
        segment_local_frame=0,
        video_pts=0,
        ros_stamp_ns=1_000_000_000,
        recv_stamp_ns=1_000_000_000,
        source_seq=0,
        dropped_before=0,
        keyframe=True,
    )
    following = previous.model_copy(
        update={
            "frame_index": 1,
            "segment_local_frame": 1,
            "video_pts": 9_000,
            "ros_stamp_ns": 1_100_000_000,
            "recv_stamp_ns": 1_100_000_000,
            "source_seq": 1,
        }
    )
    index = exporter.CameraFrameIndex(
        rows=[previous, following],
        pts_seconds=[0.0, 0.1],
    )

    assert (
        exporter._nearest_camera_frame(index, 0.05, 0.1)[0].frame_index
        == previous.frame_index
    )


def test_nearest_joint_sample_prefers_following_on_midpoint_tie() -> None:
    previous = exporter.JointSample(
        stamp_ns=1_000_000_000,
        names=["joint1"],
        positions=[0.0],
    )
    following = previous.model_copy(
        update={"stamp_ns": 1_100_000_000, "positions": [1.0]}
    )
    sample_index = exporter.JointSampleIndex(
        samples=[previous, following],
        stamps=[previous.stamp_ns, following.stamp_ns],
    )

    selected = exporter._nearest_sample(sample_index, 1_050_000_000, 0.1)

    assert selected is following


def test_joint_vectors_follow_canonical_wire_to_dataset_units() -> None:
    stream = exporter.ArmStreamSpec.model_validate(
        {
            "key": "follower",
            "namespace": "follower_arm",
            "joints": ["joint1", "gripper"],
            "joint_units": {"gripper": "percent"},
            "rx": {
                "joint_state": {"topic": STATE_TOPIC},
                "joint_command": {
                    "topic": "/unused-base-action",
                    "leader": {"topic": "/leader-action"},
                    "vr": {"topic": ACTION_TOPIC},
                },
            },
        }
    )
    stamp_ns = 1_000_000_000
    sample_indexes = {
        STATE_TOPIC: exporter.JointSampleIndex(
            samples=[
                exporter.JointSample(
                    stamp_ns=stamp_ns,
                    names=["joint1", "gripper"],
                    positions=[math.pi / 2, 0.25],
                )
            ],
            stamps=[stamp_ns],
        ),
        ACTION_TOPIC: exporter.JointSampleIndex(
            samples=[
                exporter.JointSample(
                    stamp_ns=stamp_ns,
                    names=["joint1", "gripper"],
                    positions=[-math.pi / 2, 0.5],
                )
            ],
            stamps=[stamp_ns],
        ),
    }

    state = exporter._combined_joint_values(
        arm_streams=[stream],
        samples_by_topic=sample_indexes,
        stamp_ns=stamp_ns,
        grid_index=0,
        kind="state",
        source="vr",
        max_alignment_error_s=0.1,
        last_samples_by_topic={},
    )
    action = exporter._combined_joint_values(
        arm_streams=[stream],
        samples_by_topic=sample_indexes,
        stamp_ns=stamp_ns,
        grid_index=0,
        kind="action",
        source="vr",
        max_alignment_error_s=0.1,
        last_samples_by_topic={},
    )

    assert state == pytest.approx([90.0, 25.0])
    assert action == pytest.approx([-90.0, 50.0])


def test_joint_units_refuse_keys_outside_profile_joints() -> None:
    with pytest.raises(ValueError, match="joint_units keys not in joints"):
        exporter.ArmStreamSpec.model_validate(
            {
                "key": "follower",
                "joints": ["joint1"],
                "joint_units": {"gripper": "percent"},
                "rx": {
                    "joint_state": {"topic": STATE_TOPIC},
                    "joint_command": {"topic": ACTION_TOPIC},
                },
            }
        )


@pytest.mark.parametrize("source", ["vr", "ai"])
def test_action_topic_requires_exact_configured_mux_source(source: str) -> None:
    command = exporter.JointCommandSpec(
        topic="/base-action",
        leader=exporter.TopicSpec(topic="/leader-action"),
    )
    assert command.topic_for_source("leader") == "/leader-action"

    with pytest.raises(LerobotDatasetExportError) as exc_info:
        command.topic_for_source(source)

    assert exc_info.value.code == "action_topic_not_configured"


@pytest.mark.parametrize("source", ["replay", "unexpected"])
def test_action_topic_refuses_unsupported_mux_source(source: str) -> None:
    command = exporter.JointCommandSpec(
        topic="/base-action",
        leader=exporter.TopicSpec(topic="/leader-action"),
    )

    with pytest.raises(LerobotDatasetExportError) as exc_info:
        command.topic_for_source(source)

    assert exc_info.value.code == "action_source_unsupported"


def test_nearest_camera_frame_uses_last_frame_just_after_video_end() -> None:
    frame = exporter.FrameSidecarRow(
        frame_index=2,
        segment_file="segment-000.mp4",
        segment_local_frame=2,
        video_pts=5_850,
        ros_stamp_ns=1_065_000_000,
        recv_stamp_ns=1_065_000_000,
        source_seq=2,
        dropped_before=0,
        keyframe=True,
    )
    index = exporter.CameraFrameIndex(rows=[frame], pts_seconds=[0.065])

    selected, selected_pts_s = exporter._nearest_camera_frame(
        index,
        2 / 30,
        0.1,
    )

    assert selected is frame
    assert selected_pts_s == 0.065


def test_nearest_camera_frame_rejects_video_boundary_outside_tolerance() -> None:
    frame = exporter.FrameSidecarRow(
        frame_index=0,
        segment_file="segment-000.mp4",
        segment_local_frame=0,
        video_pts=0,
        ros_stamp_ns=1_000_000_000,
        recv_stamp_ns=1_000_000_000,
        source_seq=0,
        dropped_before=0,
        keyframe=True,
    )
    index = exporter.CameraFrameIndex(rows=[frame], pts_seconds=[0.0])

    with pytest.raises(LerobotDatasetExportError) as exc_info:
        exporter._nearest_camera_frame(index, 0.101, 0.1)

    assert exc_info.value.code == "camera_alignment_error"


def test_camera_alignment_uses_persisted_video_pts_after_timebase_quantization() -> (
    None
):
    previous = exporter.FrameSidecarRow(
        frame_index=0,
        segment_file="segment-000.mp4",
        segment_local_frame=0,
        video_pts=0,
        ros_stamp_ns=0,
        recv_stamp_ns=0,
        source_seq=0,
        dropped_before=0,
        keyframe=True,
    )
    following = previous.model_copy(
        update={
            "frame_index": 1,
            "segment_local_frame": 1,
            "video_pts": 3_001,
            "ros_stamp_ns": 33_340_000,
            "recv_stamp_ns": 33_340_000,
            "source_seq": 1,
        }
    )
    query_timestamp_s = 0.016671
    assert abs(following.ros_stamp_ns / 1e9 - query_timestamp_s) < abs(
        previous.ros_stamp_ns / 1e9 - query_timestamp_s
    )
    frame_index = exporter.CameraFrameIndex(
        rows=[previous, following],
        pts_seconds=[0.0, 3_001 / 90_000],
    )

    selected, selected_pts_s = exporter._nearest_camera_frame(
        frame_index,
        query_timestamp_s,
        0.1,
    )

    assert selected.frame_index == previous.frame_index
    assert selected_pts_s == 0.0


def test_carry_allows_at_most_three_30fps_grid_frames() -> None:
    sample = exporter.JointSample(
        stamp_ns=1_000_000_000,
        names=["joint1"],
        positions=[0.1],
    )
    previous = exporter.AlignedJointSample(sample=sample, origin_grid_index=0)

    assert (
        exporter._carry_previous_value(
            previous=previous,
            age_s=0.1,
            grid_index=3,
            max_error_s=0.1,
            stream="state topic /joint_states",
            cause=LerobotDatasetExportError("sample_alignment_error", "gap"),
            allow_future=False,
        )
        is previous
    )
    with pytest.raises(LerobotDatasetExportError, match="carry_frames=4"):
        exporter._carry_previous_value(
            previous=previous,
            age_s=0.133333333,
            grid_index=4,
            max_error_s=0.1,
            stream="state topic /joint_states",
            cause=LerobotDatasetExportError("sample_alignment_error", "gap"),
            allow_future=False,
        )


def test_carry_uses_point_one_second_limit_before_three_low_fps_frames() -> None:
    frame = exporter.FrameSidecarRow(
        frame_index=0,
        segment_file="segment-000.mp4",
        segment_local_frame=0,
        video_pts=0,
        ros_stamp_ns=1_000_000_000,
        recv_stamp_ns=1_000_000_000,
        source_seq=0,
        dropped_before=0,
        keyframe=True,
    )
    previous = exporter.AlignedCameraFrame(
        frame=frame,
        pts_s=0.0,
        origin_grid_index=0,
    )

    with pytest.raises(LerobotDatasetExportError, match="max_age=0.100s"):
        exporter._carry_previous_value(
            previous=previous,
            age_s=0.2,
            grid_index=1,
            max_error_s=0.1,
            stream="camera observation.images.wrist",
            cause=LerobotDatasetExportError("camera_alignment_error", "gap"),
            allow_future=False,
        )


def test_carry_refuses_leading_gap_and_future_value() -> None:
    cause = LerobotDatasetExportError("sample_alignment_error", "gap")
    with pytest.raises(LerobotDatasetExportError, match="first grid frame"):
        exporter._carry_previous_value(
            previous=None,
            age_s=None,
            grid_index=0,
            max_error_s=0.1,
            stream="action topic /joint_ctrl",
            cause=cause,
            allow_future=False,
        )

    sample = exporter.JointSample(
        stamp_ns=1_100_000_000,
        names=["joint1"],
        positions=[0.1],
    )
    with pytest.raises(LerobotDatasetExportError, match="age=-0.100s"):
        exporter._carry_previous_value(
            previous=exporter.AlignedJointSample(sample=sample, origin_grid_index=0),
            age_s=-0.1,
            grid_index=1,
            max_error_s=0.1,
            stream="state topic /joint_states",
            cause=cause,
            allow_future=False,
        )


def test_reselecting_same_joint_sample_does_not_reset_carry_origin() -> None:
    sample = exporter.JointSample(
        stamp_ns=1_000_000_000,
        names=["joint1"],
        positions=[0.1],
    )
    sample_index = exporter.JointSampleIndex(samples=[sample], stamps=[sample.stamp_ns])
    aligned: dict[str, exporter.AlignedJointSample] = {}

    for grid_index, stamp_ns in enumerate(
        [1_000_000_000, 1_033_333_333, 1_066_666_667, 1_100_000_000]
    ):
        assert (
            exporter._joint_sample_with_carry(
                sample_index=sample_index,
                stamp_ns=stamp_ns,
                grid_index=grid_index,
                max_error_s=0.1,
                kind="state",
                topic=STATE_TOPIC,
                last_samples_by_topic=aligned,
            )
            is sample
        )

    assert aligned[STATE_TOPIC].origin_grid_index == 0
    with pytest.raises(LerobotDatasetExportError, match="carry_frames=4"):
        exporter._joint_sample_with_carry(
            sample_index=sample_index,
            stamp_ns=1_133_333_333,
            grid_index=4,
            max_error_s=0.1,
            kind="state",
            topic=STATE_TOPIC,
            last_samples_by_topic=aligned,
        )


def test_export_hardlinks_cfr_ros_pts_episode_and_lerobot_reads_it(
    tmp_path: Path,
) -> None:
    _write_episode_store(tmp_path)

    response = export_lerobot_dataset(
        request=LerobotDatasetExportRequest(
            dataset_id="dataset-1",
            dataset_name="dataset_1",
            episode_ids=[EPISODE_ID],
            fps=30,
        ),
        profile_payload=_profile_payload(),
        datasets_dir=tmp_path,
    )

    dataset_root = tmp_path / "dataset-1"
    assert response.dataset_path == str(dataset_root)
    assert response.episode_count == 1
    assert response.frame_count == 3

    info = json.loads((dataset_root / "meta" / "info.json").read_text(encoding="utf-8"))
    assert info["repo_id"] == "dataset-1"
    assert info["total_episodes"] == 1
    assert info["total_frames"] == 3
    assert info["features"]["observation.state"]["names"] == [
        "follower_arm_joint1",
        "follower_arm_joint2",
    ]
    assert "observation.images.wrist" in info["features"]
    assert info["features"]["observation.images.wrist"]["info"]["video.fps"] == 30
    assert info["video_timestamp_tolerance_s"] == pytest.approx(0.1)
    assert info["video_query_timestamp_source"] == "frame_index_over_fps"
    stats = json.loads(
        (dataset_root / "meta" / "stats.json").read_text(encoding="utf-8")
    )
    assert set(stats) == {"observation.state", "action"}

    data_rows = pq.read_table(
        dataset_root / "data" / "chunk-000" / "file-000.parquet"
    ).to_pylist()
    assert data_rows[0]["observation.state"] == pytest.approx(
        [math.degrees(0.1), math.degrees(0.2)]
    )
    assert data_rows[0]["action"] == pytest.approx(
        [math.degrees(0.4), math.degrees(0.5)]
    )
    assert data_rows[-1]["frame_index"] == 2
    assert data_rows[-1]["task_index"] == 0

    task_rows = pq.read_table(dataset_root / "meta" / "tasks.parquet").to_pylist()
    assert task_rows == [{"task_index": 0, "__index_level_0__": "pick block"}]

    episode_rows = pq.read_table(
        dataset_root / "meta" / "episodes" / "chunk-000" / "file-000.parquet"
    ).to_pylist()
    assert episode_rows[0]["tasks"] == ["pick block"]
    assert episode_rows[0]["length"] == 3

    video_path = (
        dataset_root
        / "videos"
        / "observation.images.wrist"
        / "chunk-000"
        / "file-000.mp4"
    )
    assert video_path.exists()
    assert _video_frame_count(video_path) == 3
    source_video = next(
        (tmp_path / "episodes").glob("*/*/*/videos/wrist/segment-000.mp4")
    )
    assert video_path.stat().st_ino == source_video.stat().st_ino
    dataset = LeRobotDataset(repo_id="dataset-1", root=dataset_root)
    assert dataset.num_frames == 3
    assert dataset.num_episodes == 1
    assert dataset.meta.get_video_file_path(0, "observation.images.wrist") == Path(
        "videos/observation.images.wrist/chunk-000/file-000.mp4"
    )
    item = dataset[1]
    assert item["observation.images.wrist"].shape[0] == 3
    provenance = json.loads(
        (dataset_root / "meta" / "fv_episode_export.json").read_text(encoding="utf-8")
    )
    assert provenance["source_episodes"][0]["episode_id"] == EPISODE_ID


def test_export_multiple_episodes_use_episode_video_files(tmp_path: Path) -> None:
    second_episode_id = "episode-87654321"
    _write_episode_store(tmp_path, episode_id=EPISODE_ID)
    _write_episode_store(tmp_path, episode_id=second_episode_id)

    response = export_lerobot_dataset(
        request=LerobotDatasetExportRequest(
            dataset_id="dataset-1",
            dataset_name="dataset_1",
            episode_ids=[EPISODE_ID, second_episode_id],
            fps=30,
        ),
        profile_payload=_profile_payload(),
        datasets_dir=tmp_path,
    )

    dataset_root = tmp_path / "dataset-1"
    assert response.episode_count == 2
    assert response.frame_count == 6
    first_video_path = (
        dataset_root
        / "videos"
        / "observation.images.wrist"
        / "chunk-000"
        / "file-000.mp4"
    )
    second_video_path = (
        dataset_root
        / "videos"
        / "observation.images.wrist"
        / "chunk-000"
        / "file-001.mp4"
    )
    assert _video_frame_count(first_video_path) == 3
    assert _video_frame_count(second_video_path) == 3

    episode_rows = pq.read_table(
        dataset_root / "meta" / "episodes" / "chunk-000" / "file-000.parquet"
    ).to_pylist()
    assert episode_rows[0]["videos/observation.images.wrist/file_index"] == 0
    assert episode_rows[0][
        "videos/observation.images.wrist/from_timestamp"
    ] == pytest.approx(0.0)
    assert episode_rows[1]["videos/observation.images.wrist/file_index"] == 1
    assert episode_rows[1][
        "videos/observation.images.wrist/from_timestamp"
    ] == pytest.approx(0.0)

    dataset = LeRobotDataset(repo_id="dataset-1", root=dataset_root)
    assert dataset.meta.get_video_file_path(1, "observation.images.wrist") == Path(
        "videos/observation.images.wrist/chunk-000/file-001.mp4"
    )
    assert dataset[4]["observation.images.wrist"].shape[0] == 3


def test_export_refuses_missing_selected_action_topic(tmp_path: Path) -> None:
    _write_episode_store(tmp_path, include_action=False)

    with pytest.raises(LerobotDatasetExportError) as exc_info:
        export_lerobot_dataset(
            request=LerobotDatasetExportRequest(
                dataset_id="dataset-1",
                dataset_name="dataset_1",
                episode_ids=[EPISODE_ID],
            ),
            profile_payload=_profile_payload(),
            datasets_dir=tmp_path,
        )

    assert exc_info.value.code == "episode_alignment_gap"
    assert EPISODE_ID in exc_info.value.detail
    assert ACTION_TOPIC in exc_info.value.detail

    assert not (tmp_path / "dataset-1").exists()
    assert not (tmp_path / ".lerobot_exports" / "dataset-1").exists()


def test_export_allows_absent_unused_source_action_topic(tmp_path: Path) -> None:
    _write_episode_store(tmp_path)

    response = export_lerobot_dataset(
        request=LerobotDatasetExportRequest(
            dataset_id="dataset-1",
            dataset_name="dataset_1",
            episode_ids=[EPISODE_ID],
            fps=30,
        ),
        profile_payload=_profile_payload_with_idle_leader_action(),
        datasets_dir=tmp_path,
    )

    assert response.frame_count == 3


def test_export_refuses_absent_action_for_selected_source(tmp_path: Path) -> None:
    _write_episode_store(tmp_path, mux_source="leader")

    with pytest.raises(LerobotDatasetExportError) as exc_info:
        export_lerobot_dataset(
            request=LerobotDatasetExportRequest(
                dataset_id="dataset-1",
                dataset_name="dataset_1",
                episode_ids=[EPISODE_ID],
                fps=30,
            ),
            profile_payload=_profile_payload_with_idle_leader_action(),
            datasets_dir=tmp_path,
        )

    assert exc_info.value.code == "episode_alignment_gap"
    assert IDLE_ACTION_TOPIC in exc_info.value.detail


def test_export_allows_empty_unused_source_action_topic(tmp_path: Path) -> None:
    _write_episode_store(tmp_path, include_idle_action_topic=True)

    response = export_lerobot_dataset(
        request=LerobotDatasetExportRequest(
            dataset_id="dataset-1",
            dataset_name="dataset_1",
            episode_ids=[EPISODE_ID],
            fps=30,
        ),
        profile_payload=_profile_payload_with_idle_leader_action(),
        datasets_dir=tmp_path,
    )

    assert response.frame_count == 3
    rows = pq.read_table(
        tmp_path / "dataset-1" / "data" / "chunk-000" / "file-000.parquet"
    ).to_pylist()
    assert rows[0]["action"] == pytest.approx(
        [math.degrees(0.4), math.degrees(0.5)]
    )


def test_export_allows_initial_latched_mux_status_after_first_frame(
    tmp_path: Path,
) -> None:
    _write_episode_store(tmp_path, mux_stamp_ns=1_200_000_000)

    response = export_lerobot_dataset(
        request=LerobotDatasetExportRequest(
            dataset_id="dataset-1",
            dataset_name="dataset_1",
            episode_ids=[EPISODE_ID],
            max_mux_status_age_s=0.3,
        ),
        profile_payload=_profile_payload(),
        datasets_dir=tmp_path,
    )

    assert response.frame_count == 3


def test_export_refuses_when_required_joint_stream_starts_too_late(
    tmp_path: Path,
) -> None:
    _write_episode_store(tmp_path, joint_stamp_offset_ns=166_666_667)

    with pytest.raises(LerobotDatasetExportError) as exc_info:
        export_lerobot_dataset(
            request=LerobotDatasetExportRequest(
                dataset_id="dataset-1",
                dataset_name="dataset_1",
                episode_ids=[EPISODE_ID],
            ),
            profile_payload=_profile_payload(),
            datasets_dir=tmp_path,
        )

    assert exc_info.value.code == "episode_alignment_gap"
    assert "first grid frame" in exc_info.value.detail


@pytest.mark.skipif(
    not LOCAL_VFR_READER_AVAILABLE,
    reason="requires the local LeRobot VFR timestamp reader",
)
def test_export_builds_fixed_fps_rows_from_irregular_vfr_stamps(tmp_path: Path) -> None:
    frame_stamps_ns = [
        1_000_000_000,
        1_040_000_000,
        1_075_000_000,
        1_110_000_000,
    ]
    _write_episode_store(tmp_path, frame_stamps_ns=frame_stamps_ns)

    response = export_lerobot_dataset(
        request=LerobotDatasetExportRequest(
            dataset_id="dataset-1",
            dataset_name="dataset_1",
            episode_ids=[EPISODE_ID],
            fps=30,
        ),
        profile_payload=_profile_payload(),
        datasets_dir=tmp_path,
    )

    dataset_root = tmp_path / "dataset-1"
    assert response.frame_count == 4
    rows = pq.read_table(
        dataset_root / "data" / "chunk-000" / "file-000.parquet"
    ).to_pylist()
    assert [row["timestamp"] for row in rows] == pytest.approx(
        [0.0, 1 / 30, 2 / 30, 3 / 30]
    )
    video_path = (
        dataset_root
        / "videos"
        / "observation.images.wrist"
        / "chunk-000"
        / "file-000.mp4"
    )
    source_video = next(
        (tmp_path / "episodes").glob("*/*/*/videos/wrist/segment-000.mp4")
    )
    assert video_path.stat().st_ino == source_video.stat().st_ino
    dataset = LeRobotDataset(
        repo_id="dataset-1",
        root=dataset_root,
        video_backend="pyav",
    )
    means = [
        float(dataset[index]["observation.images.wrist"].mean()) for index in range(4)
    ]
    assert means == pytest.approx([0.0, 40 / 255, 80 / 255, 120 / 255], abs=0.03)


@pytest.mark.skipif(
    not LOCAL_VFR_READER_AVAILABLE,
    reason="requires the local LeRobot VFR timestamp reader",
)
def test_export_and_reader_reuse_one_camera_frame_for_three_grid_ticks(
    tmp_path: Path,
) -> None:
    bag_stamps_ns = [
        1_000_000_000 + round(index * 1_000_000_000 / 120) for index in range(4)
    ]
    _write_episode_store(
        tmp_path,
        frame_stamps_ns=[1_000_000_000],
        bag_sample_stamps_ns=bag_stamps_ns,
    )

    response = export_lerobot_dataset(
        request=LerobotDatasetExportRequest(
            dataset_id="dataset-1",
            dataset_name="dataset_1",
            episode_ids=[EPISODE_ID],
            fps=120,
        ),
        profile_payload=_profile_payload(),
        datasets_dir=tmp_path,
    )

    assert response.frame_count == 4
    dataset = LeRobotDataset(
        repo_id="dataset-1",
        root=tmp_path / "dataset-1",
        video_backend="pyav",
    )
    means = [
        float(dataset[index]["observation.images.wrist"].mean()) for index in range(4)
    ]
    assert means == pytest.approx([0.0, 0.0, 0.0, 0.0], abs=0.01)


def test_export_refuses_fourth_reuse_of_same_camera_frame(tmp_path: Path) -> None:
    bag_stamps_ns = [
        1_000_000_000 + round(index * 1_000_000_000 / 120) for index in range(6)
    ]
    _write_episode_store(
        tmp_path,
        frame_stamps_ns=[1_000_000_000],
        bag_sample_stamps_ns=bag_stamps_ns,
    )

    with pytest.raises(LerobotDatasetExportError) as exc_info:
        export_lerobot_dataset(
            request=LerobotDatasetExportRequest(
                dataset_id="dataset-1",
                dataset_name="dataset_1",
                episode_ids=[EPISODE_ID],
                fps=120,
            ),
            profile_payload=_profile_payload(),
            datasets_dir=tmp_path,
        )

    assert exc_info.value.code == "episode_alignment_gap"
    assert "camera observation.images.wrist" in exc_info.value.detail
    assert "carry_frames=4" in exc_info.value.detail


def test_export_uses_schema_v2_video_timing_without_meta_duplicates(
    tmp_path: Path,
) -> None:
    _write_episode_store(tmp_path)

    response = export_lerobot_dataset(
        request=LerobotDatasetExportRequest(
            dataset_id="dataset-1",
            dataset_name="dataset_1",
            episode_ids=[EPISODE_ID],
        ),
        profile_payload=_profile_payload(),
        datasets_dir=tmp_path,
    )

    assert response.frame_count == 3


@pytest.mark.parametrize(
    ("schema_version", "expected_code"),
    [
        (None, "episode_schema_version_missing"),
        (1, "episode_schema_version_unsupported"),
        (3, "episode_schema_version_unsupported"),
    ],
)
def test_export_dispatches_episode_schema_explicitly(
    tmp_path: Path,
    schema_version: int | None,
    expected_code: str,
) -> None:
    _write_episode_store(tmp_path)
    meta_path = next((tmp_path / "episodes").glob("*/*/*/meta.json"))
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    if schema_version is None:
        del meta["schema_version"]
    else:
        meta["schema_version"] = schema_version
    meta_path.write_text(json.dumps(meta), encoding="utf-8")

    with pytest.raises(LerobotDatasetExportError) as exc_info:
        export_lerobot_dataset(
            request=LerobotDatasetExportRequest(
                dataset_id="dataset-1",
                dataset_name="dataset_1",
                episode_ids=[EPISODE_ID],
            ),
            profile_payload=_profile_payload(),
            datasets_dir=tmp_path,
        )

    assert exc_info.value.code == expected_code


@pytest.mark.parametrize(
    "field",
    ["profile"],
)
def test_export_refuses_incomplete_finished_episode_meta(
    tmp_path: Path, field: str
) -> None:
    _write_episode_store(tmp_path)
    meta_path = next((tmp_path / "episodes").glob("*/*/*/meta.json"))
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    del meta[field]
    meta_path.write_text(json.dumps(meta), encoding="utf-8")

    with pytest.raises(LerobotDatasetExportError) as exc_info:
        export_lerobot_dataset(
            request=LerobotDatasetExportRequest(
                dataset_id="dataset-1",
                dataset_name="dataset_1",
                episode_ids=[EPISODE_ID],
            ),
            profile_payload=_profile_payload(),
            datasets_dir=tmp_path,
        )

    assert exc_info.value.code == "episode_meta_invalid"


def test_export_does_not_use_removed_recorder_timeline_metadata(tmp_path: Path) -> None:
    _write_episode_store(tmp_path)
    meta_path = next((tmp_path / "episodes").glob("*/*/*/meta.json"))
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    meta["timeline_start_ros_ns"] = 0
    meta_path.write_text(json.dumps(meta), encoding="utf-8")

    response = export_lerobot_dataset(
        request=LerobotDatasetExportRequest(
            dataset_id="dataset-1",
            dataset_name="dataset_1",
            episode_ids=[EPISODE_ID],
        ),
        profile_payload=_profile_payload(),
        datasets_dir=tmp_path,
    )

    assert response.frame_count == 3


def test_export_rejects_sidecar_pts_that_do_not_match_ros_timestamps(
    tmp_path: Path,
) -> None:
    _write_episode_store(tmp_path)
    sidecar_path = next((tmp_path / "episodes").glob("*/*/*/videos/wrist/frames.parquet"))
    sidecar_path.chmod(0o644)
    rows = pq.read_table(sidecar_path).to_pylist()
    rows[1]["video_pts"] += 1
    pq.write_table(pa.Table.from_pylist(rows), sidecar_path)
    sidecar_path.chmod(0o444)

    with pytest.raises(LerobotDatasetExportError) as exc_info:
        export_lerobot_dataset(
            request=LerobotDatasetExportRequest(
                dataset_id="dataset-1",
                dataset_name="dataset_1",
                episode_ids=[EPISODE_ID],
            ),
            profile_payload=_profile_payload(),
            datasets_dir=tmp_path,
        )

    assert exc_info.value.code == "camera_video_pts_ros_stamp_mismatch"


def test_export_preserves_unknown_stop_metadata_as_null(tmp_path: Path) -> None:
    _write_episode_store(tmp_path)
    meta_path = next((tmp_path / "episodes").glob("*/*/*/meta.json"))
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    del meta["stopped_at"]
    del meta["duration_s"]
    meta_path.write_text(json.dumps(meta), encoding="utf-8")

    response = export_lerobot_dataset(
        request=LerobotDatasetExportRequest(
            dataset_id="dataset-1",
            dataset_name="dataset_1",
            episode_ids=[EPISODE_ID],
        ),
        profile_payload=_profile_payload(),
        datasets_dir=tmp_path,
    )

    provenance = json.loads(Path(response.provenance_path).read_text(encoding="utf-8"))
    assert provenance["source_episodes"][0]["stopped_at"] is None
    assert provenance["source_episodes"][0]["duration_s"] is None


def test_export_uses_inferred_grid_start_for_all_camera_video_offsets(
    tmp_path: Path,
) -> None:
    _write_episode_store(
        tmp_path,
        second_camera_stamps_ns=[1_033_333_333, 1_066_666_667],
    )

    response = export_lerobot_dataset(
        request=LerobotDatasetExportRequest(
            dataset_id="dataset-1",
            dataset_name="dataset_1",
            episode_ids=[EPISODE_ID],
        ),
        profile_payload=_profile_payload_with_two_cameras(),
        datasets_dir=tmp_path,
    )

    assert response.frame_count == 2
    episode_row = pq.read_table(
        tmp_path / "dataset-1" / "meta" / "episodes" / "chunk-000" / "file-000.parquet"
    ).to_pylist()[0]
    assert episode_row[
        "videos/observation.images.wrist/from_timestamp"
    ] == pytest.approx(1 / 30)
    assert episode_row["videos/observation.images.top/from_timestamp"] == pytest.approx(
        0.0
    )


def test_export_skips_profile_camera_that_is_explicitly_disabled(
    tmp_path: Path,
) -> None:
    _write_episode_store(tmp_path)
    profile = _profile_payload_with_two_cameras()
    profile["lerobot"]["cameras"][1]["enabled"] = False

    response = export_lerobot_dataset(
        request=LerobotDatasetExportRequest(
            dataset_id="dataset-1",
            dataset_name="dataset_1",
            episode_ids=[EPISODE_ID],
        ),
        profile_payload=profile,
        datasets_dir=tmp_path,
    )

    info = json.loads(
        (Path(response.dataset_path) / "meta" / "info.json").read_text(encoding="utf-8")
    )
    assert "observation.images.wrist" in info["features"]
    assert "observation.images.top" not in info["features"]


def test_export_rejects_unresolved_camera_enabled_placeholder(tmp_path: Path) -> None:
    _write_episode_store(tmp_path)
    profile = _profile_payload()
    profile["lerobot"]["cameras"][0]["enabled"] = "${camera_enabled}"

    with pytest.raises(LerobotDatasetExportError) as exc_info:
        export_lerobot_dataset(
            request=LerobotDatasetExportRequest(
                dataset_id="dataset-1",
                dataset_name="dataset_1",
                episode_ids=[EPISODE_ID],
            ),
            profile_payload=profile,
            datasets_dir=tmp_path,
        )

    assert exc_info.value.code == "profile_invalid"


def test_export_refuses_interior_fixed_fps_alignment_gap(tmp_path: Path) -> None:
    _write_episode_store(
        tmp_path,
        frame_stamps_ns=[1_000_000_000, 1_033_333_333, 1_400_000_000],
    )

    with pytest.raises(LerobotDatasetExportError) as exc_info:
        export_lerobot_dataset(
            request=LerobotDatasetExportRequest(
                dataset_id="dataset-1",
                dataset_name="dataset_1",
                episode_ids=[EPISODE_ID],
            ),
            profile_payload=_profile_payload(),
            datasets_dir=tmp_path,
        )

    assert exc_info.value.code == "episode_alignment_gap"


def test_export_refuses_camera_resolution_mismatch_between_episodes(
    tmp_path: Path,
) -> None:
    second_episode_id = "episode-87654321"
    _write_episode_store(tmp_path, episode_id=EPISODE_ID)
    _write_episode_store(tmp_path, episode_id=second_episode_id)
    second_meta_path = next(
        (tmp_path / "episodes").glob(f"*/*/*{second_episode_id[-8:]}/meta.json")
    )
    second_meta = json.loads(second_meta_path.read_text(encoding="utf-8"))
    second_meta["cameras"][0]["width"] = 8
    second_meta_path.write_text(json.dumps(second_meta), encoding="utf-8")

    with pytest.raises(LerobotDatasetExportError) as exc_info:
        export_lerobot_dataset(
            request=LerobotDatasetExportRequest(
                dataset_id="dataset-1",
                dataset_name="dataset_1",
                episode_ids=[EPISODE_ID, second_episode_id],
            ),
            profile_payload=_profile_payload(),
            datasets_dir=tmp_path,
        )

    assert exc_info.value.code == "camera_resolution_mismatch"


def test_reused_video_hardlink_failure_is_not_copied(
    monkeypatch, tmp_path: Path
) -> None:
    source = tmp_path / "source.mp4"
    destination = tmp_path / "dest.mp4"
    source.write_bytes(b"h264 source")
    source.chmod(0o444)

    def fail_hardlink(_self: Path, _target: Path) -> None:
        raise PermissionError("hardlink denied")

    monkeypatch.setattr(Path, "hardlink_to", fail_hardlink)

    with pytest.raises(LerobotDatasetExportError) as exc_info:
        exporter._hardlink_video(source, destination)

    assert exc_info.value.code == "video_hardlink_failed"
    assert not destination.exists()


def test_export_refuses_mutable_finished_video(tmp_path: Path) -> None:
    _write_episode_store(tmp_path)
    video = next((tmp_path / "episodes").glob("*/*/*/videos/wrist/segment-000.mp4"))
    video.chmod(0o644)

    with pytest.raises(LerobotDatasetExportError) as exc_info:
        export_lerobot_dataset(
            request=LerobotDatasetExportRequest(
                dataset_id="dataset-1",
                dataset_name="dataset_1",
                episode_ids=[EPISODE_ID],
            ),
            profile_payload=_profile_payload(),
            datasets_dir=tmp_path,
        )

    assert exc_info.value.code == "video_source_mutable"


def test_export_refuses_mutable_finished_sidecar(tmp_path: Path) -> None:
    _write_episode_store(tmp_path)
    sidecar = next((tmp_path / "episodes").glob("*/*/*/videos/wrist/frames.parquet"))
    sidecar.chmod(0o644)

    with pytest.raises(LerobotDatasetExportError) as exc_info:
        export_lerobot_dataset(
            request=LerobotDatasetExportRequest(
                dataset_id="dataset-1",
                dataset_name="dataset_1",
                episode_ids=[EPISODE_ID],
            ),
            profile_payload=_profile_payload(),
            datasets_dir=tmp_path,
        )

    assert exc_info.value.code == "frames_sidecar_mutable"


def test_export_refuses_episode_id_mapped_to_multiple_directories(
    tmp_path: Path,
) -> None:
    _write_episode_store(tmp_path)
    original = next((tmp_path / "episodes").glob("*/*/*/meta.json")).parent
    duplicate = tmp_path / "episodes" / "2027" / "07" / "unrelated-folder-name"
    shutil.copytree(original, duplicate)

    with pytest.raises(LerobotDatasetExportError) as exc_info:
        export_lerobot_dataset(
            request=LerobotDatasetExportRequest(
                dataset_id="dataset-1",
                dataset_name="dataset_1",
                episode_ids=[EPISODE_ID],
            ),
            profile_payload=_profile_payload(),
            datasets_dir=tmp_path,
        )

    assert exc_info.value.code == "episode_id_ambiguous"


def test_video_size_probe_uses_lerobot_video_info(tmp_path: Path) -> None:
    video_path = tmp_path / "source.mp4"
    _write_video(video_path, frame_count=1)

    assert exporter._probe_video_size(video_path) == (4, 4)


def _video_frame_count(path: Path) -> int:
    cap = cv2.VideoCapture(str(path))
    try:
        return int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    finally:
        cap.release()


def _profile_payload() -> dict[str, JsonValue]:
    return {
        "name": "piper_single_teleop",
        "lerobot": {
            "arm_streams": [
                {
                    "key": "follower",
                    "namespace": "follower_arm",
                    "joints": ["joint1", "joint2"],
                    "rx": {
                        "joint_state": {"topic": STATE_TOPIC},
                        "joint_command": {
                            "topic": ACTION_TOPIC,
                            "leader": {"topic": ACTION_TOPIC},
                            "vr": {"topic": ACTION_TOPIC},
                        },
                    },
                }
            ],
            "cameras": [
                {
                    "name": "wrist",
                    "topic": CAMERA_TOPIC,
                    "enabled": True,
                }
            ],
        },
    }


def _profile_payload_with_idle_leader_action() -> dict[str, JsonValue]:
    profile = _profile_payload()
    arm_stream = profile["lerobot"]["arm_streams"][0]
    rx = arm_stream["rx"]
    rx["joint_command"] = {
        "topic": IDLE_ACTION_TOPIC,
        "leader": {"topic": IDLE_ACTION_TOPIC},
        "vr": {"topic": ACTION_TOPIC},
    }
    return profile


def _profile_payload_with_two_cameras() -> dict[str, JsonValue]:
    profile = _profile_payload()
    cameras = profile["lerobot"]["cameras"]
    cameras.append(
        {
            "name": "top",
            "topic": SECOND_CAMERA_TOPIC,
            "enabled": True,
        }
    )
    return profile


def _write_episode_store(
    root: Path,
    *,
    episode_id: str = EPISODE_ID,
    include_action: bool = True,
    include_idle_action_topic: bool = False,
    mux_source: str = "vr",
    mux_stamp_ns: int = 999_000_000,
    joint_stamp_offset_ns: int = 0,
    frame_stamps_ns: list[int] | None = None,
    bag_sample_stamps_ns: list[int] | None = None,
    second_camera_stamps_ns: list[int] | None = None,
) -> None:
    if frame_stamps_ns is None:
        frame_stamps_ns = [1_000_000_000, 1_033_333_333, 1_066_666_667]
    episode_dir = root / "episodes" / "2026" / "07" / episode_id
    camera_dir = episode_dir / "videos" / "wrist"
    camera_dir.mkdir(parents=True)
    _write_video(camera_dir / "segment-000.mp4", frame_count=len(frame_stamps_ns))
    _remux_video_pts(camera_dir / "segment-000.mp4", frame_stamps_ns)
    _write_frames_sidecar(camera_dir / "frames.parquet", frame_stamps_ns)
    _write_bag(
        episode_dir / "bag",
        frame_stamps_ns=(
            bag_sample_stamps_ns
            if bag_sample_stamps_ns is not None
            else frame_stamps_ns
        ),
        include_action=include_action,
        include_idle_action_topic=include_idle_action_topic,
        mux_source=mux_source,
        mux_stamp_ns=mux_stamp_ns,
        joint_stamp_offset_ns=joint_stamp_offset_ns,
    )
    camera_meta: dict[str, JsonValue] = {
        "name": "wrist",
        "topic": CAMERA_TOPIC,
        "width": 4,
        "height": 4,
        "segments": [{"file": "segment-000.mp4"}],
    }
    camera_metas = [camera_meta]
    if second_camera_stamps_ns is not None:
        second_camera_dir = episode_dir / "videos" / "top"
        second_camera_dir.mkdir(parents=True)
        _write_video(
            second_camera_dir / "segment-000.mp4",
            frame_count=len(second_camera_stamps_ns),
        )
        _remux_video_pts(second_camera_dir / "segment-000.mp4", second_camera_stamps_ns)
        _write_frames_sidecar(
            second_camera_dir / "frames.parquet", second_camera_stamps_ns
        )
        camera_metas.append(
            {
                "name": "top",
                "topic": SECOND_CAMERA_TOPIC,
                "width": 4,
                "height": 4,
                "segments": [{"file": "segment-000.mp4"}],
            }
        )
    meta = {
        "schema_version": 2,
        "episode_id": episode_id,
        "state": "finished",
        "task_description": "pick block",
        "profile": "piper_single_teleop",
        "started_at": "2026-07-06T01:00:00Z",
        "stopped_at": "2026-07-06T01:00:00.200000Z",
        "duration_s": 0.2,
        "outcome": "success",
        "tags": ["dpex:record"],
        "cameras": camera_metas,
        "recorded_topics": [
            {"topic": STATE_TOPIC, "stamp_source": "message_header"},
            {"topic": ACTION_TOPIC, "stamp_source": "message_header"},
            {"topic": IDLE_ACTION_TOPIC, "stamp_source": "message_header"},
        ],
        "bag_path": "bag",
    }
    (episode_dir / "meta.json").write_text(json.dumps(meta), encoding="utf-8")
    for source in (episode_dir / "videos").glob("*/*"):
        if source.is_file():
            source.chmod(0o444)


def _write_video(path: Path, *, frame_count: int) -> None:
    writer = cv2.VideoWriter(
        str(path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        30,
        (4, 4),
    )
    if not writer.isOpened():
        raise AssertionError("test video writer did not open")
    try:
        for index in range(frame_count):
            frame = np.full((4, 4, 3), index * 40, dtype=np.uint8)
            writer.write(frame)
    finally:
        writer.release()


def _remux_video_pts(path: Path, frame_stamps_ns: list[int]) -> None:
    time_base = Fraction(1, 90_000)
    first_stamp_ns = frame_stamps_ns[0]
    temporary_path = path.with_name(f".{path.stem}.vfr{path.suffix}")
    input_container = av.open(str(path), mode="r")
    output_container = av.open(str(temporary_path), mode="w")
    packet_count = 0
    try:
        input_stream = input_container.streams.video[0]
        output_stream = output_container.add_stream_from_template(input_stream)
        output_stream.time_base = time_base
        for packet in input_container.demux(input_stream):
            if packet.dts is None:
                continue
            pts = round(
                (frame_stamps_ns[packet_count] - first_stamp_ns)
                * time_base.denominator
                / 1_000_000_000
            )
            packet.pts = pts
            packet.dts = pts
            packet.time_base = time_base
            packet.stream = output_stream
            if packet_count + 1 < len(frame_stamps_ns):
                next_pts = round(
                    (frame_stamps_ns[packet_count + 1] - first_stamp_ns)
                    * time_base.denominator
                    / 1_000_000_000
                )
                packet.duration = max(1, next_pts - pts)
            output_container.mux(packet)
            packet_count += 1
    finally:
        input_container.close()
        output_container.close()
    assert packet_count == len(frame_stamps_ns)
    temporary_path.replace(path)


def _write_frames_sidecar(path: Path, frame_stamps_ns: list[int]) -> None:
    first_stamp_ns = frame_stamps_ns[0]
    rows = [
        {
            "frame_index": index,
            "segment_file": "segment-000.mp4",
            "segment_local_frame": index,
            "video_pts": round((stamp_ns - first_stamp_ns) * 90_000 / 1_000_000_000),
            "ros_stamp_ns": stamp_ns,
            "recv_stamp_ns": stamp_ns,
            "source_seq": index,
            "dropped_before": 0,
            "keyframe": index == 0,
        }
        for index, stamp_ns in enumerate(frame_stamps_ns)
    ]
    pq.write_table(pa.Table.from_pylist(rows), path)


def _write_bag(
    path: Path,
    *,
    frame_stamps_ns: list[int],
    include_action: bool,
    include_idle_action_topic: bool = False,
    mux_source: str = "vr",
    mux_stamp_ns: int = 999_000_000,
    joint_stamp_offset_ns: int = 0,
) -> None:
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    time_cls = typestore.types["builtin_interfaces/msg/Time"]
    header_cls = typestore.types["std_msgs/msg/Header"]
    joint_state_cls = typestore.types["sensor_msgs/msg/JointState"]
    string_cls = typestore.types["std_msgs/msg/String"]

    def joint_message(stamp_ns: int, positions: list[float]):
        sec = stamp_ns // 1_000_000_000
        nanosec = stamp_ns % 1_000_000_000
        return joint_state_cls(
            header_cls(time_cls(sec=sec, nanosec=nanosec), frame_id=""),
            ["joint1", "joint2"],
            np.asarray(positions, dtype=np.float64),
            np.asarray([], dtype=np.float64),
            np.asarray([], dtype=np.float64),
        )

    with Writer(path, version=9) as writer:
        mux_connection = writer.add_connection(
            MUX_TOPIC, "std_msgs/msg/String", typestore=typestore
        )
        state_connection = writer.add_connection(
            STATE_TOPIC, "sensor_msgs/msg/JointState", typestore=typestore
        )
        action_connection = (
            writer.add_connection(
                ACTION_TOPIC, "sensor_msgs/msg/JointState", typestore=typestore
            )
            if include_action
            else None
        )
        if include_idle_action_topic:
            writer.add_connection(
                IDLE_ACTION_TOPIC, "sensor_msgs/msg/JointState", typestore=typestore
            )
        writer.write(
            mux_connection,
            mux_stamp_ns,
            typestore.serialize_cdr(
                string_cls(json.dumps({"source": mux_source})), "std_msgs/msg/String"
            ),
        )
        for index, frame_stamp_ns in enumerate(frame_stamps_ns):
            stamp_ns = frame_stamp_ns + joint_stamp_offset_ns
            writer.write(
                state_connection,
                stamp_ns,
                typestore.serialize_cdr(
                    joint_message(stamp_ns, [0.1 + index, 0.2 + index]),
                    "sensor_msgs/msg/JointState",
                ),
            )
            if action_connection is not None:
                writer.write(
                    action_connection,
                    stamp_ns,
                    typestore.serialize_cdr(
                        joint_message(stamp_ns, [0.4 + index, 0.5 + index]),
                        "sensor_msgs/msg/JointState",
                    ),
                )
