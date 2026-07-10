from __future__ import annotations

import json
from fractions import Fraction
from pathlib import Path

import av
import cv2
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
import pytest
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

import fv_lerobot_exporter.exporter as exporter
from fv_lerobot_exporter import (
    LerobotDatasetExportError,
    LerobotDatasetExportProgress,
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
    stats = json.loads(
        (dataset_root / "meta" / "stats.json").read_text(encoding="utf-8")
    )
    assert set(stats) == {"observation.state", "action"}

    data_rows = pq.read_table(
        dataset_root / "data" / "chunk-000" / "file-000.parquet"
    ).to_pylist()
    assert data_rows[0]["observation.state"] == pytest.approx([0.1, 0.2])
    assert data_rows[0]["action"] == pytest.approx([0.4, 0.5])
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


def test_export_reports_current_progress(tmp_path: Path) -> None:
    _write_episode_store(tmp_path)
    events: list[LerobotDatasetExportProgress] = []

    response = export_lerobot_dataset(
        request=LerobotDatasetExportRequest(
            dataset_id="dataset-1",
            dataset_name="dataset_1",
            episode_ids=[EPISODE_ID],
            fps=30,
        ),
        profile_payload=_profile_payload(),
        datasets_dir=tmp_path,
        progress=events.append,
    )

    phases = [event.phase for event in events]
    assert response.frame_count == 3
    assert "aligning" in phases
    assert "writing_videos" in phases
    assert "writing_stats" in phases
    assert events[-1].phase == "finalizing"
    assert any(
        event.detail and "observation.images.wrist" in event.detail for event in events
    )


def test_export_refuses_missing_action_topic(tmp_path: Path) -> None:
    _write_episode_store(tmp_path, include_action=False)

    try:
        export_lerobot_dataset(
            request=LerobotDatasetExportRequest(
                dataset_id="dataset-1",
                dataset_name="dataset_1",
                episode_ids=[EPISODE_ID],
            ),
            profile_payload=_profile_payload(),
            datasets_dir=tmp_path,
        )
    except LerobotDatasetExportError as exc:
        assert exc.code == "bag_topic_missing"
        assert EPISODE_ID in exc.detail
        assert ACTION_TOPIC in exc.detail
    else:
        raise AssertionError("missing action topic must fail closed")

    assert not (tmp_path / "dataset-1").exists()
    assert not (tmp_path / ".lerobot_exports" / "dataset-1").exists()


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
    assert rows[0]["action"] == pytest.approx([0.4, 0.5])


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


def test_export_trims_unaligned_boundary_without_reencoding_video(
    tmp_path: Path,
) -> None:
    _write_episode_store(tmp_path, joint_stamp_offset_ns=166_666_667)

    response = export_lerobot_dataset(
        request=LerobotDatasetExportRequest(
            dataset_id="dataset-1",
            dataset_name="dataset_1",
            episode_ids=[EPISODE_ID],
        ),
        profile_payload=_profile_payload(),
        datasets_dir=tmp_path,
    )

    assert response.frame_count == 1
    dataset_root = tmp_path / "dataset-1"
    rows = pq.read_table(
        dataset_root / "data" / "chunk-000" / "file-000.parquet"
    ).to_pylist()
    assert rows[0]["timestamp"] == pytest.approx(0.0)
    assert rows[0]["frame_index"] == 0
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
    assert _video_frame_count(video_path) == 3
    assert video_path.stat().st_ino == source_video.stat().st_ino
    episode_row = pq.read_table(
        dataset_root / "meta" / "episodes" / "chunk-000" / "file-000.parquet"
    ).to_pylist()[0]
    assert episode_row[
        "videos/observation.images.wrist/from_timestamp"
    ] == pytest.approx(2 / 30)


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


def test_export_refuses_undeclared_video_timing_contract(tmp_path: Path) -> None:
    _write_episode_store(tmp_path, declare_video_timing_contract=False)

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

    assert exc_info.value.code == "camera_video_timing_contract_missing"


def test_export_trims_all_cameras_to_common_nonnegative_video_window(
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

    def fail_hardlink(_self: Path, _target: Path) -> None:
        raise PermissionError("hardlink denied")

    monkeypatch.setattr(Path, "hardlink_to", fail_hardlink)

    with pytest.raises(LerobotDatasetExportError) as exc_info:
        exporter._hardlink_video(source, destination)

    assert exc_info.value.code == "video_hardlink_failed"
    assert not destination.exists()


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
                    "enabled": "true",
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
            "enabled": "true",
        }
    )
    return profile


def _write_episode_store(
    root: Path,
    *,
    episode_id: str = EPISODE_ID,
    include_action: bool = True,
    include_idle_action_topic: bool = False,
    mux_stamp_ns: int = 999_000_000,
    joint_stamp_offset_ns: int = 0,
    frame_stamps_ns: list[int] | None = None,
    second_camera_stamps_ns: list[int] | None = None,
    declare_video_timing_contract: bool = True,
) -> None:
    if frame_stamps_ns is None:
        frame_stamps_ns = [1_000_000_000, 1_033_333_333, 1_066_666_667]
    episode_dir = root / "episodes" / "2026" / "07" / episode_id
    camera_dir = episode_dir / "videos" / "wrist"
    camera_dir.mkdir(parents=True)
    _write_video(camera_dir / "segment-000.mp4", frame_count=len(frame_stamps_ns))
    if declare_video_timing_contract:
        _remux_video_pts(camera_dir / "segment-000.mp4", frame_stamps_ns)
    _write_frames_sidecar(camera_dir / "frames.parquet", frame_stamps_ns)
    _write_bag(
        episode_dir / "bag",
        frame_stamps_ns=frame_stamps_ns,
        include_action=include_action,
        include_idle_action_topic=include_idle_action_topic,
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
    if declare_video_timing_contract:
        camera_meta["video_timing_mode"] = "ros_header_stamp_to_pts"
        camera_meta["video_pts_origin_ros_ns"] = frame_stamps_ns[0]
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
                "video_timing_mode": "ros_header_stamp_to_pts",
                "video_pts_origin_ros_ns": second_camera_stamps_ns[0],
                "segments": [{"file": "segment-000.mp4"}],
            }
        )
    meta = {
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
    rows = [
        {
            "frame_index": index,
            "segment_file": "segment-000.mp4",
            "segment_local_frame": index,
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
                string_cls(json.dumps({"source": "vr"})), "std_msgs/msg/String"
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
