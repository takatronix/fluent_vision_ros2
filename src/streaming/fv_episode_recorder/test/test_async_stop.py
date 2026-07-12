from __future__ import annotations

import asyncio
from concurrent.futures import ThreadPoolExecutor
import json
import stat
import subprocess
import sys
import threading
import time
import types
from dataclasses import asdict
from fractions import Fraction
from pathlib import Path

import pytest
import pyarrow.parquet as pq

_rclpy = types.ModuleType("rclpy")
_rclpy_node = types.ModuleType("rclpy.node")
_rclpy_qos = types.ModuleType("rclpy.qos")
_sensor_msgs = types.ModuleType("sensor_msgs")
_sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
_ulid = types.ModuleType("ulid")


class _Node:
    pass


class _CompressedImage:
    pass


class _Image:
    pass


class _ULID:
    def __str__(self) -> str:
        return "01TESTASYNCSTOP0000000000"


_rclpy_node.Node = _Node
_rclpy_qos.qos_profile_sensor_data = object()
_sensor_msgs_msg.CompressedImage = _CompressedImage
_sensor_msgs_msg.Image = _Image
_ulid.ULID = _ULID
sys.modules.setdefault("rclpy", _rclpy)
sys.modules.setdefault("rclpy.node", _rclpy_node)
sys.modules.setdefault("rclpy.qos", _rclpy_qos)
sys.modules.setdefault("sensor_msgs", _sensor_msgs)
sys.modules.setdefault("sensor_msgs.msg", _sensor_msgs_msg)
sys.modules.setdefault("ulid", _ulid)

import fv_episode_recorder.api_server as api_server
import fv_episode_recorder.camera_writer as camera_writer
from fv_episode_recorder.api_server import (
    _add_episode_tags,
    _recover_episode,
    _retention_put_policy,
    _start_episode,
    _stop_episode,
)
from fv_episode_recorder.bag_recorder import (
    BAG_FINALIZE_TIMEOUT_S,
    BagRecorder,
    DetachedBagRecording,
)
from fv_episode_recorder.camera_writer import CameraWriter
from fv_episode_recorder.depth_republisher import DepthRepublisherPool, _DepthRepublisher
from fv_episode_recorder.episode_store import (
    EpisodeMeta,
    EpisodeMetadataError,
    EpisodeStore,
    read_episode_meta,
    utc_now_iso,
)
from fv_episode_recorder.episode_schema import (
    JsonValue,
    UnsupportedEpisodeSchemaVersionError,
)
from fv_episode_recorder.migrations import migrate_episode_document
from fv_episode_recorder.retention import RetentionPolicy, RetentionRunner


class _DetachedBag:
    def wait(self, timeout_s: float = 10.0) -> dict:
        time.sleep(0.05)
        return {"size_bytes": 0, "split_count": 0}


class _TimedOutDetachedBag:
    def __init__(self) -> None:
        self.timeout_s: float | None = None

    def wait(self, timeout_s: float) -> dict:
        self.timeout_s = timeout_s
        raise RuntimeError(f"bag recorder finalize timed out after {timeout_s:.1f}s")


class _ProcessThatTimesOut:
    def __init__(self) -> None:
        self.returncode: int | None = None
        self.stderr = None
        self.wait_timeouts: list[float] = []
        self.terminated = False
        self.killed = False

    def poll(self) -> int | None:
        return self.returncode

    def wait(self, timeout: float) -> int:
        self.wait_timeouts.append(timeout)
        if not self.terminated and not self.killed:
            raise subprocess.TimeoutExpired("ros2 bag record", timeout)
        self.returncode = 0
        return self.returncode

    def terminate(self) -> None:
        self.terminated = True

    def kill(self) -> None:
        self.killed = True


class _ProcessThatIgnoresTerminate(_ProcessThatTimesOut):
    def wait(self, timeout: float) -> int:
        self.wait_timeouts.append(timeout)
        if not self.killed:
            raise subprocess.TimeoutExpired("ros2 bag record", timeout)
        self.returncode = -9
        return self.returncode


def test_bag_recorder_uses_standard_ros2_cli(tmp_path: Path, monkeypatch) -> None:
    class _Process:
        def poll(self) -> None:
            return None

    commands: list[list[str]] = []

    def _popen(command: list[str], **_kwargs) -> _Process:
        commands.append(command)
        return _Process()

    monkeypatch.setattr(subprocess, "Popen", _popen)
    recorder = BagRecorder()

    recorder.start(tmp_path / "bag", ["/joint_states"])

    assert commands[0][:3] == ["ros2", "bag", "record"]
    assert commands[0][-1] == "/joint_states"


class _BagRecorder:
    def detach_for_finalize(self) -> _DetachedBag:
        return _DetachedBag()


class _DetachedCameras:
    def frame_counts(self) -> dict[str, int]:
        return {"cam": 1}

    def finalize(self, depth_frame_counts: dict[str, int]) -> tuple[list[dict], list[str]]:
        time.sleep(0.05)
        return (
            [
                {
                    "name": "cam",
                    "frame_count": 1,
                    "segments": [{"size_bytes": 10}],
                }
            ],
            [],
        )


class _CameraPool:
    def ros_now_ns(self) -> int:
        return 2_000_000_000

    def detach_all(self) -> _DetachedCameras:
        return _DetachedCameras()


class _ZeroFrameDetachedCameras:
    def frame_counts(self) -> dict[str, int]:
        return {"top_camera": 0}

    def finalize(self, depth_frame_counts: dict[str, int]) -> tuple[list[dict], list[str]]:
        return (
            [
                {
                    "name": "top_camera",
                    "frame_count": 0,
                    "segments": [{"size_bytes": 262}],
                }
            ],
            [],
        )


class _ZeroFrameCameraPool:
    def ros_now_ns(self) -> int:
        return 2_000_000_000

    def detach_all(self) -> _ZeroFrameDetachedCameras:
        return _ZeroFrameDetachedCameras()


class _DepthPool:
    def stop_all(self) -> dict[str, int]:
        return {}


class _ReadyDepthPool(_DepthPool):
    def __init__(self) -> None:
        self.started_cameras: list[dict] = []

    def start_all(self, cameras: list[dict]) -> list[dict]:
        self.started_cameras = cameras
        return [
            {
                "name": camera["name"],
                "raw_topic": camera["topic"],
                "compressed_topic": (
                    f"{str(camera['topic']).rstrip('/')}_compressed/compressedDepth"
                ),
            }
            for camera in cameras
        ]


class _FailingDepthPool:
    def __init__(self) -> None:
        self.stop_calls = 0

    def start_all(self, _cameras: list[dict]) -> None:
        raise RuntimeError("depth start failed")

    def stop_all(self) -> dict[str, int]:
        self.stop_calls += 1
        return {}


class _TrackingActiveLock:
    def __init__(self) -> None:
        self.acquired = False
        self.released = False

    def acquire(self, _episode_id: str, _episode_dir: Path) -> None:
        self.acquired = True

    def release(self) -> None:
        self.released = True


class _FailingActiveLock(_TrackingActiveLock):
    def acquire(self, _episode_id: str, _episode_dir: Path) -> None:
        self.acquired = True
        raise RuntimeError("lock unavailable")


class _MarkerManager:
    def flush(self, episode_id: str) -> list[dict]:
        return []


class _Request:
    def __init__(
        self,
        store: EpisodeStore,
        episode_id: str,
        *,
        outcome: str = "success",
    ) -> None:
        self.match_info = {"episode_id": episode_id}
        self.outcome = outcome
        self.app = {
            "store": store,
            "bag_recorder": _BagRecorder(),
            "camera_pool": _CameraPool(),
            "depth_pool": _DepthPool(),
            "marker_manager": _MarkerManager(),
            "mux_tracker": None,
            "active_lock": None,
            "finalizer_tasks": set(),
        }

    async def json(self) -> dict[str, str]:
        return {"outcome": self.outcome}


class _TagsRequest:
    def __init__(self, store: EpisodeStore, episode_id: str, tags: list[str]) -> None:
        self.match_info = {"episode_id": episode_id}
        self.app = {"store": store}
        self.tags = tags

    async def json(self) -> dict[str, list[str]]:
        return {"tags": self.tags}


class _RecoverRequest:
    def __init__(self, store: EpisodeStore, episode_id: str, action: str) -> None:
        self.match_info = {"episode_id": episode_id}
        self.app = {"store": store, "active_lock": None}
        self.action = action

    async def json(self) -> dict[str, str]:
        return {"action": self.action}


class _StartRequest:
    def __init__(
        self,
        store: EpisodeStore,
        bag_recorder,
        camera_pool,
        *,
        task_description: str = "pick",
    ) -> None:
        self.match_info = {}
        self.task_description = task_description
        self.app = {
            "store": store,
            "bag_recorder": bag_recorder,
            "camera_pool": camera_pool,
            "depth_pool": None,
            "marker_manager": _MarkerManager(),
            "mux_tracker": _MuxTracker(),
            "active_lock": None,
            "get_profile": lambda _name: _profile(),
        }

    async def json(self) -> dict:
        return {
            "task_description": self.task_description,
            "profile": "piper_single",
            "tags": ["dpex:record"],
            "expected_duration_s": 60,
        }


class _DepthStartRequest(_StartRequest):
    async def json(self) -> dict:
        body = await super().json()
        body["cameras_override"] = [
            {"name": "depth", "topic": "/camera/depth", "kind": "depth"}
        ]
        return body


class _DisabledDepthStartRequest(_StartRequest):
    def __init__(self, store: EpisodeStore, bag_recorder, camera_pool) -> None:
        super().__init__(store, bag_recorder, camera_pool)
        profile = _profile()
        profile["episode_recorder"]["cameras"].append(
            {"name": "depth", "topic": "/camera/depth", "kind": "depth"}
        )
        self.app["get_profile"] = lambda _name: profile

    async def json(self) -> dict:
        body = await super().json()
        body["cameras_override"] = [
            {
                "name": "depth",
                "topic": "/camera/depth",
                "kind": "depth",
                "enabled": False,
            }
        ]
        return body


class _DepthProfileStartRequest(_StartRequest):
    def __init__(self, store: EpisodeStore, bag_recorder, camera_pool) -> None:
        super().__init__(store, bag_recorder, camera_pool)
        profile = _profile()
        profile["episode_recorder"]["cameras"].append(
            {"name": "depth", "topic": "/camera/depth", "kind": "depth"}
        )
        self.app["get_profile"] = lambda _name: profile


class _MissingProfileStartRequest(_StartRequest):
    def __init__(self, store: EpisodeStore, bag_recorder, camera_pool) -> None:
        super().__init__(store, bag_recorder, camera_pool)
        self.app["get_profile"] = lambda _name: {}


class _EmptyOverrideStartRequest(_MissingProfileStartRequest):
    async def json(self) -> dict:
        body = await super().json()
        body["cameras_override"] = []
        return body


class _RetentionRequest:
    def __init__(self, runner, body: dict) -> None:
        self.app = {"retention_runner": runner}
        self._body = body

    async def json(self) -> dict:
        return self._body


class _MuxTracker:
    def snapshot(self) -> dict:
        return {
            "/follower_arm/teleop_mux/status": {
                "source": "vr",
            }
        }


class _ReadyBagRecorder:
    def __init__(self) -> None:
        self.started = False
        self.topics: list[str] = []

    def start(self, bag_dir: Path, topics: list[str]) -> None:
        self.started = True
        self.topics = list(topics)

    def detach_for_finalize(self) -> _DetachedBag:
        return _DetachedBag()


class _ReadyCameraPool:
    def __init__(self) -> None:
        self.started_cameras: list[dict] = []

    def start_all(self, episode_dir: Path, cameras: list[dict], fps: int = 30) -> list[dict]:
        self.started_cameras = list(cameras)
        return [
            {"name": camera["name"], "frame_count": 0, "segments": []}
            for camera in cameras
        ]

    def detach_all(self) -> _DetachedCameras:
        return _DetachedCameras()


class _DepthOnlyCameraPool(_ReadyCameraPool):
    def start_all(
        self, episode_dir: Path, cameras: list[dict], fps: int = 30
    ) -> list[dict]:
        self.started_cameras = cameras
        return [
            {
                "name": camera["name"],
                "topic": camera["topic"],
                "kind": "depth_bag",
                "frame_count": 0,
                "segments": [],
            }
            for camera in cameras
        ]

def _profile() -> dict:
    return {
        "episode_recorder": {
            "record_topics_override": [
                {
                    "topic": "/follower_arm/joint_states_single",
                    "role": "state",
                    "stamp_source": "message_header",
                },
                {
                    "topic": "/follower_arm/joint_ctrl",
                    "role": "command",
                    "stamp_source": "rosbag_recv",
                },
                {
                    "topic": "/follower_arm/teleop_mux/status",
                    "role": "mux",
                    "stamp_source": "rosbag_recv",
                },
            ],
            "cameras": [
                {
                    "name": "top_camera",
                    "topic": "/top_camera/image_raw/compressed",
                }
            ],
        },
        "lerobot": {
            "arm_streams": [
                {
                    "key": "follower",
                    "namespace": "follower_arm",
                    "joints": ["joint1"],
                    "rx": {
                        "joint_state": {"topic": "/follower_arm/joint_states_single"},
                        "joint_command": {
                            "topic": "/follower_arm/joint_cmd",
                            "vr": {"topic": "/follower_arm/joint_ctrl"},
                        },
                    },
                }
            ]
        },
    }


def _meta(episode_id: str) -> EpisodeMeta:
    return EpisodeMeta(
        episode_id=episode_id,
        state="recording",
        task_description="pick",
        profile="piper_single",
        started_at=utc_now_iso(),
    )


def _write_finished_episode(
    store: EpisodeStore,
    episode_id: str,
    tags: list[str],
    started_at: str,
) -> Path:
    ep_dir = store.output_dir / "episodes" / "piper_single" / "2026-07-08" / episode_id
    ep_dir.mkdir(parents=True)
    (ep_dir / "payload.bin").write_bytes(b"x")
    meta = EpisodeMeta(
        episode_id=episode_id,
        state="finished",
        task_description=episode_id,
        profile="piper_single",
        tags=tags,
        started_at=started_at,
        stopped_at=started_at,
        duration_s=1.0,
        outcome="success",
    )
    data = asdict(meta)
    (ep_dir / "meta.json").write_text(json.dumps(data))
    store.index.upsert(data, ep_dir)
    return ep_dir


def test_v1_migration_produces_current_episode_metadata() -> None:
    migrated = migrate_episode_document(
        {
            "schema_version": 1,
            "episode_id": "legacy-episode",
            "state": "finished",
            "profile": "piper_single",
            "started_at": "2026-07-09T00:00:00.000000Z",
            "cameras": [
                {
                    "name": "top",
                    "frame_count": 30,
                    "video_timing_mode": "ros_header_stamp_to_pts",
                    "video_pts_origin_ros_ns": 1,
                    "video_time_base_num": 1,
                    "video_time_base_den": 90_000,
                }
            ],
            "future_extension": {"preserve": True},
        }
    )
    meta = read_episode_meta(migrated)

    assert meta.schema_version == 2
    assert meta.episode_id == "legacy-episode"
    assert "timeline_start_ros_ns" not in migrated
    assert "timeline_end_ros_ns" not in migrated
    assert "stop_frame_count_per_camera" not in migrated
    assert migrated["cameras"] == [{"name": "top", "frame_count": 30}]
    assert migrated["future_extension"] == {"preserve": True}


def test_episode_store_migrates_metadata_before_opening_index(tmp_path: Path) -> None:
    meta_path = tmp_path / "episodes" / "piper_single" / "2026-07-09"
    meta_path = meta_path / "legacy" / "meta.json"
    meta_path.parent.mkdir(parents=True)
    meta_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "episode_id": "legacy",
                "state": "finished",
                "profile": "piper_single",
                "started_at": "2026-07-09T00:00:00.000000Z",
            }
        ),
        encoding="utf-8",
    )
    meta_path.chmod(0o664)

    store = EpisodeStore(tmp_path)
    migrated = json.loads(meta_path.read_text(encoding="utf-8"))

    assert migrated["schema_version"] == 2
    assert "timeline_start_ros_ns" not in migrated
    assert "timeline_end_ros_ns" not in migrated
    assert "stop_frame_count_per_camera" not in migrated
    assert stat.S_IMODE(meta_path.stat().st_mode) == 0o664
    store.index.close()


@pytest.mark.parametrize("schema_version", [3, 4])
def test_episode_store_rejects_schema_without_forward_migration(
    tmp_path: Path,
    schema_version: int,
) -> None:
    meta_path = tmp_path / "episodes" / "piper_single" / "2026-07-09"
    meta_path = meta_path / "future" / "meta.json"
    meta_path.parent.mkdir(parents=True)
    meta_path.write_text(
        json.dumps(
            {
                "schema_version": schema_version,
                "episode_id": "future",
                "state": "finished",
                "profile": "piper_single",
                "started_at": "2026-07-09T00:00:00.000000Z",
            }
        ),
        encoding="utf-8",
    )

    with pytest.raises(UnsupportedEpisodeSchemaVersionError, match="no forward migration"):
        EpisodeStore(tmp_path)


@pytest.mark.parametrize(
    ("data", "message"),
    [
        ({"episode_id": "missing-version"}, "must declare"),
        ({"schema_version": 1, "episode_id": "old"}, "not current"),
        ({"schema_version": 3, "episode_id": "unsupported"}, "not current"),
        (
            {
                "schema_version": 2,
                "episode_id": "obsolete",
                "state": "finished",
                "profile": "piper_single",
                "started_at": "2026-07-09T00:00:00.000000Z",
                "timeline_start_ros_ns": 1,
            },
            "removed fields",
        ),
    ],
)
def test_episode_schema_reader_rejects_unregistered_or_incomplete_metadata(
    data: dict[str, JsonValue],
    message: str,
) -> None:
    with pytest.raises(EpisodeMetadataError, match=message):
        read_episode_meta(data)


def test_episode_index_exposes_episode_schema_version(tmp_path: Path) -> None:
    store = EpisodeStore(tmp_path)
    _write_finished_episode(store, "schema-version", [], "2026-07-08T00:00:00.000000Z")

    rows, _cursor = store.index.list(limit=10)

    assert rows[0]["episode_schema_version"] == 2


def test_episode_index_rebuild_rejects_unsupported_schema(tmp_path: Path) -> None:
    store = EpisodeStore(tmp_path)
    meta_path = tmp_path / "episodes" / "piper_single" / "2026-07-08"
    meta_path = meta_path / "unsupported" / "meta.json"
    meta_path.parent.mkdir(parents=True)
    meta_path.write_text(
        json.dumps(
            {
                "schema_version": 3,
                "episode_id": "unsupported",
                "state": "finished",
                "profile": "piper_single",
                "started_at": "2026-07-08T00:00:00.000000Z",
            }
        )
    )

    with pytest.raises(UnsupportedEpisodeSchemaVersionError):
        store.index.rebuild_from_filesystem()


def test_retention_deletes_index_row_with_episode_dir(tmp_path: Path) -> None:
    store = EpisodeStore(tmp_path)
    ep_dir = _write_finished_episode(store, "local-ep", [], "2026-07-08T00:00:00.000000Z")

    result = RetentionRunner(store).tick(
        policy=RetentionPolicy(enabled=True, max_episodes=0, grace_period_s=0),
    )
    rows, _cursor = store.index.list(limit=10)

    assert result["deleted"][0]["episode_id"] == "local-ep"
    assert not ep_dir.exists()
    assert rows == []


def test_retention_put_enabled_false_saves_disabled_override(tmp_path: Path) -> None:
    async def run() -> None:
        store = EpisodeStore(tmp_path)
        runner = RetentionRunner(
            store,
            profile_policy_loader=lambda: RetentionPolicy(enabled=True, max_episodes=1),
        )

        response = await _retention_put_policy(_RetentionRequest(runner, {"enabled": False}))
        payload = json.loads(response.body.decode("utf-8"))

        assert response.status == 200
        assert payload["enabled"] is False
        assert runner.current_policy().enabled is False

    asyncio.run(run())


def test_depth_start_failure_cleans_resources_and_marks_episode_failed(
    tmp_path: Path,
) -> None:
    async def run() -> None:
        store = EpisodeStore(tmp_path)
        depth_pool = _FailingDepthPool()
        active_lock = _TrackingActiveLock()
        request = _DepthStartRequest(store, _ReadyBagRecorder(), _ReadyCameraPool())
        request.app["depth_pool"] = depth_pool
        request.app["active_lock"] = active_lock

        response = await _start_episode(request)
        payload = json.loads(response.body.decode("utf-8"))

        assert response.status == 500
        assert payload["error"] == "depth_republisher_failed"
        assert store.active is None
        assert depth_pool.stop_calls == 1
        assert active_lock.acquired
        assert active_lock.released
        episode = store.get_episode("01TESTASYNCSTOP0000000000")
        assert episode is not None
        assert episode[0].state == "failed"
        assert episode[0].outcome == "abort"
        assert episode[0].stale_input_events[-1]["code"] == "depth_republisher_failed"

    asyncio.run(run())


def test_active_lock_failure_uses_start_cleanup_path(tmp_path: Path) -> None:
    async def run() -> None:
        store = EpisodeStore(tmp_path)
        active_lock = _FailingActiveLock()
        request = _StartRequest(store, _ReadyBagRecorder(), _ReadyCameraPool())
        request.app["active_lock"] = active_lock

        response = await _start_episode(request)
        payload = json.loads(response.body.decode("utf-8"))

        assert response.status == 500
        assert payload["error"] == "active_lock_failed"
        assert store.active is None
        assert active_lock.acquired
        assert active_lock.released
        episode = store.get_episode("01TESTASYNCSTOP0000000000")
        assert episode is not None
        assert episode[0].state == "failed"
        assert episode[0].outcome == "abort"

    asyncio.run(run())


def test_startup_recovery_fails_finalizing_orphan_without_deleting_files(
    tmp_path: Path,
) -> None:
    first_store = EpisodeStore(tmp_path)
    episode_dir = first_store.start_episode(_meta("ep-finalizing"))
    payload_path = episode_dir / "videos" / "cam" / "0000.mp4"
    payload_path.parent.mkdir()
    payload_path.write_bytes(b"unfinished")
    first_store.begin_finalizing_active("success")

    restarted_store = EpisodeStore(tmp_path)
    recovered = restarted_store.recover_finalizing_orphans()

    assert recovered == ["ep-finalizing"]
    episode = restarted_store.get_episode("ep-finalizing")
    assert episode is not None
    assert episode[0].state == "failed"
    assert episode[0].outcome == "abort"
    assert episode[0].stale_input_events[-1]["code"] == "finalizer_orphaned"
    assert payload_path.read_bytes() == b"unfinished"
    assert restarted_store.recover_finalizing_orphans() == []


def test_startup_orphan_failure_updates_meta_and_index_immediately(
    tmp_path: Path,
) -> None:
    first_store = EpisodeStore(tmp_path)
    episode_dir = first_store.start_episode(_meta("ep-recording-orphan"))
    payload_path = episode_dir / "videos" / "cam" / "0000.mp4"
    payload_path.parent.mkdir()
    payload_path.write_bytes(b"unfinished")

    restarted_store = EpisodeStore(tmp_path)
    meta = restarted_store.mark_orphan_failed("ep-recording-orphan")
    indexed_rows, _cursor = restarted_store.index.list(limit=10)
    persisted = json.loads((episode_dir / "meta.json").read_text(encoding="utf-8"))

    assert meta.state == "failed"
    assert meta.outcome == "abort"
    assert persisted["state"] == "failed"
    assert persisted["outcome"] == "abort"
    assert indexed_rows[0]["state"] == "failed"
    assert indexed_rows[0]["outcome"] == "abort"
    assert payload_path.read_bytes() == b"unfinished"


def test_recover_discard_removes_episode_files_and_index_row(tmp_path: Path) -> None:
    async def run() -> None:
        first_store = EpisodeStore(tmp_path)
        episode_dir = first_store.start_episode(_meta("ep-discard-orphan"))
        first_store.begin_finalizing_active("abort")
        restarted_store = EpisodeStore(tmp_path)

        response = await _recover_episode(
            _RecoverRequest(restarted_store, "ep-discard-orphan", "discard")
        )

        assert response.status == 200
        assert not episode_dir.exists()
        assert restarted_store.get_episode("ep-discard-orphan") is None
        assert restarted_store.index.total_count() == 0

    asyncio.run(run())


def test_episode_store_rejects_existing_episode_id(tmp_path: Path) -> None:
    store = EpisodeStore(tmp_path)
    store.start_episode(_meta("duplicate-id"))
    store.begin_finalizing_active("abort")

    with pytest.raises(RuntimeError, match="episode_id already exists"):
        store.start_episode(_meta("duplicate-id"))


def test_episode_store_get_does_not_hide_index_miss_with_filesystem_scan(
    tmp_path: Path,
) -> None:
    store = EpisodeStore(tmp_path)
    data = asdict(_meta("duplicate-id"))
    episode_dir = tmp_path / "episodes" / "profile" / "2026" / "unindexed"
    episode_dir.mkdir(parents=True)
    (episode_dir / "meta.json").write_text(json.dumps(data), encoding="utf-8")

    assert store.get_episode("duplicate-id") is None
    store.index.ensure_consistent()
    assert store.get_episode("duplicate-id") is not None


def test_episode_index_rejects_same_id_for_different_directories(
    tmp_path: Path,
) -> None:
    store = EpisodeStore(tmp_path)
    data = asdict(_meta("duplicate-id"))
    first_dir = tmp_path / "episodes" / "profile" / "2026" / "first"
    second_dir = tmp_path / "episodes" / "profile" / "2026" / "second"
    first_dir.mkdir(parents=True)
    second_dir.mkdir(parents=True)

    store.index.upsert(data, first_dir)
    with pytest.raises(RuntimeError, match="maps to multiple directories"):
        store.index.upsert(data, second_dir)


def test_episode_index_rebuild_rejects_duplicate_ids(tmp_path: Path) -> None:
    store = EpisodeStore(tmp_path)
    data = asdict(_meta("duplicate-id"))
    for folder in ("first", "second"):
        ep_dir = tmp_path / "episodes" / "profile" / "2026" / folder
        ep_dir.mkdir(parents=True)
        (ep_dir / "meta.json").write_text(json.dumps(data), encoding="utf-8")

    with pytest.raises(RuntimeError, match="maps to multiple directories"):
        store.index.rebuild_from_filesystem()


def test_index_startup_rebuild_repairs_same_count_state_drift(tmp_path: Path) -> None:
    store = EpisodeStore(tmp_path)
    ep_dir = _write_finished_episode(
        store,
        "state-drift",
        [],
        "2026-07-08T00:00:00.000000Z",
    )
    meta_path = ep_dir / "meta.json"
    data = json.loads(meta_path.read_text(encoding="utf-8"))
    data["state"] = "failed"
    data["outcome"] = "abort"
    meta_path.write_text(json.dumps(data), encoding="utf-8")

    before, _cursor = store.index.list(limit=10)
    assert before[0]["state"] == "finished"
    store.index.ensure_consistent()
    after, _cursor = store.index.list(limit=10)
    assert after[0]["state"] == "failed"
    assert after[0]["outcome"] == "abort"


def test_tag_patch_propagates_index_failure(tmp_path: Path, monkeypatch) -> None:
    store = EpisodeStore(tmp_path)
    _write_finished_episode(
        store,
        "tag-patch",
        [],
        "2026-07-08T00:00:00.000000Z",
    )

    def fail_upsert(_data: EpisodeMeta, _ep_dir: Path) -> None:
        raise RuntimeError("index unavailable")

    monkeypatch.setattr(store.index, "upsert", fail_upsert)
    with pytest.raises(RuntimeError, match="index unavailable"):
        store.patch_episode_meta("tag-patch", {"tags": ["dpex:record"]})


def test_add_episode_tags_serializes_and_preserves_concurrent_additions(
    tmp_path: Path,
    monkeypatch,
) -> None:
    store = EpisodeStore(tmp_path)
    _write_finished_episode(
        store,
        "tag-add",
        ["vlabor:auto"],
        "2026-07-08T00:00:00.000000Z",
    )
    original_get_episode = store.get_episode
    counter_lock = threading.Lock()
    active_reads = 0
    max_active_reads = 0

    def slow_get_episode(episode_id: str):
        nonlocal active_reads, max_active_reads
        with counter_lock:
            active_reads += 1
            max_active_reads = max(max_active_reads, active_reads)
        try:
            time.sleep(0.03)
            return original_get_episode(episode_id)
        finally:
            with counter_lock:
                active_reads -= 1

    monkeypatch.setattr(store, "get_episode", slow_get_episode)
    with ThreadPoolExecutor(max_workers=2) as executor:
        futures = [
            executor.submit(store.add_episode_tags, "tag-add", [tag, tag])
            for tag in ("dpex:record", "user:赤")
        ]
        for future in futures:
            assert future.result() is not None

    episode = original_get_episode("tag-add")
    assert episode is not None
    assert max_active_reads == 1
    assert episode[0].tags[0] == "vlabor:auto"
    assert set(episode[0].tags) == {"vlabor:auto", "dpex:record", "user:赤"}


def test_add_episode_tags_endpoint_requires_tags_and_returns_merged_tags(
    tmp_path: Path,
) -> None:
    async def run() -> None:
        store = EpisodeStore(tmp_path)
        _write_finished_episode(
            store,
            "tag-api",
            ["vlabor:auto"],
            "2026-07-08T00:00:00.000000Z",
        )

        response = await _add_episode_tags(
            _TagsRequest(store, "tag-api", ["dpex:record", "dpex:record", "user:青"])
        )
        payload = json.loads(response.body.decode("utf-8"))
        invalid = await _add_episode_tags(_TagsRequest(store, "tag-api", []))

        assert response.status == 200
        assert payload == {
            "episode_id": "tag-api",
            "tags": ["vlabor:auto", "dpex:record", "user:青"],
        }
        assert invalid.status == 400

    asyncio.run(run())


@pytest.mark.parametrize("metadata_failure", ["invalid_json", "missing_file"])
def test_add_episode_tags_endpoint_reports_indexed_metadata_failure_as_server_error(
    tmp_path: Path,
    metadata_failure: str,
) -> None:
    async def run() -> None:
        store = EpisodeStore(tmp_path)
        episode_dir = _write_finished_episode(
            store,
            "tag-api-corrupt",
            ["vlabor:auto"],
            "2026-07-08T00:00:00.000000Z",
        )
        meta_path = episode_dir / "meta.json"
        if metadata_failure == "invalid_json":
            meta_path.write_text("{", encoding="utf-8")
        else:
            meta_path.unlink()

        response = await _add_episode_tags(
            _TagsRequest(store, "tag-api-corrupt", ["dpex:record"])
        )
        payload = json.loads(response.body.decode("utf-8"))

        assert response.status == 500
        assert payload["error"] == "episode_metadata_unavailable"

    asyncio.run(run())


def test_add_episode_tags_endpoint_returns_not_found_only_for_index_miss(
    tmp_path: Path,
) -> None:
    async def run() -> None:
        store = EpisodeStore(tmp_path)

        response = await _add_episode_tags(
            _TagsRequest(store, "missing-episode", ["dpex:record"])
        )

        assert response.status == 404

    asyncio.run(run())


def test_detached_bag_finalize_timeout_terminates_and_fails(tmp_path: Path) -> None:
    process = _ProcessThatTimesOut()
    recording = DetachedBagRecording(
        proc=process,
        bag_dir=tmp_path / "bag",
        topics=["/joint_states"],
    )

    with pytest.raises(RuntimeError, match="finalize timed out after 10.0s"):
        recording.wait(timeout_s=BAG_FINALIZE_TIMEOUT_S)

    assert process.wait_timeouts == [BAG_FINALIZE_TIMEOUT_S, 3.0]
    assert process.terminated
    assert not process.killed


def test_detached_bag_finalize_timeout_kills_after_terminate_timeout(
    tmp_path: Path,
) -> None:
    process = _ProcessThatIgnoresTerminate()
    recording = DetachedBagRecording(
        proc=process,
        bag_dir=tmp_path / "bag",
        topics=["/joint_states"],
    )

    with pytest.raises(RuntimeError, match="finalize timed out after 10.0s"):
        recording.wait(timeout_s=BAG_FINALIZE_TIMEOUT_S)

    assert process.wait_timeouts == [BAG_FINALIZE_TIMEOUT_S, 3.0, 3.0]
    assert process.terminated
    assert process.killed


def test_shutdown_drains_bounded_bag_finalizer_to_terminal_failure(
    tmp_path: Path,
) -> None:
    async def run() -> None:
        store = EpisodeStore(tmp_path)
        store.start_episode(_meta("bag-timeout"))
        meta, ep_dir = store.begin_finalizing_active("success")
        detached_bag = _TimedOutDetachedBag()
        task = asyncio.create_task(
            api_server._finalize_stopped_episode(
                store=store,
                meta=meta,
                ep_dir=ep_dir,
                outcome="success",
                detached_bag=detached_bag,
                detached_cameras=_DetachedCameras(),
                depth_frame_counts={},
                pending_markers=[],
                controller_at_end=None,
                initial_failures=[],
            )
        )
        app = {"finalizer_tasks": {task}}

        await asyncio.wait_for(api_server._await_finalizer_tasks(app), timeout=1.0)

        assert detached_bag.timeout_s == BAG_FINALIZE_TIMEOUT_S
        episode = store.get_episode("bag-timeout")
        assert episode is not None
        assert episode[0].state == "failed"
        assert episode[0].outcome == "abort"
        assert episode[0].stale_input_events == [
            {
                "code": "bag_write_error",
                "detail": "bag recorder finalize timed out after 10.0s",
            }
        ]

    asyncio.run(run())


def test_stop_detaches_active_episode_before_finalize(tmp_path: Path) -> None:
    async def run() -> None:
        store = EpisodeStore(tmp_path)
        store.start_episode(_meta("ep-1"))
        video_path = store.active_dir / "videos" / "cam" / "0000.mp4"
        video_path.parent.mkdir()
        video_path.write_bytes(b"video")

        response = await _stop_episode(_Request(store, "ep-1"))
        payload = json.loads(response.body.decode("utf-8"))

        assert response.status == 202
        assert payload["state"] == "finalizing"
        assert payload["duration_s"] >= 0.0
        assert store.active is None

        store.patch_episode_meta("ep-1", {"tags": ["dpex:record", "user:red"]})
        store.start_episode(_meta("ep-2"))
        await asyncio.sleep(0.2)

        first = store.get_episode("ep-1")
        assert first is not None
        assert first[0].state == "finished"
        assert first[0].tags == ["dpex:record", "user:red"]
        write_bits = stat.S_IWUSR | stat.S_IWGRP | stat.S_IWOTH
        assert video_path.stat().st_mode & write_bits == 0
        assert store.active is not None
        assert store.active.episode_id == "ep-2"

    asyncio.run(run())


def test_stop_quiesces_depth_before_detaching_bag(tmp_path: Path) -> None:
    async def run() -> None:
        events: list[str] = []

        class _OrderedDepthPool(_DepthPool):
            def stop_all(self) -> dict[str, int]:
                events.append("depth")
                return {}

        class _OrderedBagRecorder(_BagRecorder):
            def detach_for_finalize(self) -> _DetachedBag:
                events.append("bag")
                return super().detach_for_finalize()

        store = EpisodeStore(tmp_path)
        store.start_episode(_meta("ep-1"))
        request = _Request(store, "ep-1")
        request.app["depth_pool"] = _OrderedDepthPool()
        request.app["bag_recorder"] = _OrderedBagRecorder()

        response = await _stop_episode(request)

        assert response.status == 202
        assert events == ["depth", "bag"]
        await asyncio.sleep(0.1)

    asyncio.run(run())


def test_finalizer_crash_reaches_failed_terminal_state(
    tmp_path: Path, monkeypatch
) -> None:
    async def run() -> None:
        def crash(*_args) -> None:
            raise RuntimeError("finalizer crashed")

        monkeypatch.setattr(api_server, "_finalize_stopped_episode_sync", crash)
        store = EpisodeStore(tmp_path)
        store.start_episode(_meta("ep-1"))

        response = await _stop_episode(_Request(store, "ep-1"))

        assert response.status == 202
        await asyncio.sleep(0.1)
        episode = store.get_episode("ep-1")
        assert episode is not None
        assert episode[0].state == "failed"
        assert episode[0].outcome == "abort"
        assert episode[0].stale_input_events == [
            {"code": "finalizer_error", "detail": "finalizer crashed"}
        ]

    asyncio.run(run())


def test_zero_frame_color_camera_finalizes_failed(tmp_path: Path) -> None:
    async def run() -> None:
        store = EpisodeStore(tmp_path)
        store.start_episode(_meta("ep-1"))
        request = _Request(store, "ep-1")
        request.app["camera_pool"] = _ZeroFrameCameraPool()

        response = await _stop_episode(request)

        assert response.status == 202
        await asyncio.sleep(0.1)
        episode = store.get_episode("ep-1")
        assert episode is not None
        assert episode[0].state == "failed"
        assert episode[0].outcome == "abort"
        assert episode[0].stale_input_events == [
            {
                "code": "camera_write_error",
                "detail": "camera top_camera: no recorded frames",
            }
        ]

    asyncio.run(run())


def test_camera_writer_records_first_valid_frame(tmp_path: Path, monkeypatch) -> None:
    class _Stdin:
        def __init__(self) -> None:
            self.writes = 0

        def write(self, _data: bytes) -> None:
            self.writes += 1

    class _Proc:
        def __init__(self) -> None:
            self.stdin = _Stdin()

    class _Stamp:
        sec = 1
        nanosec = 2

    class _Header:
        stamp = _Stamp()

    class _Msg:
        header = _Header()
        data = b"jpeg"

    proc = _Proc()
    monkeypatch.setattr(
        camera_writer.cv2,
        "imdecode",
        lambda _arr, _flag: camera_writer.np.zeros((2, 2, 3), dtype=camera_writer.np.uint8),
    )
    monkeypatch.setattr(camera_writer, "_spawn_ffmpeg_encoder", lambda *_args: (proc, "test"))

    writer = CameraWriter("cam", "/camera", tmp_path, node=None)
    writer._encode_frame(_Msg.data, 1_000_000_002, time.time_ns())

    assert writer.frame_count == 1
    assert proc.stdin.writes == 1
    writer._sidecar.close()
    sidecar_row = pq.read_table(tmp_path / "frames.parquet").to_pylist()[0]
    assert sidecar_row["video_pts"] == 0


def test_camera_subscription_callback_only_enqueues_frame(tmp_path: Path) -> None:
    class _Node:
        def create_subscription(self, _msg_type, _topic, callback, _qos):
            self.callback = callback
            return object()

        def destroy_subscription(self, _subscription) -> None:
            pass

    class _Stamp:
        sec = 1
        nanosec = 2

    class _Header:
        stamp = _Stamp()

    class _Msg:
        header = _Header()
        data = b"jpeg"

    entered = threading.Event()
    release = threading.Event()
    writer = CameraWriter(
        "cam",
        "/camera",
        tmp_path,
        node=_Node(),
        encoder=("test", ()),
    )

    def _slow_encode(*_args) -> None:
        entered.set()
        release.wait(timeout=2)

    writer._encode_frame = _slow_encode
    writer.start()
    writer._on_image(_Msg())
    assert entered.wait(timeout=1)

    started_at = time.perf_counter()
    writer._on_image(_Msg())
    callback_elapsed = time.perf_counter() - started_at

    release.set()
    writer.stop_recording()
    assert writer._worker is not None
    writer._worker.join(timeout=1)
    writer._sidecar.close()
    assert callback_elapsed < 0.05


def test_depth_subscription_callback_only_enqueues_frame() -> None:
    class _Publisher:
        def publish(self, _message) -> None:
            pass

    class _Node:
        def create_publisher(self, _msg_type, _topic, _qos):
            return _Publisher()

        def create_subscription(self, _msg_type, _topic, callback, _qos):
            self.callback = callback
            return object()

        def destroy_subscription(self, _subscription) -> None:
            pass

        def destroy_publisher(self, _publisher) -> None:
            pass

    entered = threading.Event()
    release = threading.Event()
    republisher = _DepthRepublisher("depth", "/depth", _Node())

    def _slow_publish(_message) -> None:
        entered.set()
        release.wait(timeout=2)

    republisher._encode_and_publish = _slow_publish
    republisher.start()
    republisher._on_image(_Image())
    assert entered.wait(timeout=1)

    started_at = time.perf_counter()
    republisher._on_image(_Image())
    callback_elapsed = time.perf_counter() - started_at

    release.set()
    republisher.stop()
    assert callback_elapsed < 0.05


def test_depth_pool_counts_frames_after_worker_drain() -> None:
    class _Republisher:
        name = "depth"
        raw_topic = "/depth"
        _frame_count = 1

        def stop(self) -> None:
            self._frame_count = 3

    pool = DepthRepublisherPool(node=_Node())
    pool._pubs = [_Republisher()]

    counts = pool.stop_all()

    assert counts == {"depth": 3}


def test_camera_writer_skips_frame_without_ros_stamp(tmp_path: Path, monkeypatch) -> None:
    class _Stamp:
        sec = 0
        nanosec = 0

    class _Header:
        stamp = _Stamp()

    class _Msg:
        header = _Header()
        data = b"jpeg"

    spawn_calls = 0

    def _spawn(*_args):
        nonlocal spawn_calls
        spawn_calls += 1
        raise AssertionError("an unstamped frame must not reach the encoder")

    monkeypatch.setattr(camera_writer, "_spawn_ffmpeg_encoder", _spawn)
    writer = CameraWriter("cam", "/camera", tmp_path, node=None)

    writer._on_image(_Msg())

    assert writer.frame_count == 0
    assert spawn_calls == 0


def test_camera_writer_disables_b_frames(tmp_path: Path, monkeypatch) -> None:
    class _Probe:
        returncode = 0
        stderr = b""

    class _Process:
        pass

    commands: list[list[str]] = []
    process = _Process()
    monkeypatch.setattr(camera_writer.shutil, "which", lambda _name: "/usr/bin/ffmpeg")
    monkeypatch.setattr(camera_writer.subprocess, "run", lambda *_args, **_kwargs: _Probe())

    def _popen(command: list[str], **_kwargs):
        commands.append(command)
        return process

    monkeypatch.setattr(camera_writer.subprocess, "Popen", _popen)

    spawned = camera_writer._spawn_ffmpeg_encoder(tmp_path / "0000.mp4", 16, 16, 30)

    assert spawned == (process, "h264_nvenc")
    assert commands[0][commands[0].index("-bf") : commands[0].index("-bf") + 2] == ["-bf", "0"]


def test_camera_writer_nvenc_probe_uses_real_camera_dimensions(monkeypatch) -> None:
    class _Probe:
        returncode = 0
        stderr = b""

    commands: list[list[str]] = []
    monkeypatch.setattr(camera_writer.shutil, "which", lambda _name: "/usr/bin/ffmpeg")

    def _run(command: list[str], **_kwargs) -> _Probe:
        commands.append(command)
        return _Probe()

    monkeypatch.setattr(camera_writer.subprocess, "run", _run)

    selected = camera_writer._select_h264_encoder()

    assert selected == ("h264_nvenc", ("-preset", "p1", "-tune", "ll"))
    assert "color=c=black:s=640x480:d=0.04:r=25" in commands[0]


def test_camera_writer_rejects_unreadable_recorded_video(tmp_path: Path) -> None:
    class _Sidecar:
        def close(self) -> int:
            return 1

    writer = CameraWriter("cam", "/camera", tmp_path, node=None)
    writer._sidecar = _Sidecar()
    writer.frame_count = 1
    writer._start_wall_ns = time.time_ns()
    writer._last_recv_ns = writer._start_wall_ns
    writer._segment_file.write_bytes(b"not an mp4")

    try:
        writer.finalize()
    except RuntimeError as exc:
        assert "video" in str(exc)
    else:
        raise AssertionError("unreadable recorded video must fail finalization")


def test_camera_writer_remuxes_video_pts_from_ros_stamps(tmp_path: Path) -> None:
    video_path = tmp_path / "0000.mp4"
    container = camera_writer.av.open(str(video_path), mode="w")
    stream = container.add_stream("mpeg4", rate=30)
    stream.width = 16
    stream.height = 16
    stream.pix_fmt = "yuv420p"
    stream.time_base = Fraction(1, 30)
    stream.codec_context.time_base = Fraction(1, 30)
    try:
        for index in range(4):
            frame = camera_writer.av.VideoFrame.from_ndarray(
                camera_writer.np.zeros((16, 16, 3), dtype=camera_writer.np.uint8),
                format="bgr24",
            )
            frame.pts = index
            frame.time_base = Fraction(1, 30)
            for packet in stream.encode(frame):
                container.mux(packet)
        for packet in stream.encode():
            container.mux(packet)
    finally:
        container.close()

    frame_stamps_ns = [
        1_000_000_000,
        1_033_000_000,
        1_200_000_000,
        1_233_000_000,
    ]
    expected_pts = [
        camera_writer._timestamp_ticks(stamp_ns, frame_stamps_ns[0])
        for stamp_ns in frame_stamps_ns
    ]
    camera_writer._remux_mp4_with_ros_timestamps(
        video_path,
        frame_stamps_ns,
        expected_pts,
    )

    remuxed = camera_writer.av.open(str(video_path), mode="r")
    try:
        stream = remuxed.streams.video[0]
        pts_s = [
            round(float(frame.pts * stream.time_base), 3)
            for frame in remuxed.decode(stream)
        ]
    finally:
        remuxed.close()
    remuxed = camera_writer.av.open(str(video_path), mode="r")
    try:
        stream = remuxed.streams.video[0]
        assert stream.time_base == camera_writer.VIDEO_TIME_BASE
        packets = [packet for packet in remuxed.demux(stream) if packet.dts is not None]
        assert [packet.pts for packet in packets] == expected_pts
        assert all(
            packet.time_base == camera_writer.VIDEO_TIME_BASE for packet in packets
        )
        packet_durations_s = [
            round(float(packet.duration * packet.time_base), 3) for packet in packets
        ]
    finally:
        remuxed.close()
    assert pts_s == [0.0, 0.033, 0.2, 0.233]
    assert packet_durations_s == [0.033, 0.167, 0.033, 0.033]


def test_camera_writer_remuxes_single_frame_duration(tmp_path: Path) -> None:
    video_path = tmp_path / "0000.mp4"
    container = camera_writer.av.open(str(video_path), mode="w")
    stream = container.add_stream("mpeg4", rate=30)
    stream.width = 16
    stream.height = 16
    stream.pix_fmt = "yuv420p"
    stream.time_base = Fraction(1, 30)
    stream.codec_context.time_base = Fraction(1, 30)
    try:
        frame = camera_writer.av.VideoFrame.from_ndarray(
            camera_writer.np.zeros((16, 16, 3), dtype=camera_writer.np.uint8),
            format="bgr24",
        )
        frame.pts = 0
        frame.time_base = Fraction(1, 30)
        for packet in stream.encode(frame):
            container.mux(packet)
        for packet in stream.encode():
            container.mux(packet)
    finally:
        container.close()

    camera_writer._remux_mp4_with_ros_timestamps(video_path, [1_000_000_000])

    remuxed = camera_writer.av.open(str(video_path), mode="r")
    try:
        stream = remuxed.streams.video[0]
        packets = [packet for packet in remuxed.demux(stream) if packet.dts is not None]
    finally:
        remuxed.close()
    assert len(packets) == 1
    assert float(packets[0].pts * packets[0].time_base) == 0.0
    assert round(float(packets[0].duration * packets[0].time_base), 3) == 0.033


def test_camera_writer_summary_declares_ros_header_stamp_pts_origin(tmp_path: Path) -> None:
    writer = CameraWriter("cam", "/camera", tmp_path, node=None)
    writer._frame_stamps_ns = [1_234_567_890, 1_267_567_890]
    writer.frame_count = 2
    writer._segment_file.write_bytes(b"video")

    summary = writer.summary()

    assert "video_timing_mode" not in summary
    assert "video_pts_origin_ros_ns" not in summary
    assert "video_time_base_num" not in summary
    assert "video_time_base_den" not in summary


def test_start_launches_bag_writer_and_cameras(
    tmp_path: Path,
    monkeypatch,
) -> None:
    async def run() -> None:
        monkeypatch.setattr(api_server.BagRecorder, "available", staticmethod(lambda: True))
        store = EpisodeStore(tmp_path)
        bag_recorder = _ReadyBagRecorder()
        camera_pool = _ReadyCameraPool()

        response = await _start_episode(_StartRequest(store, bag_recorder, camera_pool))

        assert response.status == 201
        assert bag_recorder.started
        assert {
            "/follower_arm/joint_states_single",
            "/follower_arm/joint_ctrl",
            "/follower_arm/teleop_mux/status",
        }.issubset(bag_recorder.topics)
        assert [camera["name"] for camera in camera_pool.started_cameras] == [
            "top_camera"
        ]
        assert store.active is not None

    asyncio.run(run())


def test_start_adds_profile_depth_topic_to_bag(
    tmp_path: Path,
    monkeypatch,
) -> None:
    async def run() -> None:
        monkeypatch.setattr(
            api_server.BagRecorder, "available", staticmethod(lambda: True)
        )
        store = EpisodeStore(tmp_path)
        bag_recorder = _ReadyBagRecorder()
        depth_pool = _ReadyDepthPool()
        request = _DepthProfileStartRequest(
            store,
            bag_recorder,
            _ReadyCameraPool(),
        )
        request.app["depth_pool"] = depth_pool

        response = await _start_episode(request)
        payload = json.loads(response.body.decode("utf-8"))

        depth_topic = "/camera/depth_compressed/compressedDepth"
        assert response.status == 201
        assert bag_recorder.topics.count(depth_topic) == 1
        assert depth_pool.started_cameras == [
            {
                "name": "depth",
                "topic": "/camera/depth",
                "codec": "mp4v",
                "kind": "depth",
            }
        ]
        assert [
            topic
            for topic in payload["recorded_topics_resolved"]
            if topic.get("topic") == depth_topic
        ] == [
            {
                "topic": depth_topic,
                "role": "camera_depth_compressed",
                "qos": "best_effort",
                "stamp_source": "message_header",
                "msg_type": "sensor_msgs/msg/CompressedImage",
            }
        ]

    asyncio.run(run())


def test_disabled_depth_camera_is_not_started_or_required(
    tmp_path: Path,
    monkeypatch,
) -> None:
    async def run() -> None:
        monkeypatch.setattr(
            api_server.BagRecorder, "available", staticmethod(lambda: True)
        )
        store = EpisodeStore(tmp_path)
        bag_recorder = _ReadyBagRecorder()
        camera_pool = _DepthOnlyCameraPool()
        depth_pool = _ReadyDepthPool()
        request = _DisabledDepthStartRequest(store, bag_recorder, camera_pool)
        request.app["depth_pool"] = depth_pool

        response = await _start_episode(request)

        depth_topic = "/camera/depth_compressed/compressedDepth"
        assert response.status == 201
        assert depth_topic not in bag_recorder.topics
        assert camera_pool.started_cameras == []
        assert depth_pool.started_cameras == []

    asyncio.run(run())


def test_start_rejects_missing_profile_without_overrides(tmp_path: Path) -> None:
    async def run() -> None:
        store = EpisodeStore(tmp_path)

        response = await _start_episode(
            _MissingProfileStartRequest(store, _ReadyBagRecorder(), _ReadyCameraPool())
        )
        payload = json.loads(response.body.decode("utf-8"))

        assert response.status == 404
        assert payload == {
            "error": "profile_not_found",
            "detail": "recording profile not found: piper_single",
        }
        assert store.active is None

    asyncio.run(run())


def test_start_accepts_explicitly_empty_recording_inputs(
    tmp_path: Path,
    monkeypatch,
) -> None:
    async def run() -> None:
        monkeypatch.setattr(
            api_server.BagRecorder, "available", staticmethod(lambda: True)
        )
        store = EpisodeStore(tmp_path)

        response = await _start_episode(
            _EmptyOverrideStartRequest(store, _ReadyBagRecorder(), _ReadyCameraPool())
        )
        assert response.status == 201
        assert store.active is not None

    asyncio.run(run())
