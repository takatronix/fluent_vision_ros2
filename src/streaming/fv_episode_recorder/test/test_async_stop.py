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
    BagReadyStatus,
    BagRecorder,
    DetachedBagRecording,
)
from fv_episode_recorder.camera_writer import CameraWriter
from fv_episode_recorder.episode_store import EpisodeMeta, EpisodeStore, utc_now_iso
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
            raise subprocess.TimeoutExpired("fv_counting_bag_recorder", timeout)
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
            raise subprocess.TimeoutExpired("fv_counting_bag_recorder", timeout)
        self.returncode = -9
        return self.returncode


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


class _Fps60StartRequest(_StartRequest):
    async def json(self) -> dict:
        body = await super().json()
        body["fps"] = 60
        return body


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
    def __init__(self, coverage_stamp_ros_ns: int = 1_100_000_000) -> None:
        self.started = False
        self.polls = 0
        self.ready_topics: set[str] = set()
        self.topics: list[str] = []
        self.coverage_stamp_ros_ns = coverage_stamp_ros_ns

    def start(self, bag_dir: Path, topics: list[str], ready_topics: set[str] | None = None) -> None:
        self.started = True
        self.topics = list(topics)
        self.ready_topics = set(ready_topics or set())

    def ready_status(self, required_topics: set[str]) -> BagReadyStatus:
        self.polls += 1
        counts = {topic: 1 if self.polls >= 2 else 0 for topic in required_topics}
        timestamps = {
            topic: 1_000_000_000 if self.polls >= 2 else 0 for topic in required_topics
        }
        latest_timestamps = {
            topic: (
                self.coverage_stamp_ros_ns
                if self.polls >= 3
                else 1_000_000_000
                if self.polls >= 2
                else 0
            )
            for topic in required_topics
        }
        return BagReadyStatus(
            ready=all(count > 0 for count in counts.values()),
            ready_at_ros_ns=1_000_000_000 if self.polls >= 2 else None,
            counts=counts,
            first_bag_timestamp_ns=timestamps,
            latest_bag_timestamp_ns=latest_timestamps,
            timestamp_source="rosbag2_serialized_message_time_stamp",
        )

    def detach_for_finalize(self) -> _DetachedBag:
        return _DetachedBag()


class _ReadyCameraPool:
    def __init__(self) -> None:
        self.polls = 0
        self.frame_polls = 0
        self.enabled = False
        self.started_cameras: list[dict] = []

    def start_all(self, episode_dir: Path, cameras: list[dict], fps: int = 30) -> list[dict]:
        self.started_cameras = cameras
        self.enabled = bool(cameras[0].get("record_immediately"))
        return [{"name": "top_camera", "frame_count": 0, "segments": []}]

    def observed_frame_counts(self) -> dict[str, int]:
        self.polls += 1
        return {"top_camera": 1 if self.polls >= 2 else 0}

    def observed_frame_stamps_ns(self) -> dict[str, int]:
        return {"top_camera": 1_100_000_000 if self.polls >= 2 else 0}

    def ros_now_ns(self) -> int:
        return 1_200_000_000 if self.frame_polls == 0 else 1_300_000_000

    def enable_recording(self) -> None:
        self.enabled = True

    def frame_counts(self) -> dict[str, int]:
        self.frame_polls += 1
        return {"top_camera": self.frame_polls if self.enabled else 0}

    def recorded_frame_stamps_ns(self) -> dict[str, int]:
        return {
            "top_camera": (
                1_300_000_000
                if self.enabled and self.frame_polls >= 2
                else 1_100_000_000
                if self.enabled
                else 0
            )
        }

    def detach_all(self) -> _DetachedCameras:
        return _DetachedCameras()


class _NeverRecordedCameraPool(_ReadyCameraPool):
    def frame_counts(self) -> dict[str, int]:
        self.frame_polls += 1
        return {"top_camera": 0}


class _NeverCoveredBagRecorder(_ReadyBagRecorder):
    def ready_status(self, required_topics: set[str]) -> BagReadyStatus:
        status = super().ready_status(required_topics)
        return BagReadyStatus(
            ready=status.ready,
            ready_at_ros_ns=status.ready_at_ros_ns,
            counts=status.counts,
            first_bag_timestamp_ns=status.first_bag_timestamp_ns,
            latest_bag_timestamp_ns={topic: 1_000_000_000 for topic in required_topics},
            timestamp_source=status.timestamp_source,
        )


class _NeverReadyBagRecorder(_ReadyBagRecorder):
    def ready_status(self, required_topics: set[str]) -> BagReadyStatus:
        return BagReadyStatus(
            ready=False,
            ready_at_ros_ns=None,
            counts={topic: 0 for topic in required_topics},
            first_bag_timestamp_ns={topic: 0 for topic in required_topics},
            latest_bag_timestamp_ns={topic: 0 for topic in required_topics},
            timestamp_source="rosbag2_serialized_message_time_stamp",
        )


class _NeverObservedCameraPool(_ReadyCameraPool):
    def observed_frame_counts(self) -> dict[str, int]:
        return {"top_camera": 0}

    def observed_frame_stamps_ns(self) -> dict[str, int]:
        return {"top_camera": 0}


class _StaleDepthBagRecorder(_ReadyBagRecorder):
    def ready_status(self, required_topics: set[str]) -> BagReadyStatus:
        status = super().ready_status(required_topics)
        latest_timestamps = dict(status.latest_bag_timestamp_ns)
        depth_topic = "/camera/depth_compressed/compressedDepth"
        if depth_topic in latest_timestamps and status.ready:
            latest_timestamps[depth_topic] = 1_000_000_000
        return BagReadyStatus(
            ready=status.ready,
            ready_at_ros_ns=status.ready_at_ros_ns,
            counts=status.counts,
            first_bag_timestamp_ns=status.first_bag_timestamp_ns,
            latest_bag_timestamp_ns=latest_timestamps,
            timestamp_source=status.timestamp_source,
        )


class _EmptyCameraPool(_ReadyCameraPool):
    def observed_frame_counts(self) -> dict[str, int]:
        return {}

    def observed_frame_stamps_ns(self) -> dict[str, int]:
        return {}

    def recorded_frame_stamps_ns(self) -> dict[str, int]:
        return {"top_camera": 0}


class _DepthOnlyCameraPool(_ReadyCameraPool):
    def start_all(
        self, episode_dir: Path, cameras: list[dict], fps: int = 30
    ) -> list[dict]:
        self.started_cameras = cameras
        self.enabled = True
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

    def observed_frame_counts(self) -> dict[str, int]:
        return {}

    def observed_frame_stamps_ns(self) -> dict[str, int]:
        return {}

    def frame_counts(self) -> dict[str, int]:
        self.frame_polls += 1
        return {}

    def recorded_frame_stamps_ns(self) -> dict[str, int]:
        return {}


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
        timeline_start_ros_ns=1_000_000_000,
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
    first_store.begin_finalizing_active("success", 2_000_000_000)

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
        first_store.begin_finalizing_active("abort", 2_000_000_000)
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
    store.begin_finalizing_active("abort", 2_000_000_000)

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
        meta, ep_dir = store.begin_finalizing_active("success", 2_000_000_000)
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
        assert payload["timeline_end_ros_ns"] == 2_000_000_000
        assert payload["duration_s"] == 1.0
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


def test_camera_writer_warms_encoder_before_recording(tmp_path: Path, monkeypatch) -> None:
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

    writer = CameraWriter("cam", "/camera", tmp_path, node=None, record_immediately=False)
    writer._on_image(_Msg())

    assert writer.has_observed_frame()
    assert writer.frame_count == 0
    assert proc.stdin.writes == 0

    writer.enable_recording()
    writer._on_image(_Msg())

    assert writer.frame_count == 1
    assert proc.stdin.writes == 1
    writer._sidecar.close()
    sidecar_row = pq.read_table(tmp_path / "frames.parquet").to_pylist()[0]
    assert sidecar_row["video_pts"] == 0


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

    assert not writer.has_observed_frame()
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


def test_camera_writer_rejects_unreadable_recorded_video(tmp_path: Path) -> None:
    class _Sidecar:
        def close(self) -> int:
            return 1

    writer = CameraWriter("cam", "/camera", tmp_path, node=None, record_immediately=False)
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

    assert summary["video_timing_mode"] == "ros_header_stamp_to_pts"
    assert summary["video_pts_origin_ros_ns"] == 1_234_567_890
    assert summary["video_time_base_num"] == 1
    assert summary["video_time_base_den"] == 90_000


def test_start_waits_for_bag_writer_and_camera_before_enabling_camera(
    tmp_path: Path,
    monkeypatch,
) -> None:
    async def run() -> None:
        monkeypatch.setattr(api_server.BagRecorder, "available", staticmethod(lambda: True))
        store = EpisodeStore(tmp_path)
        bag_recorder = _ReadyBagRecorder()
        camera_pool = _ReadyCameraPool()

        response = await _start_episode(_StartRequest(store, bag_recorder, camera_pool))
        payload = json.loads(response.body.decode("utf-8"))

        assert response.status == 201
        assert bag_recorder.started
        assert bag_recorder.polls >= 3
        assert bag_recorder.ready_topics == {
            "/follower_arm/joint_states_single",
            "/follower_arm/joint_ctrl",
            "/follower_arm/teleop_mux/status",
        }
        assert camera_pool.polls >= 2
        assert camera_pool.enabled
        assert camera_pool.frame_polls >= 2
        assert camera_pool.started_cameras[0]["record_immediately"] is True
        assert payload["preflight"]["recording_ready"]["bag_topics"] == {
            "/follower_arm/joint_states_single": 1,
            "/follower_arm/joint_ctrl": 1,
            "/follower_arm/teleop_mux/status": 1,
        }
        coverage = payload["preflight"]["recording_coverage"]
        assert coverage["latest_bag_timestamp_ros_ns"] == {
            "/follower_arm/joint_states_single": 1_100_000_000,
            "/follower_arm/joint_ctrl": 1_100_000_000,
            "/follower_arm/teleop_mux/status": 1_100_000_000,
        }
        assert coverage["bag_alignment_tolerance_ns"] == 100_000_000
        assert coverage["frame_counts"] == {"top_camera": 2}
        assert payload["timeline_start_ros_ns"] == 1_200_000_000
        assert (
            payload["preflight"]["recording_ready"]["bag_ready_at_ros_ns"]
            == 1_000_000_000
        )
        first_camera_stamp = payload["preflight"]["recording_ready"][
            "first_camera_stamp_ros_ns"
        ]["top_camera"]
        last_camera_stamp = coverage["last_frame_stamp_ros_ns"]["top_camera"]
        assert (
            first_camera_stamp <= payload["timeline_start_ros_ns"] <= last_camera_stamp
        )

    asyncio.run(run())


def test_start_requires_profile_depth_topic_writer_coverage(
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
        assert depth_topic in bag_recorder.ready_topics
        assert (
            depth_topic
            in payload["preflight"]["recording_coverage"]["latest_bag_timestamp_ros_ns"]
        )
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


def test_start_fails_when_depth_writer_sample_is_stale_at_t0(
    tmp_path: Path,
    monkeypatch,
) -> None:
    async def run() -> None:
        monkeypatch.setattr(
            api_server.BagRecorder, "available", staticmethod(lambda: True)
        )
        monkeypatch.setattr(api_server, "_READY_TIMEOUT_S", 0.01)
        monkeypatch.setattr(api_server, "_READY_POLL_S", 0.001)
        store = EpisodeStore(tmp_path)
        depth_pool = _ReadyDepthPool()
        request = _DepthStartRequest(
            store,
            _StaleDepthBagRecorder(),
            _DepthOnlyCameraPool(),
        )
        request.app["depth_pool"] = depth_pool

        response = await _start_episode(request)
        payload = json.loads(response.body.decode("utf-8"))

        assert response.status == 503
        assert payload["error"] == "recording_coverage_timeout"
        assert "/camera/depth_compressed/compressedDepth" in payload["detail"]
        assert "earliest_acceptable_bag_stamp_ns=1100000000" in payload["detail"]
        assert store.active is None

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
        assert depth_topic not in bag_recorder.ready_topics
        assert camera_pool.started_cameras == []
        assert depth_pool.started_cameras == []

    asyncio.run(run())


def test_start_uses_three_frame_export_tolerance_for_bag_coverage(
    tmp_path: Path,
    monkeypatch,
) -> None:
    async def run() -> None:
        monkeypatch.setattr(
            api_server.BagRecorder, "available", staticmethod(lambda: True)
        )
        store = EpisodeStore(tmp_path)
        bag_recorder = _ReadyBagRecorder(coverage_stamp_ros_ns=1_150_000_000)

        response = await _start_episode(
            _Fps60StartRequest(store, bag_recorder, _ReadyCameraPool())
        )
        payload = json.loads(response.body.decode("utf-8"))

        assert response.status == 201
        coverage = payload["preflight"]["recording_coverage"]
        assert coverage["bag_alignment_tolerance_ns"] == 50_000_000
        assert set(coverage["latest_bag_timestamp_ros_ns"].values()) == {1_150_000_000}

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


def test_start_rejects_empty_override_without_ros_timestamp_evidence(
    tmp_path: Path,
    monkeypatch,
) -> None:
    async def run() -> None:
        monkeypatch.setattr(
            api_server.BagRecorder, "available", staticmethod(lambda: True)
        )
        monkeypatch.setattr(api_server, "_READY_TIMEOUT_S", 0.01)
        monkeypatch.setattr(api_server, "_READY_POLL_S", 0.001)
        store = EpisodeStore(tmp_path)

        response = await _start_episode(
            _EmptyOverrideStartRequest(store, _ReadyBagRecorder(), _EmptyCameraPool())
        )
        payload = json.loads(response.body.decode("utf-8"))

        assert response.status == 503
        assert payload == {
            "error": "recording_ready_timeout",
            "code": "recording_ready_timeout",
            "message": "recording inputs did not become ready before timeout",
            "bag_ready": True,
            "missing_bag": [],
            "missing_cameras": [],
            "bag_counts": {},
            "camera_counts": {},
            "first_bag_timestamp_ns": {},
            "first_camera_stamp_ros_ns": {},
            "bag_timestamp_source": "rosbag2_serialized_message_time_stamp",
            "timeout_s": 0.01,
        }
        assert "detail" not in payload
        assert store.active is None

    asyncio.run(run())


def test_recording_ready_timeout_returns_structured_evidence(
    tmp_path: Path,
    monkeypatch,
) -> None:
    async def run() -> None:
        monkeypatch.setattr(
            api_server.BagRecorder, "available", staticmethod(lambda: True)
        )
        monkeypatch.setattr(api_server, "_READY_TIMEOUT_S", 0.01)
        monkeypatch.setattr(api_server, "_READY_POLL_S", 0.001)
        store = EpisodeStore(tmp_path)
        first = await _start_episode(
            _StartRequest(
                store,
                _NeverReadyBagRecorder(),
                _NeverObservedCameraPool(),
            )
        )
        payload = json.loads(first.body.decode("utf-8"))
        expected_topics = [
            "/follower_arm/joint_ctrl",
            "/follower_arm/joint_states_single",
            "/follower_arm/teleop_mux/status",
        ]
        assert first.status == 503
        assert payload["error"] == "recording_ready_timeout"
        assert payload["code"] == "recording_ready_timeout"
        assert payload["message"] == (
            "recording inputs did not become ready before timeout"
        )
        assert payload["bag_ready"] is False
        assert payload["missing_bag"] == expected_topics
        assert payload["missing_cameras"] == ["top_camera"]
        assert payload["bag_counts"] == {topic: 0 for topic in expected_topics}
        assert payload["camera_counts"] == {"top_camera": 0}
        assert payload["first_bag_timestamp_ns"] == {
            topic: 0 for topic in expected_topics
        }
        assert payload["first_camera_stamp_ros_ns"] == {"top_camera": 0}
        assert payload["bag_timestamp_source"] == (
            "rosbag2_serialized_message_time_stamp"
        )
        assert payload["timeout_s"] == 0.01
        assert "detail" not in payload

    asyncio.run(run())


def test_start_fails_without_recorded_camera_frame(tmp_path: Path, monkeypatch) -> None:
    async def run() -> None:
        monkeypatch.setattr(api_server.BagRecorder, "available", staticmethod(lambda: True))
        monkeypatch.setattr(api_server, "_READY_TIMEOUT_S", 0.01)
        monkeypatch.setattr(api_server, "_READY_POLL_S", 0.001)
        store = EpisodeStore(tmp_path)
        camera_pool = _NeverRecordedCameraPool()

        response = await _start_episode(_StartRequest(store, _ReadyBagRecorder(), camera_pool))
        payload = json.loads(response.body.decode("utf-8"))

        assert response.status == 503
        assert payload["error"] == "recording_coverage_timeout"
        assert camera_pool.enabled
        assert store.active is None
        episode = store.get_episode("01TESTASYNCSTOP0000000000")
        assert episode is not None
        assert episode[0].state == "failed"
        assert episode[0].stale_input_events[0]["code"] == "recording_coverage_timeout"

    asyncio.run(run())


def test_start_fails_without_bag_sample_near_canonical_t0(
    tmp_path: Path,
    monkeypatch,
) -> None:
    async def run() -> None:
        monkeypatch.setattr(
            api_server.BagRecorder, "available", staticmethod(lambda: True)
        )
        monkeypatch.setattr(api_server, "_READY_TIMEOUT_S", 0.01)
        monkeypatch.setattr(api_server, "_READY_POLL_S", 0.001)
        store = EpisodeStore(tmp_path)

        response = await _start_episode(
            _StartRequest(store, _NeverCoveredBagRecorder(), _ReadyCameraPool())
        )
        payload = json.loads(response.body.decode("utf-8"))

        assert response.status == 503
        assert payload["error"] == "recording_coverage_timeout"
        assert "missing_bag=" in payload["detail"]
        assert "/follower_arm/joint_ctrl" in payload["detail"]
        assert "latest_bag_stamps=" in payload["detail"]
        assert "'/follower_arm/joint_ctrl': 1000000000" in payload["detail"]
        assert "earliest_acceptable_bag_stamp_ns=1100000000" in payload["detail"]
        assert store.active is None

    asyncio.run(run())


def test_bag_ready_status_reads_latest_writer_accepted_timestamps(
    tmp_path: Path,
) -> None:
    topic = "/follower_arm/joint_ctrl"
    ready_file = tmp_path / "bag_ready.json"
    ready_file.write_text(
        json.dumps(
            {
                "ready": True,
                "ready_at_ros_ns": 1_000_000_000,
                "bag_topics": {topic: 2},
                "first_bag_timestamp_ns": {topic: 1_000_000_000},
                "latest_bag_timestamp_ns": {topic: 1_100_000_000},
                "timestamp_source": "rosbag2_serialized_message_time_stamp",
            }
        ),
        encoding="utf-8",
    )
    recorder = BagRecorder()
    recorder._ready_file = ready_file

    status = recorder.ready_status({topic})

    assert status.ready
    assert status.counts == {topic: 2}
    assert status.latest_bag_timestamp_ns == {topic: 1_100_000_000}
