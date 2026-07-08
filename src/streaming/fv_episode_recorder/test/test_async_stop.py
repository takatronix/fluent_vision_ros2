from __future__ import annotations

import asyncio
import json
import sys
import time
import types
from pathlib import Path

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
from fv_episode_recorder.api_server import _start_episode, _stop_episode
from fv_episode_recorder.episode_store import EpisodeMeta, EpisodeStore, utc_now_iso


class _DetachedBag:
    def wait(self, timeout_s: float = 10.0) -> dict:
        time.sleep(0.05)
        return {"size_bytes": 0, "split_count": 0}


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
    def detach_all(self) -> _DetachedCameras:
        return _DetachedCameras()


class _DepthPool:
    def stop_all(self) -> dict[str, int]:
        return {}


class _MarkerManager:
    def flush(self, episode_id: str) -> list[dict]:
        return []


class _Request:
    def __init__(self, store: EpisodeStore, episode_id: str) -> None:
        self.match_info = {"episode_id": episode_id}
        self.app = {
            "store": store,
            "bag_recorder": _BagRecorder(),
            "camera_pool": _CameraPool(),
            "depth_pool": _DepthPool(),
            "marker_manager": _MarkerManager(),
            "mux_tracker": None,
            "active_lock": None,
        }

    async def json(self) -> dict[str, str]:
        return {"outcome": "success"}


class _StartRequest:
    def __init__(self, store: EpisodeStore, bag_recorder, camera_pool) -> None:
        self.match_info = {}
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
            "task_description": "pick",
            "profile": "piper_single",
            "tags": ["dpex:record"],
            "expected_duration_s": 60,
        }


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
        self.polls = 0

    def start(self, bag_dir: Path, topics: list[str]) -> None:
        self.started = True

    def topic_counts(self, topics: set[str]) -> dict[str, int]:
        self.polls += 1
        return {topic: 1 if self.polls >= 2 else 0 for topic in topics}


class _ReadyCameraPool:
    def __init__(self) -> None:
        self.polls = 0
        self.enabled = False
        self.started_cameras: list[dict] = []

    def start_all(self, episode_dir: Path, cameras: list[dict], fps: int = 30) -> list[dict]:
        self.started_cameras = cameras
        return [{"name": "top_camera", "frame_count": 0, "segments": []}]

    def observed_frame_counts(self) -> dict[str, int]:
        self.polls += 1
        return {"top_camera": 1 if self.polls >= 2 else 0}

    def enable_recording(self) -> None:
        self.enabled = True


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


def test_stop_detaches_active_episode_before_finalize(tmp_path: Path) -> None:
    async def run() -> None:
        store = EpisodeStore(tmp_path)
        store.start_episode(_meta("ep-1"))

        response = await _stop_episode(_Request(store, "ep-1"))
        payload = json.loads(response.body.decode("utf-8"))

        assert response.status == 202
        assert payload["state"] == "finalizing"
        assert store.active is None

        store.patch_episode_meta("ep-1", {"tags": ["dpex:record", "user:red"]})
        store.start_episode(_meta("ep-2"))
        await asyncio.sleep(0.2)

        first = store.get_episode("ep-1")
        assert first is not None
        assert first[0].state == "finished"
        assert first[0].tags == ["dpex:record", "user:red"]
        assert store.active is not None
        assert store.active.episode_id == "ep-2"

    asyncio.run(run())


def test_start_waits_for_recording_ready_before_enabling_camera(tmp_path: Path, monkeypatch) -> None:
    async def run() -> None:
        monkeypatch.setattr(api_server.BagRecorder, "available", staticmethod(lambda: True))
        store = EpisodeStore(tmp_path)
        bag_recorder = _ReadyBagRecorder()
        camera_pool = _ReadyCameraPool()

        response = await _start_episode(_StartRequest(store, bag_recorder, camera_pool))
        payload = json.loads(response.body.decode("utf-8"))

        assert response.status == 201
        assert bag_recorder.started
        assert bag_recorder.polls >= 2
        assert camera_pool.polls >= 2
        assert camera_pool.enabled
        assert camera_pool.started_cameras[0]["record_immediately"] is False
        assert payload["preflight"]["recording_ready"]["bag_topics"] == {
            "/follower_arm/joint_states_single": 1,
            "/follower_arm/joint_ctrl": 1,
            "/follower_arm/teleop_mux/status": 1,
        }
        assert store.active is not None
        assert store.active.started_at == payload["started_at"]

    asyncio.run(run())
