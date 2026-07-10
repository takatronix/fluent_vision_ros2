from __future__ import annotations

import asyncio
import json
import sys
import time
import types
from dataclasses import asdict
from fractions import Fraction
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
import fv_episode_recorder.camera_writer as camera_writer
from fv_episode_recorder.api_server import _retention_put_policy, _start_episode, _stop_episode
from fv_episode_recorder.bag_recorder import BagReadyStatus
from fv_episode_recorder.camera_writer import CameraWriter
from fv_episode_recorder.episode_store import EpisodeMeta, EpisodeStore, utc_now_iso
from fv_episode_recorder.retention import RetentionPolicy, RetentionRunner


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
    def detach_all(self) -> _ZeroFrameDetachedCameras:
        return _ZeroFrameDetachedCameras()


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
        self.polls = 0
        self.ready_topics: set[str] = set()

    def start(self, bag_dir: Path, topics: list[str], ready_topics: set[str] | None = None) -> None:
        self.started = True
        self.ready_topics = set(ready_topics or set())

    def ready_status(self, required_topics: set[str]) -> BagReadyStatus:
        self.polls += 1
        counts = {topic: 1 if self.polls >= 2 else 0 for topic in required_topics}
        return BagReadyStatus(
            ready=all(count > 0 for count in counts.values()),
            ready_at="2026-07-08T00:00:00.000000Z" if self.polls >= 2 else None,
            counts=counts,
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
        return [{"name": "top_camera", "frame_count": 0, "segments": []}]

    def observed_frame_counts(self) -> dict[str, int]:
        self.polls += 1
        return {"top_camera": 1 if self.polls >= 2 else 0}

    def enable_recording(self) -> None:
        self.enabled = True

    def frame_counts(self) -> dict[str, int]:
        self.frame_polls += 1
        return {"top_camera": 1 if self.enabled and self.frame_polls >= 2 else 0}

    def detach_all(self) -> _DetachedCameras:
        return _DetachedCameras()


class _NeverRecordedCameraPool(_ReadyCameraPool):
    def frame_counts(self) -> dict[str, int]:
        self.frame_polls += 1
        return {"top_camera": 0}


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
        assert episode[0].stale_input_events == [
            {"kind": "finalize_error", "detail": "camera top_camera: no recorded frames"}
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
    camera_writer._remux_mp4_with_ros_timestamps(video_path, frame_stamps_ns)

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
        packet_durations_s = [
            round(float(packet.duration * packet.time_base), 3)
            for packet in remuxed.demux(stream)
            if packet.dts is not None
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
        assert bag_recorder.polls >= 2
        assert bag_recorder.ready_topics == {
            "/follower_arm/joint_states_single",
            "/follower_arm/joint_ctrl",
            "/follower_arm/teleop_mux/status",
        }
        assert camera_pool.polls >= 2
        assert camera_pool.enabled
        assert camera_pool.frame_polls >= 2
        assert camera_pool.started_cameras[0]["record_immediately"] is False
        assert payload["preflight"]["recording_ready"]["bag_topics"] == {
            "/follower_arm/joint_states_single": 1,
            "/follower_arm/joint_ctrl": 1,
            "/follower_arm/teleop_mux/status": 1,
        }
        assert payload["preflight"]["first_recorded_frames"] == {"top_camera": 1}
        assert payload["started_at"] == "2026-07-08T00:00:00.000000Z"

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
        assert payload["error"] == "first_recorded_frame_timeout"
        assert camera_pool.enabled
        assert store.active is None
        episode = store.get_episode("01TESTASYNCSTOP0000000000")
        assert episode is not None
        assert episode[0].state == "failed"
        assert episode[0].stale_input_events[0]["code"] == "first_recorded_frame_timeout"

    asyncio.run(run())
