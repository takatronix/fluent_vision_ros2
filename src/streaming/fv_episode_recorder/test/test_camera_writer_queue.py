import importlib
import sys
import types
from pathlib import Path

import pytest


class _Sidecar:
    def __init__(self, _path: Path):
        self.rows = []

    def append(self, row: dict) -> None:
        self.rows.append(row)

    def close(self) -> int:
        return len(self.rows)


def _camera_writer_module(monkeypatch: pytest.MonkeyPatch):
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = object
    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.qos_profile_sensor_data = object()
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.CompressedImage = object
    sensor_msgs_msg.Image = object
    sidecar = types.ModuleType("fv_episode_recorder.frames_sidecar")
    sidecar.FramesSidecar = _Sidecar
    for name, module in {
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "rclpy.qos": rclpy_qos,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.msg": sensor_msgs_msg,
        "fv_episode_recorder.frames_sidecar": sidecar,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)
    sys.modules.pop("fv_episode_recorder.camera_writer", None)
    return importlib.import_module("fv_episode_recorder.camera_writer")


def _message(stamp_ns: int):
    stamp = types.SimpleNamespace(
        sec=stamp_ns // 1_000_000_000,
        nanosec=stamp_ns % 1_000_000_000,
    )
    return types.SimpleNamespace(
        data=b"compressed-frame", format="jpeg",
        header=types.SimpleNamespace(stamp=stamp),
    )


def test_callback_only_enqueues_header_timestamp(monkeypatch: pytest.MonkeyPatch,
                                                 tmp_path: Path) -> None:
    module = _camera_writer_module(monkeypatch)
    monkeypatch.setattr(module, "VfrMp4Writer", lambda *_args: types.SimpleNamespace())
    writer = module.CameraWriter(
        "color", "/camera", tmp_path, encoder=module.EncoderConfig("test", ()),
    )

    writer._on_image(_message(0))

    queued = writer._queue.get_nowait()
    assert queued.data == b"compressed-frame"
    assert queued.ros_stamp_ns == 0
    assert writer.frame_count == 0


def test_queue_overflow_is_reported_without_raising_from_callback(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path,
) -> None:
    module = _camera_writer_module(monkeypatch)
    monkeypatch.setattr(module, "VfrMp4Writer", lambda *_args: types.SimpleNamespace())
    writer = module.CameraWriter(
        "color", "/camera", tmp_path, encoder=module.EncoderConfig("test", ()),
        queue_size=1,
    )
    writer._on_image(_message(1))

    writer._on_image(_message(2))

    assert writer._failure is not None
    assert "queue overflow" in str(writer._failure)


def test_worker_failure_is_reported_by_status_not_callback(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path,
) -> None:
    module = _camera_writer_module(monkeypatch)

    class FailingVideo:
        def append(self, *_args):
            raise RuntimeError("decode failed")

        def abort(self):
            pass

    monkeypatch.setattr(module, "VfrMp4Writer", lambda *_args: FailingVideo())
    writer = module.CameraWriter(
        "color", "/camera", tmp_path, encoder=module.EncoderConfig("test", ()),
    )
    writer._queue.put(module._QueuedFrame(b"bad", 1, 2))
    writer._run_worker()

    writer._on_image(_message(3))

    with pytest.raises(RuntimeError, match="camera worker failed: decode failed"):
        writer.stop()


def test_worker_persists_ros_stamp_with_video_pts(monkeypatch: pytest.MonkeyPatch,
                                                  tmp_path: Path) -> None:
    module = _camera_writer_module(monkeypatch)

    class Video:
        width = 64
        height = 48

        def append(self, _data, _ros_stamp_ns):
            return types.SimpleNamespace(video_pts=42)

        def close(self):
            pass

        def abort(self):
            pass

    monkeypatch.setattr(module, "VfrMp4Writer", lambda *_args: Video())
    writer = module.CameraWriter(
        "color", "/camera", tmp_path, encoder=module.EncoderConfig("test", ()),
    )
    writer._queue.put(module._QueuedFrame(b"jpeg", 123_456_789, 987_654_321))
    writer._queue.put(module._STOP)

    writer._run_worker()

    assert writer._sidecar.rows[0]["ros_stamp_ns"] == 123_456_789
    assert writer._sidecar.rows[0]["video_pts"] == 42


def test_encoder_probe_precedes_every_subscription(monkeypatch: pytest.MonkeyPatch,
                                                   tmp_path: Path) -> None:
    module = _camera_writer_module(monkeypatch)
    events = []
    encoder = module.EncoderConfig("test", ())
    monkeypatch.setattr(module, "probe_h264_encoder", lambda: events.append("probe") or encoder)

    class Writer:
        frame_count = 0

        def __init__(self, name, **_kwargs):
            self.name = name

        def start(self):
            events.append(f"subscribe:{self.name}")

        def summary(self):
            return {"name": self.name}

    monkeypatch.setattr(module, "CameraWriter", Writer)
    pool = module.CameraWriterPool(node=object())

    pool.start_all(
        tmp_path,
        [{"name": "left", "topic": "/left"}, {"name": "right", "topic": "/right"}],
    )

    assert events == ["probe", "subscribe:left", "subscribe:right"]
