"""Encoder warm-up: the head of an episode must not be dropped while the
encoder opens (aspa3 2026-10-22: NVENC took ~2 s, 69-90 frames per camera
lost at the start). Needs rclpy / sensor_msgs / cv2 / pyarrow (ROS env); the
real-encoder round trip also needs ffmpeg."""

from __future__ import annotations

import shutil
import subprocess
import threading
import time
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")
cv2 = pytest.importorskip("cv2")
pq = pytest.importorskip("pyarrow.parquet")
pytest.importorskip("rclpy")
from sensor_msgs.msg import CompressedImage, Image  # noqa: E402

from fv_episode_recorder import camera_writer as cw  # noqa: E402


class _FakeNode:
    def create_subscription(self, *args, **kwargs):
        return object()

    def destroy_subscription(self, sub):
        pass


class _SlowStdin:
    """ffmpeg stdin whose first write blocks like an NVENC open, then the
    'encoder' writes to the output file."""

    def __init__(self, out: Path, open_s: float):
        self.out, self.open_s, self.writes = out, open_s, 0

    def write(self, data):
        if self.writes == 0:
            time.sleep(self.open_s)
        self.writes += 1
        with open(self.out, "ab") as f:
            f.write(b"x")

    def close(self):
        pass


class _FakeProc:
    def __init__(self, out: Path, open_s: float):
        self.stdin = _SlowStdin(out, open_s)

    def wait(self, timeout=None):
        return 0

    def kill(self):
        pass


def SPAWN_H264(open_s: float):
    return lambda out, w, h, fps: (_FakeProc(out, open_s), "fake")


def _jpeg_msg(i: int) -> CompressedImage:
    m = CompressedImage()
    m.format = "jpeg"
    img = np.full((48, 64, 3), i % 255, np.uint8)
    m.data = cv2.imencode(".jpg", img)[1].tobytes()
    m.header.stamp.sec, m.header.stamp.nanosec = 1000 + i // 30, (i % 30) * 33_000_000
    return m


def _depth_msg(i: int) -> Image:
    m = Image()
    m.height, m.width, m.encoding, m.step = 48, 64, "16UC1", 128
    m.data = np.full((48, 64), 500 + i, np.uint16).tobytes()
    m.header.stamp.sec, m.header.stamp.nanosec = 1000 + i // 30, (i % 30) * 33_000_000
    return m


def _feed(writer, make, n: int, fps: float = 30.0) -> list[int]:
    sent = []
    for i in range(n):
        msg = make(i)
        sent.append(time.time_ns())
        writer._on_image(msg)
        time.sleep(1.0 / fps)
    return sent


def _rows(path: Path) -> list[dict]:
    return pq.read_table(path).to_pylist()


# --- _IngestQueue ---------------------------------------------------------

def test_queue_holds_the_head_while_warming_then_bounds():
    q = cw._IngestQueue(steady=8, warm_frames=100)
    assert all(q.put(i, 10) for i in range(60))          # old queue: 52 of these dropped
    assert q.peak == 60
    got = [q.get() for _ in range(60)]
    assert [m for m, _ in got] == list(range(60))         # oldest first
    stamps = [t for _, t in got]
    assert stamps == sorted(stamps)                       # the callback's receive time
    q.end_warmup()
    assert all(q.put(i) for i in range(8))
    assert not q.put(8)                                   # steady bound: newest dropped


def test_queue_warm_bytes_cap_keeps_at_least_one_frame():
    q = cw._IngestQueue(steady=2, warm_frames=100, warm_bytes=1000)
    assert q.put("big", 5000)                             # alone over the cap: still kept
    assert not q.put("a", 1)
    assert q.get()[0] == "big"
    assert q.put("a", 400) and q.put("b", 400)
    assert not q.put("c", 400)                            # 1200 > 1000
    assert q.get()[0] == "a"
    assert q.put("c", 400)                                # bytes are released on get()


def test_queue_close_comes_after_queued_frames_and_ignores_late_ones():
    q = cw._IngestQueue()
    q.put("a")
    q.put("b")
    q.close()
    assert q.put("late")                                  # after the episode: ignored, not a drop
    assert [q.get()[0], q.get()[0], q.get()] == ["a", "b", None]


def test_queue_drop_pending_counts_what_stop_gave_up_on():
    q = cw._IngestQueue()
    for i in range(5):
        q.put(i)
    q.close()
    assert q.drop_pending() == 5
    assert q.get() is None


# --- writers: frames that arrive while the encoder opens are kept ----------

@pytest.mark.parametrize("open_s", [1.0, 2.5])
def test_camera_writer_keeps_head_while_encoder_opens(tmp_path, monkeypatch, open_s):
    monkeypatch.setattr(cw, "_spawn_ffmpeg_encoder",
                        SPAWN_H264(open_s))
    w = cw.CameraWriter("main", "/t", tmp_path / "main", fps=30, node=_FakeNode())
    w.start()
    sent = _feed(w, _jpeg_msg, 90)
    s = w.stop()
    assert s["dropped_frames"] == 0
    assert s["frame_count"] == 90
    rows = _rows(tmp_path / "main" / "frames.parquet")
    assert [r["frame_index"] for r in rows] == list(range(90))
    assert [r["ros_stamp_ns"] for r in rows] == sorted(r["ros_stamp_ns"] for r in rows)
    # recv_stamp_ns is when the callback got the frame, not when it was encoded
    # (the head was encoded ~open_s later)
    assert max(abs(r["recv_stamp_ns"] - t) for r, t in zip(rows, sent)) < 50_000_000


def test_camera_writer_old_queue_would_drop_the_head(tmp_path, monkeypatch):
    """Control: with the steady 8-frame bound from the start (the old queue)
    the same encoder open loses the head — the test above is meaningful."""
    monkeypatch.setattr(cw, "_spawn_ffmpeg_encoder",
                        SPAWN_H264(1.0))
    w = cw.CameraWriter("main", "/t", tmp_path / "main", fps=30, node=_FakeNode())
    w._queue = cw._IngestQueue(warm_frames=8)
    w._queue.end_warmup()
    w.start()
    _feed(w, _jpeg_msg, 60)
    assert w.stop()["dropped_frames"] > 15


def test_depth_writer_keeps_head_while_encoder_opens(tmp_path, monkeypatch):
    monkeypatch.setattr(cw, "_spawn_ffmpeg_hevc10_encoder",
                        lambda out, w, h, fps, cq: (_FakeProc(out, 1.0), "fake"))
    w = cw.DepthVideoWriter("depth", "/d", tmp_path / "depth", fps=30, node=_FakeNode(),
                            d_min=400, d_max=4000)
    w.start()
    _feed(w, _depth_msg, 60)
    s = w.stop()
    assert (s["dropped_frames"], s["frame_count"]) == (0, 60)
    rows = _rows(tmp_path / "depth" / "frames.parquet")
    assert [r["frame_index"] for r in rows] == list(range(60))


def test_warmup_clock_starts_at_the_first_frame(tmp_path, monkeypatch):
    """A camera that starts publishing late still gets its full warm-up."""
    monkeypatch.setattr(cw, "_WARMUP_MAX_S", 1.5)
    monkeypatch.setattr(cw, "_spawn_ffmpeg_encoder", SPAWN_H264(1.0))
    w = cw.CameraWriter("arm", "/t", tmp_path / "arm", fps=30, node=_FakeNode())
    w.start()
    time.sleep(2.0)                                       # no frames yet
    _feed(w, _jpeg_msg, 60)
    assert w.stop()["dropped_frames"] == 0


def test_warmup_ends_once_encoder_writes(tmp_path, monkeypatch):
    monkeypatch.setattr(cw, "_spawn_ffmpeg_encoder",
                        SPAWN_H264(0.5))
    w = cw.CameraWriter("main", "/t", tmp_path / "main", fps=30, node=_FakeNode())
    w.start()
    _feed(w, _jpeg_msg, 30)
    assert not w._queue.warming                           # steady bound again
    assert w._queue.peak > cw._STEADY_QUEUE
    w.stop()


# --- depth encoder chosen once, at recorder start --------------------------

def _fake_choose(calls, delay=0.0):
    def choose(cq, probe=None):
        calls.append(cq)
        time.sleep(delay)
        return ("hevc_nvenc", dict(cw.depth10.hevc10_encoder_tries(cq))["hevc_nvenc"], "p010le", True)
    return choose


def test_hevc10_choice_probed_once_whatever_cq(monkeypatch):
    calls: list[int] = []
    monkeypatch.setattr(cw, "_hevc10_pick", None)
    monkeypatch.setattr(cw, "_hevc10_failed_at", float("-inf"))
    monkeypatch.setattr(cw.depth10, "choose_hevc10_encoder", _fake_choose(calls, delay=0.3))
    out: list = []
    ts = [threading.Thread(target=lambda cq=cq: out.append(cw._choose_hevc10(cq))) for cq in (16, 18, 16, 18)]
    for t in ts:
        t.start()
    for t in ts:
        t.join()
    assert len(calls) == 1                                # one probe, the others waited for it
    for picked in out:
        assert picked[0] == "hevc_nvenc" and picked[3] is True
    assert {p[1][p[1].index("-cq") + 1] for p in out} == {"16", "18"}   # options follow each camera's cq


def test_warm_encoders_picks_the_depth_encoder(monkeypatch):
    calls: list[int] = []
    monkeypatch.setattr(cw, "_hevc10_pick", None)
    monkeypatch.setattr(cw, "_hevc10_failed_at", float("-inf"))
    monkeypatch.setattr(cw.shutil, "which", lambda name: "/usr/bin/" + name)
    monkeypatch.setattr(cw.depth10, "choose_hevc10_encoder", _fake_choose(calls))
    assert cw.warm_encoders() == {"hevc10": "hevc_nvenc"}
    cw._choose_hevc10(18)
    assert calls == [16]                                  # episodes don't probe again


def test_no_hevc10_encoder_is_shared_with_waiters_only(monkeypatch):
    calls: list[int] = []

    def none(cq, probe=None):
        calls.append(cq)
        time.sleep(0.3)

    monkeypatch.setattr(cw, "_hevc10_pick", None)
    monkeypatch.setattr(cw, "_hevc10_failed_at", float("-inf"))
    monkeypatch.setattr(cw.depth10, "choose_hevc10_encoder", none)
    out: list = []
    ts = [threading.Thread(target=lambda: out.append(cw._choose_hevc10(16))) for _ in range(3)]
    for t in ts:
        t.start()
    for t in ts:
        t.join()
    assert out == [None] * 3 and calls == [16]            # not 3 probes one after another
    assert cw._choose_hevc10(16) is None and calls == [16, 16]   # a later episode tries again


# --- real ffmpeg ------------------------------------------------------------

@pytest.mark.skipif(not shutil.which("ffmpeg"), reason="ffmpeg unavailable")
def test_real_encoder_round_trip(tmp_path):
    w = cw.CameraWriter("main", "/t", tmp_path / "main", fps=30, node=_FakeNode())
    w.start()
    _feed(w, _jpeg_msg, 45)
    s = w.stop()
    assert s["dropped_frames"] == 0 and s["frame_count"] == 45
    assert len(_rows(tmp_path / "main" / "frames.parquet")) == 45
    cap = cv2.VideoCapture(str(tmp_path / "main" / "0000.mp4"))
    n = 0
    while cap.read()[0]:
        n += 1
    assert n == 45
