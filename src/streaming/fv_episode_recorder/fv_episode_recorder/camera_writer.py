"""Per-camera writer: subscribes to a camera topic, writes mp4 + frames.parquet.

Phase 1 Step 3 minimum:
  - CompressedImage subscriber (BEST_EFFORT, qos_profile_sensor_data)
  - cv2.VideoWriter with mp4v fourcc (CPU encode, swap to h264_nvenc later)
  - 1 mp4 file per camera (segment rotation lands in Step 3.5)
  - FramesSidecar populates frames.parquet

Future steps:
  - segment rotation (10 min default)
  - h264_nvenc (Tegra hw encode)
  - sensor_msgs/Image (uncompressed) support
  - cameras_downscale for untagged pool (Phase 2.5 always_on)
"""

from __future__ import annotations

import logging
import shutil
import subprocess
import threading
import time
from collections import deque
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

from .frames_sidecar import FramesSidecar
from .executor_guard import destroy_on_executor
from . import depth10
from .calibration import camera_info_to_dict

LOG = logging.getLogger("fv_episode_recorder.camera")


# ---------------------------------------------------------------------------
# H.264 encoder selection (佐伯さん, fluent_vision_ros2 #26 / ec28653).
#
# Raw frames are piped to ffmpeg already in the encoder's input format
# (yuv420p, converted with cv2 in CameraWriter._to_pipe_bytes) so NO
# libavfilter conversion is needed. The Jetson ffmpeg package (7:8.0.1-nvidia1,
# JetPack 7) ships NVENC but only ~30 filters: no `lavfi` (the old probe's test
# source) and no `scale` (which ffmpeg inserts to convert bgr24 → yuv420p). On
# that build the lavfi probe reported every encoder as unavailable and the pipe
# would have died at the first frame anyway ("'scale' filter not present"), so
# every episode had 0 video frames (aspa2 / aspa3, 2026-10-20). Probing with two
# black rawvideo frames on stdin works on every ffmpeg build. (The depth path
# probes the same way in depth10.probe_encoder.)
# ---------------------------------------------------------------------------
_PROBE_SIDE = 256   # NVENC rejects tiny frames: a 64x64 probe made h264_nvenc look unavailable
_PROBE_BYTES_PER_PIXEL = {"yuv420p": 1.5, "bgr24": 3}
_h264_ok: set[tuple[str, str]] = set()     # (codec, stdin pix_fmt) that encoded the test frames in this process
_h264_failed: dict[tuple[str, str], tuple[float, str]] = {}   # key -> (time.monotonic() at the end, reason)
_h264_locks: dict[tuple[str, str], threading.Lock] = {}
_h264_locks_guard = threading.Lock()


def _h264_tries() -> list[tuple[str, list[str], str]]:
    return [
        # (codec, extra args, stdin pix_fmt)
        ("h264_nvenc", ["-preset", "p1", "-tune", "ll"], "yuv420p"),               # Tegra HW, fastest
        ("libx264",    ["-preset", "ultrafast", "-tune", "zerolatency"], "yuv420p"),
    ]


def _run_h264_probe(codec: str, extra: list[str], pix_fmt: str) -> Optional[str]:
    """Encode two black rawvideo frames from stdin. None = usable, else the reason (for the log)."""
    n = int(_PROBE_SIDE * _PROBE_SIDE * _PROBE_BYTES_PER_PIXEL[pix_fmt]) * 2
    try:
        test = subprocess.run(
            ["ffmpeg", "-hide_banner", "-loglevel", "error",
             "-f", "rawvideo", "-pix_fmt", pix_fmt, "-s", f"{_PROBE_SIDE}x{_PROBE_SIDE}", "-r", "25",
             "-i", "pipe:0", "-c:v", codec, *extra, "-frames:v", "2", "-f", "null", "-"],
            input=bytes(n), stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, timeout=15,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError, OSError) as exc:
        return f"probe failed: {exc}"
    err = test.stderr.decode("utf-8", errors="replace").strip()
    # "Nothing was written" comes with exit 0 when the encoder could not be opened
    # (e.g. a pixel format conversion that needs the missing `scale` filter).
    if test.returncode != 0 or "Nothing was written" in err:
        return (err or f"exit {test.returncode}")[:200]
    return None


def _probe_h264(codec: str, extra: list[str], pix_fmt: str) -> Optional[str]:
    """_run_h264_probe once per process per encoder, one at a time: cameras whose
    episode starts while the warm-up probe runs wait for its answer instead of
    opening NVENC again. A failure is shared only with those waiting; later
    calls probe again (same rule as _choose_hevc10)."""
    key = (codec, pix_fmt)
    if key in _h264_ok:
        return None
    asked = time.monotonic()
    with _h264_locks_guard:
        lock = _h264_locks.setdefault(key, threading.Lock())
    with lock:
        if key in _h264_ok:
            return None
        failed = _h264_failed.get(key)
        if failed is not None and failed[0] >= asked:
            return failed[1]
        why = _run_h264_probe(codec, extra, pix_fmt)
        if why is None:
            _h264_ok.add(key)
        else:
            _h264_failed[key] = (time.monotonic(), why)
        return why


def _spawn_ffmpeg_encoder(out_path: Path, width: int, height: int, fps: int
                          ) -> tuple[subprocess.Popen, str, str] | None:
    """Try H.264 encoders in priority order. Returns (proc, codec, stdin pix_fmt)
    or None if all encoders failed. Frames must be written in that pix_fmt
    (yuv420p = planar I420, see CameraWriter._to_pipe_bytes). Output is
    faststart mp4 for HTML5 video (browser-native H.264 4:2:0).

    Priority: h264_nvenc (Tegra HW encode) → libx264 (CPU)."""
    if not shutil.which("ffmpeg"):
        LOG.error("ffmpeg not found in PATH")
        return None
    encoder_tries = _h264_tries()
    for codec, extra, pix_fmt in encoder_tries:
        why = _probe_h264(codec, extra, pix_fmt)
        if why is not None:
            LOG.warning("[encoder probe] %s (%s in) unavailable: %s", codec, pix_fmt, why)
            continue
        cmd = [
            "ffmpeg", "-y", "-loglevel", "error",
            "-f", "rawvideo", "-pix_fmt", pix_fmt,
            "-s", f"{width}x{height}", "-r", str(fps),
            "-i", "pipe:0",
            "-c:v", codec,
            *extra,
            "-pix_fmt", "yuv420p",
            "-movflags", "+faststart",
            str(out_path),
        ]
        # Launch the real encoder pipe.
        try:
            # stderr must NOT be an unread PIPE: ffmpeg blocks once the pipe
            # buffer fills, which freezes stdin.write in the subscriber
            # callback and stalls the whole recording. Log file instead.
            with open(out_path.parent / "ffmpeg.log", "ab") as errlog:
                proc = subprocess.Popen(
                    cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=errlog,
                )
            LOG.info("ffmpeg encoder ready: %s (%s in) → %s (%dx%d @ %dfps)",
                     codec, pix_fmt, out_path.name, width, height, fps)
            return proc, codec, pix_fmt
        except Exception as exc:
            LOG.warning("[encoder] %s spawn failed: %s", codec, exc)
            continue
    LOG.error("no H.264 encoder available (tried %s)", [c for c, _, _ in encoder_tries])
    return None


def _spawn_ffmpeg_hevc10_encoder(out_path: Path, width: int, height: int,
                                 fps: int, cq: int) -> tuple[subprocess.Popen, str] | None:
    """HEVC Main10 encoder pipe for 10-bit-luma depth video (yuv420p10le stdin).
    Priority: hevc_nvenc (Tegra HW) → libx265 (CPU). Not browser-playable —
    this stream is training data, decoded via fv_episode_recorder.depth10."""
    if not shutil.which("ffmpeg"):
        LOG.error("ffmpeg not found in PATH")
        return None
    encoder_tries = depth10.hevc10_encoder_tries(cq)
    picked = _choose_hevc10(cq)
    for codec, extra in ([picked[:2]] if picked else []):
        # choose_hevc10_encoder probes with rawvideo on stdin (no lavfi — the
        # Jetson ffmpeg has none) and ffprobes the result: prefer an encoder
        # that really writes 10-bit; else keep the working one (e.g. hevc_nvenc
        # → nv12 on the ASPA Jetson) so depth is still recorded, and say so.
        if not picked[3]:
            LOG.error("[encoder probe] no verified 10-bit HEVC encoder; using %s (writes %s) — "
                      "depth is coarser than Main10 (materializer flags bit_depth_ok=false)",
                      codec, picked[2] or "unknown pix_fmt")
        cmd = depth10.hevc10_encode_cmd(codec, extra, out_path, width, height, fps)
        try:
            # stderr must NOT be an unread PIPE: ffmpeg blocks once the pipe
            # buffer fills, which freezes stdin.write in the subscriber
            # callback and stalls the whole recording. Log file instead.
            errlog = open(out_path.parent / "ffmpeg.log", "ab")
            proc = subprocess.Popen(
                cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=errlog,
            )
            errlog.close()
            LOG.info("hevc10 encoder ready: %s → %s (%dx%d @ %dfps cq%d)",
                     codec, out_path.name, width, height, fps, cq)
            return proc, codec
        except Exception as exc:
            LOG.warning("[encoder] %s spawn failed: %s", codec, exc)
            continue
    LOG.error("no HEVC Main10 encoder available (tried %s)", [c for c, _ in encoder_tries])
    return None


# ---------------------------------------------------------------------------
# Encoder warm-up.
#
# Opening an NVENC encoder takes ~2 s while aspa3's GPU is busy (CUDA context
# + encode session), and the encoder probes cost about as much again. Both
# happened on each camera's first frame while the encode thread sat blocked,
# so the 8-frame ingest queue overflowed and the head of the episode was
# dropped (2026-10-22, aspa3: 69-90 frames per camera on the first episode
# after a recorder restart, 2-24 later; aspa2 1-12). Now:
#   - warm_encoders() runs the depth encoder choice once when the recorder
#     starts (CameraWriterPool), so no episode waits for that probe;
#   - _IngestQueue keeps the frames that arrive until the camera's encoder has
#     written output (bounded), oldest first, with the time the callback got
#     them.
# ---------------------------------------------------------------------------
_WARMUP_MAX_S = 8.0               # hold at most this long's worth of frames (fps_nominal) …
_WARMUP_MAX_BYTES = 256 << 20     # … and at most this many message bytes per camera
_STEADY_QUEUE = 8                 # afterwards: bounded latency, the newest frame is dropped

_hevc10_lock = threading.Lock()
_hevc10_pick: Optional[tuple] = None
_hevc10_failed_at = float("-inf")    # time.monotonic() when the last choice found no encoder


def _choose_hevc10(cq: int) -> tuple[str, list[str], str | None, bool] | None:
    """depth10.choose_hevc10_encoder, probed once per process whatever the cq
    (which encoder works does not depend on it; its options do) and one probe
    at a time — depth cameras that start during the warm-up wait for it. "No
    encoder" is shared only with those waiting; later calls probe again."""
    global _hevc10_pick, _hevc10_failed_at
    asked = time.monotonic()
    with _hevc10_lock:
        if _hevc10_pick is None and _hevc10_failed_at < asked:
            _hevc10_pick = depth10.choose_hevc10_encoder(cq)
            if _hevc10_pick is None:
                _hevc10_failed_at = time.monotonic()
        picked = _hevc10_pick
    if picked is None:
        return None
    codec, _extra, pix_fmt, ten_bit = picked
    return codec, dict(depth10.hevc10_encoder_tries(cq))[codec], pix_fmt, ten_bit


def warm_encoders(cq: int = 16) -> dict[str, str]:
    """Probe the H.264 encoders (in the order the writers try them) and pick the
    depth encoder now. Returns {"h264": codec, "hevc10": codec} ("" = none works)."""
    out = {"h264": "", "hevc10": ""}
    if not shutil.which("ffmpeg"):
        return out
    for codec, extra, pix_fmt in _h264_tries():
        why = _probe_h264(codec, extra, pix_fmt)
        if why is None:
            out["h264"] = codec
            break
        LOG.warning("[encoder warm-up] %s (%s in) unavailable: %s", codec, pix_fmt, why)
    picked = _choose_hevc10(cq)
    out["hevc10"] = picked[0] if picked else ""
    LOG.info("encoders warmed up: %s", out)
    return out


class _IngestQueue:
    """Frames from a camera callback to its encode thread, oldest first, each
    with the time the callback got it.

    Holds `steady` frames once warmed up (put() returns False when full: the
    newest frame is dropped). Until end_warmup() — the writer's encoder has
    written output and the backlog is down to `steady` — it holds up to
    `warm_frames` frames / `warm_bytes` bytes instead: the head of the episode,
    arriving while the encoder opens."""

    def __init__(self, steady: int = _STEADY_QUEUE, warm_frames: int = 240,
                 warm_bytes: int = _WARMUP_MAX_BYTES):
        self.steady = steady
        self.warm_frames = max(warm_frames, steady)
        self.warm_bytes = warm_bytes
        self.warming = True
        self.first_frame_t: Optional[float] = None   # time.monotonic() of the first frame
        self.peak = 0                     # most frames held at once
        self._items: deque = deque()
        self._bytes = 0
        self._closed = False
        self._cv = threading.Condition()

    def put(self, msg, nbytes: int = 0) -> bool:
        """False = dropped (full). A frame after close() belongs to no episode:
        ignored, not counted."""
        recv_ns = time.time_ns()
        with self._cv:
            if self._closed:
                return True
            if self.first_frame_t is None:
                self.first_frame_t = time.monotonic()
            n = len(self._items)
            if self.warming:
                full = n >= self.warm_frames or (n > 0 and self._bytes + nbytes > self.warm_bytes)
            else:
                full = n >= self.steady
            if full:
                return False
            self._items.append((msg, recv_ns, nbytes))
            self._bytes += nbytes
            self.peak = max(self.peak, n + 1)
            self._cv.notify()
            return True

    def close(self) -> None:
        """get() returns None after the frames already queued."""
        with self._cv:
            self._closed = True
            self._items.append(None)
            self._cv.notify()

    def drop_pending(self) -> int:
        """Discard the frames still queued (stop gave up waiting); returns how
        many. get() returns None next."""
        with self._cv:
            n = sum(1 for it in self._items if it is not None)
            self._items.clear()
            self._bytes = 0
            self._closed = True
            self._items.append(None)
            self._cv.notify()
            return n

    def get(self):
        """Next (msg, recv_ns) — blocks — or None once closed."""
        with self._cv:
            while not self._items:
                self._cv.wait()
            item = self._items.popleft()
            if item is None:
                return None
            self._bytes -= item[2]
            return item[0], item[1]

    def end_warmup(self) -> None:
        with self._cv:
            self.warming = False

    def __len__(self) -> int:
        return len(self._items)


def _maybe_end_warmup(name: str, q: _IngestQueue, proc: Optional[subprocess.Popen],
                      out_path: Path) -> None:
    """Back to the steady bound once the encoder writes (ffmpeg writes the mp4
    only after the encoder has opened) and the backlog is drained — or
    _WARMUP_MAX_S after the first frame, e.g. when no encoder works at all."""
    waited = time.monotonic() - (q.first_frame_t or time.monotonic())
    writing = False
    if proc is not None and len(q) <= q.steady:
        try:
            writing = out_path.stat().st_size > 0
        except OSError:
            pass
    if writing or waited > _WARMUP_MAX_S:
        q.end_warmup()
        LOG.info("[%s] encoder %s after %.2f s; held up to %d frames meanwhile",
                 name, "up" if writing else "not writing yet", waited, q.peak)


def _unsubscribe(node, subs: list, name: str) -> str:
    """Destroy ``subs`` on the executor thread — from the HTTP thread it races
    the executor's wait-set rebuild and kills its spin thread (rclpy
    InvalidHandle), see executor_guard. Returns where it ran; "pending" = the
    executor did not get to it in time (left with it, a stop failure)."""
    def _destroy() -> None:
        for sub in subs:
            node.destroy_subscription(sub)
    try:
        return destroy_on_executor(node, _destroy)
    except Exception as exc:  # noqa: BLE001 — a teardown problem must not fail the stop
        LOG.warning("[%s] unsubscribe failed: %s", name, exc)
        return "error"


class _CameraInfoMixin:
    """Optional intrinsics captured once per camera.

    CameraWriterPool feeds the first sensor_msgs/CameraInfo from the
    profile's ``camera_info_topic`` via set_camera_info(); summary() then
    carries it as ``camera_info`` (meta.json cameras[]). Without a
    configured topic nothing is added — meta.json stays byte-compatible."""

    camera_info: Optional[dict] = None
    camera_info_topic: Optional[str] = None

    def set_camera_info(self, info: Optional[dict]) -> None:
        # first message wins: intrinsics are static per episode
        if info and self.camera_info is None:
            self.camera_info = dict(info)

    def _with_camera_info(self, summary: dict) -> dict:
        if self.camera_info_topic:
            summary["camera_info_topic"] = self.camera_info_topic
        if self.camera_info is not None:
            summary["camera_info"] = self.camera_info
        return summary


class CameraWriter(_CameraInfoMixin):
    """One worker per camera: subscribes, encodes, writes mp4 + sidecar."""

    def __init__(
        self,
        name: str,
        topic: str,
        output_dir: Path,
        fps: int = 30,
        node: Optional[Node] = None,
        raw_image: bool = False,
    ):
        self.name = name
        self.topic = topic
        self.output_dir = Path(output_dir)  # videos/<name>/
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.fps_nominal = fps
        self._node = node
        # raw_image: topic carries sensor_msgs/Image (bgr8/rgb8) instead of
        # CompressedImage — e.g. the X3 equirect stream, which has no
        # /compressed sibling. Frames feed the same ffmpeg pipe.
        self.raw_image = raw_image

        self._segment_file = self.output_dir / "0000.mp4"
        self._sidecar = FramesSidecar(self.output_dir / "frames.parquet")

        # ffmpeg subprocess (H.264 encoder) replaces cv2.VideoWriter which
        # only had MPEG-4 part 2 — that codec is not browser-playable.
        self._ffmpeg: Optional[subprocess.Popen] = None
        self._codec_in_use: str = ""
        self._pix_fmt_in: str = ""       # what ffmpeg expects on stdin (see _to_pipe_bytes)
        self._frame_index = 0
        self._segment_local = 0
        self._last_recv_ns = 0
        self._lock = threading.Lock()
        self._sub = None
        self.unsubscribe: Optional[str] = None   # where stop() destroyed the subscription
        self._first_frame_received = False
        self.frame_count = 0
        self.width = 0
        self.height = 0
        self.fps_actual = 0.0
        self._start_wall_ns = 0
        self._stopped = False
        # bounded ingest queue + worker thread: the rclpy callback only
        # enqueues; decode/encode runs here so no camera can starve the
        # executor. Overflow drops the newest frame (bounded latency) —
        # except while the encoder starts (see _IngestQueue).
        self._queue = _IngestQueue(warm_frames=int(fps * _WARMUP_MAX_S))
        self._worker = threading.Thread(
            target=self._drain, name=f"camwr-{name}", daemon=True)
        self.dropped_frames = 0

    def start(self) -> None:
        if self._node is None:
            raise RuntimeError("CameraWriter requires a rclpy Node to attach subscriber")
        # Own callback group: with the MultiThreadedExecutor this lets
        # cameras encode in parallel — the shared default group would
        # serialize every subscription behind the slowest camera.
        self._cbg = MutuallyExclusiveCallbackGroup()
        self._sub = self._node.create_subscription(
            Image if self.raw_image else CompressedImage,
            self.topic,
            self._on_image,
            qos_profile_sensor_data,
            callback_group=self._cbg,
        )
        self._start_wall_ns = time.time_ns()
        self._worker.start()
        LOG.info("camera writer started: %s topic=%s raw=%s out=%s",
                 self.name, self.topic, self.raw_image, self.output_dir)

    def _decode_frame(self, msg) -> Optional[np.ndarray]:
        """Return a HxWx3 BGR uint8 frame from either message type."""
        if not self.raw_image:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            return cv2.imdecode(arr, cv2.IMREAD_COLOR)
        enc = msg.encoding.lower()
        ch = 1 if enc == "mono8" else 3
        try:
            rows = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
            img = rows[:, : msg.width * ch].reshape(msg.height, msg.width, ch)
        except ValueError:
            return None
        if enc == "bgr8":
            return img
        if enc == "rgb8":
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if enc == "mono8":
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        LOG.warning("[%s] unsupported raw encoding %s", self.name, msg.encoding)
        return None

    def _to_pipe_bytes(self, img: np.ndarray) -> bytes:
        """BGR frame → the bytes ffmpeg expects on stdin. yuv420p (planar I420)
        is converted here with cv2 so ffmpeg needs no libavfilter `scale`
        (absent on the Jetson ffmpeg package); the encoders take it as is."""
        if self._pix_fmt_in == "yuv420p":
            return cv2.cvtColor(img, cv2.COLOR_BGR2YUV_I420).tobytes()
        return np.ascontiguousarray(img).tobytes()        # bgr24

    def _on_image(self, msg) -> None:
        """rclpy callback: enqueue only. Encoding happens on the worker
        thread — a blocking callback starves every other subscription
        (single executor thread), which is how one slow camera dropped the
        others to a few frames per minute in E2E."""
        if self._stopped:
            return
        if not self._queue.put(msg, len(msg.data)):
            self.dropped_frames += 1

    def _drain(self) -> None:
        while True:
            item = self._queue.get()
            if item is None:
                return
            try:
                self._process(*item)
            except Exception as exc:  # keep the worker alive
                LOG.warning("[%s] frame processing failed: %s", self.name, exc)
            if self._queue.warming:
                _maybe_end_warmup(self.name, self._queue, self._ffmpeg, self._segment_file)

    def _process(self, msg, recv_ns: int) -> None:
        with self._lock:
            if self._stopped:
                return
            img = self._decode_frame(msg)
            if img is None:
                LOG.warning("[%s] decode failed, skipping frame", self.name)
                return
            h, w = img.shape[:2]
            if (h & 1) or (w & 1):
                img = img[: h & ~1, : w & ~1]          # 4:2:0 needs even dimensions
                h, w = img.shape[:2]
            if self._ffmpeg is None:
                spawned = _spawn_ffmpeg_encoder(self._segment_file, w, h, self.fps_nominal)
                if spawned is None:
                    LOG.error("[%s] no H.264 encoder available — frame dropped", self.name)
                    return
                self._ffmpeg, self._codec_in_use, self._pix_fmt_in = spawned
                self.width = w
                self.height = h
                LOG.info("[%s] first frame %dx%d → %s (%s, %s in)",
                         self.name, w, h, self._segment_file, self._codec_in_use, self._pix_fmt_in)

            try:
                self._ffmpeg.stdin.write(self._to_pipe_bytes(img))
            except (BrokenPipeError, ValueError) as exc:
                LOG.warning("[%s] ffmpeg pipe write failed: %s", self.name, exc)
                return

            ros_stamp_ns = (
                msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                if msg.header.stamp.sec or msg.header.stamp.nanosec else recv_ns
            )
            self._sidecar.append({
                "frame_index": self._frame_index,
                "segment_file": self._segment_file.name,
                "segment_local_frame": self._segment_local,
                "ros_stamp_ns": ros_stamp_ns,
                "recv_stamp_ns": recv_ns,
                "source_seq": -1,
                "dropped_before": 0,  # naive (Phase 1.5 fills with seq gap detection)
                "keyframe": True,  # mp4v: every frame is intra-coded; refine later
            })
            self._frame_index += 1
            self._segment_local += 1
            self.frame_count += 1
            self._last_recv_ns = recv_ns
            self._first_frame_received = True

    def stop(self) -> dict:
        if self._sub is not None and self._node is not None:
            sub, self._sub = self._sub, None
            self.unsubscribe = _unsubscribe(self._node, [sub], self.name)
        # let queued frames finish, then wake the worker with a sentinel
        self._queue.close()
        self._worker.join(timeout=20)
        if self._worker.is_alive():
            # (a frame the encode thread has already taken but not written is
            # not counted — this only happens after a 20 s stall)
            lost = self._queue.drop_pending()
            self.dropped_frames += lost
            LOG.warning("[%s] encode thread still busy at stop: %d queued frames not written",
                        self.name, lost)
        with self._lock:
            self._stopped = True
            if self._ffmpeg is not None:
                try:
                    self._ffmpeg.stdin.close()
                except Exception:
                    pass
                try:
                    self._ffmpeg.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    LOG.warning("[%s] ffmpeg flush timeout, killing", self.name)
                    self._ffmpeg.kill()
                    self._ffmpeg.wait(timeout=3)
                self._ffmpeg = None
            self._sidecar.close()
            elapsed_s = max((self._last_recv_ns - self._start_wall_ns) / 1e9, 1e-6)
            self.fps_actual = self.frame_count / elapsed_s if self.frame_count else 0.0
        if self.dropped_frames:
            LOG.warning("[%s] %d frames dropped at the ingest queue",
                        self.name, self.dropped_frames)
        return self.summary()

    def summary(self) -> dict:
        size_bytes = self._segment_file.stat().st_size if self._segment_file.exists() else 0
        return self._with_camera_info({
            "name": self.name,
            "topic": self.topic,
            "dropped_frames": self.dropped_frames,
            "width": self.width,
            "height": self.height,
            "fps_nominal": self.fps_nominal,
            "fps_actual": round(self.fps_actual, 2),
            "frame_count": self.frame_count,
            "video_dir": str(self.output_dir.name) + "/",
            "sidecar_file": f"{self.output_dir.name}/frames.parquet",
            "segments": [
                {
                    "file": self._segment_file.name,
                    "frame_count": self.frame_count,
                    "size_bytes": size_bytes,
                }
            ],
        })


class DepthCameraWriter(_CameraInfoMixin):
    """16-bit depth (sensor_msgs/Image, encoding 16UC1) → PNG sequence sidecar.

    mp4 can't carry 16-bit single-channel losslessly without exotic codecs,
    so each frame goes to videos/<name>/<frame:06d>.png (16-bit grayscale,
    cv2.imwrite handles this natively). frames.parquet still indexes them
    so the bag + sidecar workflow stays consistent. The play modal shows
    a placeholder card pointing at the download link — no live preview."""

    def __init__(self, name: str, topic: str, output_dir: Path, fps: int = 30,
                 node: Optional[Node] = None):
        self.name = name
        self.topic = topic
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.fps_nominal = fps
        self._node = node
        self._sidecar = FramesSidecar(self.output_dir / "frames.parquet")
        self._sub = None
        self.unsubscribe: Optional[str] = None
        self._lock = threading.Lock()
        self._frame_index = 0
        self._last_recv_ns = 0
        self._start_wall_ns = 0
        self._stopped = False
        self.frame_count = 0
        self.width = 0
        self.height = 0
        self.fps_actual = 0.0
        self._encoding_seen = ""

    def start(self) -> None:
        if self._node is None:
            raise RuntimeError("DepthCameraWriter requires a rclpy Node")
        self._cbg = MutuallyExclusiveCallbackGroup()
        self._sub = self._node.create_subscription(
            Image, self.topic, self._on_image, qos_profile_sensor_data,
            callback_group=self._cbg,
        )
        self._start_wall_ns = time.time_ns()
        LOG.info("depth writer started: %s topic=%s out=%s",
                 self.name, self.topic, self.output_dir)

    def _on_image(self, msg: Image) -> None:
        if self._stopped:
            return
        with self._lock:
            if self._stopped:  # stop() ran while this callback waited for the lock
                return
            recv_ns = time.time_ns()
            self._encoding_seen = msg.encoding
            # Depth conventions: 16UC1 (mm, RealSense) or mono16 (also 16-bit).
            # 32FC1 (m, float) is converted to mm so we get a fixed-range PNG.
            try:
                if msg.encoding in ("16UC1", "mono16"):
                    arr = np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape(msg.height, msg.width)
                elif msg.encoding == "32FC1":
                    f = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.height, msg.width)
                    arr = np.nan_to_num(f * 1000.0, nan=0.0, posinf=0.0, neginf=0.0).astype(np.uint16)
                else:
                    LOG.warning("[%s] unsupported depth encoding %s, skipping",
                                self.name, msg.encoding)
                    return
            except ValueError as exc:
                LOG.warning("[%s] depth reshape failed: %s", self.name, exc)
                return
            png_path = self.output_dir / f"{self._frame_index:06d}.png"
            if not cv2.imwrite(str(png_path), arr):
                LOG.warning("[%s] cv2.imwrite failed for %s", self.name, png_path)
                return
            self.width = msg.width
            self.height = msg.height
            ros_stamp_ns = (
                msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                if msg.header.stamp.sec or msg.header.stamp.nanosec else recv_ns
            )
            self._sidecar.append({
                "frame_index": self._frame_index,
                "segment_file": png_path.name,
                "segment_local_frame": self._frame_index,
                "ros_stamp_ns": ros_stamp_ns,
                "recv_stamp_ns": recv_ns,
                "source_seq": -1,
                "dropped_before": 0,
                "keyframe": True,
            })
            self._frame_index += 1
            self.frame_count += 1
            self._last_recv_ns = recv_ns

    def stop(self) -> dict:
        # Only flag + detach under the lock: _on_image takes the same lock on
        # the executor thread, so waiting for the executor while holding it
        # would stall every callback until the destroy timed out.
        with self._lock:
            self._stopped = True
            sub, self._sub = self._sub, None
        if sub is not None and self._node is not None:
            self.unsubscribe = _unsubscribe(self._node, [sub], self.name)
        with self._lock:
            self._sidecar.close()
            elapsed_s = max((self._last_recv_ns - self._start_wall_ns) / 1e9, 1e-6)
            self.fps_actual = self.frame_count / elapsed_s if self.frame_count else 0.0
        return self.summary()

    def summary(self) -> dict:
        # No single segment file — depth is a PNG sequence. Surface that so
        # the play modal can show a download-only placeholder instead of
        # trying to embed a <video>.
        total_bytes = 0
        try:
            for p in self.output_dir.glob("*.png"):
                total_bytes += p.stat().st_size
        except OSError:
            pass
        return self._with_camera_info({
            "name": self.name,
            "topic": self.topic,
            "kind": "depth_png_seq",
            "encoding": self._encoding_seen,
            "width": self.width,
            "height": self.height,
            "fps_nominal": self.fps_nominal,
            "fps_actual": round(self.fps_actual, 2),
            "frame_count": self.frame_count,
            "video_dir": str(self.output_dir.name) + "/",
            "sidecar_file": f"{self.output_dir.name}/frames.parquet",
            "segments": [],  # PNG sequence — listed via /files/ if needed
            "total_png_bytes": total_bytes,
        })


class DepthVideoWriter(_CameraInfoMixin):
    """16-bit depth → 10-bit-luma HEVC Main10 video (lossy, ~30-60× smaller
    than the compressedDepth PNG bag path).

    Inverse-depth quantization to the Y plane (chroma flat) via
    fv_episode_recorder.depth10; encoded with hevc_nvenc (Tegra HW) or
    libx265 fallback. d_min/d_max are RAW SENSOR COUNTS. Pixels beyond
    d_max become invalid(0) — far IR noise must not decode into a phantom
    wall. depth_meta.json in the camera dir carries everything needed to
    reconstruct 16UC1 (see depth10.decode_video / python3 -m
    fv_episode_recorder.depth10)."""

    def __init__(self, name: str, topic: str, output_dir: Path, fps: int = 30,
                 node: Optional[Node] = None, *, d_min: float, d_max: float,
                 cq: int = 16, depth_scale_mm: float = 1.0):
        self.name = name
        self.topic = topic
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.fps_nominal = fps
        self._node = node
        self.d_min = float(d_min)
        self.d_max = float(d_max)
        self.cq = int(cq)
        self.depth_scale_mm = float(depth_scale_mm)
        self._pack_lut = depth10.build_pack_lut(self.d_min, self.d_max)

        self._segment_file = self.output_dir / "0000.mp4"
        self._sidecar = FramesSidecar(self.output_dir / "frames.parquet")
        self._ffmpeg: Optional[subprocess.Popen] = None
        self._codec_in_use: str = ""
        self._uv_bytes: bytes = b""
        self._sub = None
        self.unsubscribe: Optional[str] = None
        self._lock = threading.Lock()
        self._frame_index = 0
        self._last_recv_ns = 0
        self._start_wall_ns = 0
        self._stopped = False
        self.frame_count = 0
        self.width = 0
        self.height = 0
        self.fps_actual = 0.0
        self._encoding_seen = ""
        self._queue = _IngestQueue(warm_frames=int(fps * _WARMUP_MAX_S))
        self._worker = threading.Thread(
            target=self._drain, name=f"depth10-{name}", daemon=True)
        self.dropped_frames = 0

    def start(self) -> None:
        if self._node is None:
            raise RuntimeError("DepthVideoWriter requires a rclpy Node")
        self._cbg = MutuallyExclusiveCallbackGroup()
        self._sub = self._node.create_subscription(
            Image, self.topic, self._on_image, qos_profile_sensor_data,
            callback_group=self._cbg,
        )
        self._start_wall_ns = time.time_ns()
        self._worker.start()
        LOG.info("depth10 writer started: %s topic=%s d=[%.0f,%.0f]cnt cq=%d out=%s",
                 self.name, self.topic, self.d_min, self.d_max, self.cq, self.output_dir)

    def _to_counts(self, msg: Image) -> Optional[np.ndarray]:
        try:
            if msg.encoding in ("16UC1", "mono16"):
                return np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape(
                    msg.height, msg.width)
            if msg.encoding == "32FC1":
                # float meters → mm counts; d_min/d_max must then be mm
                # (configure depth_scale_mm: 1.0 for such cameras).
                f = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(
                    msg.height, msg.width)
                return np.nan_to_num(f * 1000.0, nan=0.0, posinf=0.0,
                                     neginf=0.0).astype(np.uint16)
        except ValueError as exc:
            LOG.warning("[%s] depth reshape failed: %s", self.name, exc)
            return None
        LOG.warning("[%s] unsupported depth encoding %s, skipping",
                    self.name, msg.encoding)
        return None

    def _on_image(self, msg: Image) -> None:
        """rclpy callback: enqueue only (see CameraWriter._on_image)."""
        if self._stopped:
            return
        if not self._queue.put(msg, len(msg.data)):
            self.dropped_frames += 1

    def _drain(self) -> None:
        while True:
            item = self._queue.get()
            if item is None:
                return
            try:
                self._process(*item)
            except Exception as exc:
                LOG.warning("[%s] frame processing failed: %s", self.name, exc)
            if self._queue.warming:
                _maybe_end_warmup(self.name, self._queue, self._ffmpeg, self._segment_file)

    def _process(self, msg: Image, recv_ns: int) -> None:
        with self._lock:
            if self._stopped:
                return
            self._encoding_seen = msg.encoding
            depth = self._to_counts(msg)
            if depth is None:
                return
            h, w = depth.shape
            if self._ffmpeg is None:
                spawned = _spawn_ffmpeg_hevc10_encoder(
                    self._segment_file, w, h, self.fps_nominal, self.cq)
                if spawned is None:
                    LOG.error("[%s] no HEVC Main10 encoder — frame dropped", self.name)
                    return
                self._ffmpeg, self._codec_in_use = spawned
                self.width, self.height = w, h
                # flat chroma plane (10-bit neutral 512), built once
                self._uv_bytes = np.full((h // 2, w // 2), 512,
                                         dtype="<u2").tobytes() * 2
                depth10.write_meta(
                    self.output_dir, d_min=self.d_min, d_max=self.d_max,
                    depth_scale_mm=self.depth_scale_mm, width=w, height=h,
                    fps=self.fps_nominal, codec=self._codec_in_use,
                    encoding_seen=msg.encoding, video_file=self._segment_file.name)
                LOG.info("[%s] first depth frame %dx%d → %s (%s)",
                         self.name, w, h, self._segment_file, self._codec_in_use)

            y = depth10.pack_luma10(depth, self.d_min, self.d_max, self._pack_lut)
            try:
                self._ffmpeg.stdin.write(y.astype("<u2").tobytes())
                self._ffmpeg.stdin.write(self._uv_bytes)
            except (BrokenPipeError, ValueError) as exc:
                LOG.warning("[%s] ffmpeg pipe write failed: %s", self.name, exc)
                return

            ros_stamp_ns = (
                msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                if msg.header.stamp.sec or msg.header.stamp.nanosec else recv_ns
            )
            self._sidecar.append({
                "frame_index": self._frame_index,
                "segment_file": self._segment_file.name,
                "segment_local_frame": self._frame_index,
                "ros_stamp_ns": ros_stamp_ns,
                "recv_stamp_ns": recv_ns,
                "source_seq": -1,
                "dropped_before": 0,
                "keyframe": False,
            })
            self._frame_index += 1
            self.frame_count += 1
            self._last_recv_ns = recv_ns

    def stop(self) -> dict:
        if self._sub is not None and self._node is not None:
            sub, self._sub = self._sub, None
            self.unsubscribe = _unsubscribe(self._node, [sub], self.name)
        self._queue.close()
        self._worker.join(timeout=20)
        if self._worker.is_alive():
            # (a frame the encode thread has already taken but not written is
            # not counted — this only happens after a 20 s stall)
            lost = self._queue.drop_pending()
            self.dropped_frames += lost
            LOG.warning("[%s] encode thread still busy at stop: %d queued frames not written",
                        self.name, lost)
        with self._lock:
            self._stopped = True
            if self._ffmpeg is not None:
                try:
                    self._ffmpeg.stdin.close()
                except Exception:
                    pass
                try:
                    self._ffmpeg.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    LOG.warning("[%s] ffmpeg flush timeout, killing", self.name)
                    self._ffmpeg.kill()
                    self._ffmpeg.wait(timeout=3)
                self._ffmpeg = None
            self._sidecar.close()
            elapsed_s = max((self._last_recv_ns - self._start_wall_ns) / 1e9, 1e-6)
            self.fps_actual = self.frame_count / elapsed_s if self.frame_count else 0.0
        if self.dropped_frames:
            LOG.warning("[%s] %d frames dropped at the ingest queue",
                        self.name, self.dropped_frames)
        return self.summary()

    def summary(self) -> dict:
        size_bytes = self._segment_file.stat().st_size if self._segment_file.exists() else 0
        return self._with_camera_info({
            "name": self.name,
            "topic": self.topic,
            "dropped_frames": self.dropped_frames,
            "kind": "depth_hevc10",
            "encoding": self._encoding_seen,
            "codec": self._codec_in_use,
            "d_min_counts": self.d_min,
            "d_max_counts": self.d_max,
            "depth_scale_mm_per_count": self.depth_scale_mm,
            "width": self.width,
            "height": self.height,
            "fps_nominal": self.fps_nominal,
            "fps_actual": round(self.fps_actual, 2),
            "frame_count": self.frame_count,
            "video_dir": str(self.output_dir.name) + "/",
            "sidecar_file": f"{self.output_dir.name}/frames.parquet",
            "meta_file": f"{self.output_dir.name}/{depth10.META_FILENAME}",
            "segments": [
                {
                    "file": self._segment_file.name,
                    "frame_count": self.frame_count,
                    "size_bytes": size_bytes,
                }
            ],
        })


class CameraWriterPool:
    """Manage multiple CameraWriter instances for one episode."""

    def __init__(self, node: Node, warm_up: bool = True):
        self._node = node
        self._writers: dict[str, CameraWriter | DepthCameraWriter | DepthVideoWriter] = {}
        if warm_up:
            # pick the depth encoder now, not on the first episode's first frames
            threading.Thread(target=warm_encoders, name="encoder-warmup", daemon=True).start()
        # Depth cameras don't have a writer (they go into the bag) but the
        # play modal still needs them in cameras[]. Stored here so stop_all
        # can re-emit the placeholders alongside the real writers' summaries.
        self._depth_placeholders: list[dict] = []
        # camera_info_topic subscriptions (one per camera that declares it);
        # torn down in stop_all.
        self._info_subs: list = []
        self.teardown_pending: list[str] = []   # see stop_all

    def _subscribe_camera_info(self, writer: _CameraInfoMixin, topic: str) -> None:
        """Hand the first CameraInfo on ``topic`` to ``writer``. Best effort:
        any failure is logged and the recording continues without it."""
        try:
            from sensor_msgs.msg import CameraInfo

            def _cb(msg, _w=writer) -> None:
                if _w.camera_info is None:
                    try:
                        _w.set_camera_info(camera_info_to_dict(msg))
                    except Exception as exc:  # noqa: BLE001
                        LOG.warning("camera_info convert failed (%s): %s", topic, exc)

            # sensor_data (best effort) also matches reliable publishers and
            # the usual latched-less camera_info cadence (= image rate).
            sub = self._node.create_subscription(
                CameraInfo, topic, _cb, qos_profile_sensor_data)
            self._info_subs.append(sub)
            writer.camera_info_topic = topic
        except Exception as exc:  # noqa: BLE001
            LOG.warning("camera_info subscribe failed (%s): %s", topic, exc)

    def _destroy_info_subs(self) -> str:
        subs, self._info_subs = self._info_subs, []
        if not subs:
            return "none"
        return _unsubscribe(self._node, subs, "camera_info")

    def start_all(self, episode_dir: Path, cameras: list[dict], fps: int = 30) -> list[dict]:
        """cameras: [{name, topic, kind?, codec?}, ...]. Color cameras get a
        CameraWriter (H.264 mp4); depth cameras (kind="depth") are now
        recorded into the rosbag2 sqlite instead — this method just emits a
        placeholder summary so the play modal still knows the depth track
        exists (preview reads from bag via /depth_preview endpoint). The
        topic is appended to bag_topics by topic_discovery."""
        videos_root = episode_dir / "videos"
        videos_root.mkdir(exist_ok=True)
        self._depth_placeholders = []
        for cam in cameras:
            name = cam["name"]
            topic = cam["topic"]
            kind = (cam.get("kind") or "color").lower()
            cam_fps = int(cam.get("fps") or fps)
            if kind == "depth_lossy":
                # 10-bit-luma HEVC Main10 video — NOT bagged (topic_discovery
                # only bags kind=="depth"). d_min/d_max are mandatory: a
                # wrong default range silently ruins precision, so fail loud.
                if cam.get("d_min") is None or cam.get("d_max") is None:
                    raise ValueError(
                        f"camera '{name}': kind=depth_lossy requires d_min/d_max "
                        f"(raw sensor counts, e.g. D555 400/4000, D405 700/15000)")
                writer = DepthVideoWriter(
                    name=name, topic=topic, output_dir=videos_root / name,
                    fps=cam_fps, node=self._node,
                    d_min=float(cam["d_min"]), d_max=float(cam["d_max"]),
                    cq=int(cam.get("cq") or 16),
                    depth_scale_mm=float(cam.get("depth_scale_mm") or 1.0),
                )
                writer.start()
                self._writers[name] = writer
                if cam.get("camera_info_topic"):
                    self._subscribe_camera_info(writer, str(cam["camera_info_topic"]))
                continue
            if kind == "depth":
                # No on-disk writer — depth lives in the bag. Synthesize a
                # summary entry so the UI shows the depth thumbnail and can
                # pull preview frames from /depth_preview/{cam}/{frame_idx}.
                self._depth_placeholders.append({
                    "name": name,
                    "topic": topic,
                    "kind": "depth_bag",
                    "video_dir": None,
                    "sidecar_file": None,
                    "segments": [],
                    "frame_count": 0,         # filled in at stop from bag
                    "width": 0, "height": 0,
                    "fps_actual": 0.0,
                    "fps_nominal": fps,
                })
                continue
            cam_dir = videos_root / name
            writer = CameraWriter(
                name=name, topic=topic, output_dir=cam_dir, fps=cam_fps,
                node=self._node, raw_image=bool(cam.get("raw")),
            )
            writer.start()
            self._writers[name] = writer
            if cam.get("camera_info_topic"):
                self._subscribe_camera_info(writer, str(cam["camera_info_topic"]))
        return [w.summary() for w in self._writers.values()] + self._depth_placeholders

    def apply_depth_frame_counts(self, counts: dict[str, int]) -> None:
        """Patch the depth_bag placeholder summaries with the frame counts
        sampled from the DepthRepublisher pool before its teardown. Without
        this the play modal sees frame_count=0 and renders '0 frames' even
        though the bag contains them. Idempotent — unknown names are ignored
        so a profile that wires depth differently doesn't crash here."""
        for ph in self._depth_placeholders:
            name = ph.get("name")
            if name in counts:
                ph["frame_count"] = int(counts[name])

    def stop_all(self) -> list[dict]:
        pending = ["camera_info"] if self._destroy_info_subs() == "pending" else []
        summaries = []
        for name, writer in list(self._writers.items()):
            summaries.append(writer.stop())
            if writer.unsubscribe == "pending":
                pending.append(name)
        self._writers.clear()
        # subscriptions the executor had not destroyed yet when stop gave up
        # waiting (a stop failure; the executor still runs them later)
        self.teardown_pending = pending
        # Re-emit depth placeholders so meta.cameras keeps them.
        out = summaries + list(self._depth_placeholders)
        self._depth_placeholders = []
        return out

    def is_active(self) -> bool:
        return bool(self._writers)

    def frame_counts(self) -> dict[str, int]:
        return {name: w.frame_count for name, w in self._writers.items()}
