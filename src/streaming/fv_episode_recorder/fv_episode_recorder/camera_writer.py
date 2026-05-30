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
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from .frames_sidecar import FramesSidecar

LOG = logging.getLogger("fv_episode_recorder.camera")


def _spawn_ffmpeg_encoder(out_path: Path, width: int, height: int, fps: int) -> tuple[subprocess.Popen, str] | None:
    """Try ffmpeg with H.264 encoders in priority order. Returns (proc, codec_used)
    or None if all encoders failed. Output is faststart mp4 for HTML5 video.

    Priority: h264_nvenc (Tegra HW encode) → libx264 (CPU). Both produce
    browser-native H.264 mp4 (Chrome / Safari / Firefox HW decode)."""
    if not shutil.which("ffmpeg"):
        LOG.error("ffmpeg not found in PATH")
        return None
    encoder_tries = [
        # (codec, extra args)
        ("h264_nvenc", ["-preset", "p1", "-tune", "ll"]),    # Tegra HW, fastest
        ("libx264",    ["-preset", "ultrafast", "-tune", "zerolatency"]),
    ]
    for codec, extra in encoder_tries:
        cmd = [
            "ffmpeg", "-y", "-loglevel", "error",
            "-f", "rawvideo", "-pix_fmt", "bgr24",
            "-s", f"{width}x{height}", "-r", str(fps),
            "-i", "pipe:0",
            "-c:v", codec,
            *extra,
            "-pix_fmt", "yuv420p",
            "-movflags", "+faststart",
            str(out_path),
        ]
        try:
            # Probe encoder availability first (some builds list h264_nvenc but
            # fail at runtime if no GPU). Spawn a tiny test pipe.
            test = subprocess.run(
                ["ffmpeg", "-hide_banner", "-loglevel", "error",
                 "-f", "lavfi", "-i", "color=c=black:s=64x64:d=0.04:r=25",
                 "-c:v", codec, *extra, "-f", "null", "-"],
                stderr=subprocess.PIPE, timeout=10,
            )
            if test.returncode != 0:
                LOG.warning("[encoder probe] %s unavailable: %s", codec,
                            test.stderr.decode("utf-8", errors="replace")[:200])
                continue
        except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
            LOG.warning("[encoder probe] %s probe failed: %s", codec, exc)
            continue
        # Launch the real encoder pipe.
        try:
            proc = subprocess.Popen(
                cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
            )
            LOG.info("ffmpeg encoder ready: %s → %s (%dx%d @ %dfps)",
                     codec, out_path.name, width, height, fps)
            return proc, codec
        except Exception as exc:
            LOG.warning("[encoder] %s spawn failed: %s", codec, exc)
            continue
    LOG.error("no H.264 encoder available (tried %s)", [c for c, _ in encoder_tries])
    return None


class CameraWriter:
    """One worker per camera: subscribes, encodes, writes mp4 + sidecar."""

    def __init__(
        self,
        name: str,
        topic: str,
        output_dir: Path,
        fps: int = 30,
        node: Optional[Node] = None,
    ):
        self.name = name
        self.topic = topic
        self.output_dir = Path(output_dir)  # videos/<name>/
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.fps_nominal = fps
        self._node = node

        self._segment_file = self.output_dir / "0000.mp4"
        self._sidecar = FramesSidecar(self.output_dir / "frames.parquet")

        # ffmpeg subprocess (H.264 encoder) replaces cv2.VideoWriter which
        # only had MPEG-4 part 2 — that codec is not browser-playable.
        self._ffmpeg: Optional[subprocess.Popen] = None
        self._codec_in_use: str = ""
        self._frame_index = 0
        self._segment_local = 0
        self._last_recv_ns = 0
        self._lock = threading.Lock()
        self._sub = None
        self._first_frame_received = False
        self.frame_count = 0
        self.width = 0
        self.height = 0
        self.fps_actual = 0.0
        self._start_wall_ns = 0
        self._stopped = False

    def start(self) -> None:
        if self._node is None:
            raise RuntimeError("CameraWriter requires a rclpy Node to attach subscriber")
        self._sub = self._node.create_subscription(
            CompressedImage,
            self.topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self._start_wall_ns = time.time_ns()
        LOG.info("camera writer started: %s topic=%s out=%s", self.name, self.topic, self.output_dir)

    def _on_image(self, msg: CompressedImage) -> None:
        if self._stopped:
            return
        with self._lock:
            recv_ns = time.time_ns()
            # decode jpeg/png compressed bytes
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is None:
                LOG.warning("[%s] decode failed, skipping frame", self.name)
                return
            h, w = img.shape[:2]
            if self._ffmpeg is None:
                spawned = _spawn_ffmpeg_encoder(self._segment_file, w, h, self.fps_nominal)
                if spawned is None:
                    LOG.error("[%s] no H.264 encoder available — frame dropped", self.name)
                    return
                self._ffmpeg, self._codec_in_use = spawned
                self.width = w
                self.height = h
                LOG.info("[%s] first frame %dx%d → %s (%s)",
                         self.name, w, h, self._segment_file, self._codec_in_use)

            try:
                self._ffmpeg.stdin.write(img.tobytes())
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
        with self._lock:
            self._stopped = True
            if self._sub is not None and self._node is not None:
                try:
                    self._node.destroy_subscription(self._sub)
                except Exception:
                    pass
                self._sub = None
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
        return self.summary()

    def summary(self) -> dict:
        size_bytes = self._segment_file.stat().st_size if self._segment_file.exists() else 0
        return {
            "name": self.name,
            "topic": self.topic,
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
        }


class CameraWriterPool:
    """Manage multiple CameraWriter instances for one episode."""

    def __init__(self, node: Node):
        self._node = node
        self._writers: dict[str, CameraWriter] = {}

    def start_all(self, episode_dir: Path, cameras: list[dict], fps: int = 30) -> list[dict]:
        """cameras: [{name, topic, codec?}, ...]. Returns initial summary list."""
        videos_root = episode_dir / "videos"
        videos_root.mkdir(exist_ok=True)
        for cam in cameras:
            name = cam["name"]
            topic = cam["topic"]
            cam_dir = videos_root / name
            writer = CameraWriter(name=name, topic=topic, output_dir=cam_dir, fps=fps, node=self._node)
            writer.start()
            self._writers[name] = writer
        return [w.summary() for w in self._writers.values()]

    def stop_all(self) -> list[dict]:
        summaries = []
        for name, writer in list(self._writers.items()):
            summaries.append(writer.stop())
        self._writers.clear()
        return summaries

    def is_active(self) -> bool:
        return bool(self._writers)

    def frame_counts(self) -> dict[str, int]:
        return {name: w.frame_count for name, w in self._writers.items()}
