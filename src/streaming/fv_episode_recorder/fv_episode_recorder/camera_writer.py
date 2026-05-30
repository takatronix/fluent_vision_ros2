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

        self._writer: Optional[cv2.VideoWriter] = None
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
            if self._writer is None:
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                self._writer = cv2.VideoWriter(
                    str(self._segment_file), fourcc, float(self.fps_nominal), (w, h)
                )
                if not self._writer.isOpened():
                    LOG.error("[%s] cv2.VideoWriter open failed", self.name)
                    self._writer = None
                    return
                self.width = w
                self.height = h
                LOG.info("[%s] first frame %dx%d → %s", self.name, w, h, self._segment_file)

            self._writer.write(img)

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
            if self._writer is not None:
                self._writer.release()
                self._writer = None
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
