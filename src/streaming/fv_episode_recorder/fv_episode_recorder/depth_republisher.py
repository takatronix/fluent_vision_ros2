"""Per-episode depth republisher — subscribes to raw 16-bit depth Image
and publishes CompressedImage at <topic>_compressed/compressedDepth in the
format compressed_depth_image_transport expects (12-byte ConfigHeader +
PNG payload). Lossless rvl/png ~5-10× compression, no temporal subsample.

We do this in-process (rclpy) rather than via `ros2 run image_transport
republish` because the CLI tool's `-r out:=…` remap doesn't work the way
docs suggest (topic comes out as /out/compressedDepth regardless).

**Fail-loud policy**: republisher creation failures abort the episode
start rather than silently bagging raw depth.
"""

from __future__ import annotations

import logging
import struct
import threading
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

LOG = logging.getLogger("fv_episode_recorder.depth_republish")


# compressed_depth_image_transport ConfigHeader layout:
#   int32 format      (0 = INV_DEPTH, 1 = PNG, 2 = RVL)
#   float32 depth_quantization (alpha for INV_DEPTH; ignored for raw PNG)
#   float32 depth_max  (for INV_DEPTH; encoded distance cap)
#
# For lossless 16UC1 PNG: format=1, quantization=0, max=0. The subscriber
# plugin handles this case by directly decoding the PNG payload as uint16.
_CONFIG_HEADER = struct.pack("<ifi", 1, 0.0, 0)  # format=1 (PNG), quant=0, max=0


@dataclass
class _DepthRepublisher:
    name: str
    raw_topic: str
    node: Node
    sub: object = None
    pub: object = None
    _frame_count: int = 0
    _lock: threading.Lock = field(default_factory=threading.Lock)
    _stopped: bool = False

    @property
    def compressed_topic(self) -> str:
        return f"{self.raw_topic.rstrip('/')}_compressed/compressedDepth"

    def start(self) -> None:
        # Publisher first so the bag recorder's subscriber sees a publisher
        # at subscribe time (cleaner discovery, no QoS warnings).
        self.pub = self.node.create_publisher(
            CompressedImage, self.compressed_topic, qos_profile_sensor_data,
        )
        self.sub = self.node.create_subscription(
            Image, self.raw_topic, self._on_image, qos_profile_sensor_data,
        )
        LOG.info("depth republish: %s → %s", self.raw_topic, self.compressed_topic)

    def request_stop(self) -> int:
        with self._lock:
            self._stopped = True
            return self._frame_count

    def stop(self) -> None:
        self.request_stop()
        failures: list[str] = []
        with self._lock:
            sub = self.sub
            pub = self.pub
        if sub is not None:
            try:
                self.node.destroy_subscription(sub)
            except Exception as exc:
                failures.append(f"subscription: {exc}")
            else:
                with self._lock:
                    if self.sub is sub:
                        self.sub = None
        if pub is not None:
            try:
                self.node.destroy_publisher(pub)
            except Exception as exc:
                failures.append(f"publisher: {exc}")
            else:
                with self._lock:
                    if self.pub is pub:
                        self.pub = None
        with self._lock:
            frame_count = self._frame_count
        LOG.info("depth republish stopped: %s (frames=%d)", self.raw_topic, frame_count)
        if failures:
            raise RuntimeError(
                f"depth republisher stop failed for {self.raw_topic}: "
                + "; ".join(failures)
            )

    def _on_image(self, msg: Image) -> None:
        with self._lock:
            if self._stopped or self.pub is None:
                return
        try:
            if msg.encoding in ("16UC1", "mono16"):
                arr = np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape(msg.height, msg.width)
            elif msg.encoding == "32FC1":
                f = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.height, msg.width)
                arr = np.nan_to_num(f * 1000.0, nan=0.0, posinf=0.0, neginf=0.0).astype(np.uint16)
            else:
                return
        except ValueError:
            return
        # Encode as 16-bit PNG. cv2.imencode handles uint16 grayscale lossless.
        ok, png = cv2.imencode(".png", arr, [cv2.IMWRITE_PNG_COMPRESSION, 1])
        if not ok:
            return
        out = CompressedImage()
        out.header = msg.header
        out.format = "16UC1; compressedDepth png"
        out.data = _CONFIG_HEADER + png.tobytes()
        try:
            with self._lock:
                if self._stopped or self.pub is None:
                    return
                self.pub.publish(out)
                self._frame_count += 1
        except Exception as exc:
            LOG.warning("depth republish publish failed (%s): %s", self.raw_topic, exc)


class DepthRepublisherPool:
    """Manages one in-process republisher per depth camera in the active episode."""

    def __init__(self, node: Node):
        self._node = node
        self._pubs: list[_DepthRepublisher] = []

    def start_all(self, depth_cameras: list[dict]) -> list[dict]:
        """depth_cameras: [{name, topic, ...}, ...] — only kind=depth entries.
        Returns: [{name, raw_topic, compressed_topic}, ...] for diagnostics."""
        out: list[dict] = []
        for cam in depth_cameras:
            raw_topic = cam.get("topic")
            if not raw_topic:
                continue
            r = _DepthRepublisher(name=cam.get("name") or raw_topic,
                                  raw_topic=raw_topic, node=self._node)
            r.start()
            self._pubs.append(r)
            out.append({
                "name": r.name,
                "raw_topic": r.raw_topic,
                "compressed_topic": r.compressed_topic,
            })
        return out

    def request_stop(self) -> dict[str, int]:
        """Stop accepting depth frames without destroying ROS entities."""
        return {republisher.name: republisher.request_stop() for republisher in self._pubs}

    def stop_all(self) -> dict[str, int]:
        """Tear down all republishers and return the per-camera frame count
        (republisher's internal counter, sampled before destruction so the
        camera_pool can write it into the depth placeholder summary).
        Without this the play modal sees frame_count=0 and renders the
        '0 frames' empty-state even though the bag has the frames."""
        counts: dict[str, int] = {}
        failures: list[str] = []
        remaining: list[_DepthRepublisher] = []
        for r in self._pubs:
            counts[r.name] = r._frame_count
            try:
                r.stop()
            except Exception as exc:
                failures.append(str(exc))
                remaining.append(r)
        self._pubs = remaining
        if failures:
            raise RuntimeError("; ".join(failures))
        return counts

    def has_pending_cleanup(self) -> bool:
        return bool(self._pubs)
