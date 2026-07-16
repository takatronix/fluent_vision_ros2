"""Per-camera writer: subscribes to a camera topic, writes mp4 + frames.parquet.

CompressedImage callbacks only enqueue immutable message data. A per-camera
worker decodes and persists frames, then finalizes one timestamped VFR MP4.
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

from .frames_sidecar import FramesSidecar
from .vfr_video import EncoderConfig, VfrMp4Writer, probe_h264_encoder

LOG = logging.getLogger("fv_episode_recorder.camera")


@dataclass(frozen=True)
class _QueuedFrame:
    data: bytes
    ros_stamp_ns: int
    recv_stamp_ns: int


class _Stop:
    pass


_STOP = _Stop()
_CAMERA_WRITE_TICK_PREFIX = "/fv_episode_recorder/write_tick/camera"


def _time_message(timestamp_ns: int) -> Time:
    message = Time()
    message.sec, message.nanosec = divmod(timestamp_ns, 1_000_000_000)
    return message


class CameraWriter:
    """One worker per camera: subscribes, encodes, writes mp4 + sidecar."""

    def __init__(
        self,
        name: str,
        topic: str,
        output_dir: Path,
        fps: int = 30,
        node: Optional[Node] = None,
        encoder: Optional[EncoderConfig] = None,
        queue_size: int = 120,
    ):
        self.name = name
        self.topic = topic
        self.output_dir = Path(output_dir)  # videos/<name>/
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.fps_nominal = fps
        self._node = node
        if encoder is None:
            raise RuntimeError("CameraWriter requires a probed encoder")

        self._segment_file = self.output_dir / "0000.mp4"
        self._sidecar = FramesSidecar(self.output_dir / "frames.parquet")

        self._video = VfrMp4Writer(self._segment_file, fps, encoder)
        self._queue: queue.Queue[_QueuedFrame | _Stop] = queue.Queue(maxsize=queue_size)
        self._worker: Optional[threading.Thread] = None
        self._failure: Optional[RuntimeError] = None
        self._frame_index = 0
        self._segment_local = 0
        self._last_recv_ns = 0
        self._lock = threading.Lock()
        self._sub = None
        self._tick_publisher = None
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
        self._worker = threading.Thread(
            target=self._run_worker, name=f"camera-writer-{self.name}", daemon=True,
        )
        self._worker.start()
        try:
            self._tick_publisher = self._node.create_publisher(
                Time,
                f"{_CAMERA_WRITE_TICK_PREFIX}/{self.name}",
                10,
            )
            self._sub = self._node.create_subscription(
                CompressedImage,
                self.topic,
                self._on_image,
                qos_profile_sensor_data,
            )
        except Exception:
            self._destroy_tick_publisher()
            self._queue.put(_STOP)
            self._worker.join()
            self._worker = None
            raise
        self._start_wall_ns = time.time_ns()
        LOG.info("camera writer started: %s topic=%s out=%s", self.name, self.topic, self.output_dir)

    def _on_image(self, msg: CompressedImage) -> None:
        with self._lock:
            if self._failure is not None:
                return
            if self._stopped:
                return
            recv_ns = time.time_ns()
            ros_stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            queued = _QueuedFrame(bytes(msg.data), ros_stamp_ns, recv_ns)
            try:
                self._queue.put_nowait(queued)
            except queue.Full:
                self._set_failure_locked(
                    RuntimeError(f"[{self.name}] camera frame queue overflow")
                )

    def _run_worker(self) -> None:
        try:
            while True:
                item = self._queue.get()
                if item is _STOP:
                    break
                if not isinstance(item, _QueuedFrame):
                    raise RuntimeError("invalid camera worker queue item")
                frame = self._video.append(item.data, item.ros_stamp_ns)
                self._sidecar.append(
                    {
                        "frame_index": self._frame_index,
                        "segment_file": self._segment_file.name,
                        "segment_local_frame": self._segment_local,
                        "ros_stamp_ns": item.ros_stamp_ns,
                        "video_pts": frame.video_pts,
                        "recv_stamp_ns": item.recv_stamp_ns,
                        "source_seq": -1,
                        "dropped_before": 0,
                        "keyframe": True,
                    }
                )
                if self._tick_publisher is None:
                    raise RuntimeError(f"[{self.name}] write tick publisher is unavailable")
                self._tick_publisher.publish(_time_message(time.time_ns()))
                with self._lock:
                    self._frame_index += 1
                    self._segment_local += 1
                    self.frame_count += 1
                    self._last_recv_ns = item.recv_stamp_ns
                    self._first_frame_received = True
                    self.width = self._video.width
                    self.height = self._video.height
                    failure = self._failure
                if failure is not None:
                    raise failure
            with self._lock:
                failure = self._failure
            if failure is not None:
                raise failure
            if self.frame_count == 0:
                raise RuntimeError(f"[{self.name}] no camera frames were recorded")
            self._video.close()
        except Exception as exc:
            try:
                self._video.abort()
            except Exception as cleanup_exc:
                exc = RuntimeError(f"{exc}; video cleanup failed: {cleanup_exc}")
            self._set_failure(RuntimeError(f"[{self.name}] camera worker failed: {exc}"))
        finally:
            try:
                self._sidecar.close()
            except Exception as exc:
                self._set_failure(RuntimeError(f"[{self.name}] sidecar close failed: {exc}"))

    def _set_failure(self, failure: RuntimeError) -> None:
        with self._lock:
            self._set_failure_locked(failure)

    def _set_failure_locked(self, failure: RuntimeError) -> None:
        if self._failure is None:
            self._failure = failure
            LOG.error("%s", failure)

    def request_stop(self) -> None:
        with self._lock:
            self._stopped = True

    def _destroy_subscription(self) -> None:
        with self._lock:
            sub = self._sub
        if sub is not None and self._node is not None:
            self._node.destroy_subscription(sub)
            with self._lock:
                if self._sub is sub:
                    self._sub = None

    def _destroy_tick_publisher(self) -> None:
        with self._lock:
            publisher = self._tick_publisher
        if publisher is not None and self._node is not None:
            self._node.destroy_publisher(publisher)
            with self._lock:
                if self._tick_publisher is publisher:
                    self._tick_publisher = None

    def stop(self) -> dict:
        self.request_stop()
        cleanup_failure: Optional[Exception] = None
        try:
            self._destroy_subscription()
        except Exception as exc:
            cleanup_failure = exc
        worker = self._worker
        if worker is not None and worker.is_alive():
            while worker.is_alive():
                try:
                    self._queue.put(_STOP, timeout=0.1)
                    break
                except queue.Full:
                    continue
            worker.join()
        self._worker = None
        try:
            self._destroy_tick_publisher()
        except Exception as exc:
            if cleanup_failure is None:
                cleanup_failure = exc
        with self._lock:
            elapsed_s = max((self._last_recv_ns - self._start_wall_ns) / 1e9, 1e-6)
            self.fps_actual = self.frame_count / elapsed_s if self.frame_count else 0.0
            failure = self._failure
        if cleanup_failure is not None:
            if failure is not None:
                raise RuntimeError(f"{failure}; subscription cleanup failed: {cleanup_failure}")
            raise cleanup_failure
        if failure is not None:
            raise failure
        return self.summary()

    def has_pending_cleanup(self) -> bool:
        with self._lock:
            worker = self._worker
            return self._sub is not None or (worker is not None and worker.is_alive())

    def abort(self, detail: str = "camera finalization aborted") -> None:
        self.request_stop()
        self._set_failure(RuntimeError(f"[{self.name}] {detail}"))
        try:
            while True:
                self._queue.get_nowait()
        except queue.Empty:
            pass
        try:
            self._queue.put_nowait(_STOP)
        except queue.Full:
            pass

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


class DepthCameraWriter:
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
        self._sub = self._node.create_subscription(
            Image, self.topic, self._on_image, qos_profile_sensor_data,
        )
        self._start_wall_ns = time.time_ns()
        LOG.info("depth writer started: %s topic=%s out=%s",
                 self.name, self.topic, self.output_dir)

    def _on_image(self, msg: Image) -> None:
        if self._stopped:
            return
        with self._lock:
            if self._stopped:
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

    def request_stop(self) -> None:
        with self._lock:
            self._stopped = True

    def _destroy_subscription(self) -> None:
        with self._lock:
            sub = self._sub
        if sub is not None and self._node is not None:
            self._node.destroy_subscription(sub)
            with self._lock:
                if self._sub is sub:
                    self._sub = None

    def stop(self) -> dict:
        self.request_stop()
        cleanup_failure: Optional[Exception] = None
        try:
            self._destroy_subscription()
        except Exception as exc:
            cleanup_failure = exc
        with self._lock:
            self._sidecar.close()
            elapsed_s = max((self._last_recv_ns - self._start_wall_ns) / 1e9, 1e-6)
            self.fps_actual = self.frame_count / elapsed_s if self.frame_count else 0.0
        if cleanup_failure is not None:
            raise cleanup_failure
        return self.summary()

    def has_pending_cleanup(self) -> bool:
        with self._lock:
            return self._sub is not None

    def abort(self) -> None:
        self.request_stop()

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
        return {
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
        }


class CameraWriterPool:
    """Manage multiple CameraWriter instances for one episode."""

    def __init__(self, node: Node):
        self._node = node
        self._writers: dict[str, CameraWriter | DepthCameraWriter] = {}
        # Depth cameras don't have a writer (they go into the bag) but the
        # play modal still needs them in cameras[]. Stored here so finalize_all
        # can re-emit the placeholders alongside the real writers' summaries.
        self._depth_placeholders: list[dict] = []

    def start_all(self, episode_dir: Path, cameras: list[dict], fps: int = 30) -> list[dict]:
        """cameras: [{name, topic, kind?, codec?}, ...]. Color cameras get a
        CameraWriter (H.264 mp4); depth cameras (kind="depth") are now
        recorded into the rosbag2 sqlite instead — this method just emits a
        placeholder summary so the play modal still knows the depth track
        exists (preview reads from bag via /depth_preview endpoint). The
        topic is appended to bag_topics by topic_discovery."""
        color_cameras = [
            cam for cam in cameras if (cam.get("kind") or "color").lower() != "depth"
        ]
        encoder = probe_h264_encoder() if color_cameras else None
        videos_root = episode_dir / "videos"
        videos_root.mkdir(exist_ok=True)
        self._depth_placeholders = []
        try:
            for cam in cameras:
                name = cam["name"]
                topic = cam["topic"]
                kind = (cam.get("kind") or "color").lower()
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
                    name=name, topic=topic, output_dir=cam_dir, fps=fps, node=self._node,
                    encoder=encoder,
                )
                writer.start()
                self._writers[name] = writer
        except Exception:
            for writer in self._writers.values():
                try:
                    writer.request_stop()
                except Exception as exc:
                    LOG.error("camera quiesce after start failure failed: %s", exc)
            raise
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

    def request_stop(self) -> list[dict[str, str]]:
        failures = []
        for name, writer in list(self._writers.items()):
            try:
                writer.request_stop()
            except Exception as exc:
                failures.append({
                    "component": f"camera:{name}",
                    "error_type": type(exc).__name__,
                    "detail": str(exc),
                })
        return failures

    def finalize_all(self) -> tuple[list[dict], list[dict[str, str]]]:
        summaries = []
        failures = []
        remaining = {}
        for name, writer in list(self._writers.items()):
            try:
                summaries.append(writer.stop())
            except Exception as exc:
                summaries.append(writer.summary())
                failures.append({
                    "component": f"camera:{name}",
                    "error_type": type(exc).__name__,
                    "detail": str(exc),
                })
                if writer.has_pending_cleanup():
                    remaining[name] = writer
        self._writers = remaining
        # Re-emit depth placeholders so meta.cameras keeps them.
        out = summaries + list(self._depth_placeholders)
        self._depth_placeholders = []
        return out, failures

    def abort_all(self) -> None:
        for writer in self._writers.values():
            writer.abort()

    def cleanup_pending(self) -> None:
        for name, writer in list(self._writers.items()):
            try:
                writer.stop()
            except Exception as exc:
                LOG.error("camera cleanup retry failed for %s: %s", name, exc)
            if not writer.has_pending_cleanup():
                self._writers.pop(name, None)

    def has_pending_cleanup(self) -> bool:
        return any(writer.has_pending_cleanup() for writer in self._writers.values())

    def is_active(self) -> bool:
        return bool(self._writers)

    def frame_counts(self) -> dict[str, int]:
        for writer in self._writers.values():
            if writer._failure is not None:
                raise writer._failure
        return {name: w.frame_count for name, w in self._writers.items()}
