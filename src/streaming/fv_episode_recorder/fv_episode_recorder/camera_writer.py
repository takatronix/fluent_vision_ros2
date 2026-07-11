"""Per-camera writer: subscribes to a camera topic, writes mp4 + frames.parquet.

Phase 1 Step 3 minimum:
  - CompressedImage subscriber (BEST_EFFORT, qos_profile_sensor_data)
  - ffmpeg H.264 encode, then packet remux so mp4 PTS follows ros_stamp_ns
  - 1 mp4 file per camera (segment rotation lands in Step 3.5)
  - FramesSidecar populates frames.parquet

Future steps:
  - segment rotation (10 min default)
  - sensor_msgs/Image (uncompressed) support
  - cameras_downscale for untagged pool (Phase 2.5 always_on)
"""

from __future__ import annotations

import logging
import shutil
import subprocess
import threading
import time
from dataclasses import dataclass
from fractions import Fraction
from pathlib import Path
from typing import Optional

import av
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

from .frames_sidecar import FramesSidecar

LOG = logging.getLogger("fv_episode_recorder.camera")
VIDEO_TIME_BASE = Fraction(1, 90_000)


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
            "-bf", "0",
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


def _timestamp_ticks(stamp_ns: int, first_stamp_ns: int) -> int:
    elapsed_ns = stamp_ns - first_stamp_ns
    if elapsed_ns < 0:
        raise RuntimeError("camera frame timestamps are not monotonic")
    return round(elapsed_ns * VIDEO_TIME_BASE.denominator / 1_000_000_000)


def _remux_mp4_with_ros_timestamps(
    path: Path,
    frame_stamps_ns: list[int],
    frame_pts: list[int] | None = None,
) -> None:
    if not frame_stamps_ns:
        return
    tmp_path = path.with_name(f".{path.stem}.vfr{path.suffix}")
    if tmp_path.exists():
        tmp_path.unlink()

    first_stamp_ns = frame_stamps_ns[0]
    expected_pts = frame_pts or [
        _timestamp_ticks(stamp_ns, first_stamp_ns) for stamp_ns in frame_stamps_ns
    ]
    if len(expected_pts) != len(frame_stamps_ns):
        raise RuntimeError(
            f"video PTS count mismatch: {len(expected_pts)}/{len(frame_stamps_ns)}"
        )
    previous_pts = -1
    packet_count = 0
    input_container = av.open(str(path), mode="r")
    output_container = av.open(str(tmp_path), mode="w")
    try:
        try:
            input_stream = input_container.streams.video[0]
            output_stream = output_container.add_stream_from_template(input_stream)
            output_stream.time_base = VIDEO_TIME_BASE
            output_container.start_encoding()
            if output_stream.time_base != VIDEO_TIME_BASE:
                raise RuntimeError(
                    "video muxer changed the requested time base: "
                    f"{output_stream.time_base} != {VIDEO_TIME_BASE}"
                )
            for packet in input_container.demux(input_stream):
                if packet.dts is None:
                    continue
                if packet_count >= len(frame_stamps_ns):
                    raise RuntimeError(
                        f"video packet count exceeds frames sidecar rows: {packet_count + 1}/{len(frame_stamps_ns)}"
                    )
                source_duration = packet.duration
                source_time_base = packet.time_base
                pts = expected_pts[packet_count]
                if pts <= previous_pts:
                    raise RuntimeError("camera frame timestamps are not strictly increasing")
                packet.pts = pts
                packet.dts = pts
                packet.time_base = VIDEO_TIME_BASE
                packet.stream = output_stream
                if packet_count + 1 < len(frame_stamps_ns):
                    next_pts = expected_pts[packet_count + 1]
                    packet.duration = max(1, next_pts - pts)
                elif packet_count > 0:
                    packet.duration = max(1, pts - previous_pts)
                else:
                    if source_duration is None or source_duration <= 0 or source_time_base is None:
                        raise RuntimeError("single-frame video packet has no duration")
                    packet.duration = max(
                        1,
                        round(source_duration * source_time_base / VIDEO_TIME_BASE),
                    )
                output_container.mux(packet)
                if packet.pts != pts or packet.time_base != VIDEO_TIME_BASE:
                    raise RuntimeError(
                        "video muxer changed packet PTS before write completion"
                    )
                previous_pts = pts
                packet_count += 1
            if packet_count != len(frame_stamps_ns):
                raise RuntimeError(
                    f"video packet count mismatch: {packet_count}/{len(frame_stamps_ns)}"
                )
        finally:
            input_container.close()
            output_container.close()
    except Exception:
        tmp_path.unlink(missing_ok=True)
        raise
    tmp_path.replace(path)


class CameraWriter:
    """One worker per camera: subscribes, encodes, writes mp4 + sidecar."""

    def __init__(
        self,
        name: str,
        topic: str,
        output_dir: Path,
        fps: int = 30,
        node: Optional[Node] = None,
        record_immediately: bool = True,
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
        self._frame_stamps_ns: list[int] = []
        self._frame_pts: list[int] = []
        self._lock = threading.Lock()
        self._sub = None
        self._recording_enabled = threading.Event()
        if record_immediately:
            self._recording_enabled.set()
        self._first_observed_ros_ns = 0
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
            if self._stopped:
                return
            recv_ns = time.time_ns()
            ros_stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            if ros_stamp_ns <= 0:
                LOG.warning("[%s] ROS header stamp is missing, skipping frame", self.name)
                return
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
            if self._first_observed_ros_ns == 0:
                self._first_observed_ros_ns = ros_stamp_ns
            if not self._recording_enabled.is_set():
                return

            try:
                self._ffmpeg.stdin.write(img.tobytes())
            except (BrokenPipeError, ValueError) as exc:
                LOG.warning("[%s] ffmpeg pipe write failed: %s", self.name, exc)
                return

            video_pts_origin_ros_ns = (
                self._frame_stamps_ns[0] if self._frame_stamps_ns else ros_stamp_ns
            )
            video_pts = _timestamp_ticks(
                ros_stamp_ns,
                video_pts_origin_ros_ns,
            )
            self._sidecar.append({
                "frame_index": self._frame_index,
                "segment_file": self._segment_file.name,
                "segment_local_frame": self._segment_local,
                "video_pts": video_pts,
                "ros_stamp_ns": ros_stamp_ns,
                "recv_stamp_ns": recv_ns,
                "source_seq": -1,
                "dropped_before": 0,  # naive (Phase 1.5 fills with seq gap detection)
                "keyframe": True,  # mp4v: every frame is intra-coded; refine later
            })
            self._frame_stamps_ns.append(ros_stamp_ns)
            self._frame_pts.append(video_pts)
            self._frame_index += 1
            self._segment_local += 1
            self.frame_count += 1
            self._last_recv_ns = recv_ns

    def enable_recording(self) -> None:
        if not self._recording_enabled.is_set():
            self._start_wall_ns = time.time_ns()
        self._recording_enabled.set()

    def has_observed_frame(self) -> bool:
        return self._first_observed_ros_ns > 0

    def first_observed_ros_ns(self) -> int:
        return self._first_observed_ros_ns

    def last_recorded_ros_ns(self) -> int:
        return self._frame_stamps_ns[-1] if self._frame_stamps_ns else 0

    def stop(self) -> dict:
        self.stop_recording()
        return self.finalize()

    def stop_recording(self) -> None:
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

    def finalize(self) -> dict:
        with self._lock:
            ffmpeg_error: str | None = None
            if self._ffmpeg is not None:
                try:
                    returncode = self._ffmpeg.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    LOG.warning("[%s] ffmpeg flush timeout, killing", self.name)
                    self._ffmpeg.kill()
                    returncode = self._ffmpeg.wait(timeout=3)
                if returncode != 0:
                    ffmpeg_error = f"ffmpeg exited with code {returncode}"
                self._ffmpeg = None
            sidecar_rows = self._sidecar.close()
            if ffmpeg_error is not None:
                raise RuntimeError(ffmpeg_error)
            if self.frame_count > 0:
                if sidecar_rows != self.frame_count:
                    raise RuntimeError(
                        f"frames sidecar row count mismatch: {sidecar_rows}/{self.frame_count}"
                    )
                _remux_mp4_with_ros_timestamps(
                    self._segment_file,
                    self._frame_stamps_ns,
                    self._frame_pts,
                )
                self._validate_video_readable()
            elapsed_s = max((self._last_recv_ns - self._start_wall_ns) / 1e9, 1e-6)
            self.fps_actual = self.frame_count / elapsed_s if self.frame_count else 0.0
        return self.summary()

    def _validate_video_readable(self) -> None:
        if not self._segment_file.exists() or self._segment_file.stat().st_size <= 0:
            raise RuntimeError(f"video file missing or empty: {self._segment_file}")
        cap = cv2.VideoCapture(str(self._segment_file))
        try:
            if not cap.isOpened():
                raise RuntimeError(f"video cannot be opened: {self._segment_file}")
            ok, _frame = cap.read()
            if not ok:
                raise RuntimeError(f"video frame 0 cannot be read: {self._segment_file}")
            if self.frame_count > 1:
                cap.set(cv2.CAP_PROP_POS_FRAMES, self.frame_count - 1)
                ok, _frame = cap.read()
                if not ok:
                    raise RuntimeError(
                        f"video final frame {self.frame_count - 1} cannot be read: {self._segment_file}"
                    )
        finally:
            cap.release()

    def summary(self) -> dict:
        size_bytes = self._segment_file.stat().st_size if self._segment_file.exists() else 0
        return {
            "name": self.name,
            "topic": self.topic,
            "video_timing_mode": "ros_header_stamp_to_pts",
            "video_pts_origin_ros_ns": self._frame_stamps_ns[0] if self._frame_stamps_ns else None,
            "video_time_base_num": VIDEO_TIME_BASE.numerator,
            "video_time_base_den": VIDEO_TIME_BASE.denominator,
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
            ros_stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            if ros_stamp_ns <= 0:
                LOG.warning("[%s] ROS header stamp is missing, skipping depth frame", self.name)
                return
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
            self._sidecar.append({
                "frame_index": self._frame_index,
                "segment_file": png_path.name,
                "segment_local_frame": self._frame_index,
                "video_pts": None,
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
        self.stop_recording()
        return self.finalize()

    def stop_recording(self) -> None:
        with self._lock:
            self._stopped = True
            if self._sub is not None and self._node is not None:
                try:
                    self._node.destroy_subscription(self._sub)
                except Exception:
                    pass
                self._sub = None

    def finalize(self) -> dict:
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


@dataclass
class DetachedCameraWriterPool:
    writers: list[CameraWriter | DepthCameraWriter]
    depth_placeholders: list[dict]

    def frame_counts(self) -> dict[str, int]:
        counts = {writer.name: writer.frame_count for writer in self.writers}
        for placeholder in self.depth_placeholders:
            name = placeholder.get("name")
            if name:
                counts[str(name)] = int(placeholder.get("frame_count") or 0)
        return counts

    def finalize(self, depth_frame_counts: dict[str, int]) -> tuple[list[dict], list[str]]:
        summaries: list[dict] = []
        errors: list[str] = []
        for writer in self.writers:
            try:
                summaries.append(writer.finalize())
            except Exception as exc:
                LOG.exception("camera writer finalize failed: %s", writer.name)
                errors.append(f"{writer.name}: {exc}")
                try:
                    summary = writer.summary()
                    summary["error"] = str(exc)
                    summaries.append(summary)
                except Exception:
                    pass
        for placeholder in self.depth_placeholders:
            name = placeholder.get("name")
            if name in depth_frame_counts:
                placeholder["frame_count"] = int(depth_frame_counts[name])
        return summaries + list(self.depth_placeholders), errors


class CameraWriterPool:
    """Manage multiple CameraWriter instances for one episode."""

    def __init__(self, node: Node):
        self._node = node
        self._writers: dict[str, CameraWriter | DepthCameraWriter] = {}
        # Depth cameras don't have a writer (they go into the bag) but the
        # play modal still needs them in cameras[]. Stored here so stop_all
        # can re-emit the placeholders alongside the real writers' summaries.
        self._depth_placeholders: list[dict] = []

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
                record_immediately=bool(cam.get("record_immediately", True)),
            )
            writer.start()
            self._writers[name] = writer
        return [w.summary() for w in self._writers.values()] + self._depth_placeholders

    def observed_frame_counts(self) -> dict[str, int]:
        return {
            name: 1 if writer.has_observed_frame() else 0
            for name, writer in self._writers.items()
        }

    def observed_frame_stamps_ns(self) -> dict[str, int]:
        return {
            name: writer.first_observed_ros_ns()
            for name, writer in self._writers.items()
        }

    def recorded_frame_stamps_ns(self) -> dict[str, int]:
        return {
            name: writer.last_recorded_ros_ns()
            for name, writer in self._writers.items()
        }

    def ros_now_ns(self) -> int:
        now_ns = int(self._node.get_clock().now().nanoseconds)
        if now_ns <= 0:
            raise RuntimeError("recorder ROS clock did not return a positive timestamp")
        return now_ns

    def enable_recording(self) -> None:
        for writer in self._writers.values():
            writer.enable_recording()

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

    def detach_all(self) -> DetachedCameraWriterPool:
        writers = list(self._writers.values())
        placeholders = list(self._depth_placeholders)
        for writer in writers:
            writer.stop_recording()
        self._writers.clear()
        self._depth_placeholders = []
        return DetachedCameraWriterPool(writers=writers, depth_placeholders=placeholders)

    def stop_all(self) -> list[dict]:
        summaries, errors = self.detach_all().finalize({})
        if errors:
            raise RuntimeError("; ".join(errors))
        return summaries

    def is_active(self) -> bool:
        return bool(self._writers)

    def frame_counts(self) -> dict[str, int]:
        return {name: w.frame_count for name, w in self._writers.items()}
