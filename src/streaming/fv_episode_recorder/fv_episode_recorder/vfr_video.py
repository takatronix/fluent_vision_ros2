"""VFR MP4 encoding from camera-compressed frames and ROS timestamps."""

from __future__ import annotations

import logging
from dataclasses import dataclass
from fractions import Fraction
from pathlib import Path

import av
import cv2
import numpy as np


LOG = logging.getLogger("fv_episode_recorder.camera")
VIDEO_TIME_BASE_DEN = 1_000_000
VIDEO_TIME_BASE = Fraction(1, VIDEO_TIME_BASE_DEN)


@dataclass(frozen=True)
class EncoderConfig:
    codec: str
    options: tuple[tuple[str, str], ...]


@dataclass(frozen=True)
class VideoFrame:
    ros_stamp_ns: int
    video_pts: int


_ENCODER_CANDIDATES = (
    EncoderConfig("h264_nvenc", (("preset", "p1"), ("tune", "ll"), ("bf", "0"))),
    EncoderConfig(
        "libx264", (("preset", "ultrafast"), ("tune", "zerolatency"), ("bf", "0")),
    ),
)


def probe_h264_encoder() -> EncoderConfig:
    """Select a working H.264 encoder before camera subscriptions start."""
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    for candidate in _ENCODER_CANDIDATES:
        try:
            codec = av.CodecContext.create(candidate.codec, "w")
            codec.width = 640
            codec.height = 480
            codec.pix_fmt = "yuv420p"
            codec.time_base = VIDEO_TIME_BASE
            codec.framerate = Fraction(25, 1)
            codec.options = dict(candidate.options)
            frame = av.VideoFrame.from_ndarray(image, format="bgr24")
            frame.pts = 0
            frame.time_base = VIDEO_TIME_BASE
            packets = codec.encode(frame) + codec.encode(None)
            if not packets:
                raise RuntimeError("encoder produced no probe packet")
        except Exception as exc:
            LOG.warning("[encoder probe] %s unavailable: %s", candidate.codec, exc)
            continue
        LOG.info("selected H.264 encoder: %s", candidate.codec)
        return candidate
    raise RuntimeError(
        "no working H.264 encoder available: "
        + ", ".join(candidate.codec for candidate in _ENCODER_CANDIDATES)
    )


class VfrMp4Writer:
    """Encode each frame once with a PTS derived from its ROS header stamp."""

    def __init__(self, output_path: Path, fps_nominal: int, encoder: EncoderConfig):
        self.output_path = Path(output_path)
        self.fps_nominal = fps_nominal
        self.encoder = encoder
        self.width = 0
        self.height = 0
        self._first_ros_stamp_ns: int | None = None
        self._last_frame: VideoFrame | None = None
        self._frame_count = 0
        self._container: av.container.OutputContainer | None = None
        self._stream: av.video.stream.VideoStream | None = None

    def append(self, data: bytes, ros_stamp_ns: int) -> VideoFrame:
        image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError("compressed camera frame decode failed")
        height, width = image.shape[:2]
        if self._frame_count and (width != self.width or height != self.height):
            raise RuntimeError(
                f"camera frame size changed from {self.width}x{self.height} "
                f"to {width}x{height}"
            )

        if self._first_ros_stamp_ns is None:
            self._first_ros_stamp_ns = ros_stamp_ns
            video_pts = 0
            self.width = width
            self.height = height
            self._open()
        else:
            delta_ns = ros_stamp_ns - self._first_ros_stamp_ns
            video_pts = (delta_ns * VIDEO_TIME_BASE_DEN + 500_000_000) // 1_000_000_000
            if self._last_frame is None:
                raise RuntimeError("video timestamp state is inconsistent")
            if video_pts <= self._last_frame.video_pts:
                raise RuntimeError(
                    "camera ROS timestamps are not strictly increasing at video time base: "
                    f"previous={self._last_frame.ros_stamp_ns}, current={ros_stamp_ns}"
                )

        frame = av.VideoFrame.from_ndarray(image, format="bgr24")
        frame.pts = video_pts
        frame.time_base = VIDEO_TIME_BASE
        if self._stream is None or self._container is None:
            raise RuntimeError("video encoder is not open")
        for packet in self._stream.encode(frame):
            self._container.mux(packet)
        recorded = VideoFrame(ros_stamp_ns=ros_stamp_ns, video_pts=video_pts)
        self._last_frame = recorded
        self._frame_count += 1
        return recorded

    def close(self) -> None:
        if not self._frame_count:
            self.abort()
            return
        if self._stream is None or self._container is None:
            raise RuntimeError("video encoder is not open")
        for packet in self._stream.encode(None):
            self._container.mux(packet)
        self._container.close()
        self._container = None
        self._stream = None

    def abort(self) -> None:
        container = self._container
        self._container = None
        self._stream = None
        try:
            if container is not None:
                container.close()
        finally:
            self.output_path.unlink(missing_ok=True)

    def _open(self) -> None:
        self._container = av.open(str(self.output_path), mode="w", options={"movflags": "+faststart"})
        self._stream = self._container.add_stream(self.encoder.codec, rate=self.fps_nominal)
        self._stream.width = self.width
        self._stream.height = self.height
        self._stream.pix_fmt = "yuv420p"
        self._stream.time_base = VIDEO_TIME_BASE
        self._stream.codec_context.time_base = VIDEO_TIME_BASE
        self._stream.options = dict(self.encoder.options)
