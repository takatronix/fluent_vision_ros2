"""Timestamp tolerance limits shared by export requests and dataset metadata."""

MAX_VIDEO_TIMESTAMP_TOLERANCE_S = 0.1
MAX_VIDEO_TIMESTAMP_FRAME_DISTANCE = 3


def max_video_timestamp_tolerance_s(fps: int) -> float:
    return min(
        MAX_VIDEO_TIMESTAMP_TOLERANCE_S,
        MAX_VIDEO_TIMESTAMP_FRAME_DISTANCE / fps,
    )
