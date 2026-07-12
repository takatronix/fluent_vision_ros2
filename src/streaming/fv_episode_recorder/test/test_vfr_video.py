from pathlib import Path
import json
import subprocess

import cv2
import numpy as np
import pytest

from fv_episode_recorder.vfr_video import (
    VIDEO_TIME_BASE,
    VfrMp4Writer,
    probe_h264_encoder,
)


def _jpeg(value: int) -> bytes:
    image = np.full((480, 640, 3), value, dtype=np.uint8)
    ok, encoded = cv2.imencode(".jpg", image)
    assert ok
    return encoded.tobytes()


def _probe_video(path: Path) -> dict[str, list[dict[str, str]]]:
    result = subprocess.run(
        [
            "ffprobe", "-v", "error", "-select_streams", "v:0",
            "-show_entries", "stream=time_base",
            "-show_entries", "packet=pts,dts",
            "-show_entries", "frame=pts",
            "-of", "json", str(path),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )
    return json.loads(result.stdout)


def test_vfr_mp4_uses_ros_timeline_and_validates_edge_frames(tmp_path: Path) -> None:
    writer = VfrMp4Writer(tmp_path / "0000.mp4", 30, probe_h264_encoder())
    stamps = [1_000_000_000, 1_040_000_000, 1_110_000_000]

    frames = [writer.append(_jpeg(index * 80), stamp)
              for index, stamp in enumerate(stamps)]
    writer.close()

    assert [frame.video_pts for frame in frames] == [0, 40_000, 110_000]
    assert (tmp_path / "0000.mp4").is_file()
    assert str(VIDEO_TIME_BASE) == "1/1000000"
    probe = _probe_video(tmp_path / "0000.mp4")
    assert probe["streams"] == [{"time_base": "1/1000000"}]
    entries = probe["packets_and_frames"]
    expected = [0, 40_000, 110_000]
    assert [int(row["pts"]) for row in entries if row["type"] == "packet"] == expected
    assert [int(row["dts"]) for row in entries if row["type"] == "packet"] == expected
    assert [int(row["pts"]) for row in entries if row["type"] == "frame"] == expected


def test_non_increasing_ros_timestamp_fails_instead_of_retiming(tmp_path: Path) -> None:
    writer = VfrMp4Writer(tmp_path / "0000.mp4", 30, probe_h264_encoder())
    writer.append(_jpeg(0), 2_000_000_000)

    with pytest.raises(RuntimeError, match="not strictly increasing"):
        writer.append(_jpeg(80), 2_000_000_000)

    writer.abort()
