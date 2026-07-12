from pathlib import Path

import pyarrow.parquet as pq

from fv_episode_recorder.frames_sidecar import FramesSidecar


def test_video_pts_is_persisted_with_explicit_time_base(tmp_path: Path) -> None:
    path = tmp_path / "frames.parquet"
    sidecar = FramesSidecar(path)
    sidecar.append(
        {
            "frame_index": 0,
            "segment_file": "0000.mp4",
            "segment_local_frame": 0,
            "ros_stamp_ns": 1_234_567_890,
            "video_pts": 42_000,
            "recv_stamp_ns": 2_000_000_000,
            "source_seq": -1,
            "dropped_before": 0,
            "keyframe": True,
        }
    )

    assert sidecar.close() == 1
    table = pq.read_table(path)
    assert table.column("ros_stamp_ns").to_pylist() == [1_234_567_890]
    assert table.column("video_pts").to_pylist() == [42_000]
    assert table.schema.metadata[b"video_time_base"] == b"1/1000000"
