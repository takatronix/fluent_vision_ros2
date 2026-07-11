"""Per-frame timestamp sidecar (frames.parquet) writer.

One sidecar per camera. Columns (Codex #4 minimum):
    frame_index           int64   # episode-wide sequence (0-based, increments even on drop)
    segment_file          string  # mp4 file basename, e.g. "0000.mp4"
    segment_local_frame   int32   # index within that segment
    video_pts             int64   # PTS written to the MP4 packet
    ros_stamp_ns          int64   # message header stamp (camera capture time)
    recv_stamp_ns         int64   # recorder receive time
    source_seq            int64   # publisher sequence number if available, else -1
    dropped_before        int32   # frames missed before this one (gap detection)
    keyframe              bool    # h264 keyframe (preview seek optimization)

Buffered in memory, flushed in row-group chunks for crash safety.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import pyarrow as pa
import pyarrow.parquet as pq


SCHEMA = pa.schema([
    ("frame_index", pa.int64()),
    ("segment_file", pa.string()),
    ("segment_local_frame", pa.int32()),
    ("video_pts", pa.int64()),
    ("ros_stamp_ns", pa.int64()),
    ("recv_stamp_ns", pa.int64()),
    ("source_seq", pa.int64()),
    ("dropped_before", pa.int32()),
    ("keyframe", pa.bool_()),
])


class FramesSidecar:
    def __init__(self, parquet_path: Path, flush_every: int = 600):
        self.parquet_path = Path(parquet_path)
        self.flush_every = flush_every
        self._buffer: list[dict] = []
        self._writer: Optional[pq.ParquetWriter] = None
        self.parquet_path.parent.mkdir(parents=True, exist_ok=True)

    def append(self, row: dict) -> None:
        self._buffer.append(row)
        if len(self._buffer) >= self.flush_every:
            self._flush()

    def _flush(self) -> None:
        if not self._buffer:
            return
        table = pa.Table.from_pylist(self._buffer, schema=SCHEMA)
        if self._writer is None:
            self._writer = pq.ParquetWriter(self.parquet_path, SCHEMA, compression="zstd")
        self._writer.write_table(table)
        self._buffer.clear()

    def close(self) -> int:
        """Flush remaining rows and close. Returns total frame count written."""
        self._flush()
        count = 0
        if self._writer is not None:
            self._writer.close()
            self._writer = None
        if self.parquet_path.exists():
            try:
                count = pq.read_metadata(self.parquet_path).num_rows
            except Exception:
                pass
        return count
