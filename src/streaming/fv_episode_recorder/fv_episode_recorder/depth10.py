"""10-bit-luma depth video packing (lossy depth compression).

16-bit depth is quantized to 10 bits via inverse-depth (disparity)
normalization and stored in the Y plane of a HEVC Main10 video (chroma
flat). Compared to the lossless compressedDepth PNG path this is ~30-60×
smaller; precision concentrates at near range (PoC on live D405/D555
frames 2026-07-18: grasp band 7-30cm median error 0.2mm, full working
range median 3-6mm).

Key properties:
  - luma 0 is reserved for invalid; valid pixels map to 1..1023.
  - pixels beyond d_max are mapped to INVALID (0), not clamped, so far
    IR noise doesn't decode into a phantom wall at d_max.
  - d_min/d_max are in RAW SENSOR COUNTS (device depth units), not mm.
    depth_scale_mm carries counts→mm for consumers (D555=1.0, D405=0.1).
  - encode/decode are exact inverses up to the 10-bit quantization.

`python3 -m fv_episode_recorder.depth10 <video>` decodes a recording back
to 16-bit PNG frames (or .npy) using the sidecar depth_meta.json.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

LUMA_LEVELS = 1023  # valid code points 1..1023 (0 = invalid)
META_FILENAME = "depth_meta.json"


def build_pack_lut(d_min: float, d_max: float) -> np.ndarray:
    """65536-entry uint16 LUT: raw depth count -> luma code (0 = invalid).

    Depth is uint16, so a table lookup gives bit-identical results to the
    per-pixel float math at ~memcpy speed (the float path cost ~13 ms per
    720p frame — enough to miss 30 fps once several cameras share the CPU)."""
    counts = np.arange(65536, dtype=np.float64)
    valid = (counts > 0) & (counts <= d_max)
    with np.errstate(divide="ignore"):
        inv = np.where(valid, 1.0 / np.maximum(counts, 1e-9), 0.0)
    dnorm = (inv - 1.0 / d_max) / (1.0 / d_min - 1.0 / d_max)
    dnorm = np.clip(dnorm, 0.0, 1.0)
    q = np.rint(dnorm * (LUMA_LEVELS - 1)).astype(np.int32) + 1  # 1..1023
    return np.where(valid, q, 0).astype(np.uint16)


def pack_luma10(depth: np.ndarray, d_min: float, d_max: float,
                lut: np.ndarray | None = None) -> np.ndarray:
    """uint16 depth (raw counts) -> uint16 luma 0..1023. 0/beyond-d_max -> 0."""
    if lut is None:
        lut = build_pack_lut(d_min, d_max)
    return lut[depth]


def build_unpack_lut(d_min: float, d_max: float) -> np.ndarray:
    """1024-entry uint16 LUT: luma code -> depth count (code 0 -> 0)."""
    codes = np.arange(LUMA_LEVELS + 1, dtype=np.int32)
    valid = codes > 0
    q = np.clip(codes - 1, 0, LUMA_LEVELS - 1).astype(np.float64)
    dnorm = q / (LUMA_LEVELS - 1)
    inv = 1.0 / d_max + dnorm * (1.0 / d_min - 1.0 / d_max)
    d = np.where(valid, 1.0 / np.maximum(inv, 1e-12), 0.0)
    return np.where(valid, np.rint(np.clip(d, 0, 65535)), 0).astype(np.uint16)


def unpack_luma10(y: np.ndarray, d_min: float, d_max: float,
                  lut: np.ndarray | None = None) -> np.ndarray:
    """uint16 luma 0..1023 -> uint16 depth (raw counts). 0 -> invalid(0)."""
    if lut is None:
        lut = build_unpack_lut(d_min, d_max)
    return lut[np.minimum(y, LUMA_LEVELS).astype(np.int32)]


def write_meta(cam_dir: Path, *, d_min: float, d_max: float, depth_scale_mm: float,
               width: int, height: int, fps: float, codec: str,
               encoding_seen: str, video_file: str) -> None:
    meta = {
        "format": "depth10_hevc_main10",
        "version": 1,
        "d_min_counts": d_min,
        "d_max_counts": d_max,
        "depth_scale_mm_per_count": depth_scale_mm,
        "luma_levels": LUMA_LEVELS,
        "invalid_luma": 0,
        "far_handling": "mapped_to_invalid",
        "width": width,
        "height": height,
        "fps": fps,
        "codec": codec,
        "pix_fmt": "yuv420p10le",
        "source_encoding": encoding_seen,
        "video_file": video_file,
    }
    (cam_dir / META_FILENAME).write_text(json.dumps(meta, indent=2))


def read_meta(cam_dir: Path) -> dict:
    return json.loads((cam_dir / META_FILENAME).read_text())


def decode_video(video: Path, meta: dict):
    """Yield uint16 depth frames decoded from a depth10 video (needs ffmpeg)."""
    import subprocess

    w, h = int(meta["width"]), int(meta["height"])
    frame_words = w * h + 2 * (h // 2) * (w // 2)  # Y + U + V, 10-bit in uint16
    proc = subprocess.Popen(
        ["ffmpeg", "-v", "error", "-i", str(video),
         "-f", "rawvideo", "-pix_fmt", "yuv420p10le", "pipe:1"],
        stdout=subprocess.PIPE,
    )
    d_min, d_max = float(meta["d_min_counts"]), float(meta["d_max_counts"])
    try:
        while True:
            buf = proc.stdout.read(frame_words * 2)
            if len(buf) < frame_words * 2:
                break
            y = np.frombuffer(buf, "<u2")[: w * h].reshape(h, w)
            yield unpack_luma10(y, d_min, d_max)
    finally:
        proc.stdout.close()
        proc.wait()


def _cli() -> None:
    import argparse

    ap = argparse.ArgumentParser(
        description="Decode a depth10 (10-bit-luma HEVC) recording back to 16-bit depth")
    ap.add_argument("video", type=Path, help="video file (its dir must hold depth_meta.json)")
    ap.add_argument("-o", "--out", type=Path, default=None,
                    help="output dir (default: <video_dir>/decoded)")
    ap.add_argument("--npy", action="store_true", help="write one stacked .npy instead of PNGs")
    args = ap.parse_args()

    cam_dir = args.video.parent
    meta = read_meta(cam_dir)
    out = args.out or (cam_dir / "decoded")
    out.mkdir(parents=True, exist_ok=True)
    if args.npy:
        stack = np.stack(list(decode_video(args.video, meta)))
        np.save(out / "depth.npy", stack)
        print(f"{stack.shape[0]} frames -> {out / 'depth.npy'}")
    else:
        import cv2
        n = 0
        for i, frame in enumerate(decode_video(args.video, meta)):
            cv2.imwrite(str(out / f"{i:06d}.png"), frame)
            n += 1
        print(f"{n} frames -> {out}/NNNNNN.png (16-bit)")


if __name__ == "__main__":
    _cli()
