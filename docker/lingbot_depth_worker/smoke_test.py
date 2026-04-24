#!/usr/bin/env python3
from __future__ import annotations

import io
import sys
import urllib.request

import numpy as np


def main() -> int:
    endpoint = sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:5540/infer"

    height = 48
    width = 64
    color_bgr = np.zeros((height, width, 3), dtype=np.uint8)
    color_bgr[..., 1] = 128

    depth_m = np.full((height, width), 0.75, dtype=np.float32)
    intrinsics_norm = np.eye(3, dtype=np.float32)
    intrinsics_norm[0, 0] = 600.0 / width
    intrinsics_norm[1, 1] = 600.0 / height
    intrinsics_norm[0, 2] = (width / 2.0) / width
    intrinsics_norm[1, 2] = (height / 2.0) / height

    req_buf = io.BytesIO()
    np.savez_compressed(
        req_buf,
        color_bgr=color_bgr,
        depth_m=depth_m,
        intrinsics_norm=intrinsics_norm,
        use_fp16=np.asarray(0, dtype=np.uint8),
        apply_mask=np.asarray(1, dtype=np.uint8),
        resolution_level=np.asarray(9, dtype=np.int32),
    )

    req = urllib.request.Request(
        endpoint,
        data=req_buf.getvalue(),
        headers={"Content-Type": "application/octet-stream"},
        method="POST",
    )

    with urllib.request.urlopen(req, timeout=10) as resp:
        body = resp.read()

    with np.load(io.BytesIO(body), allow_pickle=False) as data:
        depth = data["depth"]
        mask = data["mask"]
        points = data["points"]

    assert depth.shape == (height, width), f"unexpected depth shape: {depth.shape}"
    assert mask.shape == (height, width), f"unexpected mask shape: {mask.shape}"
    assert points.shape == (height, width, 3), f"unexpected points shape: {points.shape}"
    assert np.isfinite(depth).all(), "depth contains non-finite values"
    assert np.all(mask > 0), "mask should be valid in smoke test"
    assert np.isfinite(points[..., 2]).all(), "points z contains non-finite values"

    print(
        f"ok depth_shape={depth.shape} mask_shape={mask.shape} "
        f"points_shape={points.shape} depth_mean={float(depth.mean()):.4f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
