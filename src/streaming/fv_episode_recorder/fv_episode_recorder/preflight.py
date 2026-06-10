"""Disk preflight — estimate required bytes for an episode, refuse with 503
if available - safety_margin < estimated.

Codex #6 / 設計書 §6.2 / §6.5 対応.

Coefficients are order-of-magnitude only (h.264/mpeg4-compressed jpeg streams
vary widely with scene). Phase 1.5+ may learn from past episodes' size.
"""

from __future__ import annotations

import logging
import shutil
from pathlib import Path
from typing import Optional

LOG = logging.getLogger("fv_episode_recorder.preflight")

# Empirical defaults (Step 3 smoke test: top_camera 4.5s = 227KB ≈ 50KB/s
# per 640x480 compressed @30fps. Real rosbag tf-only = ~25KB/s).
PER_CAMERA_MB_PER_S = 0.5      # generous bound vs ~0.05 measured
PER_BAG_MB_PER_S = 0.2         # generous bound vs ~0.025 measured

DEFAULT_DURATION_S = 60.0      # if expected_duration_s not given
DEFAULT_SAFETY_MARGIN_GB = 5.0


def estimate_episode_bytes(
    *,
    n_cameras: int,
    n_bag_topics: int,
    duration_s: float,
    per_camera_mb_per_s: float = PER_CAMERA_MB_PER_S,
    per_bag_mb_per_s: float = PER_BAG_MB_PER_S,
) -> dict:
    duration_s = max(duration_s, 1.0)
    camera_bytes = int(n_cameras * per_camera_mb_per_s * 1024 * 1024 * duration_s)
    # bag scales weakly with topic count (most msgs are small)
    bag_bytes = int(min(1.0 + 0.2 * n_bag_topics, 4.0) * per_bag_mb_per_s * 1024 * 1024 * duration_s)
    return {
        "camera_bytes": camera_bytes,
        "bag_bytes": bag_bytes,
        "total_bytes": camera_bytes + bag_bytes,
    }


def disk_free_bytes(path: Path | str) -> int:
    path = Path(path)
    if not path.exists():
        path.mkdir(parents=True, exist_ok=True)
    return shutil.disk_usage(path).free


def run_preflight(
    *,
    output_dir: Path | str,
    n_cameras: int,
    n_bag_topics: int,
    expected_duration_s: Optional[float],
    safety_margin_gb: float = DEFAULT_SAFETY_MARGIN_GB,
    skip: bool = False,
) -> dict:
    """Return preflight result dict (always populated, never raises).

    Caller decides whether to 503 based on `passed` field.
    """
    if skip:
        return {
            "passed": True, "skipped": True,
            "estimated_bytes": 0, "available_bytes": 0, "margin_bytes": 0,
        }
    duration = expected_duration_s if expected_duration_s and expected_duration_s > 0 else DEFAULT_DURATION_S
    est = estimate_episode_bytes(
        n_cameras=n_cameras, n_bag_topics=n_bag_topics, duration_s=duration
    )
    safety_bytes = int(safety_margin_gb * 1024 ** 3)
    available = disk_free_bytes(output_dir)
    required = est["total_bytes"] + safety_bytes
    passed = available >= required
    result = {
        "passed": passed,
        "estimated_bytes": est["total_bytes"],
        "breakdown": {
            "camera_bytes": est["camera_bytes"],
            "bag_bytes": est["bag_bytes"],
        },
        "available_bytes": available,
        "margin_bytes": int(safety_bytes),
        "required_bytes": required,
        "duration_basis_s": duration,
        "n_cameras": n_cameras,
        "n_bag_topics": n_bag_topics,
        "blockers": [] if passed else [
            f"insufficient disk: required {required / 1e9:.2f}GB (est {est['total_bytes']/1e9:.2f}GB + margin {safety_margin_gb:.1f}GB) "
            f"but only {available / 1e9:.2f}GB free at {output_dir}"
        ],
    }
    if not passed:
        LOG.warning("preflight failed: %s", result["blockers"][0])
    return result
