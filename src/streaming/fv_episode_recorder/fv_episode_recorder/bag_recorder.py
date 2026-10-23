"""ros2 bag record subprocess wrapper.

Phase 1 Step 2: spawn `ros2 bag record` as subprocess into the episode's bag/
directory. Topics are passed explicitly (Step 4 wires profile-driven discovery;
for now caller passes the list).

Note: rosbag2_py Python API exists but the CLI subprocess is more robust for
arbitrary topic sets and storage options, and matches `ros2 bag info/play`
expectations downstream.
"""

from __future__ import annotations

import logging
import os
import shutil
import signal
import subprocess
import time
from pathlib import Path
from typing import Optional

LOG = logging.getLogger("fv_episode_recorder.bag")

# rosbag2 storage plugins we know how to drive. "sqlite3" stays the default
# (Humble default, what every existing reader expects). "mcap" needs
# ros-<distro>-rosbag2-storage-mcap and is opt-in per profile
# (episode_recorder.bag_storage).
SUPPORTED_STORAGE = ("sqlite3", "mcap")
DEFAULT_STORAGE = "sqlite3"
# file suffix of the split files each storage plugin writes
_STORAGE_SUFFIX = {"sqlite3": ".db3", "mcap": ".mcap"}


def build_record_command(
    bag_dir: Path,
    topics: list[str],
    *,
    storage: Optional[str] = None,
    max_bag_size_mb: int = 1024,
    storage_preset: Optional[str] = None,
) -> list[str]:
    """Build the `ros2 bag record` argv.

    storage=None/"sqlite3" (default) produces exactly the historical command
    line. storage_preset (e.g. "zstd_fast") maps to
    `--storage-preset-profile` and is only meaningful for mcap — with sqlite3
    it is ignored with a warning instead of failing the episode start.
    """
    storage = (storage or DEFAULT_STORAGE).strip().lower()
    if storage not in SUPPORTED_STORAGE:
        raise ValueError(
            f"unsupported bag storage {storage!r} (expected one of {SUPPORTED_STORAGE})")
    cmd = [
        "ros2", "bag", "record",
        "-o", str(bag_dir),
        "-s", storage,
    ]
    if storage_preset:
        if storage == "mcap":
            cmd += ["--storage-preset-profile", str(storage_preset)]
        else:
            LOG.warning("bag_storage_preset=%r ignored: only valid with bag_storage=mcap",
                        storage_preset)
    cmd += ["-b", str(int(max_bag_size_mb) * 1024 * 1024), *topics]
    return cmd


def detect_bag_storage(bag_dir: Path, default: str = DEFAULT_STORAGE) -> str:
    """storage_id for rosbag2_py readers: metadata.yaml storage_identifier,
    else guessed from split-file suffix (bag still recording → no
    metadata.yaml yet), else ``default``."""
    bag_dir = Path(bag_dir)
    meta = bag_dir / "metadata.yaml"
    if meta.exists():
        try:
            import yaml

            info = (yaml.safe_load(meta.read_text()) or {}).get(
                "rosbag2_bagfile_information") or {}
            sid = str(info.get("storage_identifier") or "").strip()
            if sid:
                return sid
        except Exception as exc:  # malformed yaml → fall through to suffix guess
            LOG.warning("metadata.yaml unreadable in %s: %s", bag_dir, exc)
    if bag_dir.is_dir():
        for sid, suffix in _STORAGE_SUFFIX.items():
            if any(bag_dir.glob(f"*{suffix}")):
                return sid
    return default


class BagRecorder:
    """Single-instance bag recorder. Owns at most one `ros2 bag record` process."""

    def __init__(self, max_bag_size_mb: int = 1024, storage: str = "sqlite3"):
        self._proc: Optional[subprocess.Popen[bytes]] = None
        self._bag_dir: Optional[Path] = None
        self._topics: list[str] = []
        self.max_bag_size_mb = max_bag_size_mb
        self.storage = storage  # "sqlite3" (rosbag2 default on Humble)
        self._storage_in_use = storage

    @property
    def active(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def start(self, bag_dir: Path, topics: list[str], *,
              storage: Optional[str] = None,
              storage_preset: Optional[str] = None) -> None:
        """storage / storage_preset: per-episode override (profile keys
        episode_recorder.bag_storage / bag_storage_preset). None keeps the
        recorder default (sqlite3)."""
        if self.active:
            raise RuntimeError("bag recorder already active")
        if not topics:
            LOG.warning("bag start with empty topic list — recording will be empty")
        bag_dir = Path(bag_dir)
        # ros2 bag record creates the directory itself; if it pre-exists with content
        # it errors. We move it aside to a stale-* sibling.
        if bag_dir.exists() and any(bag_dir.iterdir()):
            stale = bag_dir.with_name(bag_dir.name + f".stale-{int(time.time())}")
            bag_dir.rename(stale)
            LOG.warning("pre-existing non-empty bag dir moved to %s", stale)
        elif bag_dir.exists():
            # empty -> remove so `ros2 bag record -o` does not complain
            bag_dir.rmdir()

        storage = storage or self.storage
        cmd = build_record_command(
            bag_dir, topics, storage=storage,
            max_bag_size_mb=self.max_bag_size_mb, storage_preset=storage_preset,
        )
        self._storage_in_use = storage
        # ros2 needs an inherited env (ROS_DISTRO, AMENT_PREFIX_PATH, etc).
        env = os.environ.copy()
        LOG.info("starting bag recorder: %s", " ".join(cmd))
        # Use a new process group so we can SIGINT the whole thing on stop.
        self._proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            env=env,
            preexec_fn=os.setsid if hasattr(os, "setsid") else None,
        )
        self._bag_dir = bag_dir
        self._topics = topics

    def stop(self, timeout_s: float = 10.0) -> dict:
        """Stop the bag recorder gracefully (SIGINT) so metadata.yaml is finalized.

        Returns summary dict with bag_dir / size_bytes / split_count.
        """
        if not self.active:
            return self._summary()

        assert self._proc is not None
        # Send SIGINT to the whole process group so rosbag2 flushes metadata.
        try:
            if hasattr(os, "killpg"):
                os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
            else:
                self._proc.send_signal(signal.SIGINT)
        except ProcessLookupError:
            pass

        try:
            self._proc.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            LOG.warning("bag recorder did not exit on SIGINT within %.1fs, sending SIGTERM", timeout_s)
            self._proc.terminate()
            try:
                self._proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                LOG.error("bag recorder still alive, SIGKILL")
                self._proc.kill()
                self._proc.wait(timeout=3.0)

        return self._summary()

    def _summary(self) -> dict:
        bag_dir = self._bag_dir
        if bag_dir is None or not bag_dir.exists():
            return {"bag_dir": None, "size_bytes": 0, "split_count": 0}
        size = 0
        splits = 0
        suffixes = tuple(_STORAGE_SUFFIX.values())
        for p in bag_dir.rglob("*"):
            if p.is_file() and p.name.endswith(suffixes):
                size += p.stat().st_size
                splits += 1
        # also include metadata.yaml etc
        for p in bag_dir.iterdir():
            if p.is_file() and not p.name.endswith(suffixes):
                size += p.stat().st_size
        return {"bag_dir": str(bag_dir), "size_bytes": size, "split_count": splits,
                "topics": self._topics, "storage": self._storage_in_use}

    def abort(self) -> None:
        """Kill without waiting — for crash recovery / discard paths."""
        if self.active:
            try:
                if hasattr(os, "killpg"):
                    os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)
                else:
                    self._proc.kill()
            except ProcessLookupError:
                pass

    @staticmethod
    def available() -> bool:
        return shutil.which("ros2") is not None
