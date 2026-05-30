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


class BagRecorder:
    """Single-instance bag recorder. Owns at most one `ros2 bag record` process."""

    def __init__(self, max_bag_size_mb: int = 1024, storage: str = "sqlite3"):
        self._proc: Optional[subprocess.Popen[bytes]] = None
        self._bag_dir: Optional[Path] = None
        self._topics: list[str] = []
        self.max_bag_size_mb = max_bag_size_mb
        self.storage = storage  # "sqlite3" (rosbag2 default on Humble)

    @property
    def active(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def start(self, bag_dir: Path, topics: list[str]) -> None:
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

        cmd = [
            "ros2", "bag", "record",
            "-o", str(bag_dir),
            "-s", self.storage,
            "-b", str(self.max_bag_size_mb * 1024 * 1024),
            *topics,
        ]
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
        for p in bag_dir.rglob("*.db3"):
            size += p.stat().st_size
            splits += 1
        # also include metadata.yaml etc
        for p in bag_dir.iterdir():
            if p.is_file() and not p.name.endswith(".db3"):
                size += p.stat().st_size
        return {"bag_dir": str(bag_dir), "size_bytes": size, "split_count": splits, "topics": self._topics}

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
