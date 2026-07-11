"""`ros2 bag record` subprocess wrapper for episode bags."""

from __future__ import annotations

import logging
import os
import shutil
import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

LOG = logging.getLogger("fv_episode_recorder.bag")
BAG_FINALIZE_TIMEOUT_S = 10.0
_BAG_TERMINATE_TIMEOUT_S = 3.0


def _summary(bag_dir: Optional[Path], topics: list[str]) -> dict:
    if bag_dir is None or not bag_dir.exists():
        return {"bag_dir": None, "size_bytes": 0, "split_count": 0}
    size = 0
    splits = 0
    for p in bag_dir.rglob("*.db3"):
        size += p.stat().st_size
        splits += 1
    for p in bag_dir.iterdir():
        if p.is_file() and not p.name.endswith(".db3"):
            size += p.stat().st_size
    return {"bag_dir": str(bag_dir), "size_bytes": size, "split_count": splits, "topics": topics}


@dataclass
class DetachedBagRecording:
    proc: Optional[subprocess.Popen[bytes]]
    bag_dir: Optional[Path]
    topics: list[str]

    def wait(self, timeout_s: float = BAG_FINALIZE_TIMEOUT_S) -> dict:
        if self.proc is None:
            return _summary(self.bag_dir, self.topics)
        timed_out = False
        if self.proc.poll() is None:
            try:
                self.proc.wait(timeout=timeout_s)
            except subprocess.TimeoutExpired:
                timed_out = True
                LOG.warning("bag recorder did not exit on SIGINT within %.1fs, sending SIGTERM", timeout_s)
                self.proc.terminate()
                try:
                    self.proc.wait(timeout=_BAG_TERMINATE_TIMEOUT_S)
                except subprocess.TimeoutExpired:
                    LOG.error("bag recorder still alive, SIGKILL")
                    self.proc.kill()
                    self.proc.wait(timeout=_BAG_TERMINATE_TIMEOUT_S)
        if timed_out:
            raise RuntimeError(
                f"bag recorder finalize timed out after {timeout_s:.1f}s "
                f"(returncode={self.proc.returncode})"
            )
        if self.proc.returncode not in (0, 130):
            stderr = b""
            if self.proc.stderr is not None:
                stderr = self.proc.stderr.read() or b""
            raise RuntimeError(
                f"bag recorder exited with code {self.proc.returncode}: "
                f"{stderr.decode('utf-8', errors='replace')[-1000:]}"
            )
        return _summary(self.bag_dir, self.topics)


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
        if bag_dir.exists() and any(bag_dir.iterdir()):
            stale = bag_dir.with_name(bag_dir.name + f".stale-{int(time.time())}")
            bag_dir.rename(stale)
            LOG.warning("pre-existing non-empty bag dir moved to %s", stale)
        elif bag_dir.exists():
            bag_dir.rmdir()
        cmd = [
            "ros2", "bag", "record",
            "-o", str(bag_dir),
            "-s", self.storage,
            "-b", str(self.max_bag_size_mb * 1024 * 1024),
            *topics,
        ]
        env = os.environ.copy()
        LOG.info("starting bag recorder: %s", " ".join(cmd))
        self._proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            env=env,
            preexec_fn=os.setsid if hasattr(os, "setsid") else None,
        )
        self._bag_dir = bag_dir
        self._topics = topics

    def stop(self, timeout_s: float = BAG_FINALIZE_TIMEOUT_S) -> dict:
        """Stop the bag recorder gracefully (SIGINT) so metadata.yaml is finalized.

        Returns summary dict with bag_dir / size_bytes / split_count.
        """
        return self.detach_for_finalize().wait(timeout_s)

    def detach_for_finalize(self) -> DetachedBagRecording:
        proc = self._proc
        bag_dir = self._bag_dir
        topics = list(self._topics)
        if proc is not None and proc.poll() is None:
            try:
                if hasattr(os, "killpg"):
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                else:
                    proc.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass
        self._proc = None
        self._bag_dir = None
        self._topics = []
        return DetachedBagRecording(proc=proc, bag_dir=bag_dir, topics=topics)

    def _summary(self) -> dict:
        return _summary(self._bag_dir, self._topics)

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
