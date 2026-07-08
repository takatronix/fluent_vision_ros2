"""rosbag2_transport recorder subprocess wrapper.

The helper process uses rosbag2_transport::Recorder with a standard
SequentialWriter wrapper that only counts successful writer_->write() calls
for the episode ready barrier.
"""

from __future__ import annotations

import json
import logging
import os
import shutil
import signal
import subprocess
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

LOG = logging.getLogger("fv_episode_recorder.bag")


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

    def wait(self, timeout_s: float = 10.0) -> dict:
        if self.proc is None or self.proc.poll() is not None:
            return _summary(self.bag_dir, self.topics)
        try:
            self.proc.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            LOG.warning("bag recorder did not exit on SIGINT within %.1fs, sending SIGTERM", timeout_s)
            self.proc.terminate()
            try:
                self.proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                LOG.error("bag recorder still alive, SIGKILL")
                self.proc.kill()
                self.proc.wait(timeout=3.0)
        return _summary(self.bag_dir, self.topics)


@dataclass(frozen=True)
class BagReadyStatus:
    ready: bool
    ready_at: str | None
    counts: dict[str, int]


class BagRecorder:
    """Single-instance bag recorder. Owns at most one recorder helper process."""

    def __init__(self, max_bag_size_mb: int = 1024, storage: str = "sqlite3"):
        self._proc: Optional[subprocess.Popen[bytes]] = None
        self._bag_dir: Optional[Path] = None
        self._ready_file: Optional[Path] = None
        self._topics: list[str] = []
        self.max_bag_size_mb = max_bag_size_mb
        self.storage = storage  # "sqlite3" (rosbag2 default on Humble)

    @property
    def active(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def start(self, bag_dir: Path, topics: list[str], ready_topics: set[str] | None = None) -> None:
        if self.active:
            raise RuntimeError("bag recorder already active")
        if not topics:
            LOG.warning("bag start with empty topic list — recording will be empty")
        required = set(ready_topics or set())
        bag_dir = Path(bag_dir)
        ready_file = bag_dir.with_name("bag_ready.json")
        if bag_dir.exists() and any(bag_dir.iterdir()):
            stale = bag_dir.with_name(bag_dir.name + f".stale-{int(time.time())}")
            bag_dir.rename(stale)
            LOG.warning("pre-existing non-empty bag dir moved to %s", stale)
        elif bag_dir.exists():
            bag_dir.rmdir()
        try:
            ready_file.unlink()
        except FileNotFoundError:
            pass

        cmd = [
            "ros2", "run", "fv_recorder", "fv_counting_bag_recorder",
            "--output", str(bag_dir),
            "--storage", self.storage,
            "--max-bag-size", str(self.max_bag_size_mb * 1024 * 1024),
            "--ready-file", str(ready_file),
        ]
        for topic in topics:
            cmd.extend(["--topic", topic])
        for topic in sorted(required):
            cmd.extend(["--ready-topic", topic])
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
        self._ready_file = ready_file
        self._topics = topics

    def stop(self, timeout_s: float = 10.0) -> dict:
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
        self._ready_file = None
        self._topics = []
        return DetachedBagRecording(proc=proc, bag_dir=bag_dir, topics=topics)

    def ready_status(self, required_topics: set[str]) -> BagReadyStatus:
        counts = {topic: 0 for topic in required_topics}
        if not required_topics:
            return BagReadyStatus(ready=True, ready_at=_utcnow_iso(), counts=counts)
        if self._ready_file is not None and self._ready_file.exists():
            try:
                raw = json.loads(self._ready_file.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as exc:
                raise RuntimeError(f"bag ready status is unreadable: {exc}") from exc
            raw_counts = raw.get("bag_topics")
            if isinstance(raw_counts, dict):
                for topic in required_topics:
                    value = raw_counts.get(topic)
                    if isinstance(value, int):
                        counts[topic] = value
            ready = bool(raw.get("ready")) and all(counts[topic] > 0 for topic in required_topics)
            ready_at = _iso_from_unix_ns(raw.get("ready_at_unix_ns")) if ready else None
            if not ready and self._proc is not None and self._proc.poll() is not None:
                raise RuntimeError(
                    f"bag recorder exited before ready: rc={self._proc.returncode} "
                    f"stderr={self._stderr_tail()}"
                )
            return BagReadyStatus(ready=ready, ready_at=ready_at, counts=counts)
        if self._proc is not None and self._proc.poll() is not None:
            raise RuntimeError(
                f"bag recorder exited before ready: rc={self._proc.returncode} "
                f"stderr={self._stderr_tail()}"
            )
        return BagReadyStatus(ready=False, ready_at=None, counts=counts)

    def _summary(self) -> dict:
        return _summary(self._bag_dir, self._topics)

    def _stderr_tail(self) -> str:
        if self._proc is None or self._proc.stderr is None:
            return ""
        try:
            data = self._proc.stderr.read() or b""
        except OSError:
            return ""
        return data.decode("utf-8", errors="replace")[-1000:]

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


def _utcnow_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


def _iso_from_unix_ns(value) -> str | None:
    if not isinstance(value, int) or value <= 0:
        return None
    return datetime.fromtimestamp(value / 1_000_000_000.0, timezone.utc).strftime(
        "%Y-%m-%dT%H:%M:%S.%fZ"
    )
