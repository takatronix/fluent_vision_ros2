"""Active episode lock + crash recovery (設計書 §5.2 / §7.3).

Single .active_episode.lock JSON file at output_dir/ root. Created on start,
removed on graceful stop. On recorder restart, lock detection identifies
orphans (recorder crashed mid-record). Operator can recover via
POST /api/v1/episodes/{id}/recover {action: finalize_aborted | discard}.
"""

from __future__ import annotations

import json
import logging
import os
import time
import uuid
from pathlib import Path
from typing import Optional

LOG = logging.getLogger("fv_episode_recorder.lock")


class ActiveLock:
    def __init__(self, output_dir: Path | str):
        self.output_dir = Path(output_dir)
        self.lock_path = self.output_dir / ".active_episode.lock"
        self.recorder_uuid = str(uuid.uuid4())

    def acquire(self, episode_id: str, episode_dir: Path) -> None:
        """Write lock with current recorder pid + uuid + episode info."""
        payload = {
            "episode_id": episode_id,
            "episode_dir": str(episode_dir),
            "started_at": time.time(),
            "recorder_pid": os.getpid(),
            "recorder_uuid": self.recorder_uuid,
        }
        tmp = self.lock_path.with_suffix(".lock.tmp")
        with tmp.open("w") as f:
            json.dump(payload, f)
        tmp.replace(self.lock_path)

    def release(self) -> None:
        try:
            self.lock_path.unlink()
        except FileNotFoundError:
            pass

    def read(self) -> Optional[dict]:
        if not self.lock_path.exists():
            return None
        try:
            with self.lock_path.open() as f:
                return json.load(f)
        except (OSError, json.JSONDecodeError) as exc:
            LOG.warning("active lock unreadable: %s", exc)
            return None

    def detect_orphan(self) -> Optional[dict]:
        """If lock exists from a previous run, return its payload (caller decides)."""
        payload = self.read()
        if payload is None:
            return None
        pid = payload.get("recorder_pid")
        # If the pid is alive AND uuid matches a different instance, the previous
        # recorder is still running (we should not start). If uuid matches ours,
        # this is a stale lock from our own bad shutdown path.
        if pid and _pid_alive(pid) and payload.get("recorder_uuid") != self.recorder_uuid:
            LOG.error("another recorder appears alive (pid=%s); refusing to start", pid)
            return {**payload, "status": "another_alive"}
        LOG.warning("orphan lock detected: episode_id=%s pid=%s — needs recover",
                    payload.get("episode_id"), pid)
        return {**payload, "status": "orphan"}


def _pid_alive(pid: int) -> bool:
    # pid が「録画ノードとして」生きているかを確認する。コンテナ再起動で
    # PID 空間が変わると、残留ロックの pid が無関係なプロセスに再利用されて
    # 「生きている」と誤判定し、録画ノードが永久に起動拒否する
    # (2026-08-08 実害)。存在確認だけでなく cmdline まで見る。
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        pass
    try:
        cmdline = Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ")
        return b"recorder_node" in cmdline
    except OSError:
        return False
    except PermissionError:
        # process exists but we lack signal permission -> treat as alive
        return True
