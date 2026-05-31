"""Bag replay runner — Phase 3b/3c minimum (sim profile only for now).

Spawns `ros2 bag play <bag_dir>` as a subprocess so the recorded joint_cmd /
tf / mux / etc. topics are republished. Downstream subscribers (mujoco arm,
inference_bridge in policy_feed mode) pick up the messages as if it were
live.

**SAFETY**: this MVP has NO physical safety gate. It is intended for
*sim profile only* (mujoco virtual arm). For real-hardware bag_play (Phase
3c) we'll add: mux switch to bag_replay source, person-in-workspace check,
home-pose-within-tolerance check, e-stop OK check, operator ack token.
The UI shows a clear warning + sim-only badge.
"""

from __future__ import annotations

import asyncio
import logging
import os
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

LOG = logging.getLogger("fv_episode_recorder.replay")


@dataclass
class ReplayState:
    replay_id: str
    episode_id: str
    bag_dir: str
    mode: str                       # "bag_play" | "preview" | "policy_feed"
    started_at: float
    pid: Optional[int] = None
    finished_at: Optional[float] = None
    exit_code: Optional[int] = None
    last_error: Optional[str] = None
    speed: float = 1.0
    start_offset_s: float = 0.0
    duration_s: Optional[float] = None        # None = play whole bag

    def to_dict(self) -> dict:
        return {
            "replay_id": self.replay_id,
            "episode_id": self.episode_id,
            "mode": self.mode,
            "bag_dir": self.bag_dir,
            "started_at": self.started_at,
            "finished_at": self.finished_at,
            "exit_code": self.exit_code,
            "last_error": self.last_error,
            "pid": self.pid,
            "speed": self.speed,
            "start_offset_s": self.start_offset_s,
            "duration_s": self.duration_s,
            "running": self.finished_at is None and self.last_error is None,
        }


class ReplayRunner:
    """Single-replay-at-a-time runner."""

    def __init__(self):
        self._state: Optional[ReplayState] = None
        self._proc: Optional[asyncio.subprocess.Process] = None
        self._watch_task: Optional[asyncio.Task] = None

    @property
    def state(self) -> Optional[ReplayState]:
        return self._state

    def is_running(self) -> bool:
        return self._state is not None and self._state.finished_at is None and self._state.last_error is None

    async def start(
        self,
        episode_id: str,
        bag_dir: Path,
        speed: float = 1.0,
        start_offset_s: float = 0.0,
        duration_s: Optional[float] = None,
        mode: str = "bag_play",
    ) -> ReplayState:
        if self.is_running():
            raise RuntimeError(f"another replay already running: {self._state.replay_id if self._state else '?'}")

        if not bag_dir.exists():
            raise FileNotFoundError(f"bag dir does not exist: {bag_dir}")
        if not (bag_dir / "metadata.yaml").exists():
            raise FileNotFoundError(f"bag metadata.yaml missing under {bag_dir}")

        replay_id = uuid.uuid4().hex[:12]
        state = ReplayState(
            replay_id=replay_id,
            episode_id=episode_id,
            bag_dir=str(bag_dir),
            mode=mode,
            started_at=time.time(),
            speed=max(0.1, min(4.0, float(speed))),
            start_offset_s=max(0.0, float(start_offset_s)),
            duration_s=duration_s,
        )

        # Build ros2 bag play command.
        cmd = ["ros2", "bag", "play", str(bag_dir), "--rate", f"{state.speed:.3f}"]
        if state.start_offset_s > 0:
            cmd.extend(["--start-offset", f"{state.start_offset_s:.3f}"])

        # Inherit the recorder's env (CycloneDDS / ROS_DISTRO / ROS_DOMAIN_ID etc.)
        env = os.environ.copy()
        LOG.info("replay %s: spawning %s", replay_id, " ".join(cmd))
        try:
            self._proc = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.DEVNULL,
                stderr=asyncio.subprocess.PIPE,
                env=env,
            )
        except FileNotFoundError as exc:
            raise RuntimeError(f"ros2 CLI not found in PATH: {exc}") from exc

        state.pid = self._proc.pid
        self._state = state

        # Watcher: wait for process exit (or duration_s expiry → SIGINT).
        self._watch_task = asyncio.create_task(self._watch(duration_s))
        return state

    async def _watch(self, duration_s: Optional[float]) -> None:
        assert self._state is not None
        assert self._proc is not None
        try:
            if duration_s is not None and duration_s > 0:
                try:
                    await asyncio.wait_for(self._proc.wait(), timeout=duration_s)
                except asyncio.TimeoutError:
                    LOG.info("replay %s: duration_s elapsed, sending SIGINT", self._state.replay_id)
                    try:
                        self._proc.send_signal(2)  # SIGINT — ros2 bag play handles graceful shutdown
                    except ProcessLookupError:
                        pass
                    try:
                        await asyncio.wait_for(self._proc.wait(), timeout=5.0)
                    except asyncio.TimeoutError:
                        LOG.warning("replay %s: SIGINT timeout, SIGKILL", self._state.replay_id)
                        self._proc.kill()
                        await self._proc.wait()
            else:
                await self._proc.wait()
            self._state.exit_code = self._proc.returncode
            if self._proc.returncode not in (0, -2):  # 0 = normal, -2 = SIGINT from stop()
                stderr_tail = b""
                try:
                    if self._proc.stderr is not None:
                        stderr_tail = await self._proc.stderr.read()
                except Exception:
                    pass
                self._state.last_error = stderr_tail.decode("utf-8", "replace")[-500:]
                LOG.warning("replay %s exited rc=%d: %s",
                            self._state.replay_id, self._proc.returncode,
                            self._state.last_error)
        finally:
            self._state.finished_at = time.time()
            self._proc = None
            self._watch_task = None

    async def stop(self) -> bool:
        if not self.is_running() or self._proc is None:
            return False
        try:
            self._proc.send_signal(2)
        except ProcessLookupError:
            return False
        return True
