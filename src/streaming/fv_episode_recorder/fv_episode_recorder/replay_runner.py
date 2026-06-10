"""Bag replay runner — Phase 3c (mux integration, sim profile validated).

Spawns `ros2 bag play <bag_dir>` and routes the recorded command topics
through `teleop_mux_node`'s `replay` source so there is exactly one
publisher on the driver-facing command topic (no double-publish jitter
that bit us in Phase 3b — live teleop_mux output + bag_play both pushing
/follower_arm/joint_cmd produced ~2× rate and visible ガクガク).

Flow per replay start:
 1. Inspect the bag's recorded topics to detect the arm namespace and
    decide which topics to remap.
 2. POST set_source='replay' on /<arm_ns>/teleop_mux/set_source so the
    mux begins forwarding replay/* and blocks live inputs.
 3. Spawn ros2 bag play with --remap rules that move the command topics
    onto /<arm_ns>/teleop_mux/input/replay/* and isolate /tf, /tf_static,
    /<arm_ns>/teleop_mux/status under a /replay/ prefix so they do not
    fight with the live system.
 4. On stop (SIGINT or duration_s expiry or operator stop), POST
    set_source='stop' so the operator must explicitly re-select a live
    source — matches the E-STOP release semantics already in the mux.

**SAFETY**: still sim-only. Real-hardware additions (person-in-workspace
check, home-pose-within-tolerance check, operator ack token) remain Phase
3c.2 — to be added before the first real-arm bag_play.
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

import yaml

LOG = logging.getLogger("fv_episode_recorder.replay")


# Topics in the bag whose replay must route through the teleop_mux replay
# inputs (so the live command path stays single-publisher). Keyed by the
# tail of the topic name; matched against any `<prefix>/<tail>` topic in
# the bag where the prefix is the detected arm namespace.
_MUX_COMMAND_REMAPS = {
    "joint_cmd":   "teleop_mux/input/replay/joint_ctrl",
    "pos_cmd":     "teleop_mux/input/replay/pos_cmd",
    "gripper_cmd": "teleop_mux/input/replay/gripper_cmd",
}

# Global topics that conflict with the live system if bag_play publishes
# them onto their original names; isolate under /replay/.
_GLOBAL_REPLAY_PREFIX_REMAPS = ("/tf", "/tf_static")


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


def _read_bag_topics(bag_dir: Path) -> list[str]:
    """Pull the topic name list from rosbag2 metadata.yaml. Returns [] on
    parse failure rather than raising — replay falls back to a no-remap
    bag_play (which is the Phase 3b behaviour) so the operator at least
    gets motion, with a warning logged."""
    try:
        with (bag_dir / "metadata.yaml").open() as f:
            meta = yaml.safe_load(f) or {}
        topics_block = (
            meta.get("rosbag2_bagfile_information", {})
                .get("topics_with_message_count", [])
        )
        return [
            (t.get("topic_metadata", {}) or {}).get("name", "")
            for t in topics_block
            if isinstance(t, dict)
        ]
    except Exception as exc:
        LOG.warning("could not parse bag metadata.yaml under %s: %s", bag_dir, exc)
        return []


def _detect_arm_namespace(bag_topics: list[str]) -> Optional[str]:
    """Pull the arm namespace out of a topic like
    `/follower_arm/teleop_mux/status` (we always record this).
    Returns `/follower_arm` (no trailing slash) or None if not present —
    in which case the recorder will skip the mux switch and the operator
    sees the legacy double-publish behaviour with a warning."""
    suffix = "/teleop_mux/status"
    for t in bag_topics:
        if t and t.endswith(suffix):
            return t[: -len(suffix)] or None
    return None


def _build_remap_args(arm_ns: Optional[str], bag_topics: list[str]) -> list[str]:
    """Return the --remap arguments that route command topics through the
    mux replay inputs and isolate tf-style globals under /replay/."""
    remaps: list[tuple[str, str]] = []
    if arm_ns:
        for tail, mux_path in _MUX_COMMAND_REMAPS.items():
            full = f"{arm_ns}/{tail}"
            if full in bag_topics:
                remaps.append((full, f"{arm_ns}/{mux_path}"))
        # The mux's own status topic is recorded for diagnostic value; the
        # live mux republishes its own status, so isolate the recorded one
        # to /replay/ to avoid confusing dashboard listeners.
        live_status = f"{arm_ns}/teleop_mux/status"
        if live_status in bag_topics:
            remaps.append((live_status, f"/replay{live_status}"))
    for gtopic in _GLOBAL_REPLAY_PREFIX_REMAPS:
        if gtopic in bag_topics:
            remaps.append((gtopic, f"/replay{gtopic}"))
    if not remaps:
        return []
    args = ["--remap"]
    args.extend(f"{src}:={dst}" for src, dst in remaps)
    return args


async def _call_set_source(arm_ns: str, source: str, timeout_s: float = 3.0) -> bool:
    """Best-effort ros2 service call to set the mux source. We use the CLI
    rather than rclpy so this stays decoupled from the recorder node and
    works whether or not the recorder happens to have the service client
    pre-created. Returns True on success."""
    service = f"{arm_ns}/teleop_mux/set_source"
    payload = "{source: '" + source + "'}"
    cmd = [
        "ros2", "service", "call", service,
        "vlabor_msgs/srv/SetControlSource", payload,
    ]
    try:
        proc = await asyncio.create_subprocess_exec(
            *cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
        try:
            stdout, stderr = await asyncio.wait_for(proc.communicate(), timeout=timeout_s)
        except asyncio.TimeoutError:
            proc.kill()
            await proc.wait()
            LOG.warning("set_source(%s) on %s timed out after %.1fs", source, service, timeout_s)
            return False
    except FileNotFoundError:
        LOG.warning("ros2 CLI missing — cannot switch mux source")
        return False
    out = (stdout or b"").decode("utf-8", "replace")
    err = (stderr or b"").decode("utf-8", "replace")
    if proc.returncode != 0 or "success=True" not in out:
        LOG.warning("set_source(%s) failed: rc=%s stdout=%s stderr=%s",
                    source, proc.returncode, out.strip()[-200:], err.strip()[-200:])
        return False
    LOG.info("mux source -> %s via %s", source, service)
    return True


class ReplayRunner:
    """Single-replay-at-a-time runner."""

    def __init__(self):
        self._state: Optional[ReplayState] = None
        self._proc: Optional[asyncio.subprocess.Process] = None
        self._watch_task: Optional[asyncio.Task] = None
        self._mux_arm_ns: Optional[str] = None  # set when we switch mux to 'replay'

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

        # Phase 3c: route command topics through teleop_mux's replay source.
        # Without this, bag_play and the live teleop both publish on
        # /<arm>/joint_cmd → 2× rate → driver/sim ガクガク (see
        # docs/episode_recorder_phase3c.md). With the remap + mux switch
        # the driver sees exactly one publisher.
        bag_topics = _read_bag_topics(bag_dir)
        arm_ns = _detect_arm_namespace(bag_topics)
        remap_args = _build_remap_args(arm_ns, bag_topics)
        if arm_ns:
            switched = await _call_set_source(arm_ns, "replay")
            if switched:
                self._mux_arm_ns = arm_ns
            else:
                LOG.warning(
                    "replay %s: mux set_source('replay') failed on %s — "
                    "live teleop will keep publishing alongside bag_play "
                    "(expect jitter). Add the replay source to your "
                    "teleop_mux build or check the namespace.",
                    replay_id, arm_ns,
                )
        else:
            LOG.warning(
                "replay %s: could not detect arm namespace from bag topics "
                "(no */teleop_mux/status). Skipping mux switch — bag_play "
                "runs alongside live teleop. Expect jitter.", replay_id,
            )

        # Build ros2 bag play command.
        # `--clock 100` publishes /clock at 100Hz so time-aware subscribers
        # (mujoco bridge, anything using ros::Time::now) stay in lockstep with
        # the bag's timeline → less jitter perceived on the replay.
        # `--read-ahead-queue-size 10000` (default 1000) keeps the publisher
        # ahead enough to absorb decode hiccups, especially for large mp4
        # interleaved bags.
        cmd = [
            "ros2", "bag", "play", str(bag_dir),
            "--rate", f"{state.speed:.3f}",
            "--clock", "100",
            "--read-ahead-queue-size", "10000",
        ]
        if state.start_offset_s > 0:
            cmd.extend(["--start-offset", f"{state.start_offset_s:.3f}"])
        if remap_args:
            cmd.extend(remap_args)

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
            # bag_play didn't start — undo the mux switch so we don't strand
            # the operator with a 'replay' source that has no publisher.
            if self._mux_arm_ns:
                try:
                    await _call_set_source(self._mux_arm_ns, "stop")
                except Exception:
                    pass
                self._mux_arm_ns = None
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
            # Restore mux to 'stop' so the operator must explicitly re-pick
            # a live source. Mirrors the E-STOP-release semantics already
            # baked into teleop_mux_node — replay finishing does NOT auto-
            # resume the previous source.
            if self._mux_arm_ns:
                try:
                    await _call_set_source(self._mux_arm_ns, "stop")
                except Exception as exc:
                    LOG.warning("replay teardown: set_source('stop') raised: %s", exc)
                self._mux_arm_ns = None

    async def stop(self) -> bool:
        if not self.is_running() or self._proc is None:
            return False
        try:
            self._proc.send_signal(2)
        except ProcessLookupError:
            return False
        return True
