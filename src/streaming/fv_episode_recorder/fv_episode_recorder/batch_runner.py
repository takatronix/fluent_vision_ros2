"""Auto-loop episode recording (LeRobot-style: N iterations × [setup, record]).

Solves the "100回 click 同じ場所" problem: operator sets a target count,
setup countdown (time to reset the scene), and recording duration; the
runner cycles through each iteration calling the same start/stop episode
codepath the manual UI uses. Single-batch-at-a-time (recorder enforces
single active episode anyway).

Phase machine:
    idle → setup (N seconds, scene reset) → recording (N seconds, capture)
         ↻ repeats per remaining iteration
         → done (loop completed) / aborted (operator stopped)
"""

from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass, field
from typing import Awaitable, Callable, Optional

LOG = logging.getLogger("fv_episode_recorder.batch")


# Caller-supplied async functions so the runner doesn't have to import
# api_server (avoids a circular dependency). Both take the same shape as
# the existing _start_episode / _stop_episode handlers.
StartEpisodeFn = Callable[[dict], Awaitable[dict]]
StopEpisodeFn = Callable[[str, str], Awaitable[dict]]


@dataclass
class BatchConfig:
    task_description: str
    profile: str
    count: int                  # number of episodes to record
    record_s: float             # length of each recording (seconds)
    setup_s: float = 5.0        # countdown before each recording starts
    tags: list[str] = field(default_factory=list)
    record_topics_override: Optional[list] = None
    outcome_on_timeout: str = "success"   # outcome stamped when record_s expires


@dataclass
class BatchState:
    batch_id: str
    config: BatchConfig
    phase: str = "idle"          # idle | setup | recording | done | aborted | failed
    current_index: int = 0       # 1-based when running
    current_episode_id: Optional[str] = None
    phase_started_at: float = 0.0
    phase_duration_s: float = 0.0
    started_at: float = 0.0
    finished_at: Optional[float] = None
    succeeded: int = 0
    failed: int = 0
    last_error: Optional[str] = None

    def remaining_s(self) -> float:
        if self.phase not in ("setup", "recording"):
            return 0.0
        return max(0.0, self.phase_duration_s - (time.time() - self.phase_started_at))

    def to_dict(self) -> dict:
        c = self.config
        return {
            "batch_id": self.batch_id,
            "phase": self.phase,
            "current_index": self.current_index,
            "total": c.count,
            "current_episode_id": self.current_episode_id,
            "phase_remaining_s": round(self.remaining_s(), 1),
            "phase_duration_s": self.phase_duration_s,
            "started_at": self.started_at,
            "finished_at": self.finished_at,
            "succeeded": self.succeeded,
            "failed": self.failed,
            "last_error": self.last_error,
            "config": {
                "task_description": c.task_description,
                "profile": c.profile,
                "count": c.count,
                "record_s": c.record_s,
                "setup_s": c.setup_s,
                "tags": c.tags,
            },
        }


class BatchRunner:
    """Single-batch-at-a-time runner."""

    def __init__(self):
        self._state: Optional[BatchState] = None
        self._task: Optional[asyncio.Task] = None
        self._stop_evt: Optional[asyncio.Event] = None
        # Per-iteration control. retry = discard current ep + redo same index;
        # skip = stop current ep with the configured outcome + advance to next.
        self._retry_evt: Optional[asyncio.Event] = None
        self._skip_evt: Optional[asyncio.Event] = None

    @property
    def state(self) -> Optional[BatchState]:
        return self._state

    def is_running(self) -> bool:
        return self._state is not None and self._state.phase in ("setup", "recording")

    async def start(
        self,
        config: BatchConfig,
        start_fn: StartEpisodeFn,
        stop_fn: StopEpisodeFn,
    ) -> BatchState:
        if self.is_running():
            raise RuntimeError(f"batch already running: {self._state.batch_id if self._state else '?'}")
        import uuid
        batch_id = uuid.uuid4().hex[:12]
        self._state = BatchState(
            batch_id=batch_id,
            config=config,
            phase="setup",
            started_at=time.time(),
            phase_started_at=time.time(),
            phase_duration_s=config.setup_s,
        )
        self._stop_evt = asyncio.Event()
        self._retry_evt = asyncio.Event()
        self._skip_evt = asyncio.Event()
        self._task = asyncio.create_task(self._run(start_fn, stop_fn))
        return self._state

    async def stop(self) -> bool:
        """Request abort. Currently-recording episode is stopped with the
        configured outcome (default discard for abort)."""
        if not self.is_running() or self._stop_evt is None:
            return False
        self._stop_evt.set()
        return True

    def retry(self) -> bool:
        """Operator-triggered: this iteration was bad; discard the current
        episode and redo the same index from the setup phase."""
        if not self.is_running() or self._retry_evt is None:
            return False
        self._retry_evt.set()
        return True

    def skip(self) -> bool:
        """Operator-triggered: this iteration finished early; finalize the
        current episode with the configured outcome and advance to the next."""
        if not self.is_running() or self._skip_evt is None:
            return False
        self._skip_evt.set()
        return True

    async def _wait(self, seconds: float) -> str:
        """Wait `seconds` or until one of the control events fires. Returns
        one of: 'timeout' (full duration elapsed), 'abort', 'retry', 'skip'."""
        if self._stop_evt is None:
            await asyncio.sleep(seconds)
            return 'timeout'
        sleep_t = asyncio.create_task(asyncio.sleep(seconds))
        evts = {
            'abort': asyncio.create_task(self._stop_evt.wait()),
            'retry': asyncio.create_task(self._retry_evt.wait()) if self._retry_evt else None,
            'skip':  asyncio.create_task(self._skip_evt.wait())  if self._skip_evt  else None,
        }
        waitset = {sleep_t} | {t for t in evts.values() if t is not None}
        done, pending = await asyncio.wait(waitset, return_when=asyncio.FIRST_COMPLETED)
        for t in pending:
            t.cancel()
        # Priority: abort > retry > skip > timeout.
        if self._stop_evt.is_set():
            return 'abort'
        if self._retry_evt is not None and self._retry_evt.is_set():
            self._retry_evt.clear()
            return 'retry'
        if self._skip_evt is not None and self._skip_evt.is_set():
            self._skip_evt.clear()
            return 'skip'
        return 'timeout'

    async def _run(self, start_fn: StartEpisodeFn, stop_fn: StopEpisodeFn) -> None:
        assert self._state is not None
        state = self._state
        cfg = state.config
        try:
            i = 1
            while i <= cfg.count:
                state.current_index = i
                # ---- setup countdown ----
                state.phase = "setup"
                state.phase_started_at = time.time()
                state.phase_duration_s = cfg.setup_s
                LOG.info("batch %s: ep %d/%d setup (%.1fs)", state.batch_id, i, cfg.count, cfg.setup_s)
                setup_action = await self._wait(cfg.setup_s)
                if setup_action == 'abort':
                    state.phase = 'aborted'
                    LOG.info("batch %s: aborted during setup at %d/%d", state.batch_id, i, cfg.count)
                    return
                # retry / skip during setup just shorten or restart — both advance to recording

                # ---- start episode ----
                width = max(3, len(str(cfg.count)))
                seq = f"{i:0{width}d}/{cfg.count}"
                req_body = {
                    "task_description": f"{cfg.task_description} #{seq}",
                    "profile": cfg.profile,
                    "tags": cfg.tags + [f"batch:{state.batch_id}", f"batch_idx:{seq}"],
                }
                if cfg.record_topics_override:
                    req_body["record_topics_override"] = cfg.record_topics_override
                try:
                    resp = await start_fn(req_body)
                    state.current_episode_id = resp.get("episode_id")
                except Exception as exc:
                    state.failed += 1
                    state.last_error = f"start failed: {exc}"
                    LOG.warning("batch %s: ep %d start failed: %s", state.batch_id, i, exc)
                    i += 1
                    continue

                # ---- recording ----
                state.phase = "recording"
                state.phase_started_at = time.time()
                state.phase_duration_s = cfg.record_s
                LOG.info("batch %s: ep %d/%d recording (%.1fs) id=%s",
                         state.batch_id, i, cfg.count, cfg.record_s, state.current_episode_id)
                rec_action = await self._wait(cfg.record_s)

                # ---- decide outcome from action ----
                if rec_action == 'abort':
                    outcome = 'discard'
                elif rec_action == 'retry':
                    outcome = 'discard'
                elif rec_action == 'skip':
                    outcome = cfg.outcome_on_timeout   # treat as if record_s elapsed
                else:  # 'timeout'
                    outcome = cfg.outcome_on_timeout

                try:
                    if state.current_episode_id:
                        await stop_fn(state.current_episode_id, outcome)
                    if outcome != 'discard':
                        state.succeeded += 1
                    elif rec_action == 'retry':
                        # retry doesn't count as failure — operator just bailing
                        pass
                except Exception as exc:
                    state.failed += 1
                    state.last_error = f"stop failed: {exc}"
                    LOG.warning("batch %s: ep %d stop failed: %s", state.batch_id, i, exc)

                state.current_episode_id = None
                if rec_action == 'abort':
                    state.phase = 'aborted'
                    LOG.info("batch %s: aborted after %d/%d", state.batch_id, i, cfg.count)
                    return
                if rec_action == 'retry':
                    LOG.info("batch %s: retry requested at %d/%d", state.batch_id, i, cfg.count)
                    continue  # same i, redo setup → record
                # timeout or skip → advance
                i += 1

            state.phase = "done"
            LOG.info("batch %s: done (succeeded=%d failed=%d)",
                     state.batch_id, state.succeeded, state.failed)
        except Exception as exc:
            state.phase = "failed"
            state.last_error = str(exc)
            LOG.exception("batch %s: unexpected failure", state.batch_id)
        finally:
            state.finished_at = time.time()
            self._stop_evt = None
            self._retry_evt = None
            self._skip_evt = None
            self._task = None
