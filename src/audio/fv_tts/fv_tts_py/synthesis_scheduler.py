from __future__ import annotations

from collections import deque
from collections.abc import Callable
from threading import Condition, Thread
from typing import Literal

from .contracts import SayRequest
from .voicevox_backend import SynthesizedAudio


SubmitStatus = Literal["accepted", "stale", "cancelled"]
_MAX_SYSTEM_ABORT_TOMBSTONES = 64


def agent_floor_epoch(utterance_id: str) -> int:
    parts = utterance_id.split("-", 2)
    if len(parts) != 3 or parts[0] != "agent":
        raise ValueError("agent utterance_id must be agent-<floor_epoch>-<id>")
    try:
        epoch = int(parts[1])
    except ValueError as exc:
        raise ValueError("agent utterance_id floor epoch is invalid") from exc
    if epoch < 0:
        raise ValueError("agent utterance_id floor epoch is negative")
    return epoch


class SynthesisScheduler:
    """Serializes voicevox_core calls while prioritizing valid SYSTEM speech."""

    def __init__(
        self,
        synthesize: Callable[[str], SynthesizedAudio],
        completed: Callable[[SayRequest, SynthesizedAudio], None],
        failed: Callable[[SayRequest, Exception], None],
        cancelled: Callable[[SayRequest], None],
    ) -> None:
        self._synthesize = synthesize
        self._completed = completed
        self._failed = failed
        self._cancelled = cancelled
        self._agent: deque[SayRequest] = deque()
        self._system: deque[SayRequest] = deque()
        self._minimum_agent_epoch = 0
        self._cancelled_system: set[str] = set()
        self._cancelled_system_order: deque[str] = deque()
        self._active: SayRequest | None = None
        self._closed = False
        self._condition = Condition()
        self._thread = Thread(
            target=self._run,
            name="voicevox-core",
            daemon=True,
        )
        self._thread.start()

    def submit(self, request: SayRequest) -> SubmitStatus:
        epoch = agent_floor_epoch(request.utterance_id) if request.kind == "agent" else None
        with self._condition:
            if self._closed:
                raise RuntimeError("synthesis scheduler is closed")
            if request.kind == "agent":
                if epoch < self._minimum_agent_epoch:
                    status: SubmitStatus = "stale"
                else:
                    self._agent.append(request)
                    status = "accepted"
            else:
                if request.utterance_id in self._cancelled_system:
                    self._forget_cancelled_system(request.utterance_id)
                    status = "cancelled"
                else:
                    self._system.append(request)
                    status = "accepted"
            if status == "accepted":
                self._condition.notify()
        if status != "accepted":
            self._report_cancelled(request)
        return status

    def advance_agent_floor(self, floor_epoch: int) -> tuple[str, ...]:
        if floor_epoch < 0:
            raise ValueError("agent floor epoch must be non-negative")
        with self._condition:
            self._minimum_agent_epoch = max(self._minimum_agent_epoch, floor_epoch)
            kept: deque[SayRequest] = deque()
            removed: list[SayRequest] = []
            while self._agent:
                request = self._agent.popleft()
                if agent_floor_epoch(request.utterance_id) < self._minimum_agent_epoch:
                    removed.append(request)
                else:
                    kept.append(request)
            self._agent = kept
        for request in removed:
            self._report_cancelled(request)
        return tuple(request.utterance_id for request in removed)

    def cancel_system(self, utterance_id: str) -> bool:
        if not utterance_id:
            raise ValueError("system cancellation requires utterance_id")
        with self._condition:
            kept: deque[SayRequest] = deque()
            removed: list[SayRequest] = []
            while self._system:
                request = self._system.popleft()
                if request.utterance_id == utterance_id:
                    removed.append(request)
                else:
                    kept.append(request)
            self._system = kept
            active = (
                self._active is not None
                and self._active.kind == "system"
                and self._active.utterance_id == utterance_id
            )
            if active or not removed:
                self._remember_cancelled_system(utterance_id)
        for request in removed:
            self._report_cancelled(request)
        return bool(removed) or active

    def close(self) -> None:
        with self._condition:
            if self._closed:
                return
            self._closed = True
            self._agent.clear()
            self._system.clear()
            self._condition.notify()
        self._thread.join()

    def _run(self) -> None:
        while True:
            with self._condition:
                self._condition.wait_for(
                    lambda: self._closed or self._system or self._agent
                )
                if self._closed:
                    return
                request = self._system.popleft() if self._system else self._agent.popleft()
                self._active = request
            try:
                audio = self._synthesize(request.text)
            except Exception as exc:  # Native synthesis boundary.
                if not self._commit_failure_if_current(request, exc):
                    self._report_cancelled(request)
            else:
                if not self._commit_audio_if_current(request, audio):
                    self._report_cancelled(request)
            finally:
                with self._condition:
                    if request.kind == "system":
                        self._forget_cancelled_system(request.utterance_id)
                    self._active = None

    def _is_current_locked(self, request: SayRequest) -> bool:
        """Return eligibility while the scheduler condition is held."""
        if self._closed:
            return False
        if request.kind == "agent":
            return agent_floor_epoch(request.utterance_id) >= self._minimum_agent_epoch
        return request.utterance_id not in self._cancelled_system

    def _commit_audio_if_current(
        self, request: SayRequest, audio: SynthesizedAudio
    ) -> bool:
        # Eligibility and PCM publication are one linearized commit. A floor
        # advance or SYSTEM abort that returns first therefore cannot be
        # followed by stale PCM from this scheduler.
        with self._condition:
            if not self._is_current_locked(request):
                return False
            try:
                self._completed(request, audio)
            except Exception as exc:
                self._report_failure(request, exc)
            return True

    def _commit_failure_if_current(self, request: SayRequest, exc: Exception) -> bool:
        with self._condition:
            if not self._is_current_locked(request):
                return False
            self._report_failure(request, exc)
            return True

    def _report_failure(self, request: SayRequest, exc: Exception) -> None:
        try:
            self._failed(request, exc)
        except Exception:
            pass

    def _report_cancelled(self, request: SayRequest) -> None:
        try:
            self._cancelled(request)
        except Exception as exc:
            self._report_failure(request, exc)

    def _remember_cancelled_system(self, utterance_id: str) -> None:
        if utterance_id in self._cancelled_system:
            return
        if len(self._cancelled_system_order) >= _MAX_SYSTEM_ABORT_TOMBSTONES:
            oldest = self._cancelled_system_order.popleft()
            self._cancelled_system.discard(oldest)
        self._cancelled_system.add(utterance_id)
        self._cancelled_system_order.append(utterance_id)

    def _forget_cancelled_system(self, utterance_id: str) -> None:
        if utterance_id not in self._cancelled_system:
            return
        self._cancelled_system.remove(utterance_id)
        self._cancelled_system_order.remove(utterance_id)
