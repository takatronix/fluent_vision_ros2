"""Keep the recorder's ROS executor alive across rclpy's entity-destruction race.

Camera subscriptions are created at episode start and destroyed at episode
stop from the HTTP thread, while a SingleThreadedExecutor spins in its own
thread. rclpy (humble, jazzy) can then raise inside the executor while it
rebuilds its wait set — ``InvalidHandle: cannot use Destroyable because
destruction was requested`` from ``EventHandler.add_to_wait_set`` — if a
subscription's QoS event handle is being torn down at that moment. With a
bare ``executor.spin()`` the exception kills the spin thread silently: the
HTTP API and the bag keep working, every later episode is written with 0
video frames, and nothing reports it (2026-09-27 and 2026-10-25: 11 and 13
episodes lost).

Two layers, both without rclpy imports so they stay unit-testable:

- ``spin_forever``: spin in a loop and survive executor exceptions instead
  of letting one end the thread. ``SpinState.healthy()`` turns false when it
  gives up (or the thread ends); the API then refuses new episodes and does
  not save the running one as a success.
- ``destroy_on_executor``: hand the destruction to the executor thread
  itself (``Executor.create_task``), so the wait set is never rebuilt while
  an entity is half-destroyed. The destroy runs exactly once. If the
  executor does not get to it in time, the request stays with the executor
  (a stop failure, counted in ``SpinState``) — it is only run on the calling
  thread once the spin thread has really ended, and whatever is still
  pending when the spin loop exits is run there on its way out.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Callable, Optional

LOG = logging.getLogger("fv_episode_recorder.executor_guard")


class _DestroyRequest:
    """One destroy, run by whichever thread claims it first (never twice)."""

    def __init__(self, destroy: Callable[[], Any],
                 on_done: Optional[Callable[["_DestroyRequest"], None]] = None) -> None:
        self._destroy = destroy
        self._on_done = on_done
        self._lock = threading.Lock()
        self._claimed = False
        self.done = threading.Event()
        self.ran_on: Optional[str] = None
        self.error: Optional[BaseException] = None

    def run(self, where: str) -> bool:
        """Run the destroy here unless another thread already took it.
        Returns True when this call ran it. Exceptions are kept, not raised."""
        with self._lock:
            if self._claimed:
                return False
            self._claimed = True
        try:
            self._destroy()
        except Exception as exc:  # noqa: BLE001 — reported by the caller
            self.error = exc
        finally:
            self.ran_on = where
            self.done.set()
            if self._on_done is not None:
                self._on_done(self)
        return True


class SpinState:
    """What the spin loop is doing, for /healthz, the episode API and tests."""

    def __init__(self) -> None:
        self.alive = False
        self.gave_up = False
        self.exit_reason: Optional[str] = None  # shutdown | gave_up | crashed: ...
        self.failures = 0
        self.consecutive_failures = 0
        self.last_error: Optional[str] = None
        self.last_error_at: Optional[float] = None
        # destroys the executor did not run within the timeout (stop failures)
        self.destroy_timeouts = 0
        self.last_destroy_timeout_at: Optional[float] = None
        self.thread: Optional[threading.Thread] = None
        self._pending: set[_DestroyRequest] = set()
        self._pending_lock = threading.Lock()
        self._closed = False  # the exiting spin loop has run its last pending destroy

    def thread_ended(self) -> bool:
        """True only once the spin thread has really finished (it can no
        longer touch the wait set)."""
        return self.thread is not None and not self.thread.is_alive()

    def healthy(self) -> bool:
        """Callbacks are being dispatched: camera frames reach the writers."""
        if self.gave_up or not self.alive:
            return False
        return self.thread is None or self.thread.is_alive()

    def snapshot(self) -> dict:
        with self._pending_lock:
            pending = len(self._pending)
        return {
            "alive": self.alive,
            "healthy": self.healthy(),
            "gave_up": self.gave_up,
            "exit_reason": self.exit_reason,
            "failures": self.failures,
            "consecutive_failures": self.consecutive_failures,
            "last_error": self.last_error,
            "last_error_at": self.last_error_at,
            "destroy_timeouts": self.destroy_timeouts,
            "last_destroy_timeout_at": self.last_destroy_timeout_at,
            "pending_destroys": pending,
        }

    def _add_pending(self, req: _DestroyRequest) -> bool:
        """False once the exiting spin loop has stopped taking requests (it
        will not touch the wait set again: the caller may destroy directly)."""
        with self._pending_lock:
            if self._closed:
                return False
            self._pending.add(req)
            return True

    def _drop_pending(self, req: _DestroyRequest) -> None:
        with self._pending_lock:
            self._pending.discard(req)

    def _run_pending_on_exit(self, logger: Any) -> None:
        """The spin loop is ending: nobody else will run the destroys left
        with the executor, so run them on this (the spin) thread — including
        ones handed over while these run — then stop taking requests."""
        while True:
            with self._pending_lock:
                if not self._pending:
                    self._closed = True
                    return
                pending, self._pending = list(self._pending), set()
            for req in pending:
                if req.run("spin_exit") and req.error is not None:
                    logger.warning(f"pending destroy raised at spin exit: {req.error!r}")


def attach(executor: Any, thread: threading.Thread, state: SpinState) -> None:
    """Tell ``destroy_on_executor`` which thread spins ``executor``."""
    state.thread = thread
    executor._fv_spin_state = state  # type: ignore[attr-defined]


def spin_forever(
    spin_once: Callable[[], None],
    is_ok: Callable[[], bool],
    *,
    state: Optional[SpinState] = None,
    logger: Any = LOG,
    retry_sleep_s: float = 0.05,
    max_consecutive_failures: int = 200,
) -> SpinState:
    """Call ``spin_once`` until ``is_ok()`` is false, surviving exceptions.

    An exception is logged and counted, then spinning continues after a short
    sleep; ``max_consecutive_failures`` in a row means the executor is wedged
    and the loop gives up (logged as an error, ``state.gave_up``) rather than
    spinning hot.
    """
    st = state or SpinState()
    st.alive = True
    reason = "shutdown"
    try:
        while is_ok():
            try:
                spin_once()
                st.consecutive_failures = 0
            except Exception as exc:  # noqa: BLE001 — anything from rclpy must not end the thread
                st.failures += 1
                st.consecutive_failures += 1
                st.last_error = f"{type(exc).__name__}: {exc}"
                st.last_error_at = time.time()
                # pre-formatted: rclpy's node logger takes no positional args
                logger.error(
                    f"executor spin failed ({st.failures} total, "
                    f"{st.consecutive_failures} in a row): {st.last_error} — continuing")
                if st.consecutive_failures >= max_consecutive_failures:
                    logger.error("executor keeps failing; giving up the spin loop — "
                                 "camera frames are no longer recorded, new episodes are refused")
                    st.gave_up = True
                    reason = "gave_up"
                    break
                time.sleep(retry_sleep_s)
    except BaseException as exc:
        reason = f"crashed: {type(exc).__name__}: {exc}"
        raise
    finally:
        st.exit_reason = reason
        st.alive = False
        st._run_pending_on_exit(logger)
    return st


def destroy_on_executor(
    node: Any,
    destroy: Callable[[], Any],
    *,
    timeout_s: float = 2.0,
    poll_s: float = 0.05,
    logger: Any = LOG,
) -> str:
    """Run ``destroy`` on ``node``'s executor thread if it has one; never twice.

    Returns
    - ``"executor"``: the spin thread ran it (also on its way out).
    - ``"direct"``: run on the calling thread — no executor, called from the
      spin thread, or the spin thread has ended (nothing can race it).
    - ``"pending"``: the executor did not run it within ``timeout_s`` while
      its spin thread is still alive. The request stays with the executor and
      is counted as a stop failure (``SpinState.destroy_timeouts``); running
      it here would bring the wait-set race back.

    An exception from ``destroy`` is logged, not raised.
    """
    executor = getattr(node, "executor", None)
    state: Optional[SpinState] = getattr(executor, "_fv_spin_state", None) if executor is not None else None
    req = _DestroyRequest(destroy, on_done=state._drop_pending if state is not None else None)

    def _here(where: str = "direct") -> str:
        req.run(where)
        if req.error is not None:
            logger.warning(f"destroy raised {req.error!r}")
        return where

    if executor is None:
        return _here()
    if state is not None and (threading.current_thread() is state.thread or state.thread_ended()):
        return _here()

    if state is not None and not state._add_pending(req):
        return _here()  # the spin loop already ran its last pending destroy and is ending
    try:
        executor.create_task(req.run, "executor")
    except Exception as exc:  # noqa: BLE001 — e.g. executor already shut down
        # create_task may have queued the task before failing; the claim in
        # _DestroyRequest keeps it from running twice either way.
        logger.warning(f"could not schedule destroy on executor ({exc!r})")
    deadline = time.monotonic() + timeout_s
    while not req.done.is_set():
        if state is not None and state.thread_ended():
            # the spin thread is gone: either it ran the request on its way
            # out (claimed), or nothing ever will and it is safe here
            req.run("direct")
            break
        left = deadline - time.monotonic()
        if left <= 0:
            break
        req.done.wait(min(poll_s, left))
    if req.done.is_set():
        if req.error is not None:
            logger.warning(f"destroy raised {req.error!r}")
        return "direct" if req.ran_on == "direct" else "executor"
    if state is not None:
        state.destroy_timeouts += 1
        state.last_destroy_timeout_at = time.time()
    logger.error(f"executor did not run destroy within {timeout_s:.1f}s; "
                 "left with the executor (stop failure) — not destroying from this thread")
    return "pending"
