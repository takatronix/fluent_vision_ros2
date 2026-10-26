"""Keep the recorder's ROS executor alive across rclpy's entity-destruction race.

Camera subscriptions are created at episode start and destroyed at episode
stop from the HTTP thread, while a SingleThreadedExecutor spins in its own
thread. rclpy (humble) can then raise inside the executor while it rebuilds
its wait set — ``InvalidHandle: cannot use Destroyable because destruction
was requested`` — if a subscription's QoS event handle is being torn down at
that moment. With a bare ``executor.spin()`` the exception kills the spin
thread silently: the HTTP API and the bag keep working, every later episode
is written with 0 video frames, and nothing reports it (2026-09-27 and
2026-10-25: 11 and 13 episodes lost).

Two layers, both without rclpy imports so they stay unit-testable:

- ``spin_forever``: spin in a loop and survive executor exceptions instead
  of letting one end the thread.
- ``destroy_on_executor``: hand the destruction to the executor thread
  itself (``Executor.create_task``), so the wait set is never rebuilt while
  an entity is half-destroyed. Falls back to a direct destroy when the
  executor is unavailable or does not get around to the task in time.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Callable, Optional

LOG = logging.getLogger("fv_episode_recorder.executor_guard")


class SpinState:
    """What the spin loop is doing, for /healthz and tests."""

    def __init__(self) -> None:
        self.alive = False
        self.failures = 0
        self.consecutive_failures = 0
        self.last_error: Optional[str] = None
        self.last_error_at: Optional[float] = None

    def snapshot(self) -> dict:
        return {
            "alive": self.alive,
            "failures": self.failures,
            "consecutive_failures": self.consecutive_failures,
            "last_error": self.last_error,
            "last_error_at": self.last_error_at,
        }


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
    and the loop gives up (logged as an error) rather than spinning hot.
    """
    st = state or SpinState()
    st.alive = True
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
                    logger.error("executor keeps failing; giving up the spin loop")
                    break
                time.sleep(retry_sleep_s)
    finally:
        st.alive = False
    return st


def destroy_on_executor(
    node: Any,
    destroy: Callable[[], Any],
    *,
    timeout_s: float = 2.0,
    poll_s: float = 0.005,
    logger: Any = LOG,
) -> str:
    """Run ``destroy`` on ``node``'s executor thread if it has one.

    Returns ``"executor"`` when the executor ran it, ``"direct"`` when it was
    run on the calling thread (no executor, called from the executor thread,
    or the executor did not run the task within ``timeout_s`` — e.g. its spin
    thread is dead).
    """
    executor = getattr(node, "executor", None)
    spin_thread = getattr(executor, "_fv_spin_thread", None) if executor is not None else None
    if executor is None or threading.current_thread() is spin_thread:
        destroy()
        return "direct"
    try:
        task = executor.create_task(destroy)
    except Exception as exc:  # noqa: BLE001
        logger.warning(f"could not schedule destroy on executor ({exc!r}); destroying directly")
        destroy()
        return "direct"
    deadline = time.monotonic() + timeout_s
    while not task.done() and time.monotonic() < deadline:
        time.sleep(poll_s)
    if task.done():
        exc = task.exception() if hasattr(task, "exception") else None
        if exc is not None:
            logger.warning(f"destroy on executor raised {exc!r}")
        return "executor"
    logger.warning(f"executor did not run destroy within {timeout_s:.1f}s; destroying directly")
    try:
        if hasattr(task, "cancel"):
            task.cancel()
    except Exception:  # noqa: BLE001
        pass
    destroy()
    return "direct"
