"""executor_guard: the spin loop survives executor exceptions, and subscription teardown is
routed to the executor thread (pure pytest, no ROS runtime)."""

from __future__ import annotations

import threading

from fv_episode_recorder.executor_guard import SpinState, destroy_on_executor, spin_forever


class _Log:
    def __init__(self) -> None:
        self.lines: list[str] = []

    def error(self, msg):  # rclpy-style: message only, no positional args
        self.lines.append(msg)

    def warning(self, msg):
        self.lines.append(msg)


def test_spin_survives_exceptions_and_keeps_counting():
    calls = {"n": 0}

    def spin_once():
        calls["n"] += 1
        if calls["n"] in (2, 3):
            raise RuntimeError("cannot use Destroyable because destruction was requested")

    state = SpinState(); log = _Log()
    spin_forever(spin_once, lambda: calls["n"] < 6, state=state, logger=log, retry_sleep_s=0.0)
    assert calls["n"] == 6                      # kept spinning after the two failures
    assert state.failures == 2 and state.consecutive_failures == 0
    assert "destruction was requested" in state.last_error and len(log.lines) == 2
    assert state.alive is False                 # loop ended normally


def test_spin_gives_up_after_too_many_consecutive_failures():
    def spin_once():
        raise RuntimeError("wedged")
    state = SpinState()
    spin_forever(spin_once, lambda: True, state=state, logger=_Log(), retry_sleep_s=0.0, max_consecutive_failures=5)
    assert state.failures == 5 and state.consecutive_failures == 5 and state.alive is False


class _Task:
    def __init__(self, run_now: bool, exc: Exception | None = None):
        self._done = run_now; self._exc = exc; self.cancelled = False

    def done(self):
        return self._done

    def exception(self):
        return self._exc

    def cancel(self):
        self.cancelled = True


class _Executor:
    def __init__(self, run_now=True, raise_on_create=False):
        self.run_now = run_now; self.raise_on_create = raise_on_create; self.created = []
        self._fv_spin_thread = None

    def create_task(self, cb):
        if self.raise_on_create:
            raise RuntimeError("no executor")
        self.created.append(cb)
        if self.run_now:
            cb()
        return _Task(self.run_now)


class _Node:
    def __init__(self, executor):
        self.executor = executor; self.destroyed = 0


def test_destroy_runs_on_executor_when_it_is_responsive():
    ex = _Executor(run_now=True); node = _Node(ex); calls = []
    where = destroy_on_executor(node, lambda: calls.append("x"), logger=_Log())
    assert where == "executor" and calls == ["x"] and len(ex.created) == 1


def test_destroy_falls_back_when_executor_is_dead_or_absent():
    ex = _Executor(run_now=False); node = _Node(ex); calls = []; log = _Log()
    where = destroy_on_executor(node, lambda: calls.append("x"), timeout_s=0.02, poll_s=0.001, logger=log)
    assert where == "direct" and calls == ["x"] and any("did not run" in l for l in log.lines)
    node2 = _Node(None); calls2 = []
    assert destroy_on_executor(node2, lambda: calls2.append("y"), logger=_Log()) == "direct" and calls2 == ["y"]
    ex3 = _Executor(raise_on_create=True); node3 = _Node(ex3); calls3 = []
    assert destroy_on_executor(node3, lambda: calls3.append("z"), logger=_Log()) == "direct" and calls3 == ["z"]


def test_destroy_is_direct_when_called_from_the_spin_thread():
    ex = _Executor(run_now=False); ex._fv_spin_thread = threading.current_thread(); node = _Node(ex); calls = []
    assert destroy_on_executor(node, lambda: calls.append("x"), logger=_Log()) == "direct" and calls == ["x"] and ex.created == []
