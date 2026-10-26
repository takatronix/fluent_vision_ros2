"""executor_guard: the spin loop survives executor exceptions, and subscription teardown is
handed to the executor thread — exactly once, never re-run on the caller while the spin
thread lives (pure pytest, no ROS runtime: a real thread + Events stand in for the executor)."""

from __future__ import annotations

import queue
import threading
import time

from fv_episode_recorder.executor_guard import (
    SpinState, _DestroyRequest, attach, destroy_on_executor, spin_forever,
)


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
    assert state.alive is False and state.exit_reason == "shutdown" and not state.gave_up


def test_spin_gives_up_after_too_many_consecutive_failures():
    def spin_once():
        raise RuntimeError("wedged")
    state = SpinState()
    spin_forever(spin_once, lambda: True, state=state, logger=_Log(), retry_sleep_s=0.0, max_consecutive_failures=5)
    assert state.failures == 5 and state.consecutive_failures == 5 and state.alive is False
    assert state.gave_up and state.exit_reason == "gave_up" and not state.healthy()
    snap = state.snapshot()
    assert snap["healthy"] is False and snap["gave_up"] is True


class _ThreadExecutor:
    """Stands in for rclpy's executor: create_task queues the callback, a real
    thread running spin_forever takes tasks one at a time (like spin_once).
    ``gate`` cleared = the executor is busy (e.g. in a long callback);
    ``before_task`` runs on the executor thread before each task."""

    def __init__(self) -> None:
        self._q: queue.Queue = queue.Queue()
        self.gate = threading.Event(); self.gate.set()
        self.before_task = None
        self._stop = threading.Event()
        self.state = SpinState()
        self.log = _Log()
        self.thread = threading.Thread(
            target=spin_forever, args=(self._spin_once, lambda: not self._stop.is_set()),
            kwargs={"state": self.state, "logger": self.log, "retry_sleep_s": 0.0},
            daemon=True, name="fake-spin")
        attach(self, self.thread, self.state)
        self.thread.start()

    def create_task(self, cb, *args):
        self._q.put((cb, args))
        return object()

    def _spin_once(self):
        try:
            cb, args = self._q.get(timeout=0.01)
        except queue.Empty:
            return
        self.gate.wait()
        if self._stop.is_set():
            self.leftover = (cb, args)        # the executor stops without running it
            return
        if self.before_task is not None:
            self.before_task()
        cb(*args)

    def stop(self):
        self._stop.set(); self.gate.set(); self.thread.join(timeout=2)
        assert not self.thread.is_alive()


class _Node:
    def __init__(self, executor):
        self.executor = executor


class _Destroy:
    """Counts runs and remembers the thread; ``block`` holds a run in progress."""

    def __init__(self, block: threading.Event | None = None):
        self.threads: list[threading.Thread] = []
        self.started = threading.Event()
        self.block = block

    def __call__(self):
        self.threads.append(threading.current_thread())
        self.started.set()
        if self.block is not None:
            assert self.block.wait(5)


def _wait(cond, timeout=2.0):
    end = time.monotonic() + timeout
    while not cond() and time.monotonic() < end:
        time.sleep(0.002)
    return cond()


def test_destroy_runs_on_the_spin_thread():
    ex = _ThreadExecutor()
    try:
        d = _Destroy()
        where = destroy_on_executor(_Node(ex), d, logger=_Log())
        assert where == "executor" and d.threads == [ex.thread]
        assert ex.state.snapshot()["pending_destroys"] == 0 and ex.state.destroy_timeouts == 0
    finally:
        ex.stop()


def test_timeout_leaves_the_destroy_with_the_executor_and_it_runs_once():
    ex = _ThreadExecutor()
    try:
        ex.gate.clear()                                  # executor busy past the timeout
        d = _Destroy(); log = _Log()
        where = destroy_on_executor(_Node(ex), d, timeout_s=0.05, poll_s=0.005, logger=log)
        assert where == "pending" and d.threads == []    # not run on the calling thread
        assert ex.state.destroy_timeouts == 1 and ex.state.snapshot()["pending_destroys"] == 1
        assert any("left with the executor" in line for line in log.lines)
        ex.gate.set()                                    # executor gets to it later
        assert _wait(lambda: len(d.threads) == 1)
        time.sleep(0.05)
        assert d.threads == [ex.thread]                  # once, on the spin thread
        assert ex.state.snapshot()["pending_destroys"] == 0
    finally:
        ex.stop()


def test_timeout_while_the_executor_is_mid_destroy_does_not_destroy_again():
    ex = _ThreadExecutor()
    try:
        release = threading.Event()
        d = _Destroy(block=release)                      # the executor started it, still running
        out = {}
        t = threading.Thread(target=lambda: out.setdefault(
            "where", destroy_on_executor(_Node(ex), d, timeout_s=0.1, poll_s=0.005, logger=_Log())))
        t.start()
        assert d.started.wait(2)
        t.join(timeout=2)
        assert out["where"] == "pending"                 # timed out, but did not run it itself
        release.set()
        assert _wait(lambda: ex.state.snapshot()["pending_destroys"] == 0)
        assert d.threads == [ex.thread]
    finally:
        release.set(); ex.stop()


def test_spin_exit_runs_what_is_still_pending_exactly_once():
    ex = _ThreadExecutor()
    ex.gate.clear()
    d = _Destroy()
    assert destroy_on_executor(_Node(ex), d, timeout_s=0.05, poll_s=0.005, logger=_Log()) == "pending"
    ex.stop()                                            # the loop ends without running the task
    assert d.threads == [ex.thread]                      # ... but runs it on its way out
    cb, args = ex.leftover
    cb(*args)                                            # the stale executor task fires later anyway
    assert len(d.threads) == 1


def test_requests_handed_over_while_the_exit_drain_runs_are_not_lost():
    ex = _ThreadExecutor()
    ex.gate.clear()
    release_a = threading.Event()
    a = _Destroy(block=release_a)
    assert destroy_on_executor(_Node(ex), a, timeout_s=0.05, poll_s=0.005, logger=_Log()) == "pending"
    ex._stop.set(); ex.gate.set()               # loop ends; its exit drain starts on A and blocks
    assert a.started.wait(2)
    b = _Destroy()
    assert destroy_on_executor(_Node(ex), b, timeout_s=0.05, poll_s=0.005, logger=_Log()) == "pending"
    assert b.threads == []                      # spin thread still alive: not run here
    release_a.set()
    ex.thread.join(timeout=2)
    assert not ex.thread.is_alive()
    assert a.threads == [ex.thread] and b.threads == [ex.thread]   # the same drain took B
    assert ex.state._add_pending(_DestroyRequest(lambda: None)) is False   # closed after the drain
    c = _Destroy()
    assert destroy_on_executor(_Node(ex), c, logger=_Log()) == "direct" and len(c.threads) == 1


def test_direct_only_once_the_spin_thread_has_ended():
    ex = _ThreadExecutor(); ex.stop()
    d = _Destroy()
    assert destroy_on_executor(_Node(ex), d, logger=_Log()) == "direct"
    assert d.threads == [threading.current_thread()]


def test_spin_thread_ending_during_the_wait_hands_over_without_a_second_run():
    ex = _ThreadExecutor()
    ex.gate.clear()
    d = _Destroy(); out = {}
    t = threading.Thread(target=lambda: out.setdefault(
        "where", destroy_on_executor(_Node(ex), d, timeout_s=5.0, poll_s=0.005, logger=_Log())))
    t.start()
    assert _wait(lambda: ex.state.snapshot()["pending_destroys"] == 1)
    ex.stop()                                            # spin thread ends while the caller waits
    t.join(timeout=2)
    assert not t.is_alive() and out["where"] == "executor"   # run by the spin thread on exit
    assert d.threads == [ex.thread]


def test_destroy_request_runs_once_under_a_race():
    for _ in range(200):
        n = []
        req = _DestroyRequest(lambda: n.append(1))
        go = threading.Barrier(3)

        def _race():
            go.wait(); req.run("x")
        ts = [threading.Thread(target=_race) for _ in range(2)]
        for t in ts:
            t.start()
        go.wait()
        for t in ts:
            t.join()
        assert n == [1] and req.done.is_set()


def test_destroy_error_is_logged_not_raised():
    ex = _ThreadExecutor()
    try:
        log = _Log()

        def boom():
            raise RuntimeError("already gone")
        assert destroy_on_executor(_Node(ex), boom, logger=log) == "executor"
        assert any("already gone" in line for line in log.lines)
        assert ex.state.failures == 0                    # the spin loop did not see it
    finally:
        ex.stop()


def test_no_executor_or_called_from_the_spin_thread_is_direct():
    d = _Destroy()
    assert destroy_on_executor(_Node(None), d, logger=_Log()) == "direct" and len(d.threads) == 1
    ex = _ThreadExecutor()
    try:
        d2 = _Destroy(); out = {}
        ex.create_task(lambda: out.setdefault("where", destroy_on_executor(_Node(ex), d2, logger=_Log())))
        assert _wait(lambda: "where" in out)
        assert out["where"] == "direct" and d2.threads == [ex.thread]   # no self-deadlock
    finally:
        ex.stop()


def test_unknown_spin_thread_never_falls_back_to_a_direct_destroy():
    class _Exec:                                   # an executor nobody attached a SpinState to
        def __init__(self):
            self.tasks = []

        def create_task(self, cb, *args):
            self.tasks.append((cb, args))
    ex = _Exec(); d = _Destroy()
    assert destroy_on_executor(_Node(ex), d, timeout_s=0.02, poll_s=0.005, logger=_Log()) == "pending"
    assert d.threads == []
    cb, args = ex.tasks[0]; cb(*args); cb(*args)
    assert len(d.threads) == 1
