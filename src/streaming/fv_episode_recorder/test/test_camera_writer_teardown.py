"""Writers hand the unsubscribe to the executor thread without holding the lock
their executor-thread callbacks take, and a slow executor leaves the teardown
pending instead of destroying from the HTTP thread (#33 review). Needs
rclpy / sensor_msgs / cv2 / pyarrow (ROS env); the executor is a real thread
driven by spin_forever, the node is a stand-in."""

from __future__ import annotations

import functools
import queue
import threading
import time

import pytest

pytest.importorskip("numpy")
pytest.importorskip("cv2")
pytest.importorskip("pyarrow.parquet")
pytest.importorskip("rclpy")
from sensor_msgs.msg import Image  # noqa: E402

from fv_episode_recorder import camera_writer as cw  # noqa: E402
from fv_episode_recorder import executor_guard as eg  # noqa: E402


class _Log:
    def error(self, msg):
        pass

    def warning(self, msg):
        pass


class _Exec:
    """create_task queues; a spin_forever thread runs the tasks. ``gate``
    cleared = busy; ``before_task`` = what an executor callback does first."""

    def __init__(self) -> None:
        self._q: queue.Queue = queue.Queue()
        self.gate = threading.Event(); self.gate.set()
        self.before_task = None
        self._stop = threading.Event()
        self.state = eg.SpinState()
        self.thread = threading.Thread(
            target=eg.spin_forever, args=(self._spin_once, lambda: not self._stop.is_set()),
            kwargs={"state": self.state, "logger": _Log(), "retry_sleep_s": 0.0}, daemon=True)
        eg.attach(self, self.thread, self.state)
        self.thread.start()

    def create_task(self, cb, *args):
        self._q.put((cb, args))

    def _spin_once(self):
        try:
            cb, args = self._q.get(timeout=0.01)
        except queue.Empty:
            return
        self.gate.wait()
        if self.before_task is not None:
            self.before_task()
        cb(*args)

    def stop(self):
        self._stop.set(); self.gate.set(); self.thread.join(timeout=2)


class _Node:
    def __init__(self, executor):
        self.executor = executor
        self.destroyed: list[tuple[object, threading.Thread]] = []

    def create_subscription(self, *args, **kwargs):
        return object()

    def destroy_subscription(self, sub):
        self.destroyed.append((sub, threading.current_thread()))
        return True


@pytest.fixture
def ex():
    e = _Exec()
    yield e
    e.stop()


def test_depth_writer_stop_waits_for_the_executor_outside_its_lock(tmp_path, ex):
    node = _Node(ex)
    w = cw.DepthCameraWriter("d", "/d/image", tmp_path / "d", node=node)
    w.start()

    def _callback_takes_writer_lock():   # what DepthCameraWriter._on_image does on the executor
        assert w._lock.acquire(timeout=5)
        w._lock.release()
    ex.before_task = _callback_takes_writer_lock
    t0 = time.monotonic()
    w.stop()
    dt = time.monotonic() - t0
    # holding the lock while waiting = the executor stalls until the 2 s timeout
    assert dt < 1.0, dt
    assert w.unsubscribe == "executor"
    assert len(node.destroyed) == 1 and node.destroyed[0][1] is ex.thread


def test_depth_callback_that_waited_for_the_lock_does_not_write_after_stop(tmp_path, ex):
    w = cw.DepthCameraWriter("d", "/d/image", tmp_path / "d", node=_Node(ex))
    w.start()
    msg = Image(); msg.encoding = "16UC1"; msg.width = 2; msg.height = 2; msg.step = 4
    msg.data = bytes(8)
    w._lock.acquire()                         # stop() is in its locked section
    t = threading.Thread(target=w._on_image, args=(msg,))
    t.start()                                 # passed the unlocked _stopped check, waits for the lock
    time.sleep(0.05)
    w._stopped = True
    w._lock.release()
    t.join(timeout=2)
    assert w.frame_count == 0 and not list((tmp_path / "d").glob("*.png"))
    w.stop()


def test_pool_teardown_goes_to_the_executor_and_a_busy_one_leaves_it_pending(tmp_path, ex, monkeypatch):
    monkeypatch.setattr(cw, "destroy_on_executor",
                        functools.partial(eg.destroy_on_executor, timeout_s=0.1, poll_s=0.01))
    node = _Node(ex)
    pool = cw.CameraWriterPool(node=node, warm_up=False)
    ep = tmp_path / "ep"; ep.mkdir()
    cams = [{"name": "c0", "topic": "/c0/image/compressed", "camera_info_topic": "/c0/camera_info"}]

    pool.start_all(ep, cams)
    pool.stop_all()
    assert pool.teardown_pending == []
    assert len(node.destroyed) == 2 and all(th is ex.thread for _, th in node.destroyed)

    node.destroyed.clear()
    ep2 = tmp_path / "ep2"; ep2.mkdir()
    pool.start_all(ep2, cams)
    ex.gate.clear()                           # executor stuck in a long callback
    pool.stop_all()
    assert sorted(pool.teardown_pending) == ["c0", "camera_info"]
    assert node.destroyed == []               # nothing destroyed from this (HTTP) thread
    assert ex.state.destroy_timeouts == 2
    ex.gate.set()
    end = time.monotonic() + 2
    while len(node.destroyed) < 2 and time.monotonic() < end:
        time.sleep(0.01)
    time.sleep(0.05)
    assert len(node.destroyed) == 2 and all(th is ex.thread for _, th in node.destroyed)
    assert ex.state.snapshot()["pending_destroys"] == 0


def test_depth_republisher_failing_unsubscribe_still_destroys_the_publisher(ex):
    from fv_episode_recorder.depth_republisher import _DepthRepublisher

    class _N(_Node):
        def create_publisher(self, *args, **kwargs):
            return object()

        def destroy_subscription(self, sub):
            raise RuntimeError("already gone")

        def destroy_publisher(self, pub):
            self.destroyed.append((pub, threading.current_thread()))
    node = _N(ex)
    r = _DepthRepublisher(name="d", raw_topic="/d/depth", node=node)
    r.start()
    pub = r.pub
    assert r.stop() == "executor"
    assert node.destroyed == [(pub, ex.thread)]
    assert r.sub is None and r.pub is None
