"""executor_guard against the real rclpy executor (skipped without ROS).

The race behind the 0-frame episodes: subscriptions destroyed from another
thread while SingleThreadedExecutor rebuilds its wait set make
EventHandler.add_to_wait_set raise InvalidHandle, and a bare executor.spin()
thread dies. A short GIL switch interval makes the window easy to hit (on
Jazzy, origin/main's spin thread died within a second under this load).

- destroys routed through destroy_on_executor: no executor error at all, the
  spin keeps dispatching, every "episode" gets frames;
- destroys from the other thread (what the recorder did before): spin_forever
  survives whatever it hits and frames keep coming.
"""

from __future__ import annotations

import random
import sys
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("sensor_msgs.msg")
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import CompressedImage  # noqa: E402

from fv_episode_recorder.executor_guard import SpinState, attach, destroy_on_executor, spin_forever  # noqa: E402

N_CAMS = 3
RATE_HZ = 200.0


@pytest.fixture
def ros():
    """Own context on a random domain, a 'camera' node publishing frames on
    N_CAMS topics + a heartbeat, and a 'recorder' node spun like recorder_node."""
    ctx = rclpy.Context()
    rclpy.init(context=ctx, domain_id=random.randint(150, 230))
    tag = f"t{random.randint(0, 1 << 30):x}"
    cam = rclpy.create_node(f"fv_cam_{tag}", context=ctx)
    rec = rclpy.create_node(f"fv_rec_{tag}", context=ctx)
    topics = [f"/{tag}/cam{i}/image/compressed" for i in range(N_CAMS)]
    pubs = [cam.create_publisher(CompressedImage, t, qos_profile_sensor_data) for t in topics]
    hb_pub = cam.create_publisher(CompressedImage, f"/{tag}/heartbeat", qos_profile_sensor_data)
    payload = bytes(1024)

    def _tick():
        m = CompressedImage(); m.format = "jpeg"; m.data = payload
        for p in pubs:
            p.publish(m)
        hb_pub.publish(m)
    cam.create_timer(1.0 / RATE_HZ, _tick)
    cam_exec = SingleThreadedExecutor(context=ctx); cam_exec.add_node(cam)
    cam_thread = threading.Thread(target=cam_exec.spin, daemon=True)
    cam_thread.start()

    hb = {"n": 0}
    rec.create_subscription(CompressedImage, f"/{tag}/heartbeat",
                            lambda _m: hb.__setitem__("n", hb["n"] + 1), qos_profile_sensor_data)
    executor = SingleThreadedExecutor(context=ctx); executor.add_node(rec)
    state = SpinState()
    stop = threading.Event()
    thread = threading.Thread(
        target=spin_forever,
        args=(lambda: executor.spin_once(timeout_sec=0.2), lambda: ctx.ok() and not stop.is_set()),
        kwargs={"state": state, "logger": rec.get_logger(), "retry_sleep_s": 0.0},
        daemon=True, name="fv-recorder-spin")
    attach(executor, thread, state)
    thread.start()
    old_switch = sys.getswitchinterval()
    sys.setswitchinterval(1e-4)
    try:
        yield {"rec": rec, "topics": topics, "state": state, "thread": thread, "hb": hb}
    finally:
        sys.setswitchinterval(old_switch)
        stop.set()
        thread.join(timeout=2)
        executor.shutdown(timeout_sec=1.0)
        cam_exec.shutdown(timeout_sec=1.0)
        cam_thread.join(timeout=2)
        rec.destroy_node(); cam.destroy_node()
        rclpy.shutdown(context=ctx)


def _episodes(ros, destroy, seconds: float) -> dict:
    """Start/stop 'episodes' back to back from this (the HTTP) thread."""
    rec, topics = ros["rec"], ros["topics"]
    end = time.monotonic() + seconds
    episodes = zero = 0
    where: dict[str, int] = {}
    while time.monotonic() < end:
        counts = [0] * len(topics)
        subs = []
        for i, t in enumerate(topics):
            subs.append(rec.create_subscription(
                CompressedImage, t, lambda _m, i=i: counts.__setitem__(i, counts[i] + 1),
                qos_profile_sensor_data, callback_group=MutuallyExclusiveCallbackGroup()))
        deadline = time.monotonic() + 2.0
        while min(counts) == 0 and time.monotonic() < deadline:
            time.sleep(0.001)
        time.sleep(random.random() * 0.005)
        for s in subs:
            w = destroy(s)
            where[w] = where.get(w, 0) + 1
        episodes += 1
        zero += min(counts) == 0
    return {"episodes": episodes, "zero": zero, "where": where}


def _still_dispatching(ros) -> bool:
    n0 = ros["hb"]["n"]
    end = time.monotonic() + 2.0
    while ros["hb"]["n"] < n0 + 20 and time.monotonic() < end:
        time.sleep(0.01)
    return ros["hb"]["n"] >= n0 + 20


def test_routed_destroys_never_break_the_executor(ros):
    rec = ros["rec"]
    res = _episodes(ros, lambda s: destroy_on_executor(rec, lambda: rec.destroy_subscription(s)), 4.0)
    st = ros["state"]
    assert res["episodes"] > 20
    assert res["zero"] == 0, res
    assert set(res["where"]) == {"executor"}, res
    assert st.failures == 0, st.snapshot()           # no InvalidHandle: the race is gone
    assert st.healthy() and ros["thread"].is_alive()
    assert _still_dispatching(ros)


def test_spin_forever_survives_destroys_from_another_thread(ros):
    rec = ros["rec"]

    def _direct(s):
        rec.destroy_subscription(s)
        return "direct"
    res = _episodes(ros, _direct, 4.0)
    st = ros["state"]
    assert res["episodes"] > 20
    assert res["zero"] == 0, (res, st.snapshot())
    assert st.healthy() and ros["thread"].is_alive(), st.snapshot()
    if st.failures:
        assert "InvalidHandle" in (st.last_error or "") or "destruction" in (st.last_error or "")
    assert _still_dispatching(ros)
