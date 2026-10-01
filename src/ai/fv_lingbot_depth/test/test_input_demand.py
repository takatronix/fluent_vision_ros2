"""Real ROS graph regression test, using synthetic images in isolated domain 136."""
import sys
from pathlib import Path
import threading
import time

import numpy as np
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger


sys.path.insert(0, str(Path(__file__).parents[1]))
from fv_lingbot_depth_py import lingbot_depth_node as module


@pytest.mark.parametrize("static_intrinsics", [False, True])
def test_demand_disconnect_resume_and_capture(static_intrinsics):
    args = ["--ros-args", "-p", "backend:=http", "-p", "pointcloud_topic:=''",
            "-p", "color_topic:=/test_lingbot/color",
            "-p", "depth_topic:=/test_lingbot/depth",
            "-p", "camera_info_topic:=/test_lingbot/info",
            "-p", "fallback_passthrough:=false", "-p", "capture_timeout_sec:=3.0",
            "-p", "demand_poll_sec:=0.05"]
    if static_intrinsics:
        args.extend(["-p", "static_intrinsics:=[100.0,100.0,16.0,12.0]"])
    rclpy.init(args=args, domain_id=136)
    node = module.FvLingbotDepthNode()
    probe = rclpy.create_node("lingbot_input_demand_probe")
    calls = []

    def refine(color, depth, intrinsics, **kwargs):
        calls.append(kwargs)
        return depth, np.ones(depth.shape, dtype=bool), None

    node._run_remote_model = refine
    inputs = [probe.create_publisher(kind, topic, qos_profile_sensor_data)
              for kind, topic in [(Image, "/test_lingbot/color"),
                                  (Image, "/test_lingbot/depth"),
                                  (CameraInfo, "/test_lingbot/info")]]
    capture_thread = None

    def send_frame():
        stamp = probe.get_clock().now().to_msg()
        color = Image(height=24, width=32, encoding="bgr8", step=96)
        color.header.stamp = stamp
        color.data = np.full((24, 32, 3), 100, dtype=np.uint8).tobytes()
        depth = Image(height=24, width=32, encoding="16UC1", step=64)
        depth.header.stamp = stamp
        depth.data = np.full((24, 32), 1000, dtype=np.uint16).tobytes()
        info = CameraInfo(height=24, width=32)
        info.header.stamp = stamp
        info.k = [100.0, 0.0, 16.0, 0.0, 100.0, 12.0, 0.0, 0.0, 1.0]
        for publisher, message in zip(inputs, (color, depth, info)):
            publisher.publish(message)

    def until(predicate, publish=False, timeout=4.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if publish:
                send_frame()
            rclpy.spin_once(node, timeout_sec=0.01)
            rclpy.spin_once(probe, timeout_sec=0.01)
            if predicate():
                return
        raise AssertionError("demand lifecycle timeout")

    def connected():
        return [p.get_subscription_count() for p in inputs] == (
            [1, 1, 0] if static_intrinsics else [1, 1, 1])

    def disconnected():
        return all(p.get_subscription_count() == 0 for p in inputs)

    try:
        assert node.sync is None
        assert disconnected()
        received = []
        output = probe.create_subscription(Image, node.refined_depth_topic,
                                           received.append, qos_profile_sensor_data)
        until(connected)
        first_sync = node.sync
        until(lambda: bool(received), publish=True)
        assert received[-1].encoding == "16UC1"
        assert calls
        probe.destroy_subscription(output)
        until(disconnected)
        assert node.sync is None

        # A different real consumer resumes inputs with an empty sync queue.
        masks = []
        output = probe.create_subscription(Image, node.mask_topic,
                                           masks.append, qos_profile_sensor_data)
        until(connected)
        assert node.sync is not first_sync
        assert all(not queue for queue in node.sync.queues)
        until(lambda: bool(masks), publish=True)
        node._on_depth_source(String(data="raw"))
        until(disconnected)

        # A capture wakes input subscriptions even with the selector paused.
        for source_selected in (False, True):
            probe.destroy_subscription(output) if output is not None else None
            output = None
            node._source_selected = source_selected
            until(disconnected)
            response = Trigger.Response()
            capture_thread = threading.Thread(
                target=node._on_capture, args=(Trigger.Request(), response))
            capture_thread.start()
            until(connected)
            until(lambda: not capture_thread.is_alive(), publish=True)
            capture_thread.join()
            assert response.success, response.message
            assert "continuous mode" not in response.message
            assert calls[-1]["resolution_level"] == node.capture_resolution_level
            until(disconnected)
            assert node.sync is None

        # Capture failure/timeout also releases input subscriptions.
        node.capture_timeout_sec = 0.4
        response = Trigger.Response()
        capture_thread = threading.Thread(
            target=node._on_capture, args=(Trigger.Request(), response))
        capture_thread.start()
        until(lambda: not capture_thread.is_alive())
        assert not response.success
        until(disconnected)
    finally:
        if capture_thread is not None:
            capture_thread.join(timeout=4)
        probe.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
