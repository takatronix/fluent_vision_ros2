import asyncio
import json
import socket
import threading
import time

import cv2
import numpy as np
import pytest


pytest.importorskip("rclpy")

import rclpy  # noqa: E402
from aiohttp import WSMsgType, web  # noqa: E402
from fv_episode_msgs.msg import EnvironmentChange, EnvironmentEvent  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import CompressedImage  # noqa: E402

from fv_episode_recorder.moss_realtime_adapter_node import (  # noqa: E402
    MossRealtimeAdapterNode,
)


class FakeMossRuntime:
    def __init__(self) -> None:
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(("127.0.0.1", 0))
        self.socket.listen(128)
        self.socket.setblocking(False)
        self.port = int(self.socket.getsockname()[1])
        self.ready = threading.Event()
        self.focused = threading.Event()
        self.frame_received = threading.Event()
        self.stop_requested = threading.Event()
        self.frame_bytes: bytes | None = None
        self.errors: list[Exception] = []
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self.thread.start()
        assert self.ready.wait(3.0)

    def stop(self) -> None:
        self.stop_requested.set()
        self.thread.join(timeout=3.0)
        assert not self.thread.is_alive()
        assert self.errors == []

    def _run(self) -> None:
        try:
            asyncio.run(self._serve())
        except Exception as exc:
            self.errors.append(exc)
            self.ready.set()

    async def _serve(self) -> None:
        app = web.Application()
        app.router.add_get("/v1/realtime", self._websocket)
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.SockSite(runner, self.socket)
        await site.start()
        self.ready.set()
        try:
            while not self.stop_requested.is_set():
                await asyncio.sleep(0.01)
        finally:
            await runner.cleanup()

    async def _websocket(self, request: web.Request) -> web.WebSocketResponse:
        websocket = web.WebSocketResponse()
        await websocket.prepare(request)
        expecting_frame = False
        async for message in websocket:
            if message.type == WSMsgType.TEXT:
                payload = json.loads(message.data)
                if payload.get("type") == "start":
                    await websocket.send_json({"type": "ready"})
                elif payload.get("type") == "prompt":
                    self.focused.set()
                elif payload.get("type") == "frame":
                    expecting_frame = True
                elif payload.get("type") == "stop":
                    break
            elif message.type == WSMsgType.BINARY and expecting_frame:
                expecting_frame = False
                self.frame_bytes = bytes(message.data)
                self.frame_received.set()
                await websocket.send_json({
                    "type": "frame_ack",
                    "dropped_oldest": False,
                })
                await websocket.send_json({
                    "type": "output",
                    "text": (
                        "<|round_start|><|response|>赤い工具箱が机に置かれた。"
                        "<|round_end|>"
                    ),
                })
        return websocket


def _spin_until(
    executor: SingleThreadedExecutor,
    condition,
    timeout_sec: float = 5.0,
) -> None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if condition():
            return
    raise AssertionError("timed out waiting for ROS pipeline")


def test_compressed_ros_topic_to_moss_annotation_event(tmp_path) -> None:
    runtime = FakeMossRuntime()
    runtime.start()
    database_path = tmp_path / "annotations.db"
    rclpy.init(args=[
        "--ros-args",
        "-p", f"runtime_url:=ws://127.0.0.1:{runtime.port}/v1/realtime",
        "-p", f"database_path:={database_path}",
        "-p", "image_topic:=/perception/moss/image/compressed",
        "-p", "sample_fps:=30.0",
    ])
    adapter = MossRealtimeAdapterNode()
    driver = Node("compressed_adapter_e2e_driver")
    executor = SingleThreadedExecutor()
    executor.add_node(adapter)
    executor.add_node(driver)
    received_events: list[EnvironmentEvent] = []
    change_pub = driver.create_publisher(
        EnvironmentChange,
        "/environment/change",
        10,
    )
    image_pub = driver.create_publisher(
        CompressedImage,
        "/perception/moss/image/compressed",
        10,
    )
    event_sub = driver.create_subscription(
        EnvironmentEvent,
        "/environment/event",
        received_events.append,
        10,
    )

    source = np.zeros((24, 32, 3), dtype=np.uint8)
    source[:, :, 2] = 220
    ok, encoded = cv2.imencode(".jpg", source)
    assert ok
    image = CompressedImage()
    image.format = "bgr8; jpeg compressed bgr8"
    image.data = encoded.tobytes()
    change = EnvironmentChange()
    change.episode_id = "compressed-e2e"
    change.state = "started"

    try:
        _spin_until(
            executor,
            lambda: change_pub.get_subscription_count() > 0
            and image_pub.get_subscription_count() > 0,
        )
        change_pub.publish(change)
        _spin_until(executor, runtime.focused.is_set)
        image_pub.publish(image)
        _spin_until(executor, runtime.frame_received.is_set)
        _spin_until(executor, lambda: len(received_events) == 1)

        assert runtime.frame_bytes == bytes(image.data)
        assert received_events[0].episode_id == "compressed-e2e"
        assert received_events[0].text == "赤い工具箱が机に置かれた。"
    finally:
        driver.destroy_subscription(event_sub)
        executor.remove_node(adapter)
        executor.remove_node(driver)
        adapter.destroy_node()
        driver.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        runtime.stop()
