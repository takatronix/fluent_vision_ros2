#!/usr/bin/env python3
"""
fv_browser_camera — Browser webcam bridge for online demos.

One aiohttp server (single port — free-tier hosts expose only one) serves:
  GET /            demo page (getUserMedia + model toggle + YOLOE prompt box)
  WS  /ws          binary up:   JPEG camera frames from the browser
                   text   up:   JSON control {"type": "model"|"prompt", ...}
                   binary down: JPEG overlay frames of the selected model
                   text   down: JSON status {"type": "status", ...}

Frames are decoded and published as sensor_msgs/Image to the *selected*
model's input topic only, so the idle detector receives nothing and sleeps.
Switching models is therefore instant (both detectors stay resident).

The first WebSocket connection is the "driver" (its camera is used); later
connections are viewers. When the driver leaves, the next sender takes over.
Frames are processed in memory only — nothing is written to disk.
"""

import asyncio
import json
import threading
import time
from pathlib import Path

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String

try:
    from aiohttp import web, WSMsgType
except ImportError as e:  # pragma: no cover
    raise SystemExit("fv_browser_camera requires aiohttp (pip install aiohttp)") from e


def _best_effort_qos(depth: int = 2) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class BrowserCameraNode(Node):
    """ROS side: publishes camera frames, republishes overlays to asyncio land."""

    def __init__(self):
        super().__init__('browser_camera')

        self.declare_parameter('port', 8096)
        self.declare_parameter('frame_id', 'browser_camera')
        # Models the page can switch between. Per model: where camera frames go
        # and where the annotated overlay comes back from.
        self.declare_parameter('models.coco.input_topic', '/fv/browser_cam/coco/image_raw')
        self.declare_parameter('models.coco.overlay_topic', '/fv/browser_demo/coco/overlay')
        self.declare_parameter('models.coco.overlay_compressed', False)
        self.declare_parameter('models.yoloe.input_topic', '/fv/browser_cam/yoloe/image_raw')
        self.declare_parameter('models.yoloe.overlay_topic', '/fv/browser_demo/yoloe/overlay/compressed')
        self.declare_parameter('models.yoloe.overlay_compressed', True)
        self.declare_parameter('yoloe_prompt_topic', '/fv/yoloe_1/set_prompt')
        self.declare_parameter('default_model', 'coco')
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('max_fps', 15.0)

        self.port = int(self.get_parameter('port').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.max_fps = float(self.get_parameter('max_fps').value)

        qos = _best_effort_qos()
        self.models = {}
        for key in ('coco', 'yoloe'):
            input_topic = str(self.get_parameter(f'models.{key}.input_topic').value)
            overlay_topic = str(self.get_parameter(f'models.{key}.overlay_topic').value)
            compressed = bool(self.get_parameter(f'models.{key}.overlay_compressed').value)
            entry = {
                'publisher': self.create_publisher(Image, input_topic, qos),
                'overlay_topic': overlay_topic,
                'compressed': compressed,
            }
            if compressed:
                self.create_subscription(
                    CompressedImage, overlay_topic,
                    lambda msg, k=key: self._on_overlay_jpeg(k, bytes(msg.data)), qos)
            else:
                self.create_subscription(
                    Image, overlay_topic,
                    lambda msg, k=key: self._on_overlay_raw(k, msg), qos)
            self.models[key] = entry

        self.prompt_pub = self.create_publisher(
            String, str(self.get_parameter('yoloe_prompt_topic').value), 10)

        self.active_model = str(self.get_parameter('default_model').value)
        if self.active_model not in self.models:
            self.active_model = 'coco'

        # Handoff to the asyncio side: latest overlay JPEG of the active model.
        self.overlay_lock = threading.Lock()
        self.latest_overlay: bytes | None = None
        self.overlay_seq = 0
        self.frames_in = 0
        self.last_frame_wall = 0.0

        self.get_logger().info(
            f'browser camera bridge on :{self.port} '
            f'(models: {", ".join(self.models)}, active: {self.active_model})')

    # ---- called from asyncio thread ----

    def publish_frame(self, jpeg: bytes) -> bool:
        now = time.monotonic()
        if now - self.last_frame_wall < 1.0 / self.max_fps:
            return False  # rate-limit: drop excess browser frames
        arr = np.frombuffer(jpeg, dtype=np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            return False
        self.last_frame_wall = now
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = bgr.shape[:2]
        msg.encoding = 'bgr8'
        msg.step = msg.width * 3
        msg.data = bgr.tobytes()
        self.models[self.active_model]['publisher'].publish(msg)
        self.frames_in += 1
        return True

    def set_model(self, key: str) -> bool:
        if key not in self.models:
            return False
        self.active_model = key
        with self.overlay_lock:
            self.latest_overlay = None  # don't show the other model's stale frame
        self.get_logger().info(f'model switched to {key}')
        return True

    def set_prompt(self, prompt: str):
        msg = String()
        msg.data = prompt[:200]
        self.prompt_pub.publish(msg)
        self.get_logger().info(f'YOLOE prompt set: {msg.data!r}')

    # ---- ROS callbacks ----

    def _on_overlay_jpeg(self, key: str, jpeg: bytes):
        if key != self.active_model:
            return
        with self.overlay_lock:
            self.latest_overlay = jpeg
            self.overlay_seq += 1

    def _on_overlay_raw(self, key: str, msg: Image):
        if key != self.active_model:
            return
        if msg.encoding not in ('bgr8', 'rgb8', 'mono8'):
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding == 'mono8':
            img = arr.reshape(msg.height, msg.width)
        else:
            img = arr.reshape(msg.height, msg.width, 3)
            if msg.encoding == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        ok, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if ok:
            self._on_overlay_jpeg(key, jpeg.tobytes())


class DemoServer:
    """aiohttp side: demo page + bidirectional WebSocket."""

    def __init__(self, node: BrowserCameraNode, web_root: Path):
        self.node = node
        self.web_root = web_root
        self.clients: set[web.WebSocketResponse] = set()
        self.driver: web.WebSocketResponse | None = None

    def build_app(self) -> web.Application:
        app = web.Application(client_max_size=4 * 1024 * 1024)
        app.router.add_get('/', self.handle_index)
        app.router.add_get('/ws', self.handle_ws)
        app.router.add_get('/healthz', lambda r: web.Response(text='ok'))
        return app

    async def handle_index(self, _request):
        return web.FileResponse(self.web_root / 'index.html')

    async def handle_ws(self, request):
        ws = web.WebSocketResponse(heartbeat=20)
        await ws.prepare(request)
        self.clients.add(ws)
        role = 'viewer'
        if self.driver is None or self.driver.closed:
            self.driver = ws
            role = 'driver'
        await ws.send_json({'type': 'hello', 'role': role,
                            'model': self.node.active_model})
        try:
            async for msg in ws:
                if msg.type == WSMsgType.BINARY:
                    if ws is self.driver:
                        self.node.publish_frame(msg.data)
                elif msg.type == WSMsgType.TEXT:
                    await self._handle_control(ws, msg.data)
                elif msg.type == WSMsgType.ERROR:
                    break
        finally:
            self.clients.discard(ws)
            if ws is self.driver:
                self.driver = None  # next sender may claim
        return ws

    async def _handle_control(self, ws, raw: str):
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            return
        kind = data.get('type')
        if kind == 'claim' and (self.driver is None or self.driver.closed):
            self.driver = ws
            await ws.send_json({'type': 'hello', 'role': 'driver',
                                'model': self.node.active_model})
        elif kind == 'model':
            if self.node.set_model(str(data.get('value', ''))):
                await self._broadcast_json({'type': 'model',
                                            'value': self.node.active_model})
        elif kind == 'prompt':
            self.node.set_prompt(str(data.get('value', '')))
            await ws.send_json({'type': 'prompt_ack'})

    async def _broadcast_json(self, payload: dict):
        for ws in list(self.clients):
            if not ws.closed:
                try:
                    await ws.send_json(payload)
                except ConnectionError:
                    pass

    async def overlay_pump(self):
        """Push new overlay JPEGs of the active model to all clients."""
        last_seq = -1
        while True:
            await asyncio.sleep(0.03)
            with self.node.overlay_lock:
                seq = self.node.overlay_seq
                data = self.node.latest_overlay
            if seq == last_seq or data is None:
                continue
            last_seq = seq
            for ws in list(self.clients):
                if not ws.closed:
                    try:
                        await ws.send_bytes(data)
                    except ConnectionError:
                        pass


def main(args=None):
    rclpy.init(args=args)
    node = BrowserCameraNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    web_root = Path(__file__).resolve().parent.parent / 'web'
    if not web_root.exists():  # installed layout
        from ament_index_python.packages import get_package_share_directory
        web_root = Path(get_package_share_directory('fv_browser_camera')) / 'web'

    server = DemoServer(node, web_root)
    app = server.build_app()

    async def run():
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, '0.0.0.0', node.port)
        await site.start()
        node.get_logger().info(f'demo page: http://0.0.0.0:{node.port}/')
        await server.overlay_pump()  # runs forever

    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
