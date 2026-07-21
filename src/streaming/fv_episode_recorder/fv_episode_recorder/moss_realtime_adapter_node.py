"""ROS 2 adapter for the official MOSS-VL-Realtime WebSocket service."""

from __future__ import annotations

import asyncio
import json
import queue
import threading
import time
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import rclpy
from aiohttp import ClientSession, ClientTimeout, ClientWebSocketResponse, WSMsgType
from fv_episode_msgs.msg import EnvironmentChange, EnvironmentEvent
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

from .moss_rounds import EpisodeRoundOwnership, MossRoundParser


@dataclass(frozen=True)
class EncodedFrame:
    timestamp: float
    data: bytes


@dataclass(frozen=True)
class FocusPrompt:
    episode_id: str
    phase: str
    text: str


class MossRealtimeAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("moss_realtime_adapter")

        self.declare_parameter("image_topic", "/aspa/restamped/color_compressed")
        self.declare_parameter("image_type", "compressed")
        self.declare_parameter("environment_change_topic", "/environment/change")
        self.declare_parameter("environment_event_topic", "/environment/event")
        self.declare_parameter(
            "environment_annotation_topic", "/environment/annotation"
        )
        self.declare_parameter("runtime_url", "ws://127.0.0.1:18081/v1/realtime")
        self.declare_parameter("sample_fps", 1.0)
        self.declare_parameter("jpeg_quality", 85)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.image_type = str(self.get_parameter("image_type").value).strip().lower()
        self.change_topic = str(self.get_parameter("environment_change_topic").value)
        self.event_topic = str(self.get_parameter("environment_event_topic").value)
        self.annotation_topic = str(
            self.get_parameter("environment_annotation_topic").value
        )
        self.runtime_url = str(self.get_parameter("runtime_url").value)
        self.sample_fps = max(0.01, float(self.get_parameter("sample_fps").value))
        self.jpeg_quality = min(100, max(1, int(self.get_parameter("jpeg_quality").value)))

        self.event_pub = self.create_publisher(EnvironmentEvent, self.event_topic, 10)
        self.annotation_pub = self.create_publisher(
            EnvironmentEvent, self.annotation_topic, 10
        )
        self.change_sub = self.create_subscription(
            EnvironmentChange,
            self.change_topic,
            self._on_environment_change,
            10,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        if self.image_type == "compressed":
            self.image_sub = self.create_subscription(
                CompressedImage,
                self.image_topic,
                self._on_compressed_image,
                sensor_qos,
            )
        elif self.image_type == "raw":
            self.image_sub = self.create_subscription(
                Image,
                self.image_topic,
                self._on_image,
                sensor_qos,
            )
        else:
            raise ValueError("image_type must be 'compressed' or 'raw'")

        self._frames: queue.Queue[EncodedFrame] = queue.Queue(maxsize=1)
        self._prompts: queue.Queue[FocusPrompt] = queue.Queue()
        self._episode_lock = threading.Lock()
        self._ownership = EpisodeRoundOwnership()
        self._steered_episode_ids: set[str] = set()
        self._focus_started_at: dict[str, float] = {}
        self._frame_acks = 0
        self._runtime_frame_drops = 0
        self._stream_started = time.monotonic()
        self._last_frame_at = 0.0
        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._run_worker, daemon=True)
        self._worker.start()

        self.get_logger().info(
            "MOSS realtime adapter started: "
            f"image={self.image_topic} image_type={self.image_type} "
            f"runtime={self.runtime_url} sample_fps={self.sample_fps:g}"
        )

    def _on_environment_change(self, msg: EnvironmentChange) -> None:
        if not msg.episode_id:
            return
        if msg.state == "started":
            with self._episode_lock:
                self._ownership.start(msg.episode_id)
                self._focus_started_at = {msg.episode_id: time.monotonic()}
            self._prompts.put(FocusPrompt(
                episode_id=msg.episode_id,
                phase="active",
                text=(
                    "An anomaly episode has started. Focus on the current scene change. "
                    "Describe only what visually changed. Stay silent until there is "
                    "meaningful evidence."
                ),
            ))
        elif msg.state == "ended":
            with self._episode_lock:
                finalizing = self._ownership.end(msg.episode_id)
            if finalizing:
                self._prompts.put(FocusPrompt(
                    episode_id=msg.episode_id,
                    phase="finalizing",
                    text=(
                        "The anomaly episode has ended. Complete or correct the concise "
                        "description now. Do not start a new unrelated observation."
                    ),
                ))
        else:
            self.get_logger().warning(f"ignored invalid environment state: {msg.state}")

    def _on_compressed_image(self, msg: CompressedImage) -> None:
        now = time.monotonic()
        if not self._should_sample(now):
            return
        data = bytes(msg.data)
        if not self._is_supported_encoded_image(data):
            self.get_logger().warning(
                f"unsupported compressed image format: {msg.format or 'unknown'}"
            )
            return
        self._queue_frame(now, data)

    def _on_image(self, msg: Image) -> None:
        now = time.monotonic()
        if not self._should_sample(now):
            return
        try:
            image = self._image_to_bgr(msg)
            ok, encoded = cv2.imencode(
                ".jpg",
                image,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
            )
            if not ok:
                raise ValueError("JPEG encoding failed")
        except Exception as exc:
            self.get_logger().warning(f"frame encoding failed: {exc}")
            return
        self._queue_frame(now, encoded.tobytes())

    def _should_sample(self, now: float) -> bool:
        if now - self._last_frame_at < 1.0 / self.sample_fps:
            return False
        self._last_frame_at = now
        return True

    def _queue_frame(self, now: float, data: bytes) -> None:
        frame = EncodedFrame(
            timestamp=max(0.0, now - self._stream_started),
            data=data,
        )
        try:
            self._frames.put_nowait(frame)
        except queue.Full:
            try:
                self._frames.get_nowait()
            except queue.Empty:
                pass
            self._frames.put_nowait(frame)

    @staticmethod
    def _is_supported_encoded_image(data: bytes) -> bool:
        return data.startswith(b"\xff\xd8\xff") or data.startswith(
            b"\x89PNG\r\n\x1a\n"
        )

    def _run_worker(self) -> None:
        try:
            asyncio.run(self._connection_loop())
        except Exception as exc:
            if not self._stop.is_set():
                self.get_logger().error(f"MOSS adapter worker stopped: {exc}")

    async def _connection_loop(self) -> None:
        timeout = ClientTimeout(total=None, connect=5.0, sock_connect=5.0)
        async with ClientSession(timeout=timeout) as session:
            while not self._stop.is_set():
                try:
                    async with session.ws_connect(
                        self.runtime_url,
                        heartbeat=20.0,
                        max_msg_size=0,
                    ) as websocket:
                        await self._start_session(websocket)
                        await self._serve_session(websocket)
                except asyncio.CancelledError:
                    raise
                except Exception as exc:
                    if not self._stop.is_set():
                        self.get_logger().warning(f"MOSS runtime disconnected: {exc}")
                        await asyncio.sleep(1.0)

    async def _start_session(self, websocket: ClientWebSocketResponse) -> None:
        await websocket.send_json({
            "type": "start",
            "system_prompt": "You are a realtime visual perception system.",
            "prompt": (
                "Observe the continuous video and describe important environmental "
                "changes as they happen. "
                "Stay silent when there is no relevant update."
            ),
            "frame_queue_size": 256,
            "max_tokens_per_second": 12,
            "max_new_tokens": 4096,
            "do_sample": False,
            "repetition_penalty": 1.0,
        })
        while not self._stop.is_set():
            message = await websocket.receive(timeout=30.0)
            if message.type != WSMsgType.TEXT:
                raise ConnectionError(f"unexpected MOSS start response: {message.type}")
            payload = json.loads(message.data)
            if payload.get("type") == "ready":
                self.get_logger().info("MOSS realtime session ready")
                return
            if payload.get("type") == "error":
                raise RuntimeError(str(payload.get("message") or payload))
        raise ConnectionError("MOSS adapter stopped before session became ready")

    async def _serve_session(self, websocket: ClientWebSocketResponse) -> None:
        parser = MossRoundParser()
        with self._episode_lock:
            focused = self._ownership.route_episode()
            finalizing = (
                focused is not None and self._ownership.is_finalizing(focused)
            )
        if focused is not None:
            await websocket.send_json({
                "type": "prompt",
                "text": (
                    "The anomaly episode has ended. Complete or correct the concise "
                    "description now. Do not start a new unrelated observation."
                    if finalizing else
                    "An anomaly episode is active. Focus on the current scene change "
                    "and stay silent until there is meaningful evidence."
                ),
            })

        while not self._stop.is_set() and not websocket.closed:
            await self._send_pending(websocket)
            try:
                message = await websocket.receive(timeout=0.02)
            except asyncio.TimeoutError:
                await asyncio.sleep(0.005)
                continue

            if message.type == WSMsgType.TEXT:
                self._handle_server_message(message.data, parser)
            elif message.type in (WSMsgType.CLOSE, WSMsgType.CLOSED, WSMsgType.ERROR):
                raise ConnectionError("MOSS WebSocket closed")

        if not websocket.closed:
            await websocket.send_json({"type": "stop"})

    async def _send_pending(self, websocket: ClientWebSocketResponse) -> None:
        while True:
            try:
                prompt = self._prompts.get_nowait()
            except queue.Empty:
                break
            with self._episode_lock:
                valid = (
                    self._ownership.is_active(prompt.episode_id)
                    if prompt.phase == "active"
                    else self._ownership.is_finalizing(prompt.episode_id)
                )
            if not valid:
                continue
            await websocket.send_json({"type": "prompt", "text": prompt.text})

        try:
            frame = self._frames.get_nowait()
        except queue.Empty:
            return
        await websocket.send_json({"type": "frame", "timestamp": frame.timestamp})
        await websocket.send_bytes(frame.data)

    def _handle_server_message(self, raw_message: str, parser: MossRoundParser) -> None:
        try:
            payload = json.loads(raw_message)
        except json.JSONDecodeError:
            self.get_logger().warning("ignored malformed MOSS message")
            return
        message_type = payload.get("type")
        if message_type == "output":
            with self._episode_lock:
                routed_episode_id = self._ownership.route_episode()
            routed_rounds = parser.push_routed(
                str(payload.get("text") or ""),
                episode_id=routed_episode_id,
            )
            for routed_round in routed_rounds:
                if routed_round.episode_id is None or routed_round.text is None:
                    continue
                with self._episode_lock:
                    accepted = self._ownership.claim_completion(
                        routed_round.episode_id
                    )
                    focus_started_at = self._focus_started_at.get(
                        routed_round.episode_id
                    )
                if accepted:
                    self._commit_semantic_round(
                        routed_round.text,
                        routed_round.episode_id,
                        focus_started_at,
                    )
        elif message_type == "frame_ack":
            self._frame_acks += 1
            if bool(payload.get("dropped_oldest")):
                self._runtime_frame_drops += 1
                self.get_logger().warning(
                    "MOSS runtime dropped oldest pending frame: "
                    f"drops={self._runtime_frame_drops} acks={self._frame_acks}"
                )
        elif message_type == "error":
            self.get_logger().warning(f"MOSS runtime error: {payload.get('message') or payload}")
        elif message_type == "session_end":
            raise ConnectionError("MOSS realtime session ended")

    def _commit_semantic_round(
        self,
        semantic_text: str,
        episode_id: str,
        focus_started_at: Optional[float],
    ) -> None:
        annotation_id = f"{episode_id}:moss"
        event = EnvironmentEvent()
        event.episode_id = episode_id
        event.annotation_id = annotation_id
        event.text = semantic_text

        # Every completed round updates the recorder marker. Only the first
        # round enters the live dialogue stream; later rounds are corrections.
        self.annotation_pub.publish(event)
        is_first = episode_id not in self._steered_episode_ids
        self._steered_episode_ids.add(episode_id)
        if not is_first:
            self.get_logger().info(
                f"MOSS annotation corrected: episode={episode_id}"
            )
            return

        self.event_pub.publish(event)
        if focus_started_at is not None:
            latency_ms = (time.monotonic() - focus_started_at) * 1000.0
            self.get_logger().info(
                f"MOSS first semantic response: episode={episode_id} latency_ms={latency_ms:.1f}"
            )

    @staticmethod
    def _image_to_bgr(msg: Image) -> np.ndarray:
        encoding = msg.encoding.lower()
        channels = {
            "bgr8": 3,
            "8uc3": 3,
            "rgb8": 3,
            "rgba8": 4,
            "bgra8": 4,
            "mono8": 1,
            "8uc1": 1,
            "bayer_rggb8": 1,
        }.get(encoding)
        if channels is None:
            raise ValueError(f"unsupported encoding: {msg.encoding}")
        packed_width = int(msg.width) * channels
        step = int(msg.step) if msg.step else packed_width
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(int(msg.height), step)
        packed = rows[:, :packed_width]
        image = (
            packed.reshape(int(msg.height), int(msg.width), channels)
            if channels > 1 else packed
        )
        if encoding in ("bgr8", "8uc3"):
            return image
        if encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if encoding == "rgba8":
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        if encoding == "bgra8":
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        if encoding in ("mono8", "8uc1"):
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        return cv2.cvtColor(image, cv2.COLOR_BayerRG2BGR)

    def destroy_node(self):
        self._stop.set()
        self._worker.join(timeout=3.0)
        if self._worker.is_alive():
            self.get_logger().warning("MOSS adapter worker did not stop before shutdown")
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MossRealtimeAdapterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
