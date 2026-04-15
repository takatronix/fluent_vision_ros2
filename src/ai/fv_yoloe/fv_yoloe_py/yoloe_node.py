#!/usr/bin/env python3
"""YOLOE open-vocabulary detection & segmentation ROS2 node.

Text prompt で任意の物体をリアルタイム検出。
プロンプトは ROS2 パラメータまたはサービスで動的に変更可能。
"""
from __future__ import annotations

import time
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger


class FvYoloeNode(Node):
    def __init__(self) -> None:
        super().__init__("fv_yoloe")

        # --- Parameters ---
        self.declare_parameter("input_image_topic", "~/color/image_raw")
        self.declare_parameter("overlay_topic", "yoloe/overlay")
        self.declare_parameter("overlay_compressed_topic", "yoloe/overlay/compressed")
        self.declare_parameter("detections_topic", "yoloe/detections")

        # Model
        self.declare_parameter("model_name", "yoloe-11s-seg.pt")
        self.declare_parameter("device", "auto")  # auto|cuda|cpu
        self.declare_parameter("conf_threshold", 0.25)
        self.declare_parameter("iou_threshold", 0.5)

        # Text prompt (comma-separated class names)
        self.declare_parameter("text_prompt", "cube")
        # Topic to receive prompt updates (publish String to change classes)
        self.declare_parameter("prompt_topic", "~/set_prompt")

        # Performance
        self.declare_parameter("processing_frequency", 10.0)
        self.declare_parameter("qos_reliability", "best_effort")
        self.declare_parameter("log_every_n_frames", 30)
        self.declare_parameter("jpeg_quality", 80)

        # Read params
        self.input_topic = str(self.get_parameter("input_image_topic").value)
        self.overlay_topic = str(self.get_parameter("overlay_topic").value)
        self.overlay_compressed_topic = str(self.get_parameter("overlay_compressed_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)

        self.model_name = str(self.get_parameter("model_name").value)
        self.device_mode = str(self.get_parameter("device").value)
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)

        self.text_prompt = str(self.get_parameter("text_prompt").value)
        self.prompt_topic = str(self.get_parameter("prompt_topic").value)

        self.processing_frequency = float(self.get_parameter("processing_frequency").value)
        self.qos_reliability = str(self.get_parameter("qos_reliability").value).strip().lower()
        self.log_every_n_frames = max(1, int(self.get_parameter("log_every_n_frames").value))
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        # Publishers
        self.overlay_pub = self.create_publisher(Image, self.overlay_topic, 10)
        self.overlay_compressed_pub = self.create_publisher(
            CompressedImage, self.overlay_compressed_topic, 10
        )
        self.detections_pub = self.create_publisher(String, self.detections_topic, 10)

        # Load model
        self._model = None
        self._current_classes: List[str] = []
        self._load_model()

        # Subscriber
        qos = self._sensor_qos()
        self.image_sub = self.create_subscription(
            Image, self.input_topic, self._on_image, qos
        )

        # Prompt update subscriber
        self.prompt_sub = self.create_subscription(
            String, self.prompt_topic, self._on_prompt, 10
        )

        # Service to get current prompt
        self.create_service(Trigger, "~/get_prompt", self._srv_get_prompt)

        self._frame_counter = 0
        self._proc_ms_acc = 0.0
        self._last_proc_time = 0.0

        self.get_logger().info(
            f"fv_yoloe started: model={self.model_name} prompt='{self.text_prompt}' "
            f"input={self.input_topic} overlay={self.overlay_compressed_topic}"
        )

    def _sensor_qos(self) -> QoSProfile:
        reliable = self.qos_reliability == "reliable"
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

    def _load_model(self) -> None:
        try:
            from ultralytics import YOLOE
            import torch

            if self.device_mode == "cuda":
                device = "cuda"
            elif self.device_mode == "cpu":
                device = "cpu"
            else:
                device = "cuda" if torch.cuda.is_available() else "cpu"

            self.get_logger().info(f"Loading YOLOE model: {self.model_name} on {device}...")
            self._model = YOLOE(self.model_name)
            self._model.to(device)
            self._device = device

            # Set initial text prompt
            self._set_classes(self.text_prompt)

            self.get_logger().info(f"YOLOE model loaded: {self.model_name} on {device}")
        except Exception as e:
            self.get_logger().error(f"YOLOE model load failed: {e}")
            self._model = None

    def _set_classes(self, prompt_str: str) -> None:
        if self._model is None:
            return

        names = [n.strip() for n in prompt_str.split(",") if n.strip()]
        if not names:
            self.get_logger().warning("Empty prompt, keeping current classes")
            return

        if names == self._current_classes:
            return

        try:
            text_pe = self._model.get_text_pe(names)
            self._model.set_classes(names, text_pe)
            self._current_classes = names
            self.get_logger().info(f"YOLOE classes set: {names}")
        except Exception as e:
            self.get_logger().error(f"Failed to set classes: {e}")

    def _on_prompt(self, msg: String) -> None:
        new_prompt = msg.data.strip()
        if new_prompt:
            self.get_logger().info(f"Prompt update: '{new_prompt}'")
            self._set_classes(new_prompt)

    def _srv_get_prompt(self, request, response):
        response.success = self._model is not None
        response.message = ",".join(self._current_classes)
        return response

    def _on_image(self, msg: Image) -> None:
        if self._model is None or not self._current_classes:
            return

        # Throttle
        if self.processing_frequency > 0:
            now = time.monotonic()
            interval = 1.0 / self.processing_frequency
            if (now - self._last_proc_time) < interval:
                return
            self._last_proc_time = now

        start = time.perf_counter()

        # Decode image
        try:
            img_bgr = self._imgmsg_to_bgr(msg)
        except Exception as e:
            self.get_logger().warning(f"Image decode failed: {e}")
            return

        # Inference
        try:
            results = self._model.predict(
                img_bgr,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                verbose=False,
            )
        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}")
            return

        result = results[0]

        # Draw overlay
        overlay = result.plot()

        header = msg.header

        # Publish raw overlay
        overlay_msg = self._cv2_to_imgmsg(overlay, "bgr8", header)
        self.overlay_pub.publish(overlay_msg)

        # Publish compressed for dashboard
        _, jpeg_data = cv2.imencode(".jpg", overlay, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        comp_msg = CompressedImage()
        comp_msg.header = header
        comp_msg.format = "jpeg"
        comp_msg.data = jpeg_data.tobytes()
        self.overlay_compressed_pub.publish(comp_msg)

        # Publish detections as JSON string
        if result.boxes is not None and len(result.boxes) > 0:
            import json
            dets = []
            for i, box in enumerate(result.boxes):
                cls_id = int(box.cls[0])
                cls_name = result.names.get(cls_id, str(cls_id))
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                dets.append({
                    "class": cls_name, "conf": round(conf, 3),
                    "bbox": [round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1)]
                })
            det_msg = String()
            det_msg.data = json.dumps(dets)
            self.detections_pub.publish(det_msg)

        # Stats
        self._frame_counter += 1
        proc_ms = (time.perf_counter() - start) * 1000.0
        self._proc_ms_acc += proc_ms
        if self._frame_counter % self.log_every_n_frames == 0:
            avg = self._proc_ms_acc / float(self.log_every_n_frames)
            fps = 1000.0 / avg if avg > 0 else 0
            n_det = len(result.boxes) if result.boxes is not None else 0
            self._proc_ms_acc = 0.0
            self.get_logger().info(
                f"frames={self._frame_counter} avg={avg:.1f}ms ({fps:.1f}fps) "
                f"detections={n_det} classes={self._current_classes}"
            )

    # --- Image conversion (no cv_bridge) ---
    def _imgmsg_to_bgr(self, msg: Image) -> np.ndarray:
        encoding = msg.encoding.lower()
        if encoding in ("bgr8", "8uc3"):
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        elif encoding == "rgb8":
            rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        elif encoding in ("mono8", "8uc1"):
            gray = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        else:
            raise ValueError(f"Unsupported encoding: {msg.encoding}")

    def _cv2_to_imgmsg(self, img: np.ndarray, encoding: str, header) -> Image:
        msg = Image()
        msg.header = header
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = encoding
        if img.ndim == 3:
            msg.step = img.shape[1] * img.shape[2] * img.dtype.itemsize
        else:
            msg.step = img.shape[1] * img.dtype.itemsize
        msg.is_bigendian = False
        msg.data = img.tobytes()
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FvYoloeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
