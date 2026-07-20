#!/usr/bin/env python3
"""Visual condition detector ROS2 node.

Input: sensor_msgs/Image or sensor_msgs/CompressedImage
Output: JSON status/event on std_msgs/String
"""
from __future__ import annotations

import json
import os
import time
import uuid
from typing import Dict, List, Optional, Sequence

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from fv_episode_msgs.msg import EnvironmentChange

from fv_visual_condition_detector_py.vad_state import VadState


class VisualConditionDetectorNode(Node):
    def __init__(
        self,
        node_name: str = "fv_visual_condition_detector",
        publish_legacy: bool = True,
        publish_environment: bool = False,
        require_model: bool = True,
    ) -> None:
        super().__init__(node_name)
        self.require_model = require_model

        self.declare_parameter("input_image_topic", "~/image_raw")
        self.declare_parameter("input_image_type", "raw")
        self.declare_parameter("compressed_relay_topic", "")
        self.declare_parameter("compressed_relay_frequency", 1.0)
        self.declare_parameter("status_topic", "visual_condition/status")
        self.declare_parameter("event_topic", "visual_condition/events")
        self.declare_parameter("environment_change_topic", "/environment/change")

        self.declare_parameter("model_path", "")
        self.declare_parameter("providers", "TensorrtExecutionProvider,CUDAExecutionProvider,CPUExecutionProvider")
        self.declare_parameter("input_width", 224)
        self.declare_parameter("input_height", 224)
        self.declare_parameter("mean", [0.485, 0.456, 0.406])
        self.declare_parameter("std", [0.229, 0.224, 0.225])

        self.declare_parameter("threshold", 0.50)
        self.declare_parameter("min_on_sec", 0.30)
        self.declare_parameter("min_off_sec", 0.30)
        self.declare_parameter("processing_frequency", 10.0)
        self.declare_parameter("qos_reliability", "best_effort")
        self.declare_parameter("log_every_n_frames", 30)

        self.declare_parameter(
            "condition_labels",
            ["soiling", "fog", "rain", "snow", "night", "blur", "exposure", "occlusion"],
        )
        self.declare_parameter("domain_labels", ["driving", "driving_fisheye", "field"])

        self.input_image_topic = str(self.get_parameter("input_image_topic").value)
        self.input_image_type = str(
            self.get_parameter("input_image_type").value
        ).strip().lower()
        self.compressed_relay_topic = str(
            self.get_parameter("compressed_relay_topic").value
        )
        self.compressed_relay_frequency = max(
            0.01,
            float(self.get_parameter("compressed_relay_frequency").value),
        )
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.event_topic = str(self.get_parameter("event_topic").value)
        self.environment_change_topic = str(
            self.get_parameter("environment_change_topic").value
        )
        self.model_path = str(self.get_parameter("model_path").value)
        self.providers = self._split_csv(str(self.get_parameter("providers").value))
        self.input_w = int(self.get_parameter("input_width").value)
        self.input_h = int(self.get_parameter("input_height").value)
        self.mean = np.asarray(self._float_list("mean", 3), dtype=np.float32).reshape(1, 1, 3)
        self.std = np.asarray(self._float_list("std", 3), dtype=np.float32).reshape(1, 1, 3)
        self.threshold = float(self.get_parameter("threshold").value)
        self.processing_frequency = float(self.get_parameter("processing_frequency").value)
        self.qos_reliability = str(self.get_parameter("qos_reliability").value).strip().lower()
        self.log_every_n_frames = max(1, int(self.get_parameter("log_every_n_frames").value))
        self.condition_labels = [str(x) for x in self.get_parameter("condition_labels").value]
        self.domain_labels = [str(x) for x in self.get_parameter("domain_labels").value]

        min_on_sec = float(self.get_parameter("min_on_sec").value)
        min_off_sec = float(self.get_parameter("min_off_sec").value)
        self.vad = VadState(self.threshold, min_on_sec, min_off_sec)

        self._session = None
        self._input_name = ""
        self._output_names: List[str] = []
        self._load_model()

        self.status_pub = (
            self.create_publisher(String, self.status_topic, 10)
            if publish_legacy else None
        )
        self.event_pub = (
            self.create_publisher(String, self.event_topic, 10)
            if publish_legacy else None
        )
        self.environment_change_pub = (
            self.create_publisher(EnvironmentChange, self.environment_change_topic, 10)
            if publish_environment else None
        )
        self._anomaly_episode_id: Optional[str] = None
        if self.compressed_relay_topic:
            if self.input_image_type != "compressed":
                raise ValueError(
                    "compressed_relay_topic requires input_image_type='compressed'"
                )
            self.compressed_relay_pub = self.create_publisher(
                CompressedImage,
                self.compressed_relay_topic,
                self._sensor_qos(),
            )
        else:
            self.compressed_relay_pub = None
        self._last_compressed_relay_time = 0.0
        if self.input_image_type == "compressed":
            self.image_sub = self.create_subscription(
                CompressedImage,
                self.input_image_topic,
                self._on_image,
                self._sensor_qos(),
            )
        elif self.input_image_type == "raw":
            self.image_sub = self.create_subscription(
                Image,
                self.input_image_topic,
                self._on_image,
                self._sensor_qos(),
            )
        else:
            raise ValueError("input_image_type must be 'compressed' or 'raw'")

        self._last_proc_time = 0.0
        self._frame_counter = 0
        self._proc_ms_acc = 0.0

        self.get_logger().info(
            "fv_visual_condition_detector started: "
            f"input={self.input_image_topic} input_type={self.input_image_type} "
            f"compressed_relay={self.compressed_relay_topic or 'disabled'} "
            f"status={self.status_topic} event={self.event_topic} "
            f"model={self.model_path} threshold={self.threshold:.3f} "
            f"min_on={min_on_sec:.2f}s min_off={min_off_sec:.2f}s"
        )

    def _float_list(self, parameter_name: str, expected_len: int) -> List[float]:
        values = [float(x) for x in self.get_parameter(parameter_name).value]
        if len(values) != expected_len:
            raise ValueError(f"{parameter_name} must have {expected_len} values")
        return values

    def _sensor_qos(self) -> QoSProfile:
        reliable = self.qos_reliability == "reliable"
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

    def _load_model(self) -> None:
        if not self.model_path or not os.path.isfile(self.model_path):
            message = f"Model file not found: {self.model_path}"
            self.get_logger().error(message)
            if self.require_model:
                raise RuntimeError(message)
            return

        try:
            import onnxruntime as ort

            available = ort.get_available_providers()
            providers = [provider for provider in self.providers if provider in available]
            if "CPUExecutionProvider" not in providers:
                providers.append("CPUExecutionProvider")

            sess_opts = ort.SessionOptions()
            sess_opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            self._session = ort.InferenceSession(
                self.model_path,
                sess_options=sess_opts,
                providers=providers,
            )
            self._input_name = self._session.get_inputs()[0].name
            self._output_names = [x.name for x in self._session.get_outputs()]
            self.get_logger().info(
                f"ONNX loaded provider={self._session.get_providers()[0]} "
                f"input={self._input_name} outputs={self._output_names}"
            )
        except Exception as exc:
            message = f"ONNX model load failed: {exc}"
            self.get_logger().error(message)
            self._session = None
            if self.require_model:
                raise RuntimeError(message) from exc

    def _on_image(self, msg: Image | CompressedImage) -> None:
        now = time.monotonic()
        if isinstance(msg, CompressedImage):
            self._relay_compressed_image(msg, now)
        if self._session is None:
            return

        if self.processing_frequency > 0.0:
            min_interval = 1.0 / self.processing_frequency
            if now - self._last_proc_time < min_interval:
                return
            self._last_proc_time = now

        started = time.perf_counter()
        try:
            image_bgr = self._imgmsg_to_bgr(msg)
            tensor = self._preprocess(image_bgr)
            outputs = self._session.run(None, {self._input_name: tensor})
            parsed = self._parse_outputs(outputs)
        except Exception as exc:
            self.get_logger().error(f"inference failed: {exc}")
            return

        latency_ms = (time.perf_counter() - started) * 1000.0
        raw_bad = parsed["worsen_score"] >= self.threshold
        event = self.vad.update(parsed["worsen_score"], now)
        status = {
            "stamp": {"sec": int(msg.header.stamp.sec), "nanosec": int(msg.header.stamp.nanosec)},
            "frame_id": msg.header.frame_id,
            "frame": self._frame_counter,
            "worsen_score": parsed["worsen_score"],
            "raw_bad": raw_bad,
            "bad": self.vad.state,
            "threshold": self.threshold,
            "condition_scores": parsed["condition_scores"],
            "condition_top": parsed["condition_top"],
            "domain_probs": parsed["domain_probs"],
            "domain_top": parsed["domain_top"],
            "latency_ms": latency_ms,
            "provider": self._session.get_providers()[0],
        }
        if self.status_pub is not None:
            self._publish_json(self.status_pub, status)

        if event is not None:
            event_payload = {
                "action": event.action,
                "bad": event.bad,
                "stamp": status["stamp"],
                "frame_id": status["frame_id"],
                "frame": status["frame"],
                "worsen_score": status["worsen_score"],
                "condition_top": status["condition_top"],
                "domain_top": status["domain_top"],
                "tags": [
                    "visual_condition",
                    "worsen" if event.bad else "recovered",
                    f"condition:{status['condition_top']}",
                    f"domain:{status['domain_top']}",
                ],
            }
            if self.event_pub is not None:
                self._publish_json(self.event_pub, event_payload)
            if self.environment_change_pub is not None:
                self._publish_environment_change(event.action)

        self._frame_counter += 1
        self._proc_ms_acc += latency_ms
        if self._frame_counter % self.log_every_n_frames == 0:
            avg_ms = self._proc_ms_acc / float(self.log_every_n_frames)
            fps = 1000.0 / avg_ms if avg_ms > 0.0 else 0.0
            self._proc_ms_acc = 0.0
            self.get_logger().info(
                f"frames={self._frame_counter} avg={avg_ms:.1f}ms ({fps:.1f}fps) "
                f"score={status['worsen_score']:.3f} bad={status['bad']} "
                f"condition={status['condition_top']} domain={status['domain_top']}"
            )

    def _relay_compressed_image(self, msg: CompressedImage, now: float) -> None:
        if self.compressed_relay_pub is None:
            return
        if (
            now - self._last_compressed_relay_time
            < 1.0 / self.compressed_relay_frequency
        ):
            return
        self._last_compressed_relay_time = now
        self.compressed_relay_pub.publish(msg)

    def _preprocess(self, image_bgr: np.ndarray) -> np.ndarray:
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        image_rgb = cv2.resize(image_rgb, (self.input_w, self.input_h), interpolation=cv2.INTER_AREA)
        image = image_rgb.astype(np.float32) / 255.0
        image = (image - self.mean) / self.std
        image = np.transpose(image, (2, 0, 1))[None, :, :, :]
        return np.ascontiguousarray(image, dtype=np.float32)

    def _parse_outputs(self, outputs: Sequence[np.ndarray]) -> Dict[str, object]:
        by_name = dict(zip(self._output_names, outputs))
        worsen = self._first_available(by_name, ["worsen_score", "bad_score", "worsen"])
        condition = self._first_available(by_name, ["condition_scores", "condition_logits", "condition"])
        domain = self._first_available(by_name, ["domain_probs", "domain_logits", "domain"])

        if worsen is None and outputs:
            worsen = outputs[0]
        if condition is None and len(outputs) > 1:
            condition = outputs[1]
        if domain is None and len(outputs) > 2:
            domain = outputs[2]

        worsen_score = self._scalar(worsen)
        condition_vec = self._vector(condition)
        domain_vec = self._vector(domain)

        return {
            "worsen_score": worsen_score,
            "condition_scores": self._label_scores(condition_vec, self.condition_labels),
            "condition_top": self._top_label(condition_vec, self.condition_labels),
            "domain_probs": self._label_scores(domain_vec, self.domain_labels),
            "domain_top": self._top_label(domain_vec, self.domain_labels),
        }

    @staticmethod
    def _first_available(by_name: Dict[str, np.ndarray], names: Sequence[str]) -> Optional[np.ndarray]:
        for name in names:
            if name in by_name:
                return by_name[name]
        return None

    @staticmethod
    def _scalar(value: Optional[np.ndarray]) -> float:
        if value is None:
            return 0.0
        array = np.asarray(value, dtype=np.float32).reshape(-1)
        return float(array[0]) if array.size else 0.0

    @staticmethod
    def _vector(value: Optional[np.ndarray]) -> np.ndarray:
        if value is None:
            return np.zeros((0,), dtype=np.float32)
        return np.asarray(value, dtype=np.float32).reshape(-1)

    @staticmethod
    def _label_scores(values: np.ndarray, labels: Sequence[str]) -> Dict[str, float]:
        return {
            label: float(values[index])
            for index, label in enumerate(labels)
            if index < values.size
        }

    @staticmethod
    def _top_label(values: np.ndarray, labels: Sequence[str]) -> str:
        if values.size == 0 or not labels:
            return ""
        index = int(np.argmax(values))
        if index >= len(labels):
            return str(index)
        return labels[index]

    @staticmethod
    def _split_csv(value: str) -> List[str]:
        return [item.strip() for item in value.split(",") if item.strip()]

    @staticmethod
    def _publish_json(publisher, payload: Dict[str, object]) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        publisher.publish(msg)

    def _publish_environment_change(self, action: str) -> None:
        if action == "start":
            self._anomaly_episode_id = str(uuid.uuid4())
            state = "started"
        elif action == "stop" and self._anomaly_episode_id is not None:
            state = "ended"
        else:
            return

        msg = EnvironmentChange()
        msg.episode_id = self._anomaly_episode_id
        msg.state = state
        self.environment_change_pub.publish(msg)
        if state == "ended":
            self._anomaly_episode_id = None

    # Image conversion without cv_bridge.
    @staticmethod
    def _imgmsg_to_bgr(msg: Image | CompressedImage) -> np.ndarray:
        if isinstance(msg, CompressedImage):
            encoded = np.frombuffer(msg.data, dtype=np.uint8)
            image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError(f"Unable to decode compressed image: {msg.format}")
            return image

        encoding = msg.encoding.lower()
        if encoding in ("bgr8", "8uc3"):
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if encoding == "rgb8":
            rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if encoding == "rgba8":
            rgba = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4)
            return cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
        if encoding == "bgra8":
            bgra = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4)
            return cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
        if encoding in ("mono8", "8uc1"):
            gray = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        if encoding == "bayer_rggb8":
            bayer = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            return cv2.cvtColor(bayer, cv2.COLOR_BayerRG2BGR)
        raise ValueError(f"Unsupported encoding: {msg.encoding}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualConditionDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
