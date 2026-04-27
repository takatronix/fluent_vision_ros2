#!/usr/bin/env python3
from __future__ import annotations

import math
import re
import zlib
from typing import Optional

import rclpy
from geometry_msgs.msg import Point, Quaternion, TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster
from vision_msgs.msg import Detection3DArray

from fv_msgs.msg import Object3D, Object3DArray


def _safe_token(value: str, fallback: str) -> str:
    token = re.sub(r"[^a-zA-Z0-9_]+", "_", (value or "").strip()).strip("_").lower()
    return token or fallback


def _hash_int32(value: str, fallback: int) -> int:
    text = (value or "").strip()
    if not text:
        return fallback
    try:
        parsed = int(text, 10)
        if -(2 ** 31) <= parsed < 2 ** 31:
            return parsed
    except ValueError:
        pass
    hashed = zlib.crc32(text.encode("utf-8"))
    if hashed >= 2 ** 31:
        hashed -= 2 ** 32
    return int(hashed)


def _is_nonzero_quaternion(q: Quaternion) -> bool:
    return any(abs(v) > 1e-9 for v in (q.x, q.y, q.z, q.w))


class FoundationPoseBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("fv_foundationpose_bridge")

        self.declare_parameter("input_topic", "/pose_estimation/output")
        self.declare_parameter("output_topic", "~/objects_3d")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("tf_parent_frame", "")
        self.declare_parameter("tf_child_frame_prefix", "fp_object")
        self.declare_parameter("tf_use_label", False)
        self.declare_parameter("confidence_threshold", 0.0)
        self.declare_parameter("default_label", "object")
        self.declare_parameter("fallback_class_id", 0)
        self.declare_parameter("prefer_bbox_center", True)
        self.declare_parameter("qos.reliability", "reliable")

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.tf_parent_frame = str(self.get_parameter("tf_parent_frame").value)
        self.tf_child_frame_prefix = str(self.get_parameter("tf_child_frame_prefix").value)
        self.tf_use_label = bool(self.get_parameter("tf_use_label").value)
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.default_label = str(self.get_parameter("default_label").value)
        self.fallback_class_id = int(self.get_parameter("fallback_class_id").value)
        self.prefer_bbox_center = bool(self.get_parameter("prefer_bbox_center").value)
        reliability_name = str(self.get_parameter("qos.reliability").value).strip().lower()

        qos = QoSProfile(depth=10)
        if reliability_name == "best_effort":
            qos.reliability = ReliabilityPolicy.BEST_EFFORT
        else:
            qos.reliability = ReliabilityPolicy.RELIABLE

        self.pub = self.create_publisher(Object3DArray, self.output_topic, qos)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.sub = self.create_subscription(Detection3DArray, self.input_topic, self._on_detections, qos)

        self.get_logger().info(
            f"fv_foundationpose_bridge ready: input={self.input_topic} output={self.output_topic} publish_tf={self.publish_tf}"
        )

    def _on_detections(self, msg: Detection3DArray) -> None:
        out = Object3DArray()
        out.header = msg.header

        tf_messages = []
        for index, detection in enumerate(msg.detections):
            converted = self._convert_detection(msg, detection, index)
            if converted is None:
                continue
            obj, tf_msg = converted
            out.objects.append(obj)
            if tf_msg is not None:
                tf_messages.append(tf_msg)

        self.pub.publish(out)
        if self.tf_broadcaster and tf_messages:
            self.tf_broadcaster.sendTransform(tf_messages)

    def _convert_detection(self, array_msg: Detection3DArray, detection, index: int) -> Optional[tuple[Object3D, Optional[TransformStamped]]]:
        result = detection.results[0] if detection.results else None
        class_key = ""
        score = 0.0
        if result is not None:
            hypothesis = getattr(result, "hypothesis", None)
            if hypothesis is not None:
                class_key = str(getattr(hypothesis, "class_id", "") or "")
                score = float(getattr(hypothesis, "score", 0.0) or 0.0)

        if score < self.confidence_threshold:
            return None

        label = class_key or self.default_label
        detection_id_text = str(getattr(detection, "id", "") or "")
        object_id = _hash_int32(detection_id_text, index + 1)
        class_id = _hash_int32(class_key, self.fallback_class_id)

        obj = Object3D()
        obj.header = detection.header if getattr(detection.header, "frame_id", "") else array_msg.header
        obj.id = object_id
        obj.class_id = class_id
        obj.label = label
        obj.shape = self._shape_from_label(label)
        obj.confidence = float(score)

        center_position, center_orientation = self._extract_pose(detection, result)
        obj.centroid = center_position
        obj.orientation = center_orientation

        bbox = getattr(detection, "bbox", None)
        if bbox is not None:
            obj.extent.x = float(getattr(bbox.size, "x", 0.0))
            obj.extent.y = float(getattr(bbox.size, "y", 0.0))
            obj.extent.z = float(getattr(bbox.size, "z", 0.0))

        obj.distance_m = math.sqrt(
            float(obj.centroid.x) ** 2 + float(obj.centroid.y) ** 2 + float(obj.centroid.z) ** 2
        )
        obj.num_points = 0

        tf_msg = None
        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header = obj.header
            if self.tf_parent_frame:
                tf_msg.header.frame_id = self.tf_parent_frame
            tf_msg.child_frame_id = self._tf_child_name(label, detection_id_text, index)
            tf_msg.transform.translation.x = float(obj.centroid.x)
            tf_msg.transform.translation.y = float(obj.centroid.y)
            tf_msg.transform.translation.z = float(obj.centroid.z)
            tf_msg.transform.rotation = obj.orientation
        return obj, tf_msg

    def _extract_pose(self, detection, result) -> tuple:
        if self.prefer_bbox_center:
            bbox = getattr(detection, "bbox", None)
            center = getattr(bbox, "center", None) if bbox is not None else None
            if center is not None:
                orientation = center.orientation
                if not _is_nonzero_quaternion(orientation):
                    orientation = Quaternion(w=1.0)
                return center.position, orientation

        if result is not None:
            pose_with_cov = getattr(result, "pose", None)
            pose = getattr(pose_with_cov, "pose", None) if pose_with_cov is not None else None
            if pose is not None:
                orientation = pose.orientation
                if not _is_nonzero_quaternion(orientation):
                    orientation = Quaternion(w=1.0)
                return pose.position, orientation

        bbox = getattr(detection, "bbox", None)
        center = getattr(bbox, "center", None) if bbox is not None else None
        if center is not None:
            return center.position, Quaternion(w=1.0)
        return Point(), Quaternion(w=1.0)

    def _tf_child_name(self, label: str, detection_id_text: str, index: int) -> str:
        suffix = _safe_token(detection_id_text, f"{index + 1}")
        if self.tf_use_label:
            return f"{_safe_token(self.tf_child_frame_prefix, 'fp_object')}_{_safe_token(label, 'object')}_{suffix}"
        return f"{_safe_token(self.tf_child_frame_prefix, 'fp_object')}_{suffix}"

    @staticmethod
    def _shape_from_label(label: str) -> int:
        text = (label or "").strip().lower()
        if "cube" in text or "box" in text:
            return Object3D.SHAPE_CUBE
        if "plate" in text:
            return Object3D.SHAPE_PLATE
        if "cyl" in text:
            return Object3D.SHAPE_CYLINDER
        if "sphere" in text or "ball" in text:
            return Object3D.SHAPE_SPHERE
        return Object3D.SHAPE_UNKNOWN


def main() -> None:
    rclpy.init()
    node = FoundationPoseBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
