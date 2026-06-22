#!/usr/bin/env python3
from __future__ import annotations

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


class FoundationPoseAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("fv_foundationpose_adapter")

        self.declare_parameter("color_input_topic", "/d405_color/image_raw")
        self.declare_parameter("camera_info_input_topic", "/d405_color/camera_info")
        self.declare_parameter("depth_input_topic", "/d405_depth/image_rect_raw")
        self.declare_parameter("mask_input_topic", "/d405_perception/mask")

        self.declare_parameter("color_output_topic", "/rgb/image_rect_color")
        self.declare_parameter("camera_info_output_topic", "/rgb/camera_info")
        self.declare_parameter("depth_output_topic", "/depth_registered/image_rect")
        self.declare_parameter("mask_output_topic", "/segmentation")

        self.declare_parameter("depth_scale_16uc1", 0.0001)
        self.declare_parameter("output_depth_encoding", "32FC1")
        self.declare_parameter("force_output_frame_id", "")
        self.declare_parameter("input_qos.reliability", "best_effort")
        self.declare_parameter("output_qos.reliability", "reliable")

        self.color_input_topic = str(self.get_parameter("color_input_topic").value)
        self.camera_info_input_topic = str(self.get_parameter("camera_info_input_topic").value)
        self.depth_input_topic = str(self.get_parameter("depth_input_topic").value)
        self.mask_input_topic = str(self.get_parameter("mask_input_topic").value)

        self.color_output_topic = str(self.get_parameter("color_output_topic").value)
        self.camera_info_output_topic = str(self.get_parameter("camera_info_output_topic").value)
        self.depth_output_topic = str(self.get_parameter("depth_output_topic").value)
        self.mask_output_topic = str(self.get_parameter("mask_output_topic").value)

        self.depth_scale_16uc1 = float(self.get_parameter("depth_scale_16uc1").value)
        self.output_depth_encoding = str(self.get_parameter("output_depth_encoding").value).strip()
        self.force_output_frame_id = str(self.get_parameter("force_output_frame_id").value).strip()
        input_reliability_name = str(self.get_parameter("input_qos.reliability").value).strip().lower()
        output_reliability_name = str(self.get_parameter("output_qos.reliability").value).strip().lower()

        input_qos = QoSProfile(depth=10)
        input_qos.reliability = (
            ReliabilityPolicy.RELIABLE if input_reliability_name == "reliable" else ReliabilityPolicy.BEST_EFFORT
        )
        output_qos = QoSProfile(depth=10)
        output_qos.reliability = (
            ReliabilityPolicy.RELIABLE if output_reliability_name == "reliable" else ReliabilityPolicy.BEST_EFFORT
        )

        self.bridge = CvBridge()
        self.latest_camera_info: CameraInfo | None = None

        self.color_pub = self.create_publisher(Image, self.color_output_topic, output_qos)
        self.camera_info_pub = self.create_publisher(CameraInfo, self.camera_info_output_topic, output_qos)
        self.depth_pub = self.create_publisher(Image, self.depth_output_topic, output_qos)
        self.mask_pub = self.create_publisher(Image, self.mask_output_topic, output_qos)

        self.create_subscription(Image, self.color_input_topic, self._on_color, input_qos)
        self.create_subscription(CameraInfo, self.camera_info_input_topic, self._on_camera_info, input_qos)
        self.create_subscription(Image, self.depth_input_topic, self._on_depth, input_qos)
        self.create_subscription(Image, self.mask_input_topic, self._on_mask, input_qos)

        self.get_logger().info(
            "fv_foundationpose_adapter ready: "
            f"color={self.color_input_topic}->{self.color_output_topic} "
            f"camera_info={self.camera_info_input_topic}->{self.camera_info_output_topic} "
            f"depth={self.depth_input_topic}->{self.depth_output_topic} "
            f"mask={self.mask_input_topic}->{self.mask_output_topic} "
            f"depth_scale_16uc1={self.depth_scale_16uc1}"
        )

    def _patch_frame_id(self, header):
        if self.force_output_frame_id:
            header.frame_id = self.force_output_frame_id
        elif self.latest_camera_info and self.latest_camera_info.header.frame_id:
            header.frame_id = self.latest_camera_info.header.frame_id
        return header

    def _on_color(self, msg: Image) -> None:
        out = Image()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.encoding = msg.encoding
        out.is_bigendian = msg.is_bigendian
        out.step = msg.step
        out.data = msg.data
        out.header = self._patch_frame_id(out.header)
        self.color_pub.publish(out)

    def _on_camera_info(self, msg: CameraInfo) -> None:
        out = CameraInfo()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.distortion_model = msg.distortion_model
        out.d = list(msg.d)
        out.k = list(msg.k)
        out.r = list(msg.r)
        out.p = list(msg.p)
        out.binning_x = msg.binning_x
        out.binning_y = msg.binning_y
        out.roi = msg.roi
        if self.force_output_frame_id:
            out.header.frame_id = self.force_output_frame_id
        self.latest_camera_info = out
        self.camera_info_pub.publish(out)

    def _on_mask(self, msg: Image) -> None:
        out = Image()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.encoding = msg.encoding
        out.is_bigendian = msg.is_bigendian
        out.step = msg.step
        out.data = msg.data
        out.header = self._patch_frame_id(out.header)
        self.mask_pub.publish(out)

    def _on_depth(self, msg: Image) -> None:
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if msg.encoding.lower() in ("16uc1", "mono16") or depth.dtype == np.uint16:
            depth_m = depth.astype(np.float32) * self.depth_scale_16uc1
        elif msg.encoding.lower() == "32fc1" or depth.dtype == np.float32:
            depth_m = depth.astype(np.float32, copy=False)
        else:
            self.get_logger().warning(f"Unsupported depth encoding: {msg.encoding}")
            return

        out = self.bridge.cv2_to_imgmsg(depth_m, encoding=self.output_depth_encoding)
        out.header = msg.header
        out.header = self._patch_frame_id(out.header)
        self.depth_pub.publish(out)


def main() -> None:
    rclpy.init()
    node = FoundationPoseAdapterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
