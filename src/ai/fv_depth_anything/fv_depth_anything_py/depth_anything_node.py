#!/usr/bin/env python3
"""Depth Anything V3 monocular depth estimation ROS2 node.

Supports ONNX Runtime (arm64/x64) and TensorRT backends.
Input: RGB image → Output: depth map (32FC1), colorized depth, optional point cloud.
"""
from __future__ import annotations

import os
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2, PointField


class FvDepthAnythingNode(Node):
    def __init__(self) -> None:
        super().__init__("fv_depth_anything")

        # --- Parameters ---
        self.declare_parameter("input_image_topic", "~/color/image_raw")
        self.declare_parameter("camera_info_topic", "~/camera_info")
        self.declare_parameter("depth_topic", "depth_anything/image")
        self.declare_parameter("depth_colored_topic", "depth_anything/image_color")
        self.declare_parameter("pointcloud_topic", "depth_anything/points")
        self.declare_parameter("frame_id_override", "")

        # Model
        self.declare_parameter("backend", "onnx")  # onnx | tensorrt
        self.declare_parameter("model_path", "")  # .onnx or .engine
        self.declare_parameter("model_variant", "small")  # small | large
        self.declare_parameter("inference_width", 518)
        self.declare_parameter("inference_height", 518)

        # Depth
        self.declare_parameter("min_depth_m", 0.1)
        self.declare_parameter("max_depth_m", 10.0)
        self.declare_parameter("metric_depth", False)  # True for DA3METRIC models
        self.declare_parameter("point_stride", 2)
        self.declare_parameter("publish_pointcloud", False)
        self.declare_parameter("publish_colored", True)
        self.declare_parameter("colormap", 20)  # cv2.COLORMAP_INFERNO

        # QoS / Performance
        self.declare_parameter("qos_reliability", "best_effort")
        self.declare_parameter("processing_frequency", 0.0)  # 0 = every frame
        self.declare_parameter("log_every_n_frames", 30)

        # Read parameters
        self.input_image_topic = str(self.get_parameter("input_image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.depth_colored_topic = str(self.get_parameter("depth_colored_topic").value)
        self.pointcloud_topic = str(self.get_parameter("pointcloud_topic").value)
        self.frame_id_override = str(self.get_parameter("frame_id_override").value)

        self.backend = str(self.get_parameter("backend").value).lower()
        self.model_path = str(self.get_parameter("model_path").value)
        self.model_variant = str(self.get_parameter("model_variant").value).lower()
        self.infer_w = int(self.get_parameter("inference_width").value)
        self.infer_h = int(self.get_parameter("inference_height").value)

        self.min_depth = float(self.get_parameter("min_depth_m").value)
        self.max_depth = float(self.get_parameter("max_depth_m").value)
        self.metric_depth = bool(self.get_parameter("metric_depth").value)
        self.point_stride = max(1, int(self.get_parameter("point_stride").value))
        self.publish_pointcloud = bool(self.get_parameter("publish_pointcloud").value)
        self.publish_colored = bool(self.get_parameter("publish_colored").value)
        self.colormap = int(self.get_parameter("colormap").value)

        self.qos_reliability = str(self.get_parameter("qos_reliability").value).strip().lower()
        self.processing_frequency = float(self.get_parameter("processing_frequency").value)
        self.log_every_n_frames = max(1, int(self.get_parameter("log_every_n_frames").value))

        # Publishers
        self.depth_pub = self.create_publisher(Image, self.depth_topic, 10)
        self.depth_color_pub = self.create_publisher(Image, self.depth_colored_topic, 10)
        # CompressedImage for dashboard compatibility
        self.depth_color_compressed_pub = self.create_publisher(
            CompressedImage, self.depth_colored_topic + "/compressed", 10
        )
        if self.publish_pointcloud:
            self.points_pub = self.create_publisher(PointCloud2, self.pointcloud_topic, 10)
        else:
            self.points_pub = None

        # Model session
        self._session = None
        self._input_name = None
        self._output_name = None
        self._load_model()

        # Subscriber
        qos = self._sensor_qos()
        self.image_sub = self.create_subscription(
            Image, self.input_image_topic, self._on_image, qos
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self._on_camera_info, qos
        )

        self._camera_info: Optional[CameraInfo] = None
        self._frame_counter = 0
        self._proc_ms_acc = 0.0
        self._last_proc_time = 0.0

        self.get_logger().info(
            f"fv_depth_anything started: backend={self.backend} model={self.model_path} "
            f"input={self.input_image_topic} depth={self.depth_topic} "
            f"metric={self.metric_depth} infer_size={self.infer_w}x{self.infer_h}"
        )

    def _sensor_qos(self) -> QoSProfile:
        reliable = self.qos_reliability == "reliable"
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

    def _load_model(self) -> None:
        if not self.model_path or not os.path.isfile(self.model_path):
            self.get_logger().error(f"Model file not found: {self.model_path}")
            return

        if self.backend == "onnx":
            self._load_onnx()
        elif self.backend == "tensorrt":
            self._load_tensorrt()
        else:
            self.get_logger().error(f"Unknown backend: {self.backend}")

        # Detect model input/output format after loading
        if self._session is not None:
            inp = self._session.get_inputs()[0]
            self._input_ndim = len(inp.shape)
            self._input_name = inp.name
            self._output_name = self._session.get_outputs()[0].name

            # Detect fixed input size from model (for METRIC-LARGE style)
            self._model_h = None
            self._model_w = None
            if self._input_ndim == 4 and isinstance(inp.shape[2], int) and isinstance(inp.shape[3], int):
                self._model_h = inp.shape[2]
                self._model_w = inp.shape[3]
                self.get_logger().info(f"Fixed input model: {self._model_w}x{self._model_h}")

            self.get_logger().info(
                f"Model I/O: input={self._input_name} ndim={self._input_ndim} "
                f"output={self._output_name}"
            )

    def _load_onnx(self) -> None:
        try:
            import onnxruntime as ort

            providers = []
            available = ort.get_available_providers()
            if "TensorrtExecutionProvider" in available:
                providers.append("TensorrtExecutionProvider")
            if "CUDAExecutionProvider" in available:
                providers.append("CUDAExecutionProvider")
            providers.append("CPUExecutionProvider")

            sess_opts = ort.SessionOptions()
            sess_opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL

            self._session = ort.InferenceSession(
                self.model_path, sess_options=sess_opts, providers=providers
            )
            self._input_name = self._session.get_inputs()[0].name
            self._output_name = self._session.get_outputs()[0].name

            active_provider = self._session.get_providers()[0]
            self.get_logger().info(
                f"ONNX model loaded: {self.model_path} provider={active_provider}"
            )
        except Exception as e:
            self.get_logger().error(f"ONNX model load failed: {e}")
            self._session = None

    def _load_tensorrt(self) -> None:
        try:
            import onnxruntime as ort

            providers = [
                ("TensorrtExecutionProvider", {
                    "trt_engine_cache_enable": True,
                    "trt_engine_cache_path": os.path.dirname(self.model_path),
                    "trt_fp16_enable": True,
                }),
                "CUDAExecutionProvider",
            ]

            sess_opts = ort.SessionOptions()
            self._session = ort.InferenceSession(
                self.model_path, sess_options=sess_opts, providers=providers
            )
            self._input_name = self._session.get_inputs()[0].name
            self._output_name = self._session.get_outputs()[0].name

            active_provider = self._session.get_providers()[0]
            self.get_logger().info(
                f"TensorRT model loaded: {self.model_path} provider={active_provider}"
            )
        except Exception as e:
            self.get_logger().error(f"TensorRT model load failed: {e}")
            self._session = None

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _on_image(self, msg: Image) -> None:
        if self._session is None:
            return

        # Throttle if processing_frequency is set
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

        if img_bgr is None:
            return

        orig_h, orig_w = img_bgr.shape[:2]

        # Preprocess: resize to inference size, normalize
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

        # Use model's fixed size if available, otherwise use configured size
        target_w = self._model_w if self._model_w else self.infer_w
        target_h = self._model_h if self._model_h else self.infer_h

        img_resized = cv2.resize(img_rgb, (target_w, target_h), interpolation=cv2.INTER_LINEAR)
        img_float = img_resized.astype(np.float32) / 255.0

        # Normalize with ImageNet mean/std
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        img_float = (img_float - mean) / std

        # Build input tensor based on model format
        img_chw = np.transpose(img_float, (2, 0, 1)).astype(np.float32)
        if self._input_ndim == 5:
            # DA3 multi-view format: [B, N, 3, H, W] with N=1
            input_tensor = img_chw[np.newaxis, np.newaxis, ...]
        else:
            # Standard NCHW: [B, 3, H, W]
            input_tensor = img_chw[np.newaxis, ...]

        # Inference
        try:
            outputs = self._session.run(
                [self._output_name], {self._input_name: input_tensor}
            )
            depth_pred = outputs[0].squeeze()
        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}")
            return

        # Resize depth back to original resolution
        if depth_pred.shape != (orig_h, orig_w):
            depth_pred = cv2.resize(
                depth_pred, (orig_w, orig_h), interpolation=cv2.INTER_LINEAR
            )

        # For non-metric (relative) depth, normalize to [min_depth, max_depth]
        if not self.metric_depth:
            d_min = depth_pred.min()
            d_max = depth_pred.max()
            if d_max - d_min > 1e-6:
                depth_pred = (depth_pred - d_min) / (d_max - d_min)
                depth_pred = depth_pred * (self.max_depth - self.min_depth) + self.min_depth
            else:
                depth_pred = np.full_like(depth_pred, self.min_depth)
        else:
            depth_pred = np.clip(depth_pred, self.min_depth, self.max_depth)

        depth_f32 = depth_pred.astype(np.float32)

        # Build header
        header = msg.header
        if self.frame_id_override:
            header.frame_id = self.frame_id_override

        # Publish depth (32FC1)
        depth_msg = self._cv2_to_imgmsg(depth_f32, "32FC1", header)
        self.depth_pub.publish(depth_msg)

        # Publish depth visualization
        if self.publish_colored:
            d_norm = depth_f32 - depth_f32.min()
            d_max = d_norm.max()
            if d_max > 1e-6:
                d_norm = (d_norm / d_max * 255.0).astype(np.uint8)
            else:
                d_norm = np.zeros_like(depth_f32, dtype=np.uint8)

            if self.colormap >= 0:
                # Colormap mode (e.g. INFERNO, JET)
                vis = cv2.applyColorMap(d_norm, self.colormap)
                encoding = "bgr8"
            else:
                # Raw grayscale depth (colormap = -1)
                vis = d_norm
                encoding = "mono8"

            color_msg = self._cv2_to_imgmsg(vis, encoding, header)
            self.depth_color_pub.publish(color_msg)

            # Publish compressed for dashboard
            _, jpeg_data = cv2.imencode(".jpg", vis, [cv2.IMWRITE_JPEG_QUALITY, 80])
            comp_msg = CompressedImage()
            comp_msg.header = header
            comp_msg.format = "jpeg"
            comp_msg.data = jpeg_data.tobytes()
            self.depth_color_compressed_pub.publish(comp_msg)

        # Publish point cloud (with color)
        if self.points_pub is not None and self._camera_info is not None:
            cloud = self._depth_to_pointcloud(depth_f32, self._camera_info, header, img_bgr)
            self.points_pub.publish(cloud)

        # Stats
        self._frame_counter += 1
        proc_ms = (time.perf_counter() - start) * 1000.0
        self._proc_ms_acc += proc_ms
        if self._frame_counter % self.log_every_n_frames == 0:
            avg = self._proc_ms_acc / float(self.log_every_n_frames)
            fps = 1000.0 / avg if avg > 0 else 0
            self._proc_ms_acc = 0.0
            self.get_logger().info(
                f"frames={self._frame_counter} avg={avg:.1f}ms ({fps:.1f}fps) "
                f"backend={self.backend}"
            )

    # --- Image conversion (no cv_bridge dependency) ---
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
        elif encoding == "bayer_rggb8":
            bayer = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            return cv2.cvtColor(bayer, cv2.COLOR_BayerRG2BGR)
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

    def _depth_to_pointcloud(
        self, depth: np.ndarray, info: CameraInfo, header,
        color_bgr: Optional[np.ndarray] = None
    ) -> PointCloud2:
        k = np.asarray(info.k, dtype=np.float32).reshape(3, 3)
        fx, fy = float(k[0, 0]), float(k[1, 1])
        cx, cy = float(k[0, 2]), float(k[1, 2])

        h, w = depth.shape
        # Scale intrinsics if camera_info resolution differs
        if info.width > 0 and info.width != w:
            scale_x = float(w) / float(info.width)
            scale_y = float(h) / float(info.height)
            fx *= scale_x
            fy *= scale_y
            cx *= scale_x
            cy *= scale_y

        stride = self.point_stride
        u = np.arange(0, w, stride, dtype=np.float32)
        v = np.arange(0, h, stride, dtype=np.float32)
        uu, vv = np.meshgrid(u, v)
        z = depth[::stride, ::stride].astype(np.float32)

        valid = np.isfinite(z) & (z >= self.min_depth) & (z <= self.max_depth)
        z_valid = z[valid]
        x_valid = (uu[valid] - cx) * z_valid / fx
        y_valid = (vv[valid] - cy) * z_valid / fy

        # Pack XYZRGB if color available
        has_color = color_bgr is not None
        if has_color:
            # Resize color to match depth if needed
            if color_bgr.shape[:2] != (h, w):
                color_bgr = cv2.resize(color_bgr, (w, h), interpolation=cv2.INTER_LINEAR)
            bgr_sampled = color_bgr[::stride, ::stride]
            b = bgr_sampled[valid, 0].astype(np.uint8)
            g = bgr_sampled[valid, 1].astype(np.uint8)
            r = bgr_sampled[valid, 2].astype(np.uint8)
            # Pack RGB into float32 (ROS PointCloud2 convention)
            rgb_uint32 = (r.astype(np.uint32) << 16) | (g.astype(np.uint32) << 8) | b.astype(np.uint32)
            rgb_float = rgb_uint32.view(np.float32)

            xyzrgb = np.zeros((z_valid.shape[0], 4), dtype=np.float32)
            xyzrgb[:, 0] = x_valid
            xyzrgb[:, 1] = y_valid
            xyzrgb[:, 2] = z_valid
            xyzrgb[:, 3] = rgb_float

            msg = PointCloud2()
            msg.header = header
            msg.height = 1
            msg.width = int(xyzrgb.shape[0])
            msg.is_bigendian = False
            msg.is_dense = False
            msg.point_step = 16
            msg.row_step = 16 * msg.width
            msg.fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            msg.data = xyzrgb.tobytes()
        else:
            xyz = np.stack([x_valid, y_valid, z_valid], axis=-1).astype(np.float32)
            msg = PointCloud2()
            msg.header = header
            msg.height = 1
            msg.width = int(xyz.shape[0])
            msg.is_bigendian = False
            msg.is_dense = False
            msg.point_step = 12
            msg.row_step = 12 * msg.width
            msg.fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            msg.data = xyz.tobytes()
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FvDepthAnythingNode()
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
