#!/usr/bin/env python3
from __future__ import annotations

import io
import time
from typing import Optional, Tuple
import urllib.error
import urllib.request

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


class FvLingbotDepthNode(Node):
    def __init__(self) -> None:
        super().__init__("fv_lingbot_depth")
        self.bridge = CvBridge()

        # I/O
        self.declare_parameter("color_topic", "~/color/image_raw")
        self.declare_parameter("depth_topic", "~/depth/image_rect_raw")
        self.declare_parameter("camera_info_topic", "~/camera_info")
        self.declare_parameter("refined_depth_topic", "depth_refined/image_rect_raw")
        self.declare_parameter("mask_topic", "depth_refined/mask")
        self.declare_parameter("pointcloud_topic", "depth_refined/points")
        self.declare_parameter("frame_id_override", "")

        # Model/runtime
        self.declare_parameter("backend", "direct")  # direct|http
        self.declare_parameter("worker_endpoint", "http://127.0.0.1:5540/infer")
        self.declare_parameter("worker_timeout_sec", 5.0)
        self.declare_parameter("model_id", "robbyant/lingbot-depth-pretrain-vitl-14")
        self.declare_parameter("local_model_path", "")
        self.declare_parameter("device", "auto")  # auto|cuda|cpu
        self.declare_parameter("use_fp16", True)
        self.declare_parameter("apply_mask", True)
        self.declare_parameter("resolution_level", 9)
        self.declare_parameter("fallback_passthrough", True)

        # Depth/point settings
        self.declare_parameter("depth_scale_16uc1", 0.001)
        self.declare_parameter("min_depth_m", 0.05)
        self.declare_parameter("max_depth_m", 3.0)
        self.declare_parameter("point_stride", 2)

        # Sync/QoS
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop_sec", 0.03)
        self.declare_parameter("qos_reliability", "best_effort")

        # Logging
        self.declare_parameter("log_every_n_frames", 30)

        self.color_topic = str(self.get_parameter("color_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.refined_depth_topic = str(self.get_parameter("refined_depth_topic").value)
        self.mask_topic = str(self.get_parameter("mask_topic").value)
        self.pointcloud_topic = str(self.get_parameter("pointcloud_topic").value)
        self.frame_id_override = str(self.get_parameter("frame_id_override").value)

        self.backend = str(self.get_parameter("backend").value).strip().lower()
        self.worker_endpoint = str(self.get_parameter("worker_endpoint").value).strip()
        self.worker_timeout_sec = float(self.get_parameter("worker_timeout_sec").value)
        self.model_id = str(self.get_parameter("model_id").value)
        self.local_model_path = str(self.get_parameter("local_model_path").value)
        self.device_mode = str(self.get_parameter("device").value)
        self.use_fp16 = bool(self.get_parameter("use_fp16").value)
        self.apply_mask = bool(self.get_parameter("apply_mask").value)
        self.resolution_level = int(self.get_parameter("resolution_level").value)
        self.fallback_passthrough = bool(self.get_parameter("fallback_passthrough").value)

        self.depth_scale_16uc1 = float(self.get_parameter("depth_scale_16uc1").value)
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.point_stride = max(1, int(self.get_parameter("point_stride").value))

        self.sync_queue_size = int(self.get_parameter("sync_queue_size").value)
        self.sync_slop_sec = float(self.get_parameter("sync_slop_sec").value)
        self.qos_reliability = str(self.get_parameter("qos_reliability").value).strip().lower()

        self.log_every_n_frames = max(1, int(self.get_parameter("log_every_n_frames").value))

        self.depth_pub = self.create_publisher(Image, self.refined_depth_topic, 10)
        self.mask_pub = self.create_publisher(Image, self.mask_topic, 10)
        self.points_pub = self.create_publisher(PointCloud2, self.pointcloud_topic, 10)

        if self.backend not in ("direct", "http"):
            self.get_logger().warning(
                f"Unknown backend='{self.backend}', falling back to direct"
            )
            self.backend = "direct"

        self._model = None
        self._torch = None
        self._device = None
        self._model_error = ""
        if self.backend == "direct":
            self._try_load_model()
        else:
            self.get_logger().info(
                f"LingBot-Depth worker backend enabled endpoint={self.worker_endpoint}"
            )

        qos = self._sensor_qos_profile()
        self.color_sub = Subscriber(self, Image, self.color_topic, qos_profile=qos)
        self.depth_sub = Subscriber(self, Image, self.depth_topic, qos_profile=qos)
        self.info_sub = Subscriber(self, CameraInfo, self.camera_info_topic, qos_profile=qos)
        self.sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.info_sub],
            queue_size=self.sync_queue_size,
            slop=self.sync_slop_sec,
            allow_headerless=False,
        )
        self.sync.registerCallback(self._on_synced)

        self._frame_counter = 0
        self._proc_ms_acc = 0.0

        model_source = self.local_model_path if self.local_model_path else self.model_id
        self.get_logger().info(
            f"fv_lingbot_depth started color={self.color_topic} depth={self.depth_topic} "
            f"info={self.camera_info_topic} out_depth={self.refined_depth_topic} "
            f"out_points={self.pointcloud_topic} backend={self.backend} model={model_source}"
        )

    def _sensor_qos_profile(self) -> QoSProfile:
        reliable = self.qos_reliability == "reliable"
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

    def _try_load_model(self) -> None:
        try:
            import torch
            from mdm.model.v2 import MDMModel
        except Exception as exc:
            self._model = None
            self._torch = None
            self._device = None
            self._model_error = str(exc)
            self.get_logger().error(
                f"LingBot-Depth import failed. Install lingbot-depth deps. error={self._model_error}"
            )
            return

        try:
            if self.device_mode == "cuda":
                device = "cuda"
            elif self.device_mode == "cpu":
                device = "cpu"
            else:
                device = "cuda" if torch.cuda.is_available() else "cpu"

            model_source = self.local_model_path.strip() if self.local_model_path.strip() else self.model_id
            model = MDMModel.from_pretrained(model_source).to(device).eval()

            self._torch = torch
            self._device = device
            self._model = model
            self._model_error = ""
            self.get_logger().info(f"LingBot-Depth model loaded on {device} ({model_source})")
        except Exception as exc:
            self._model = None
            self._torch = None
            self._device = None
            self._model_error = str(exc)
            self.get_logger().error(f"LingBot-Depth model load failed: {self._model_error}")

    def _on_synced(self, color_msg: Image, depth_msg: Image, info_msg: CameraInfo) -> None:
        start = time.perf_counter()

        try:
            color_bgr = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth_m = self._depth_to_meters(depth_msg)
        except Exception as exc:
            self.get_logger().warning(f"Input conversion failed: {exc}")
            return

        if color_bgr is None or depth_m is None:
            return

        if color_bgr.shape[:2] != depth_m.shape[:2]:
            color_bgr = cv2.resize(
                color_bgr,
                (depth_m.shape[1], depth_m.shape[0]),
                interpolation=cv2.INTER_LINEAR,
            )

        intr_norm = self._normalized_intrinsics(info_msg, depth_m.shape[1], depth_m.shape[0])
        if intr_norm is None:
            return

        if self.backend == "http":
            refined_depth, mask, points = self._run_remote_model(color_bgr, depth_m, intr_norm)
            if refined_depth is None:
                if not self.fallback_passthrough:
                    return
                refined_depth = depth_m
                mask = np.isfinite(refined_depth) & (refined_depth > 0.0)
                points = self._depth_to_points(refined_depth, intr_norm, mask)
        elif self._model is not None:
            refined_depth, mask, points = self._run_model(color_bgr, depth_m, intr_norm)
            if refined_depth is None:
                if not self.fallback_passthrough:
                    return
                refined_depth = depth_m
                mask = np.isfinite(refined_depth) & (refined_depth > 0.0)
                points = self._depth_to_points(refined_depth, intr_norm, mask)
        else:
            if not self.fallback_passthrough:
                return
            refined_depth = depth_m
            mask = np.isfinite(refined_depth) & (refined_depth > 0.0)
            points = self._depth_to_points(refined_depth, intr_norm, mask)

        header = depth_msg.header
        if self.frame_id_override:
            header.frame_id = self.frame_id_override

        depth_out = np.nan_to_num(refined_depth.astype(np.float32), nan=0.0, posinf=0.0, neginf=0.0)
        depth_image_msg = self.bridge.cv2_to_imgmsg(depth_out, encoding="32FC1")
        depth_image_msg.header = header
        self.depth_pub.publish(depth_image_msg)

        if mask is None:
            mask = np.isfinite(depth_out) & (depth_out > 0.0)
        mask_u8 = np.where(mask, 255, 0).astype(np.uint8)
        mask_msg = self.bridge.cv2_to_imgmsg(mask_u8, encoding="mono8")
        mask_msg.header = header
        self.mask_pub.publish(mask_msg)

        cloud_msg = self._to_pointcloud2(points, header)
        self.points_pub.publish(cloud_msg)

        self._frame_counter += 1
        proc_ms = (time.perf_counter() - start) * 1000.0
        self._proc_ms_acc += proc_ms
        if self._frame_counter % self.log_every_n_frames == 0:
            avg = self._proc_ms_acc / float(self.log_every_n_frames)
            self._proc_ms_acc = 0.0
            mode = "on" if self._model is not None else "fallback"
            self.get_logger().info(
                f"processed={self._frame_counter} avg_proc_ms={avg:.1f} model={mode}"
            )

    def _run_remote_model(
        self, color_bgr: np.ndarray, depth_m: np.ndarray, intr_norm: np.ndarray
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        payload = io.BytesIO()
        np.savez_compressed(
            payload,
            color_bgr=color_bgr.astype(np.uint8, copy=False),
            depth_m=depth_m.astype(np.float32, copy=False),
            intrinsics_norm=intr_norm.astype(np.float32, copy=False),
            use_fp16=np.asarray(self.use_fp16, dtype=np.uint8),
            apply_mask=np.asarray(self.apply_mask, dtype=np.uint8),
            resolution_level=np.asarray(self.resolution_level, dtype=np.int32),
        )

        req = urllib.request.Request(
            self.worker_endpoint,
            data=payload.getvalue(),
            headers={"Content-Type": "application/octet-stream"},
            method="POST",
        )

        try:
            with urllib.request.urlopen(req, timeout=self.worker_timeout_sec) as resp:
                body = resp.read()
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace").strip()
            if detail:
                self.get_logger().error(
                    f"worker request failed status={exc.code} detail={detail}"
                )
            else:
                self.get_logger().error(f"worker request failed status={exc.code}")
            return None, None, None
        except Exception as exc:
            self.get_logger().error(f"worker request failed: {exc}")
            return None, None, None

        try:
            with np.load(io.BytesIO(body), allow_pickle=False) as result:
                depth_np = None
                points_np = None
                mask_np = None

                if "depth" in result.files:
                    depth_np = result["depth"].astype(np.float32, copy=False)
                if "mask" in result.files:
                    mask_np = result["mask"].astype(bool, copy=False)
                if "points" in result.files:
                    points_np = result["points"].astype(np.float32, copy=False)

                if points_np is None and depth_np is not None:
                    points_np = self._depth_to_points(depth_np, intr_norm, mask_np)
                return depth_np, mask_np, points_np
        except Exception as exc:
            self.get_logger().error(f"invalid worker response: {exc}")
            return None, None, None

    def _depth_to_meters(self, msg: Image) -> np.ndarray:
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if depth is None:
            raise RuntimeError("depth image is empty")
        if depth.ndim != 2:
            raise RuntimeError(f"unexpected depth shape: {depth.shape}")

        if msg.encoding.lower() in ("16uc1", "mono16") or depth.dtype == np.uint16:
            depth_m = depth.astype(np.float32) * self.depth_scale_16uc1
        elif msg.encoding.lower() in ("32fc1",) or depth.dtype == np.float32:
            depth_m = depth.astype(np.float32)
        else:
            depth_m = depth.astype(np.float32)
        return np.nan_to_num(depth_m, nan=0.0, posinf=0.0, neginf=0.0)

    def _normalized_intrinsics(self, info: CameraInfo, width: int, height: int) -> Optional[np.ndarray]:
        k = np.asarray(info.k, dtype=np.float32).reshape(3, 3)
        fx = float(k[0, 0])
        fy = float(k[1, 1])
        cx = float(k[0, 2])
        cy = float(k[1, 2])
        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().warning(f"invalid camera intrinsics fx={fx:.3f} fy={fy:.3f}")
            return None

        out = np.eye(3, dtype=np.float32)
        out[0, 0] = fx / float(width)
        out[1, 1] = fy / float(height)
        out[0, 2] = cx / float(width)
        out[1, 2] = cy / float(height)
        return out

    def _run_model(
        self, color_bgr: np.ndarray, depth_m: np.ndarray, intr_norm: np.ndarray
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        if self._model is None or self._torch is None:
            return None, None, None

        try:
            torch = self._torch
            color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0

            image_t = torch.from_numpy(color_rgb).to(device=self._device, dtype=torch.float32)
            image_t = image_t.permute(2, 0, 1).unsqueeze(0)
            depth_t = torch.from_numpy(depth_m.astype(np.float32)).to(device=self._device).unsqueeze(0)
            intr_t = torch.from_numpy(intr_norm).to(device=self._device).unsqueeze(0)

            with torch.inference_mode():
                out = self._model.infer(
                    image_t,
                    depth_in=depth_t,
                    intrinsics=intr_t,
                    use_fp16=self.use_fp16,
                    apply_mask=self.apply_mask,
                    resolution_level=self.resolution_level,
                )

            depth_out = out.get("depth", None)
            points_out = out.get("points", None)
            mask_out = out.get("mask", None)

            depth_np = None
            points_np = None
            mask_np = None
            if depth_out is not None:
                depth_np = depth_out.detach().float().cpu().numpy().squeeze().astype(np.float32)
            if points_out is not None:
                points_np = points_out.detach().float().cpu().numpy().squeeze().astype(np.float32)
            if mask_out is not None:
                mask_np = mask_out.detach().cpu().numpy().squeeze().astype(bool)

            if points_np is None and depth_np is not None:
                points_np = self._depth_to_points(depth_np, intr_norm, mask_np)

            return depth_np, mask_np, points_np
        except Exception as exc:
            self.get_logger().error(f"inference failed: {exc}")
            return None, None, None

    def _depth_to_points(
        self, depth_m: np.ndarray, intr_norm: np.ndarray, mask: Optional[np.ndarray]
    ) -> np.ndarray:
        h, w = depth_m.shape
        fx = float(intr_norm[0, 0]) * float(w)
        fy = float(intr_norm[1, 1]) * float(h)
        cx = float(intr_norm[0, 2]) * float(w)
        cy = float(intr_norm[1, 2]) * float(h)

        u, v = np.meshgrid(
            np.arange(w, dtype=np.float32),
            np.arange(h, dtype=np.float32),
        )
        z = depth_m.astype(np.float32)
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        pts = np.stack([x, y, z], axis=-1)

        valid = np.isfinite(z) & (z > 0.0)
        if mask is not None:
            valid &= mask
        pts[~valid] = np.inf
        return pts

    def _to_pointcloud2(self, points: Optional[np.ndarray], header) -> PointCloud2:
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.is_bigendian = False
        msg.is_dense = False
        msg.point_step = 12
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        if points is None or points.ndim != 3 or points.shape[2] != 3:
            msg.width = 0
            msg.row_step = 0
            msg.data = b""
            return msg

        sampled = points[:: self.point_stride, :: self.point_stride, :].reshape(-1, 3)
        if sampled.size == 0:
            msg.width = 0
            msg.row_step = 0
            msg.data = b""
            return msg

        valid = (
            np.isfinite(sampled[:, 0])
            & np.isfinite(sampled[:, 1])
            & np.isfinite(sampled[:, 2])
            & (sampled[:, 2] >= self.min_depth_m)
            & (sampled[:, 2] <= self.max_depth_m)
        )
        xyz = sampled[valid].astype(np.float32, copy=False)
        msg.width = int(xyz.shape[0])
        msg.row_step = msg.point_step * msg.width
        msg.data = xyz.tobytes()
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FvLingbotDepthNode()
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
