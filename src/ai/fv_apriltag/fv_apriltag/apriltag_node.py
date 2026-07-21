"""AprilTag detection node (Python, pupil-apriltags backend).

One instance per camera. Subscribes to image + camera_info, runs
detection, publishes:
- apriltag_msgs/AprilTagDetectionArray on ~/detections
- geometry_msgs/PoseArray on ~/poses (camera-frame tag poses)
- sensor_msgs/Image on ~/image_annotated (overlay) if enabled
- tf2 transforms: <camera_frame> -> <tag_frame_prefix><id>
"""
from __future__ import annotations

from collections import deque
import json
import math
import time
from typing import Deque, Optional

from ament_index_python.packages import get_package_share_directory
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from apriltag_msgs.msg import Point as ATPoint
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .tag_registry import TagRegistryError, TagSizeRegistry, TagSizeResolver


def _rotation_matrix_to_quaternion(R: np.ndarray) -> tuple:
    """3x3 rotation matrix -> (x, y, z, w) quaternion."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 2.0 * math.sqrt(trace + 1.0)
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


class ApriltagNode(Node):
    def __init__(self) -> None:
        super().__init__('apriltag_node')

        self.declare_parameter('family', 'tag36h11')
        self.declare_parameter('nthreads', 2)
        self.declare_parameter('quad_decimate', 2.0)
        self.declare_parameter('quad_sigma', 0.0)
        self.declare_parameter('refine_edges', 1)
        self.declare_parameter('decode_sharpening', 0.25)
        # Runtime source of truth is config/tag_registry.yaml. A positive
        # tag_size is an explicit homogeneous black-edge override.
        self.declare_parameter('tag_registry', '')
        self.declare_parameter('tag_size', 0.0)
        # Optional per-ID black-edge override: [id, metres, id, metres, ...].
        self.declare_parameter('tag_sizes', [])
        self.declare_parameter('min_decision_margin', 50.0)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('tag_frame_prefix', 'tag')
        self.declare_parameter('publish_tf', True)
        # Runtime on/off gate (same pattern as fv_yoloe): while false the
        # image callback returns before any decode/detection work, so the
        # node costs nothing; re-enabling via `ros2 param set` (or the
        # perception MCP) is instant.
        self.declare_parameter('enabled', True)
        # Optional depth source for per-tag accurate distance overlay.
        # Empty string disables. Topic should be aligned to the color
        # image (e.g. /d405_depth/image_rect_raw paired with
        # /d405_color/image_raw).
        self.declare_parameter('depth_topic', '')
        self.declare_parameter('depth_info_topic', '')

        self.family = self.get_parameter('family').get_parameter_value().string_value
        self.min_decision_margin = float(
            self.get_parameter('min_decision_margin').value
        )
        self.publish_annotated = bool(
            self.get_parameter('publish_annotated').value
        )
        self.tag_frame_prefix = self.get_parameter(
            'tag_frame_prefix'
        ).get_parameter_value().string_value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.enabled = bool(self.get_parameter('enabled').value)
        self.add_on_set_parameters_callback(self._on_param_update)

        registry_path = self.get_parameter(
            'tag_registry'
        ).get_parameter_value().string_value
        if not registry_path:
            registry_path = str(
                get_package_share_directory('fv_apriltag')
                + '/config/tag_registry.yaml'
            )
        try:
            registry = TagSizeRegistry.from_file(registry_path)
            if registry.family != self.family:
                raise TagRegistryError(
                    f'registry family {registry.family!r} does not match '
                    f'detector family {self.family!r}'
                )
            self._tag_size_resolver = TagSizeResolver.create(
                registry=registry,
                single_size_m=self.get_parameter('tag_size').value,
                per_id_raw=self.get_parameter('tag_sizes').value,
            )
            self._pose_reference_size = (
                self._tag_size_resolver.reference_size()
            )
        except TagRegistryError as exc:
            self.get_logger().fatal(f'unsafe AprilTag size config: {exc}')
            raise RuntimeError('invalid AprilTag size configuration') from exc
        self._missing_size_ids = set()

        try:
            from pupil_apriltags import Detector
        except ImportError as exc:
            self.get_logger().fatal(
                f'pupil_apriltags not available: {exc}. '
                'Install via `pip3 install pupil-apriltags` (handled by '
                'container entrypoint).'
            )
            raise

        self._detector = Detector(
            families=self.family,
            nthreads=int(self.get_parameter('nthreads').value),
            quad_decimate=float(self.get_parameter('quad_decimate').value),
            quad_sigma=float(self.get_parameter('quad_sigma').value),
            refine_edges=int(self.get_parameter('refine_edges').value),
            decode_sharpening=float(
                self.get_parameter('decode_sharpening').value
            ),
        )

        self._bridge = CvBridge()
        self._cam_info: Optional[CameraInfo] = None
        self._undistort_map: Optional[tuple] = None
        self._undistort_K: Optional[np.ndarray] = None
        # Latest depth image (metres). Resized to color resolution if
        # the depth stream is a different size.
        self._depth_image: Optional[np.ndarray] = None
        self._depth_K: Optional[np.ndarray] = None

        # Separate callback groups so the image processing pipeline
        # doesn't block on housekeeping callbacks (camera_info, depth,
        # depth_info). With the default single-threaded executor those
        # serialise behind detection, capping callback rate well below
        # the source frame rate even when detection itself is fast.
        self._cb_image = ReentrantCallbackGroup()
        self._cb_aux = ReentrantCallbackGroup()

        self._sub_image = self.create_subscription(
            Image, 'image', self._on_image, qos_profile_sensor_data,
            callback_group=self._cb_image,
        )
        self._sub_info = self.create_subscription(
            CameraInfo, 'camera_info', self._on_camera_info,
            qos_profile_sensor_data,
            callback_group=self._cb_aux,
        )

        depth_topic = self.get_parameter(
            'depth_topic'
        ).get_parameter_value().string_value
        depth_info_topic = self.get_parameter(
            'depth_info_topic'
        ).get_parameter_value().string_value
        if depth_topic:
            self.create_subscription(
                Image, depth_topic, self._on_depth,
                qos_profile_sensor_data,
                callback_group=self._cb_aux,
            )
        if depth_info_topic:
            self.create_subscription(
                CameraInfo, depth_info_topic, self._on_depth_info,
                qos_profile_sensor_data,
                callback_group=self._cb_aux,
            )

        self._pub_detections = self.create_publisher(
            AprilTagDetectionArray, 'detections', 10
        )
        self._pub_poses = self.create_publisher(PoseArray, 'poses', 10)
        # Annotated image uses sensor-data QoS (BEST_EFFORT, depth 5) to
        # match fv_realsense / dashboard subscriber conventions. RELIABLE
        # +depth=1 was leaving the dashboard tile blank because the web
        # consumer can't keep up with reliable backpressure.
        # Both raw BGR and JPEG-compressed are exposed — dashboard /
        # foxglove can pick whichever is lighter. Compressed is ~50KB vs
        # 921KB raw at 640x480, removing the serialise/transport cost
        # spike that pulled callback throughput down.
        self._pub_annotated = (
            self.create_publisher(
                Image, 'image_annotated', qos_profile_sensor_data
            )
            if self.publish_annotated
            else None
        )
        self._pub_annotated_compressed = (
            self.create_publisher(
                CompressedImage, 'image_annotated/compressed',
                qos_profile_sensor_data,
            )
            if self.publish_annotated
            else None
        )
        self._pub_stats = self.create_publisher(String, 'stats', 10)
        self._tf_broadcaster = (
            TransformBroadcaster(self) if self.publish_tf else None
        )

        # Timing stats: rolling windows over the last ~2 seconds.
        self._detect_ms_window: Deque[float] = deque(maxlen=60)
        self._total_ms_window: Deque[float] = deque(maxlen=60)
        self._frame_t_window: Deque[float] = deque(maxlen=60)
        self._last_stats_log_t = 0.0

        self.get_logger().info(
            f'apriltag_node started family={self.family} '
            f'tag_size_mode={self._tag_size_resolver.mode_description()} '
            f'tag_registry={registry_path} '
            f'pose_reference_size={self._pose_reference_size}m '
            f'frame_prefix={self.tag_frame_prefix}'
        )

    def _tag_size_for(self, tag_id: int) -> Optional[float]:
        return self._tag_size_resolver.size_for(tag_id)

    def _warn_missing_size(self, tag_id: int) -> None:
        if tag_id in self._missing_size_ids:
            return
        self._missing_size_ids.add(tag_id)
        self.get_logger().warning(
            f'tag ID {tag_id} has no physical size in the active registry; '
            'suppressing detection, pose, and TF'
        )

    def _on_depth_info(self, msg: CameraInfo) -> None:
        if self._depth_K is None:
            self._depth_K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def _on_depth(self, msg: Image) -> None:
        try:
            if msg.encoding == '16UC1':
                arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                    msg.height, msg.width
                )
                self._depth_image = arr.astype(np.float32) / 1000.0
            elif msg.encoding == '32FC1':
                self._depth_image = np.frombuffer(
                    msg.data, dtype=np.float32
                ).reshape(msg.height, msg.width).copy()
        except Exception as exc:
            self.get_logger().warning(f'depth decode failed: {exc}')

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self._cam_info is not None:
            return
        self._cam_info = msg
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.d, dtype=np.float64) if msg.d else np.zeros(5)
        # Only build undistort map when distortion is non-trivial.
        if D.size > 0 and float(np.max(np.abs(D))) > 1e-6:
            map1, map2 = cv2.initUndistortRectifyMap(
                K, D, None, K, (msg.width, msg.height), cv2.CV_16SC2
            )
            self._undistort_map = (map1, map2)
            self.get_logger().info(
                f'Built undistort map (max|D|={float(np.max(np.abs(D))):.4f})'
            )
        else:
            self.get_logger().info('Distortion is zero, skipping undistort')
        self._undistort_K = K
        self.get_logger().info(
            f'Camera intrinsics: fx={K[0, 0]:.2f} fy={K[1, 1]:.2f} '
            f'cx={K[0, 2]:.2f} cy={K[1, 2]:.2f} '
            f'frame={msg.header.frame_id}'
        )

    def _on_param_update(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name in ('family', 'tag_registry', 'tag_size', 'tag_sizes'):
                return SetParametersResult(
                    successful=False,
                    reason=f'{p.name} is startup-only; restart the profile',
                )
            if p.name == 'enabled':
                new_enabled = bool(p.value)
                if new_enabled != self.enabled:
                    self.enabled = new_enabled
                    state = 'ENABLED' if new_enabled else \
                        'DISABLED - detector idle, no CPU work'
                    self.get_logger().info(f'AprilTag detection {state}')
        return SetParametersResult(successful=True)

    def _on_image(self, msg: Image) -> None:
        if not self.enabled:
            return
        if self._cam_info is None or self._undistort_K is None:
            return

        t_callback_start = time.perf_counter()
        try:
            if msg.encoding in ('mono8', '8UC1'):
                gray = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
                color_for_overlay = None
            else:
                bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                if self._undistort_map is not None:
                    bgr = cv2.remap(
                        bgr, self._undistort_map[0], self._undistort_map[1],
                        cv2.INTER_LINEAR,
                    )
                gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
                color_for_overlay = bgr
        except Exception as exc:
            self.get_logger().warning(f'image decode failed: {exc}')
            return

        if msg.encoding in ('mono8', '8UC1') and self._undistort_map is not None:
            gray = cv2.remap(
                gray, self._undistort_map[0], self._undistort_map[1],
                cv2.INTER_LINEAR,
            )

        K = self._undistort_K
        camera_params = (
            float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2])
        )

        # pupil-apriltags accepts one tag_size per call. Rotation is size
        # invariant and translation is linear in tag_size, so estimate at a
        # validated reference size and rescale per ID below.
        t_detect_start = time.perf_counter()
        detections = self._detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=camera_params,
            tag_size=self._pose_reference_size,
        )
        detect_ms = (time.perf_counter() - t_detect_start) * 1000.0

        msg_arr = AprilTagDetectionArray()
        msg_arr.header.stamp = msg.header.stamp
        msg_arr.header.frame_id = self._cam_info.header.frame_id
        pose_arr = PoseArray()
        pose_arr.header = msg_arr.header
        tf_msgs = []

        for det in detections:
            if det.decision_margin < self.min_decision_margin:
                continue

            tag_id = int(det.tag_id)
            tag_size = self._tag_size_for(tag_id)
            if tag_size is None:
                self._warn_missing_size(tag_id)
                continue
            scale = tag_size / self._pose_reference_size
            pose_t = det.pose_t * scale

            atd = AprilTagDetection()
            atd.family = self.family
            atd.id = tag_id
            atd.hamming = int(det.hamming)
            atd.goodness = 0.0
            atd.decision_margin = float(det.decision_margin)
            atd.centre = ATPoint(
                x=float(det.center[0]), y=float(det.center[1])
            )
            # pupil-apriltags returns corners in (x, y), 4 points
            for i in range(4):
                atd.corners[i] = ATPoint(
                    x=float(det.corners[i][0]),
                    y=float(det.corners[i][1]),
                )
            H = np.array(det.homography, dtype=np.float64).flatten().tolist()
            for i in range(9):
                atd.homography[i] = H[i]
            msg_arr.detections.append(atd)

            qx, qy, qz, qw = _rotation_matrix_to_quaternion(det.pose_R)
            pose = Pose()
            pose.position.x = float(pose_t[0])
            pose.position.y = float(pose_t[1])
            pose.position.z = float(pose_t[2])
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
            pose_arr.poses.append(pose)

            if self._tf_broadcaster is not None:
                tf = TransformStamped()
                tf.header.stamp = msg.header.stamp
                tf.header.frame_id = self._cam_info.header.frame_id
                tf.child_frame_id = f'{self.tag_frame_prefix}{tag_id}'
                tf.transform.translation.x = float(pose_t[0])
                tf.transform.translation.y = float(pose_t[1])
                tf.transform.translation.z = float(pose_t[2])
                tf.transform.rotation.x = qx
                tf.transform.rotation.y = qy
                tf.transform.rotation.z = qz
                tf.transform.rotation.w = qw
                tf_msgs.append(tf)

        self._pub_detections.publish(msg_arr)
        self._pub_poses.publish(pose_arr)
        if tf_msgs and self._tf_broadcaster is not None:
            self._tf_broadcaster.sendTransform(tf_msgs)

        # Frame-rate tracking from callback timestamps.
        now = time.perf_counter()
        self._frame_t_window.append(now)
        fps = self._compute_fps()
        n_kept = len(msg_arr.detections)

        total_ms = (now - t_callback_start) * 1000.0
        self._detect_ms_window.append(detect_ms)
        self._total_ms_window.append(total_ms)

        if self._pub_annotated is not None and color_for_overlay is not None:
            raw_subs = self._pub_annotated.get_subscription_count()
            comp_subs = (
                self._pub_annotated_compressed.get_subscription_count()
                if self._pub_annotated_compressed is not None else 0
            )
            if raw_subs > 0 or comp_subs > 0:
                self._draw_overlay(
                    color_for_overlay, detections,
                    detect_ms=detect_ms, total_ms=total_ms,
                    fps=fps, n_kept=n_kept,
                )
                if raw_subs > 0:
                    try:
                        out = self._bridge.cv2_to_imgmsg(
                            color_for_overlay, encoding='bgr8'
                        )
                        out.header = msg.header
                        self._pub_annotated.publish(out)
                    except Exception as exc:
                        self.get_logger().warning(
                            f'overlay raw publish failed: {exc}'
                        )
                if comp_subs > 0:
                    try:
                        ok, jpg = cv2.imencode(
                            '.jpg', color_for_overlay,
                            [int(cv2.IMWRITE_JPEG_QUALITY), 80],
                        )
                        if ok:
                            comp = CompressedImage()
                            comp.header = msg.header
                            comp.format = 'jpeg'
                            comp.data = jpg.tobytes()
                            self._pub_annotated_compressed.publish(comp)
                    except Exception as exc:
                        self.get_logger().warning(
                            f'overlay compressed publish failed: {exc}'
                        )

        # Stats topic (1 Hz to avoid spam).
        if now - self._last_stats_log_t > 1.0:
            self._last_stats_log_t = now
            stats = {
                'detect_ms': round(detect_ms, 2),
                'detect_ms_avg': round(
                    sum(self._detect_ms_window) / max(1, len(self._detect_ms_window)),
                    2,
                ),
                'total_ms': round(total_ms, 2),
                'total_ms_avg': round(
                    sum(self._total_ms_window) / max(1, len(self._total_ms_window)),
                    2,
                ),
                'fps': round(fps, 2),
                'n_detections': n_kept,
                'n_raw': len(detections),
            }
            self._pub_stats.publish(String(data=json.dumps(stats)))

    def _compute_fps(self) -> float:
        if len(self._frame_t_window) < 2:
            return 0.0
        dt = self._frame_t_window[-1] - self._frame_t_window[0]
        if dt <= 0.0:
            return 0.0
        return (len(self._frame_t_window) - 1) / dt

    def _draw_overlay(
        self,
        bgr: np.ndarray,
        detections,
        detect_ms: float,
        total_ms: float,
        fps: float,
        n_kept: int,
    ) -> None:
        for det in detections:
            if det.decision_margin < self.min_decision_margin:
                continue
            corners = det.corners.astype(int)
            tag_size = self._tag_size_for(int(det.tag_id))
            edge_color = (0, 255, 0) if tag_size is not None else (0, 0, 255)
            for i in range(4):
                p0 = tuple(corners[i])
                p1 = tuple(corners[(i + 1) % 4])
                cv2.line(bgr, p0, p1, edge_color, 2)
            cx, cy = int(det.center[0]), int(det.center[1])
            cv2.circle(bgr, (cx, cy), 4, (0, 0, 255), -1)

            pnp_z = 0.0
            if tag_size is not None and det.pose_t is not None:
                pnp_z = float(
                    det.pose_t[2] * tag_size / self._pose_reference_size
                )
            depth_z = self._sample_depth_at(cx, cy)

            lines = [f'id={det.tag_id}']
            if tag_size is None:
                lines.append('SIZE UNKNOWN - NO POSE')
            if depth_z is not None:
                lines.append(f'd={depth_z * 100:.1f}cm')
                if pnp_z > 0.01:
                    delta_mm = abs(depth_z - pnp_z) * 1000.0
                    lines.append(
                        f'pnp={pnp_z * 100:.1f}cm '
                        f'(Δ{delta_mm:.0f}mm)'
                    )
            elif pnp_z > 0.01:
                lines.append(f'pnp={pnp_z * 100:.1f}cm')
            lines.append(f'm={det.decision_margin:.0f}')

            # Label box anchored at top-right of the tag, with a small
            # offset so it doesn't sit on the tag corner.
            tag_xmax = int(np.max(corners[:, 0]))
            tag_ymin = int(np.min(corners[:, 1]))
            self._draw_label_box(
                bgr, lines, anchor=(tag_xmax + 4, max(4, tag_ymin - 2)),
            )

        # Stats banner: top-left, white text on semi-opaque black.
        avg_detect = (
            sum(self._detect_ms_window) / len(self._detect_ms_window)
            if self._detect_ms_window else 0.0
        )
        avg_total = (
            sum(self._total_ms_window) / len(self._total_ms_window)
            if self._total_ms_window else 0.0
        )
        banner_lines = [
            f'detect {detect_ms:5.1f} ms  (avg {avg_detect:5.1f})',
            f'total  {total_ms:5.1f} ms  (avg {avg_total:5.1f})',
            f'fps    {fps:5.1f}    tags {n_kept}',
        ]
        pad = 6
        line_h = 16
        bw = 240
        bh = pad * 2 + line_h * len(banner_lines)
        overlay = bgr.copy()
        cv2.rectangle(overlay, (4, 4), (4 + bw, 4 + bh), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.55, bgr, 0.45, 0, dst=bgr)
        for i, line in enumerate(banner_lines):
            cv2.putText(
                bgr, line, (4 + pad, 4 + pad + line_h * (i + 1) - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1,
                cv2.LINE_AA,
            )

    def _sample_depth_at(self, cx: int, cy: int) -> Optional[float]:
        if self._depth_image is None:
            return None
        depth = self._depth_image
        # Map color pixel to depth pixel if resolutions differ. D405's
        # color and depth streams are already aligned and same-resolution
        # in our pipeline so this is usually identity.
        dh, dw = depth.shape
        if self._cam_info is not None:
            ch, cw = self._cam_info.height, self._cam_info.width
            if (dh, dw) != (ch, cw):
                cx = int(round(cx * dw / max(1, cw)))
                cy = int(round(cy * dh / max(1, ch)))
        if not (0 <= cx < dw and 0 <= cy < dh):
            return None
        x0 = max(0, cx - 3)
        x1 = min(dw, cx + 4)
        y0 = max(0, cy - 3)
        y1 = min(dh, cy + 4)
        patch = depth[y0:y1, x0:x1]
        valid = patch[(patch > 0.01) & np.isfinite(patch)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    @staticmethod
    def _draw_label_box(bgr, lines, anchor) -> None:
        x, y = anchor
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.45
        thick = 1
        line_h = 14
        pad = 4
        w = 0
        for line in lines:
            (tw, _), _ = cv2.getTextSize(line, font, scale, thick)
            w = max(w, tw)
        bw = w + pad * 2
        bh = line_h * len(lines) + pad * 2
        # Clamp inside image.
        h_img, w_img = bgr.shape[:2]
        x = min(x, w_img - bw - 2)
        y = min(y, h_img - bh - 2)
        y = max(y, 2)
        x = max(x, 2)
        overlay = bgr.copy()
        cv2.rectangle(overlay, (x, y), (x + bw, y + bh), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.55, bgr, 0.45, 0, dst=bgr)
        for i, line in enumerate(lines):
            cv2.putText(
                bgr, line,
                (x + pad, y + pad + line_h * (i + 1) - 3),
                font, scale, (0, 255, 0), thick, cv2.LINE_AA,
            )


def main(args=None):
    rclpy.init(args=args)
    node = ApriltagNode()
    # MultiThreadedExecutor was measured slower on AGX Thor for this
    # workload (Python callbacks are GIL-bound — extra threads just add
    # contention). Stick with the default single-threaded spin.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
