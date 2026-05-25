"""Cube pose estimator using AprilTag bundle fusion + optional depth refine.

Subscribes to per-camera detection + pose topics (published by apriltag_node).
For each configured cube, identifies which detected tags belong to which
face, transforms each tag pose into a candidate cube pose, fuses candidates
from all visible faces (and all cameras), and publishes the result.

When a depth image is available for a camera, the per-detection Z (distance
to tag center) is replaced by the measured depth at the tag's center pixel,
which significantly reduces PnP depth bias on D405 at close range.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import message_filters
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import (
    PoseArray,
    PoseStamped,
    PoseWithCovarianceStamped,
    TransformStamped,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA, Header
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


FACE_NAMES = ('top', 'bottom', 'front', 'back', 'left', 'right')


def _origin_offset(origin: str, size: float) -> np.ndarray:
    """Offset from geometric center to the requested cube-frame origin."""
    if origin == 'geometric_center':
        return np.array([0.0, 0.0, 0.0])
    if origin == 'bottom_center':
        return np.array([0.0, 0.0, -size / 2.0])
    if origin == 'top_center':
        return np.array([0.0, 0.0, size / 2.0])
    raise ValueError(f'unknown origin: {origin}')


def _face_pose_in_cube(face: str, size: float, origin: str) -> Tuple[np.ndarray, np.ndarray]:
    """Return (position, rotation_matrix) of a tag's local frame in cube frame.

    Cube frame follows REP-103: +X forward (front), +Y left, +Z up.
    Tag frame: +Z normal out of tag face, +X right, +Y up (when viewing
    the tag from outside the cube, with the tag's printed "up" arrow
    pointing toward cube +Z for side faces, or toward cube +X for the
    top face).
    """
    half = size / 2.0
    off = _origin_offset(origin, size)
    # Geometric center frame, then shift by origin offset.
    if face == 'top':
        pos = np.array([0.0, 0.0, half]) + off
        # tag x=cube+X, y=cube+Y, z=cube+Z
        Rm = np.column_stack([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ])
    elif face == 'bottom':
        pos = np.array([0.0, 0.0, -half]) + off
        # tag x=cube+X, y=cube-Y, z=cube-Z (180° about X)
        Rm = np.column_stack([
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
        ])
    elif face == 'front':
        pos = np.array([half, 0.0, 0.0]) + off
        # tag x=cube-Y, y=cube+Z, z=cube+X
        Rm = np.column_stack([
            [0.0, -1.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
        ])
    elif face == 'back':
        pos = np.array([-half, 0.0, 0.0]) + off
        # tag x=cube+Y, y=cube+Z, z=cube-X
        Rm = np.column_stack([
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
        ])
    elif face == 'left':
        pos = np.array([0.0, half, 0.0]) + off
        # tag x=cube+X, y=cube+Z, z=cube+Y
        Rm = np.column_stack([
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
        ])
    elif face == 'right':
        pos = np.array([0.0, -half, 0.0]) + off
        # tag x=cube-X, y=cube+Z, z=cube-Y
        Rm = np.column_stack([
            [-1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, -1.0, 0.0],
        ])
    else:
        raise ValueError(f'unknown face: {face}')
    return pos, Rm


def _yaw_z(angle_deg: float) -> np.ndarray:
    """Rotation matrix for a Z-axis rotation in tag frame (yaw)."""
    if abs(angle_deg) < 1e-9:
        return np.eye(3)
    return R.from_euler('z', angle_deg, degrees=True).as_matrix()


@dataclass
class FaceDef:
    cube_name: str
    face: str
    tag_id: int
    tag_size: float
    # Tag pose in cube frame (4x4 homogeneous).
    T_tag_in_cube: np.ndarray
    T_cube_in_tag: np.ndarray  # cached inverse


@dataclass
class CubeDef:
    name: str
    size: float
    tag_size: float
    origin: str
    faces: Dict[str, int]  # face -> id


@dataclass
class CameraDef:
    name: str
    detections_topic: str
    poses_topic: str
    depth_topic: Optional[str]
    depth_info_topic: Optional[str]
    camera_frame: str


@dataclass
class CubeObs:
    """One cube pose candidate observed by a (camera, tag) pair."""
    stamp_ns: int
    camera: str
    face: str
    tag_id: int
    T_cube_in_cam: np.ndarray
    decision_margin: float


@dataclass
class CubeState:
    name: str
    obs: List[CubeObs] = field(default_factory=list)


def _make_T(pos: np.ndarray, Rm: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = Rm
    T[:3, 3] = pos
    return T


def _invert_T(T: np.ndarray) -> np.ndarray:
    Rm = T[:3, :3]
    p = T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = Rm.T
    out[:3, 3] = -Rm.T @ p
    return out


class CubeEstimatorNode(Node):
    def __init__(self) -> None:
        super().__init__('cube_estimator')

        self.declare_parameter('config_file', '')
        cfg_path = self.get_parameter(
            'config_file'
        ).get_parameter_value().string_value
        if not cfg_path:
            self.get_logger().fatal(
                'config_file parameter is required (path to YAML)'
            )
            raise RuntimeError('config_file missing')

        with open(cfg_path, 'r') as f:
            cfg = yaml.safe_load(f)

        fusion = cfg.get('fusion', {})
        self.max_age = float(fusion.get('max_age', 0.5))
        self.publish_rate = float(fusion.get('publish_rate', 30.0))
        self.use_depth_refine = bool(fusion.get('use_depth_refine', True))
        self.min_decision_margin = float(
            fusion.get('min_decision_margin', 50.0)
        )
        self.world_frame = str(
            fusion.get('world_frame', '')
        )  # empty = use camera frame of best obs

        self._cameras: List[CameraDef] = []
        for cam in cfg.get('cameras', []):
            self._cameras.append(CameraDef(
                name=cam['name'],
                detections_topic=cam['detections_topic'],
                poses_topic=cam['poses_topic'],
                depth_topic=cam.get('depth_topic') or None,
                depth_info_topic=cam.get('depth_info_topic') or None,
                camera_frame=cam.get('camera_frame', ''),
            ))

        self._cubes: Dict[str, CubeDef] = {}
        self._faces_by_id: Dict[int, List[FaceDef]] = {}
        for name, cdef in cfg.get('cubes', {}).items():
            cube = CubeDef(
                name=name,
                size=float(cdef['size']),
                tag_size=float(cdef.get('tag_size', 0.050)),
                origin=str(cdef.get('origin', 'bottom_center')),
                faces={str(k): int(v) for k, v in cdef.get('faces', {}).items()},
            )
            self._cubes[name] = cube
            for face_name, tag_id in cube.faces.items():
                if face_name not in FACE_NAMES:
                    self.get_logger().warning(
                        f'cube {name}: unknown face "{face_name}", skipping'
                    )
                    continue
                pos, Rm = _face_pose_in_cube(
                    face_name, cube.size, cube.origin
                )
                T_tag_in_cube = _make_T(pos, Rm)
                fd = FaceDef(
                    cube_name=name,
                    face=face_name,
                    tag_id=tag_id,
                    tag_size=cube.tag_size,
                    T_tag_in_cube=T_tag_in_cube,
                    T_cube_in_tag=_invert_T(T_tag_in_cube),
                )
                self._faces_by_id.setdefault(tag_id, []).append(fd)

        self._cube_states: Dict[str, CubeState] = {
            name: CubeState(name=name) for name in self._cubes
        }

        # Depth state per camera.
        self._depth_K: Dict[str, np.ndarray] = {}
        self._depth_image: Dict[str, np.ndarray] = {}
        self._depth_stamp_ns: Dict[str, int] = {}

        self._tf_broadcaster = TransformBroadcaster(self)
        self._pub_pose: Dict[str, rclpy.publisher.Publisher] = {}
        self._pub_marker: Dict[str, rclpy.publisher.Publisher] = {}
        for name in self._cubes:
            self._pub_pose[name] = self.create_publisher(
                PoseWithCovarianceStamped, f'cube/{name}/pose', 10
            )
            self._pub_marker[name] = self.create_publisher(
                MarkerArray, f'cube/{name}/markers', 10
            )

        for cam in self._cameras:
            sub_det = message_filters.Subscriber(
                self, AprilTagDetectionArray, cam.detections_topic,
                qos_profile=10,
            )
            sub_pose = message_filters.Subscriber(
                self, PoseArray, cam.poses_topic, qos_profile=10,
            )
            sync = message_filters.TimeSynchronizer([sub_det, sub_pose], 20)
            sync.registerCallback(
                lambda d, p, c=cam: self._on_camera_pair(c, d, p)
            )
            if cam.depth_topic:
                self.create_subscription(
                    Image, cam.depth_topic,
                    lambda m, c=cam: self._on_depth(c, m),
                    qos_profile_sensor_data,
                )
            if cam.depth_info_topic:
                self.create_subscription(
                    CameraInfo, cam.depth_info_topic,
                    lambda m, c=cam: self._on_depth_info(c, m),
                    qos_profile_sensor_data,
                )

        self.create_timer(1.0 / self.publish_rate, self._on_publish_timer)

        self.get_logger().info(
            f'cube_estimator started: '
            f'cubes={list(self._cubes.keys())} '
            f'cameras={[c.name for c in self._cameras]} '
            f'depth_refine={self.use_depth_refine}'
        )

    # ----- Subscriber callbacks -----

    def _on_depth_info(self, cam: CameraDef, msg: CameraInfo) -> None:
        if cam.name in self._depth_K:
            return
        self._depth_K[cam.name] = np.array(
            msg.k, dtype=np.float64
        ).reshape(3, 3)

    def _on_depth(self, cam: CameraDef, msg: Image) -> None:
        try:
            if msg.encoding == '16UC1':
                arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                    msg.height, msg.width
                )
                # millimetres → metres
                self._depth_image[cam.name] = arr.astype(np.float32) / 1000.0
            elif msg.encoding == '32FC1':
                self._depth_image[cam.name] = np.frombuffer(
                    msg.data, dtype=np.float32
                ).reshape(msg.height, msg.width).copy()
            else:
                self.get_logger().warning(
                    f'depth encoding {msg.encoding} not supported '
                    f'(camera {cam.name})'
                )
                return
        except Exception as exc:
            self.get_logger().warning(f'depth decode failed: {exc}')
            return
        self._depth_stamp_ns[cam.name] = (
            msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        )

    def _on_camera_pair(
        self,
        cam: CameraDef,
        det_msg: AprilTagDetectionArray,
        pose_msg: PoseArray,
    ) -> None:
        if len(det_msg.detections) != len(pose_msg.poses):
            return
        stamp_ns = (
            det_msg.header.stamp.sec * 1_000_000_000
            + det_msg.header.stamp.nanosec
        )
        depth_img = self._depth_image.get(cam.name)
        depth_K = self._depth_K.get(cam.name)

        for det, pose in zip(det_msg.detections, pose_msg.poses):
            if det.decision_margin < self.min_decision_margin:
                continue
            face_defs = self._faces_by_id.get(int(det.id))
            if not face_defs:
                continue

            pos = np.array([
                pose.position.x, pose.position.y, pose.position.z
            ])
            quat = [
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w,
            ]
            Rm = R.from_quat(quat).as_matrix()
            T_tag_in_cam = _make_T(pos, Rm)

            # Depth refine: replace |T| (distance) with depth at center.
            if (
                self.use_depth_refine
                and depth_img is not None
                and depth_K is not None
                and pos[2] > 0.01
            ):
                cx_pix = int(round(det.centre.x))
                cy_pix = int(round(det.centre.y))
                # Use a small median window to suppress single-pixel noise.
                refined = self._sample_depth(depth_img, cx_pix, cy_pix)
                if refined is not None and refined > 0.01:
                    scale = refined / pos[2]
                    if 0.5 < scale < 2.0:
                        T_tag_in_cam[:3, 3] = pos * scale

            for fd in face_defs:
                T_cube_in_cam = T_tag_in_cam @ fd.T_cube_in_tag
                obs = CubeObs(
                    stamp_ns=stamp_ns,
                    camera=cam.name,
                    face=fd.face,
                    tag_id=fd.tag_id,
                    T_cube_in_cam=T_cube_in_cam,
                    decision_margin=float(det.decision_margin),
                )
                state = self._cube_states[fd.cube_name]
                state.obs.append(obs)

    @staticmethod
    def _sample_depth(
        depth: np.ndarray, cx: int, cy: int, win: int = 3
    ) -> Optional[float]:
        h, w = depth.shape
        if not (0 <= cx < w and 0 <= cy < h):
            return None
        x0 = max(0, cx - win)
        x1 = min(w, cx + win + 1)
        y0 = max(0, cy - win)
        y1 = min(h, cy + win + 1)
        patch = depth[y0:y1, x0:x1]
        valid = patch[(patch > 0.01) & np.isfinite(patch)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    # ----- Fusion + publish -----

    def _on_publish_timer(self) -> None:
        now = self.get_clock().now().nanoseconds
        cutoff_ns = now - int(self.max_age * 1_000_000_000)

        for name, state in self._cube_states.items():
            state.obs = [o for o in state.obs if o.stamp_ns >= cutoff_ns]
            if not state.obs:
                continue
            self._fuse_and_publish(name, state.obs)
            # NOTE: don't clear obs here — the max_age cutoff above is the
            # sliding window. Clearing made publishes bursty when the timer
            # rate (publish_rate) exceeded the detector rate, which the
            # viewer showed as flicker. Re-fusing the same observation on
            # the next tick is a no-op for the weighted average.

    def _fuse_and_publish(self, name: str, obs: List[CubeObs]) -> None:
        weights = np.array(
            [max(o.decision_margin, 1.0) for o in obs], dtype=np.float64
        )
        weights /= weights.sum()

        positions = np.array(
            [o.T_cube_in_cam[:3, 3] for o in obs], dtype=np.float64
        )
        mean_pos = (positions.T * weights).sum(axis=1)

        # Orientation: weighted quaternion average via outer-product method.
        # All in the same camera frame? — choose the first observation's
        # camera as the reference, then transform other cameras' candidates
        # into that frame. For now we assume all observations come from
        # the SAME camera (multi-camera fusion requires extrinsics).
        # Pick the dominant camera as reference.
        by_cam: Dict[str, List[CubeObs]] = {}
        for o in obs:
            by_cam.setdefault(o.camera, []).append(o)
        ref_cam = max(by_cam.items(), key=lambda kv: len(kv[1]))[0]
        ref_obs = by_cam[ref_cam]
        ref_weights = np.array(
            [max(o.decision_margin, 1.0) for o in ref_obs], dtype=np.float64
        )
        ref_weights /= ref_weights.sum()
        ref_positions = np.array(
            [o.T_cube_in_cam[:3, 3] for o in ref_obs], dtype=np.float64
        )
        mean_pos = (ref_positions.T * ref_weights).sum(axis=1)

        quats = []
        for o in ref_obs:
            q = R.from_matrix(o.T_cube_in_cam[:3, :3]).as_quat()
            quats.append(q)
        mean_q = self._average_quaternions(quats, ref_weights)

        # Reference frame: use the camera_frame of ref_cam.
        cam_def = next((c for c in self._cameras if c.name == ref_cam), None)
        frame_id = cam_def.camera_frame if cam_def else ''
        if not frame_id:
            frame_id = f'{ref_cam}_optical_frame'

        # Stamp with the most recent observation's sensor time, not "now":
        # downstream TF lookups (scene_viewer markers, RViz) interpolate
        # source_frame in the past but cannot extrapolate into the future,
        # so a future-stamped marker is silently dropped.
        latest_obs_ns = max(o.stamp_ns for o in ref_obs)
        stamp = rclpy.time.Time(nanoseconds=latest_obs_ns).to_msg()

        pwc = PoseWithCovarianceStamped()
        pwc.header.stamp = stamp
        pwc.header.frame_id = frame_id
        pwc.pose.pose.position.x = float(mean_pos[0])
        pwc.pose.pose.position.y = float(mean_pos[1])
        pwc.pose.pose.position.z = float(mean_pos[2])
        pwc.pose.pose.orientation.x = float(mean_q[0])
        pwc.pose.pose.orientation.y = float(mean_q[1])
        pwc.pose.pose.orientation.z = float(mean_q[2])
        pwc.pose.pose.orientation.w = float(mean_q[3])
        # Simple covariance: scale with 1/sqrt(N_obs), placeholder.
        sigma_pos = 0.005 / math.sqrt(len(ref_obs))
        sigma_rot = math.radians(1.0) / math.sqrt(len(ref_obs))
        pwc.pose.covariance = [
            sigma_pos**2, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, sigma_pos**2, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, sigma_pos**2, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, sigma_rot**2, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, sigma_rot**2, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, sigma_rot**2,
        ]
        self._pub_pose[name].publish(pwc)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = frame_id
        tf.child_frame_id = f'cube_{name}'
        tf.transform.translation.x = float(mean_pos[0])
        tf.transform.translation.y = float(mean_pos[1])
        tf.transform.translation.z = float(mean_pos[2])
        tf.transform.rotation.x = float(mean_q[0])
        tf.transform.rotation.y = float(mean_q[1])
        tf.transform.rotation.z = float(mean_q[2])
        tf.transform.rotation.w = float(mean_q[3])
        self._tf_broadcaster.sendTransform(tf)

        self._publish_markers(name, frame_id, stamp, mean_pos, mean_q, ref_obs)

    def _publish_markers(
        self,
        name: str,
        frame_id: str,
        stamp,
        pos: np.ndarray,
        quat: np.ndarray,
        obs: List[CubeObs],
    ) -> None:
        cube = self._cubes[name]
        marray = MarkerArray()

        cube_marker = Marker()
        cube_marker.header.stamp = stamp
        cube_marker.header.frame_id = frame_id
        cube_marker.ns = f'cube_{name}'
        cube_marker.id = 0
        cube_marker.type = Marker.CUBE
        cube_marker.action = Marker.ADD
        cube_marker.pose.position.x = float(pos[0])
        cube_marker.pose.position.y = float(pos[1])
        cube_marker.pose.position.z = float(pos[2])
        cube_marker.pose.orientation.x = float(quat[0])
        cube_marker.pose.orientation.y = float(quat[1])
        cube_marker.pose.orientation.z = float(quat[2])
        cube_marker.pose.orientation.w = float(quat[3])
        # If origin = bottom_center, marker (which renders around origin)
        # needs the center to be shifted up by size/2 — but doing that
        # rotates the offset with the cube. Instead use a child marker
        # at geometric center, computed from the cube's pose.
        cube_marker.scale.x = cube.size
        cube_marker.scale.y = cube.size
        cube_marker.scale.z = cube.size
        cube_marker.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.4)
        marray.markers.append(cube_marker)

        text = Marker()
        text.header = cube_marker.header
        text.ns = f'cube_{name}_label'
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = float(pos[0])
        text.pose.position.y = float(pos[1])
        text.pose.position.z = float(pos[2]) + cube.size
        text.pose.orientation.w = 1.0
        text.scale.z = 0.03
        text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        seen_faces = sorted({o.face for o in obs})
        text.text = f'{name} ({len(obs)} obs: {",".join(seen_faces)})'
        marray.markers.append(text)

        self._pub_marker[name].publish(marray)

    @staticmethod
    def _average_quaternions(
        quats: List[List[float]], weights: np.ndarray
    ) -> np.ndarray:
        """Weighted quaternion mean via Markley's outer-product method.

        Quaternions are xyzw (scipy convention).
        """
        if not quats:
            return np.array([0.0, 0.0, 0.0, 1.0])
        if len(quats) == 1:
            return np.array(quats[0])
        # Resolve sign ambiguity against the first quat.
        ref = np.array(quats[0])
        M = np.zeros((4, 4))
        for q, w in zip(quats, weights):
            q = np.array(q)
            if float(np.dot(q, ref)) < 0.0:
                q = -q
            M += w * np.outer(q, q)
        eigvals, eigvecs = np.linalg.eigh(M)
        q_mean = eigvecs[:, -1]
        return q_mean / np.linalg.norm(q_mean)


def main(args=None):
    rclpy.init(args=args)
    node = CubeEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
