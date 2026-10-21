#!/usr/bin/env python3
from __future__ import annotations

import base64
import json
import math
import threading
import time
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET
from collections import deque
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np
import yaml
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image, JointState
from std_msgs.msg import Bool, ColorRGBA, Float32MultiArray, String
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

try:
    from vlabor_msgs.srv import LoadVlaProfile
    HAS_LOAD_VLA_SRV = True
except ImportError:
    HAS_LOAD_VLA_SRV = False


@dataclass
class ImageSlot:
    name: str
    topic: str
    image: Optional[np.ndarray] = None
    # Original compressed frame, forwarded verbatim (no decode/re-encode) to
    # match the DPEX inference_bridge passthrough.
    raw: Optional[bytes] = None
    raw_codec: str = "jpg"
    stamp_sec: float = 0.0
    # Transport decided at configuration time so the subscription can be
    # torn down while idle and rebuilt identically on the next trigger.
    compressed: bool = False


@dataclass
class UrdfJoint:
    name: str
    joint_type: str
    parent: str
    child: str
    xyz: np.ndarray
    rpy: np.ndarray
    axis: np.ndarray
    lower: Optional[float] = None
    upper: Optional[float] = None


class SimpleUrdfFk:
    def __init__(self, urdf_xml: str, base_link: str, tip_link: str) -> None:
        self.base_link = base_link
        self.tip_link = tip_link
        self._joints_by_child = self._parse_joints(urdf_xml)
        self.chain = self._build_chain(base_link, tip_link)

    def _parse_joints(self, urdf_xml: str) -> dict[str, UrdfJoint]:
        root = ET.fromstring(urdf_xml)
        joints: dict[str, UrdfJoint] = {}
        for elem in root.findall("joint"):
            name = elem.attrib.get("name", "")
            joint_type = elem.attrib.get("type", "fixed")
            parent_elem = elem.find("parent")
            child_elem = elem.find("child")
            if parent_elem is None or child_elem is None:
                continue
            origin_elem = elem.find("origin")
            axis_elem = elem.find("axis")
            limit_elem = elem.find("limit")
            xyz = self._parse_vector(origin_elem.attrib.get("xyz", "0 0 0") if origin_elem is not None else "0 0 0")
            rpy = self._parse_vector(origin_elem.attrib.get("rpy", "0 0 0") if origin_elem is not None else "0 0 0")
            axis = self._parse_vector(axis_elem.attrib.get("xyz", "0 0 1") if axis_elem is not None else "0 0 1")
            lower = self._parse_optional_float(limit_elem.attrib.get("lower")) if limit_elem is not None else None
            upper = self._parse_optional_float(limit_elem.attrib.get("upper")) if limit_elem is not None else None
            joints[child_elem.attrib["link"]] = UrdfJoint(
                name=name,
                joint_type=joint_type,
                parent=parent_elem.attrib["link"],
                child=child_elem.attrib["link"],
                xyz=xyz,
                rpy=rpy,
                axis=axis,
                lower=lower,
                upper=upper,
            )
        return joints

    @staticmethod
    def _parse_optional_float(value: Optional[str]) -> Optional[float]:
        if value is None or value == "":
            return None
        try:
            return float(value)
        except ValueError:
            return None

    @staticmethod
    def _parse_vector(value: str) -> np.ndarray:
        parts = [float(x) for x in value.replace(",", " ").split()]
        while len(parts) < 3:
            parts.append(0.0)
        return np.asarray(parts[:3], dtype=np.float64)

    def _build_chain(self, base_link: str, tip_link: str) -> list[UrdfJoint]:
        chain: list[UrdfJoint] = []
        link = tip_link
        while link != base_link:
            joint = self._joints_by_child.get(link)
            if joint is None:
                raise ValueError(f"URDF chain not found from {base_link} to {tip_link}: missing parent joint for {link}")
            chain.append(joint)
            link = joint.parent
        chain.reverse()
        return chain

    @staticmethod
    def _translation_matrix(xyz: np.ndarray) -> np.ndarray:
        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, 3] = xyz
        return matrix

    @staticmethod
    def _rotation_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
        norm = float(np.linalg.norm(axis))
        if norm <= 1e-9:
            return np.eye(4, dtype=np.float64)
        x, y, z = axis / norm
        c = math.cos(angle)
        s = math.sin(angle)
        t = 1.0 - c
        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = np.asarray(
            [
                [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
                [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
                [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
            ],
            dtype=np.float64,
        )
        return matrix

    @classmethod
    def _rpy_matrix(cls, rpy: np.ndarray) -> np.ndarray:
        roll, pitch, yaw = [float(x) for x in rpy]
        return (
            cls._rotation_matrix(np.asarray([0.0, 0.0, 1.0]), yaw)
            @ cls._rotation_matrix(np.asarray([0.0, 1.0, 0.0]), pitch)
            @ cls._rotation_matrix(np.asarray([1.0, 0.0, 0.0]), roll)
        )

    def position(self, joint_positions: dict[str, float]) -> np.ndarray:
        transform = np.eye(4, dtype=np.float64)
        for joint in self.chain:
            transform = transform @ self._translation_matrix(joint.xyz) @ self._rpy_matrix(joint.rpy)
            if joint.joint_type in {"revolute", "continuous"}:
                position = self._joint_position(joint, joint_positions)
                transform = transform @ self._rotation_matrix(joint.axis, position)
            elif joint.joint_type == "prismatic":
                position = self._joint_position(joint, joint_positions)
                transform = transform @ self._translation_matrix(joint.axis * position)
        return transform[:3, 3].copy()

    @staticmethod
    def _joint_position(joint: UrdfJoint, joint_positions: dict[str, float]) -> float:
        position = float(joint_positions.get(joint.name, 0.0))
        if not math.isfinite(position):
            return 0.0
        if joint.joint_type == "continuous":
            return position
        if joint.lower is not None:
            position = max(joint.lower, position)
        if joint.upper is not None:
            position = min(joint.upper, position)
        return position


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class FvPolicyRunnerNode(Node):
    def __init__(self) -> None:
        super().__init__("fv_policy_runner")
        self.bridge = CvBridge()
        self._lock = threading.Lock()
        self._executor = ThreadPoolExecutor(max_workers=1)

        self.declare_parameter("image_topics", [""])
        self.declare_parameter("image_names", [""])
        # When no images are configured, fall back to the ALOHA 3-camera layout.
        # Set False for state-only policies (no vision) so the runner stays image-free.
        self.declare_parameter("default_aloha_cameras", True)
        self.declare_parameter("state_topic", "~/state/joint_states")
        self.declare_parameter("trigger_topic", "~/trigger/start")
        self.declare_parameter("stop_topic", "~/trigger/stop")
        self.declare_parameter("policy_select_topic", "~/policy/select")
        self.declare_parameter("prompt_topic", "~/policy/prompt")
        self.declare_parameter("action_topic", "policy/action")
        self.declare_parameter("joint_action_topic", "policy/joint_action")
        self.declare_parameter("status_topic", "policy/status")
        self.declare_parameter("trajectory_topic", "policy/trajectory_markers")
        # Live per-policy trajectory control (dashboard / MCP): toggle drawing
        # on/off and override the colour without a relaunch.
        self.declare_parameter("trajectory_enable_topic", "~/trajectory/enable")
        self.declare_parameter("trajectory_color_topic", "~/trajectory/color")
        self.declare_parameter("action_joint_names", [""])
        self.declare_parameter("profile_path", "")
        self.declare_parameter("profile_arm_namespace", "")

        self.declare_parameter("backend_endpoint", "http://127.0.0.1:8000/infer")
        self.declare_parameter("backend_timeout_sec", 30.0)
        self.declare_parameter("observation_format", "aloha")
        self.declare_parameter("default_policy_id", "pi0")
        self.declare_parameter("compare_policy_ids", [""])
        self.declare_parameter("default_prompt", "do something")
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("replenish_threshold_steps", 5)
        self.declare_parameter("replace_action_queue_on_infer", False)
        self.declare_parameter("observation_max_age_sec", 0.5)
        self.declare_parameter("image_transport_encoding", "rgb8")
        self.declare_parameter("image_topics_are_compressed", False)
        self.declare_parameter("image_codec", "jpg")
        self.declare_parameter("image_quality", 90)
        self.declare_parameter("auto_start", False)
        self.declare_parameter("trajectory_publish", True)
        self.declare_parameter("trajectory_include_current_tip", True)
        self.declare_parameter("trajectory_anchor_to_current_tip", True)
        self.declare_parameter("trajectory_anchor_mode", "joint")
        self.declare_parameter("trajectory_mode", "action_xyz")
        self.declare_parameter("trajectory_frame_id", "base_link")
        self.declare_parameter("trajectory_action_indices", [0, 1, 2])
        self.declare_parameter("trajectory_scale", 1.0)
        self.declare_parameter("trajectory_z_offset", 0.0)
        self.declare_parameter("trajectory_offset_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("trajectory_line_width", 0.02)
        self.declare_parameter("trajectory_label_height", 0.08)
        self.declare_parameter("trajectory_lifetime_sec", 2.0)
        self.declare_parameter("trajectory_refresh_interval_sec", 0.0)
        # Operator-chosen trajectory colour (hex, e.g. "#00d9ff"). Empty → the
        # positional palette (legacy behaviour). Stable per policy when set.
        self.declare_parameter("trajectory_color", "")
        self.declare_parameter("policy_angle_unit", "rad")
        self.declare_parameter("policy_angle_joint_names", ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"])
        self.declare_parameter("policy_gripper_state_override", False)
        self.declare_parameter("policy_gripper_joint_name", "gripper")
        self.declare_parameter("policy_gripper_state_value", 5.6)
        self.declare_parameter("policy_gripper_output_scale", 1.0)
        # Live observation extras for state-based RL policies (off by default so
        # joint/VLA policies are unaffected). send_joint_velocities adds qvel to the
        # request; send_object_pose subscribes an external object pose (PoseStamped on
        # object_pose_topic) and adds it — required by policies whose obs needs it.
        self.declare_parameter("send_joint_velocities", False)
        self.declare_parameter("send_object_pose", False)
        self.declare_parameter("object_pose_topic", "")
        self.declare_parameter("object_pose_max_age_sec", 0.5)
        self.declare_parameter("trajectory_fk_urdf_topic", "")
        self.declare_parameter("trajectory_fk_urdf_topic_timeout_sec", 3.0)
        self.declare_parameter("trajectory_fk_urdf_path", "")
        self.declare_parameter("trajectory_fk_base_link", "base_link")
        self.declare_parameter("trajectory_fk_tip_link", "gripper_tip")
        self.declare_parameter("trajectory_fk_joint_names", ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"])

        self.default_aloha_cameras = bool(self.get_parameter("default_aloha_cameras").value)
        image_topics = [str(x).strip() for x in self.get_parameter("image_topics").value if str(x).strip()]
        image_names = [str(x).strip() for x in self.get_parameter("image_names").value if str(x).strip()]
        # When no images are configured, default to the ALOHA 3-camera layout for
        # back-compat — UNLESS default_aloha_cameras is False, which leaves the runner
        # image-free for state-only policies (e.g. state-based RL: joint+object pose,
        # no vision). Without this, a state-only policy would block on phantom camera
        # slots that never receive frames.
        if not image_topics and self.default_aloha_cameras:
            image_topics = ["~/image/cam_high", "~/image/cam_left_wrist", "~/image/cam_right_wrist"]
        if not image_names and self.default_aloha_cameras:
            image_names = ["cam_high", "cam_left_wrist", "cam_right_wrist"]
        if len(image_topics) != len(image_names):
            raise ValueError("image_topics and image_names must have the same length")

        self.profile_path = str(self.get_parameter("profile_path").value)
        self.profile_arm_namespace = str(self.get_parameter("profile_arm_namespace").value)
        profile_cfg = self._load_profile_config()

        self.state_topic = str(self.get_parameter("state_topic").value)
        self.trigger_topic = str(self.get_parameter("trigger_topic").value)
        self.stop_topic = str(self.get_parameter("stop_topic").value)
        self.policy_select_topic = str(self.get_parameter("policy_select_topic").value)
        self.prompt_topic = str(self.get_parameter("prompt_topic").value)
        self.action_topic = str(self.get_parameter("action_topic").value)
        self.joint_action_topic = str(self.get_parameter("joint_action_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.trajectory_topic = str(self.get_parameter("trajectory_topic").value)
        self.trajectory_enable_topic = str(self.get_parameter("trajectory_enable_topic").value)
        self.trajectory_color_topic = str(self.get_parameter("trajectory_color_topic").value)
        self.action_joint_names = [
            str(x).strip()
            for x in self.get_parameter("action_joint_names").value
            if str(x).strip()
        ]
        profile_follower = self._profile_follower(profile_cfg)
        self._apply_profile_topic_defaults(profile_follower)

        self.backend_endpoint = str(self.get_parameter("backend_endpoint").value)
        self.backend_timeout_sec = float(self.get_parameter("backend_timeout_sec").value)
        self.observation_format = str(self.get_parameter("observation_format").value).strip().lower()
        self.default_policy_id = str(self.get_parameter("default_policy_id").value).strip()
        self.compare_policy_ids = [
            str(x).strip()
            for x in self.get_parameter("compare_policy_ids").value
            if str(x).strip()
        ]
        self.default_prompt = str(self.get_parameter("default_prompt").value)
        self.publish_hz = max(0.1, float(self.get_parameter("publish_hz").value))
        self.replenish_threshold_steps = max(1, int(self.get_parameter("replenish_threshold_steps").value))
        self.replace_action_queue_on_infer = bool(self.get_parameter("replace_action_queue_on_infer").value)
        self.observation_max_age_sec = max(0.01, float(self.get_parameter("observation_max_age_sec").value))
        self.send_joint_velocities = bool(self.get_parameter("send_joint_velocities").value)
        self.send_object_pose = bool(self.get_parameter("send_object_pose").value)
        self.object_pose_topic = str(self.get_parameter("object_pose_topic").value).strip()
        self.object_pose_max_age_sec = max(0.0, float(self.get_parameter("object_pose_max_age_sec").value))
        self.image_transport_encoding = str(self.get_parameter("image_transport_encoding").value)
        self.image_topics_are_compressed = bool(self.get_parameter("image_topics_are_compressed").value)
        self.image_codec = str(self.get_parameter("image_codec").value).strip().lower()
        self.image_quality = max(1, min(100, int(self.get_parameter("image_quality").value)))
        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.trajectory_publish = bool(self.get_parameter("trajectory_publish").value)
        self._traj_color = self._parse_hex_color(
            str(self.get_parameter("trajectory_color").value))
        self.trajectory_include_current_tip = bool(self.get_parameter("trajectory_include_current_tip").value)
        self.trajectory_anchor_to_current_tip = bool(self.get_parameter("trajectory_anchor_to_current_tip").value)
        self.trajectory_anchor_mode = str(self.get_parameter("trajectory_anchor_mode").value).strip().lower()
        if self.trajectory_anchor_mode not in {"joint", "cartesian"}:
            self.get_logger().warning(f"unsupported trajectory_anchor_mode={self.trajectory_anchor_mode}; using joint")
            self.trajectory_anchor_mode = "joint"
        self.trajectory_mode = str(self.get_parameter("trajectory_mode").value).strip().lower()
        self.trajectory_frame_id = str(self.get_parameter("trajectory_frame_id").value)
        self.trajectory_action_indices = [
            int(x) for x in self.get_parameter("trajectory_action_indices").value
        ]
        if not self.trajectory_action_indices:
            self.trajectory_action_indices = [0, 1, 2]
        while len(self.trajectory_action_indices) < 3:
            self.trajectory_action_indices.append(self.trajectory_action_indices[-1])
        self.trajectory_scale = float(self.get_parameter("trajectory_scale").value)
        self.trajectory_z_offset = float(self.get_parameter("trajectory_z_offset").value)
        self.trajectory_offset_xyz = np.asarray(
            [float(x) for x in self.get_parameter("trajectory_offset_xyz").value],
            dtype=np.float64,
        )
        if self.trajectory_offset_xyz.size < 3:
            self.trajectory_offset_xyz = np.pad(self.trajectory_offset_xyz, (0, 3 - self.trajectory_offset_xyz.size))
        self.trajectory_offset_xyz = self.trajectory_offset_xyz[:3]
        self.trajectory_line_width = max(0.001, float(self.get_parameter("trajectory_line_width").value))
        self.trajectory_label_height = max(0.001, float(self.get_parameter("trajectory_label_height").value))
        self.trajectory_lifetime_sec = max(0.0, float(self.get_parameter("trajectory_lifetime_sec").value))
        self.trajectory_refresh_interval_sec = max(
            0.0, float(self.get_parameter("trajectory_refresh_interval_sec").value)
        )
        self.policy_angle_unit = str(self.get_parameter("policy_angle_unit").value).strip().lower()
        if self.policy_angle_unit not in {"rad", "radian", "radians", "deg", "degree", "degrees"}:
            self.get_logger().warning(f"unsupported policy_angle_unit={self.policy_angle_unit}; using rad")
            self.policy_angle_unit = "rad"
        self.policy_angle_joint_names = {
            str(x).strip()
            for x in self.get_parameter("policy_angle_joint_names").value
            if str(x).strip()
        }
        self.policy_gripper_state_override = bool(self.get_parameter("policy_gripper_state_override").value)
        self.policy_gripper_joint_name = str(self.get_parameter("policy_gripper_joint_name").value).strip()
        self.policy_gripper_state_value = float(self.get_parameter("policy_gripper_state_value").value)
        self.policy_gripper_output_scale = float(self.get_parameter("policy_gripper_output_scale").value)
        self.trajectory_fk_urdf_topic = str(self.get_parameter("trajectory_fk_urdf_topic").value)
        self.trajectory_fk_urdf_topic_timeout_sec = max(
            0.1,
            float(self.get_parameter("trajectory_fk_urdf_topic_timeout_sec").value),
        )
        self.trajectory_fk_urdf_path = str(self.get_parameter("trajectory_fk_urdf_path").value)
        self.trajectory_fk_base_link = str(self.get_parameter("trajectory_fk_base_link").value)
        self.trajectory_fk_tip_link = str(self.get_parameter("trajectory_fk_tip_link").value)
        self.trajectory_fk_joint_names = [
            str(x).strip()
            for x in self.get_parameter("trajectory_fk_joint_names").value
            if str(x).strip()
        ]
        self._apply_profile_joint_defaults(profile_follower)
        self._fk_solver: Optional[SimpleUrdfFk] = None
        if self.trajectory_mode == "fk":
            self._fk_solver = self._load_fk_solver()

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._image_slots = [ImageSlot(name=name, topic=topic) for name, topic in zip(image_names, image_topics)]
        self._image_subs = []
        self._sensor_qos = sensor_qos
        for slot in self._image_slots:
            slot.compressed = bool(
                self.image_topics_are_compressed
                or slot.topic.endswith("/compressed"))
        # Camera subscriptions exist only while running (idle backoff):
        # rclpy deserialises every incoming frame BEFORE the callback can
        # early-return, so an idle runner with live image subs still paid
        # for ~900KB x 30Hz per camera - measured ~20% of a Thor core per
        # runner, and every registered DPEX policy keeps one runner up.
        # _on_trigger creates them; _on_stop tears them down.
        if self.auto_start:
            self._create_obs_subs()

        # state_topic may be empty when the profile defers wiring to
        # /load_profile. Skip the sub creation here and let the service
        # handler build it later.
        # Deferred to _create_obs_subs (idle backoff) together with the
        # camera subs - ~50Hz joint_states wakeups cost real CPU in rclpy
        # even when the callback does almost nothing.
        self._state_sub = None
        self._trigger_sub = self.create_subscription(Bool, self.trigger_topic, self._on_trigger, 10)
        self._stop_sub = self.create_subscription(Bool, self.stop_topic, self._on_stop, 10)
        self._policy_sub = self.create_subscription(String, self.policy_select_topic, self._on_policy_select, 10)
        self._prompt_sub = self.create_subscription(String, self.prompt_topic, self._on_prompt, 10)
        self._traj_enable_sub = self.create_subscription(
            Bool, self.trajectory_enable_topic, self._on_trajectory_enable, 10)
        self._traj_color_sub = self.create_subscription(
            String, self.trajectory_color_topic, self._on_trajectory_color, 10)
        # Optional external object pose (PoseStamped) for state-based RL policies whose
        # observation includes an object pose (e.g. cube pick). Off unless a profile sets
        # send_object_pose + object_pose_topic. The M1 sim bridge publishes the MuJoCo
        # cube ground-truth here; M2 swaps in an fv_3d_detector bridge — same contract.
        self._object_pose_sub = None
        if self.send_object_pose and self.object_pose_topic:
            self._object_pose_sub = self.create_subscription(
                PoseStamped,
                self.object_pose_topic,
                self._on_object_pose,
                QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                ),
            )

        self._action_pub = self.create_publisher(Float32MultiArray, self.action_topic, 10)
        self._joint_action_pub = self.create_publisher(JointState, self.joint_action_topic, 10)
        self._status_pub = self.create_publisher(String, self.status_topic, 10)
        self._trajectory_pub = self.create_publisher(MarkerArray, self.trajectory_topic, 10)

        self._state_vector: Optional[np.ndarray] = None
        self._state_names: list[str] = []
        self._state_stamp_sec = 0.0
        self._state_velocity: Optional[np.ndarray] = None
        self._object_pose: Optional[np.ndarray] = None  # [x,y,z, qw,qx,qy,qz]
        self._object_pose_stamp_sec = 0.0

        self._policy_id = self.default_policy_id
        self._prompt = self.default_prompt
        self._running = self.auto_start
        self._last_trigger = False
        self._queue: deque[np.ndarray] = deque()
        # RTC 用: policy 単位のままの生アクション (ROS 単位へ変換する前)。
        # gateway へ「まだ再生していない前チャンクの残り」として渡すため、
        # _queue と常に同じ長さで前後させる。
        self._queue_raw: deque[list[float]] = deque()
        # RTC: 推論 submit 時点のキュー残数。到着時との差 = 推論中に消費した
        # ステップ数で、新チャンクの先頭の同数ステップは「もう過ぎた時刻」の
        # 分なので捨てる (これが無いと毎リフィルで ~delay 本を巻き戻し再生する)。
        self._infer_submit_queue_len = 0
        # shim から受け取る RTC 情報 (0 = 未較正/RTC無効 = 従来の枯渇時推論)
        self._rtc_refill_threshold: int = 0
        self._rtc_mode: str = ""
        self._rtc_phase: str = ""
        self._infer_future: Optional[Future] = None
        self._next_trajectory_refresh_time = 0.0
        self._last_error = ""
        self._last_infer_ms = 0.0
        self._last_compare_ms = 0.0
        self._last_infer_wall_ms = 0.0
        self._last_trajectory_models: list[str] = []
        self._last_policy_infer_ms: dict[str, float] = {}

        # Live profile swap: destroys + rebuilds image/state subs and updates
        # backend URL / model / prompt in one atomic call. Dashboard and MCP
        # call this after resolving a policy profile against the registry.
        # (NOTE: srv name `LoadVlaProfile` is the legacy-named contract; the
        # mux source `vla` was unified into `policy` but the srv message
        # name is kept for one release of compat. To be renamed alongside
        # the dashboard `_vla_*` python identifiers in a follow-up PR.)
        self._sensor_qos = sensor_qos
        if HAS_LOAD_VLA_SRV:
            self.create_service(
                LoadVlaProfile, "~/load_profile", self._on_load_vla_profile)

        self._timer = self.create_timer(1.0 / self.publish_hz, self._on_tick)
        self._publish_status("startup")
        self.get_logger().info(
            f"fv_policy_runner started backend={self.backend_endpoint} "
            f"policy={self._policy_id} format={self.observation_format}"
        )

    # ------------------------------------------------------------------
    # Live profile reconfiguration
    # ------------------------------------------------------------------

    def _create_obs_subs(self) -> None:
        """(Re)create the observation subscriptions (cameras + joint
        state) for the current configuration. Idempotent."""
        if not self._image_subs:
            for slot in self._image_slots:
                if slot.compressed:
                    self._image_subs.append(
                        self.create_subscription(
                            CompressedImage, slot.topic,
                            lambda msg, current=slot: self._on_compressed_image(current, msg),
                            self._sensor_qos,
                        )
                    )
                else:
                    self._image_subs.append(
                        self.create_subscription(
                            Image, slot.topic,
                            lambda msg, current=slot: self._on_image(current, msg),
                            self._sensor_qos,
                        )
                    )
        if self._state_sub is None and self.state_topic:
            self._state_sub = self.create_subscription(
                JointState, self.state_topic, self._on_state, self._sensor_qos)

    def _drop_obs_subs(self) -> None:
        """Idle backoff: tear down camera + joint-state subscriptions and
        clear the buffered observation (it would be stale by the next
        trigger; the snapshot logic then waits for fresh data instead of
        silently reusing old frames)."""
        subs, self._image_subs = self._image_subs, []
        for sub in subs:
            try:
                self.destroy_subscription(sub)
            except Exception:
                pass
        for slot in self._image_slots:
            slot.image = None
            slot.raw = None
            slot.stamp_sec = 0.0
        if self._state_sub is not None:
            try:
                self.destroy_subscription(self._state_sub)
            except Exception:
                pass
            self._state_sub = None
        self._state_vector = None
        self._state_names = []
        self._state_stamp_sec = 0.0
        self._state_velocity = None

    def _destroy_input_subscriptions(self) -> None:
        for sub in getattr(self, "_image_subs", []) or []:
            try:
                self.destroy_subscription(sub)
            except Exception:
                pass
        self._image_subs = []
        self._image_slots = []
        state_sub = getattr(self, "_state_sub", None)
        if state_sub is not None:
            try:
                self.destroy_subscription(state_sub)
            except Exception:
                pass
        self._state_sub = None

    def _build_input_subscriptions(
        self,
        image_topics: List[str],
        image_names: List[str],
        image_transports: List[str],
        state_topic: str,
    ) -> None:
        qos = self._sensor_qos
        self._image_slots = [
            ImageSlot(name=n, topic=t) for n, t in zip(image_names, image_topics)
        ]
        self._image_subs = []
        for i, slot in enumerate(self._image_slots):
            transport = (
                image_transports[i].lower() if i < len(image_transports) and image_transports[i] else ""
            )
            slot.compressed = (
                transport == "compressed"
                or (not transport and (self.image_topics_are_compressed or slot.topic.endswith("/compressed")))
            )
        # Idle backoff: only subscribe while running; the next trigger
        # creates the subs for these (possibly new) slots.
        if self._running:
            self._create_obs_subs()
        elif state_topic:
            self._state_sub = self.create_subscription(
                JointState, state_topic, self._on_state, qos)
        # Clear buffered state + action queue — they belong to the previous
        # configuration and would be stale after an I/O swap.
        self._state_vector = None
        self._state_names = []
        self._state_stamp_sec = 0.0
        self._state_velocity = None
        with self._lock:
            self._queue.clear()
            self._queue_raw.clear()

    def _on_load_vla_profile(self, request, response):
        try:
            with self._lock:
                pass  # reserved for fairness; subscription ops need node thread
            topo_change = False
            if request.endpoint_url:
                self.backend_endpoint = str(request.endpoint_url)
            if request.endpoint_timeout_sec > 0.0:
                self.backend_timeout_sec = float(request.endpoint_timeout_sec)
            if request.model:
                self._policy_id = str(request.model)
                self.default_policy_id = self._policy_id
            if request.prompt:
                self._prompt = str(request.prompt)
                self.default_prompt = self._prompt
            if request.state_joints:
                self.state_joint_names = list(request.state_joints)
            if request.action_joints:
                self.action_joint_names = list(request.action_joints)
            new_img_topics = list(request.image_topics or [])
            new_img_names = list(request.image_names or [])
            new_img_transports = list(request.image_transports or [])
            new_state = str(request.state_topic or "")
            need_image_rebuild = bool(new_img_topics) and (
                new_img_topics != [s.topic for s in self._image_slots]
                or new_img_names != [s.name for s in self._image_slots]
            )
            need_state_rebuild = bool(new_state) and new_state != self.state_topic
            if need_image_rebuild or need_state_rebuild:
                topo_change = True
                current_names = new_img_names or [s.name for s in self._image_slots]
                current_topics = new_img_topics or [s.topic for s in self._image_slots]
                current_transports = new_img_transports or [""] * len(current_names)
                current_state = new_state or self.state_topic
                self._destroy_input_subscriptions()
                self._build_input_subscriptions(
                    current_topics, current_names, current_transports, current_state)
                self.state_topic = current_state
            if request.action_topic and request.action_topic != self.joint_action_topic:
                try:
                    self.destroy_publisher(self._joint_action_pub)
                except Exception:
                    pass
                self.joint_action_topic = str(request.action_topic)
                self._joint_action_pub = self.create_publisher(
                    JointState, self.joint_action_topic, 10)
                topo_change = True
            if request.auto_start:
                self._running = True
                self._create_obs_subs()
            response.success = True
            response.current_endpoint = self.backend_endpoint
            response.current_model = self._policy_id
            response.current_prompt = self._prompt
            response.message = "topology rebuilt" if topo_change else "config updated"
            self._publish_status("load_profile")
            self.get_logger().info(
                f"load_profile: endpoint={self.backend_endpoint} "
                f"model={self._policy_id} topo_change={topo_change}")
        except Exception as exc:
            response.success = False
            response.message = f"load_profile failed: {exc}"
            self.get_logger().error(response.message)
        return response

    def _load_profile_config(self) -> dict:
        if not self.profile_path:
            return {}
        try:
            with Path(self.profile_path).open("r", encoding="utf-8") as file:
                data = yaml.safe_load(file) or {}
            profile = data.get("profile", data)
            if isinstance(profile, dict):
                self.get_logger().info(f"loaded profile config: {self.profile_path}")
                return profile
        except Exception as exc:
            self.get_logger().warning(f"profile load failed path={self.profile_path}: {exc}")
        return {}

    def _profile_follower(self, profile: dict) -> dict:
        if not profile:
            return {}
        follower = profile.get("lerobot", {}).get("follower", {})
        if not isinstance(follower, dict):
            return {}
        namespace = str(follower.get("namespace", "")).strip()
        if self.profile_arm_namespace and namespace != self.profile_arm_namespace:
            return {}
        return follower

    def _apply_profile_topic_defaults(self, follower: dict) -> None:
        if not follower:
            return
        profile_state_topic = str(follower.get("topic", "")).strip()
        profile_action_topic = str(follower.get("action_topic", "")).strip()
        if profile_state_topic and self.state_topic == "~/state/joint_states":
            self.state_topic = profile_state_topic
        if profile_action_topic and self.joint_action_topic == "policy/joint_action":
            self.joint_action_topic = profile_action_topic
        self.get_logger().info(
            f"profile lerobot topics applied state_topic={self.state_topic} "
            f"joint_action_topic={self.joint_action_topic}"
        )

    def _apply_profile_joint_defaults(self, follower: dict) -> None:
        if not follower:
            return
        profile_joints = [str(x).strip() for x in follower.get("joints", []) if str(x).strip()]
        if profile_joints:
            if not self.action_joint_names:
                self.action_joint_names = profile_joints
            self.trajectory_fk_joint_names = profile_joints
        self.get_logger().info(
            f"profile lerobot joints applied joints={self.trajectory_fk_joint_names}"
        )

    def _on_image(self, slot: ImageSlot, msg: Image) -> None:
        # Skip decode when idle so an inactive runner doesn't burn CPU on
        # every camera frame. Trigger -> first frame after that gets stored.
        if not self._running:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.image_transport_encoding)
            stamp_sec = _stamp_to_sec(msg.header.stamp)
        except Exception as exc:
            self.get_logger().warning(f"image conversion failed topic={slot.topic} error={exc}")
            return
        with self._lock:
            slot.image = image
            slot.stamp_sec = stamp_sec

    def _on_compressed_image(self, slot: ImageSlot, msg: CompressedImage) -> None:
        # Skip work when idle (see _on_image).
        if not self._running:
            return
        # Forward the camera's original compressed frame verbatim, matching the
        # DPEX inference_bridge passthrough. Decoding then re-encoding at JPEG q90
        # double-compresses the image the policy grounds on; the gateway worker
        # decodes "jpg"/"compressed" identically, so passthrough is exact parity.
        fmt = str(msg.format or "").strip().lower()
        codec = "png" if "png" in fmt else "jpg"
        stamp_sec = _stamp_to_sec(msg.header.stamp)
        with self._lock:
            slot.raw = bytes(msg.data)
            slot.raw_codec = codec
            slot.stamp_sec = stamp_sec

    def _on_state(self, msg: JointState) -> None:
        with self._lock:
            self._state_vector = np.asarray(msg.position, dtype=np.float32)
            self._state_names = list(msg.name)
            self._state_stamp_sec = _stamp_to_sec(msg.header.stamp)
            # Velocities are paired with position/name; kept for policies that need
            # them (send_joint_velocities). None when the publisher omits velocity.
            self._state_velocity = (
                np.asarray(msg.velocity, dtype=np.float32)
                if len(msg.velocity) == len(msg.position)
                else None
            )

    def _on_object_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        with self._lock:
            # Store as [x,y,z, qw,qx,qy,qz] — serve/MuJoCo use wxyz, PoseStamped is
            # xyzw. The publisher is expected to emit in the policy's base frame
            # (M1 sim bridge / M2 fv_3d_detector bridge both republish in base_link).
            self._object_pose = np.asarray(
                [p.x, p.y, p.z, q.w, q.x, q.y, q.z], dtype=np.float32
            )
            self._object_pose_stamp_sec = _stamp_to_sec(msg.header.stamp)

    def _on_trigger(self, msg: Bool) -> None:
        # Treat any True as "start" rather than requiring a False→True
        # edge. Controllers that send idempotent triggers (dashboard,
        # MCP) shouldn't need to track the runner's previous state.
        self._last_trigger = bool(msg.data)
        if not msg.data:
            return
        with self._lock:
            was_running = self._running
            self._running = True
            self._queue.clear()
            self._queue_raw.clear()
        self._create_obs_subs()
        if not was_running:
            self._publish_status("trigger")

    def _on_stop(self, msg: Bool) -> None:
        if not msg.data:
            return
        with self._lock:
            self._running = False
            self._queue.clear()
            self._queue_raw.clear()
        self._drop_obs_subs()
        self._publish_status("stop")

    def _on_policy_select(self, msg: String) -> None:
        policy_id = msg.data.strip()
        if not policy_id:
            return
        with self._lock:
            self._policy_id = policy_id
            self._queue.clear()
            self._queue_raw.clear()
        self._publish_status("policy_select")

    def _on_prompt(self, msg: String) -> None:
        with self._lock:
            self._prompt = msg.data
        self._publish_status("prompt")

    def _on_trajectory_enable(self, msg: Bool) -> None:
        """Live on/off of the inferred-trajectory drawing (per policy)."""
        self.trajectory_publish = bool(msg.data)
        self._publish_status("trajectory_enable")

    def _on_trajectory_color(self, msg: String) -> None:
        """Live per-policy trajectory colour override (hex). Ignored if unparseable."""
        color = self._parse_hex_color(msg.data)
        if color is not None:
            self._traj_color = color

    def _snapshot_observation(self) -> Optional[dict]:
        now = time.time()
        with self._lock:
            # RTC: まだ再生していない前チャンクの残り (policy 単位)。shim 経由で
            # gateway へ渡すと、その区間を保ったまま新チャンクが合成される
            # =チャンク境界で動きが途切れない。
            previous_original_actions = [list(a) for a in self._queue_raw]
            self._infer_submit_queue_len = len(self._queue)
            if self._state_vector is None:
                self._last_error = "state not received yet"
                return None
            state_vector = self._ros_state_to_policy_state(self._state_vector, self._state_names)
            images = []
            for slot in self._image_slots:
                # Compressed-source slots forward the original frame verbatim
                # (no re-encode) to match the DPEX inference_bridge passthrough.
                if slot.raw is not None:
                    if now - slot.stamp_sec > self.observation_max_age_sec:
                        self._last_error = f"image stale: {slot.name}"
                        return None
                    images.append(
                        {
                            "name": slot.name,
                            "codec": slot.raw_codec,
                            "data": base64.b64encode(slot.raw).decode("ascii"),
                        }
                    )
                    continue
                if slot.image is None:
                    self._last_error = f"image missing: {slot.name}"
                    return None
                if now - slot.stamp_sec > self.observation_max_age_sec:
                    self._last_error = f"image stale: {slot.name}"
                    return None
                ok, encoded = cv2.imencode(
                    ".jpg" if self.image_codec == "jpg" else ".png",
                    cv2.cvtColor(slot.image, cv2.COLOR_RGB2BGR),
                    [int(cv2.IMWRITE_JPEG_QUALITY), self.image_quality] if self.image_codec == "jpg" else [],
                )
                if not ok:
                    self._last_error = f"image encode failed: {slot.name}"
                    return None
                images.append(
                    {
                        "name": slot.name,
                        "codec": self.image_codec,
                        "data": base64.b64encode(encoded.tobytes()).decode("ascii"),
                    }
                )
            snapshot = {
                "policy_id": self._policy_id,
                "observation_format": self.observation_format,
                "prompt": self._prompt,
                "state": state_vector.tolist(),
                "images": images,
                "joint_names": self._state_names,
                # RTC: まだ再生していない前チャンクの残り (policy 単位)。
                # backend (shim) が RTC 有効なら gateway がこの区間を保ったまま
                # 新チャンクを合成する。無効な backend は未知キーを無視するだけ。
                "previous_original_actions": previous_original_actions,
            }
            # Live observation extras for state-based RL policies. Velocities are
            # best-effort (the backend warns + zero-fills if absent). object_pose is
            # load-bearing for a pick, so a missing/stale pose fails closed (skip the
            # tick) rather than feeding a stale target — no silent fallback.
            if (
                self.send_joint_velocities
                and self._state_velocity is not None
                and self._state_velocity.size == state_vector.size
            ):
                snapshot["joint_velocities"] = self._state_velocity.astype(float).tolist()
            if self.send_object_pose:
                if self._object_pose is None:
                    self._last_error = "object_pose requested but not received yet"
                    return None
                age = now - self._object_pose_stamp_sec
                if self.object_pose_max_age_sec > 0.0 and age > self.object_pose_max_age_sec:
                    self._last_error = (
                        f"object_pose stale ({age:.2f}s > {self.object_pose_max_age_sec}s); "
                        "skipping inference"
                    )
                    return None
                snapshot["object_pose"] = self._object_pose.astype(float).tolist()
            return snapshot

    def _policy_uses_degrees(self) -> bool:
        return self.policy_angle_unit in {"deg", "degree", "degrees"}

    def _ros_state_to_policy_state(self, values: np.ndarray, names: list[str]) -> np.ndarray:
        state = np.asarray(values, dtype=np.float32).copy()
        if not self._policy_uses_degrees():
            return state
        for index, name in enumerate(names):
            if index >= state.size:
                break
            if name in self.policy_angle_joint_names:
                state[index] = math.degrees(float(state[index]))
            elif name == self.policy_gripper_joint_name:
                if self.policy_gripper_state_override:
                    state[index] = float(self.policy_gripper_state_value)
                elif self.policy_gripper_output_scale not in (0.0, 1.0):
                    # Gripper proprioception must be symmetric with the action
                    # scale (DPEX inference_bridge uses one per-joint factor:
                    # obs ×S, action ÷S). The arm reports the gripper in meters;
                    # the policy was trained on dataset units (≈0–3.8). Without
                    # this the model sees the gripper as ~always closed and emits
                    # a narrow, oscillating grasp that never reaches the object.
                    state[index] = float(state[index]) / self.policy_gripper_output_scale
        return state

    def _policy_action_to_ros_action(self, action: np.ndarray) -> np.ndarray:
        converted = np.asarray(action, dtype=np.float32).copy()
        if not self._policy_uses_degrees():
            return converted
        joint_names = self.action_joint_names or self.trajectory_fk_joint_names
        for index, name in enumerate(joint_names):
            if index >= converted.size:
                break
            if name in self.policy_angle_joint_names:
                converted[index] = math.radians(float(converted[index]))
        return converted

    def _submit_infer(self) -> None:
        snapshot = self._snapshot_observation()
        if snapshot is None:
            return
        self._infer_future = self._executor.submit(self._infer_request, snapshot)

    def _post_infer(self, snapshot: dict) -> tuple[list[np.ndarray], list[list[float]], float]:
        raw = json.dumps(snapshot).encode("utf-8")
        request = urllib.request.Request(
            self.backend_endpoint,
            data=raw,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=self.backend_timeout_sec) as resp:
            payload = json.loads(resp.read().decode("utf-8"))
        if not payload.get("ok", False):
            raise RuntimeError(payload.get("error", "inference failed"))
        actions = [np.asarray(x, dtype=np.float32) for x in payload.get("actions", [])]
        # ポリシー空間の生値 (shim: gateway action.original_values)。RTC の
        # previous_original_actions は**この空間でなければならない** — 後処理済みの
        # actions (度/グリッパ%) を返すと正規化空間に実単位が混入して行動が壊れる。
        original_actions = [
            [float(v) for v in row] for row in payload.get("original_actions", [])
        ]
        infer_ms = float(payload.get("timing", {}).get("infer_ms", 0.0))
        # RTC (shim が返す): refill_threshold_steps>0 なら「残りがこれ以下になったら
        # 先行推論する」。gateway が前チャンクと合成するのでチャンク境界で止まらない。
        rtc = payload.get("rtc") or {}
        if isinstance(rtc, dict) and rtc.get("refill_threshold_steps") is not None:
            try:
                self._rtc_refill_threshold = max(0, int(rtc["refill_threshold_steps"]))
                self._rtc_mode = str(rtc.get("mode") or "")
                self._rtc_phase = str(rtc.get("phase") or "")
            except (TypeError, ValueError):
                pass
        return actions, original_actions, infer_ms

    def _infer_request(
        self, snapshot: dict
    ) -> tuple[list[np.ndarray], list[list[float]], float, float, float,
               dict[str, list[np.ndarray]], dict[str, float]]:
        wall_start = time.perf_counter()
        primary_policy_id = str(snapshot.get("policy_id", "")).strip()
        requests: dict[str, dict] = {}
        if primary_policy_id:
            requests[primary_policy_id] = snapshot

        for policy_id in self.compare_policy_ids:
            if not policy_id or policy_id == primary_policy_id:
                continue
            compare_snapshot = dict(snapshot)
            compare_snapshot["policy_id"] = policy_id
            requests[policy_id] = compare_snapshot

        if not requests:
            return [], [], 0.0, 0.0, 0.0, {}, {}

        max_workers = min(len(requests), max(1, 1 + len(self.compare_policy_ids)))
        with ThreadPoolExecutor(max_workers=max_workers) as executor:
            futures = {
                policy_id: executor.submit(self._post_infer, request)
                for policy_id, request in requests.items()
            }
            results = {policy_id: future.result() for policy_id, future in futures.items()}

        trajectories = {policy_id: actions for policy_id, (actions, _, _) in results.items()}
        policy_infer_ms = {policy_id: infer_ms for policy_id, (_, _, infer_ms) in results.items()}
        actions = trajectories.get(primary_policy_id, [])
        original_actions = results.get(primary_policy_id, ([], [], 0.0))[1]
        infer_ms = policy_infer_ms.get(primary_policy_id, 0.0)
        compare_ms = sum(
            infer_ms_value
            for policy_id, infer_ms_value in policy_infer_ms.items()
            if policy_id != primary_policy_id
        )
        wall_ms = (time.perf_counter() - wall_start) * 1000.0

        return actions, original_actions, infer_ms, compare_ms, wall_ms, trajectories, policy_infer_ms

    def _consume_future(self) -> None:
        if self._infer_future is None or not self._infer_future.done():
            return
        future = self._infer_future
        self._infer_future = None
        try:
            (actions, original_actions, infer_ms, compare_ms, wall_ms,
             trajectories, policy_infer_ms) = future.result()
            # RTC の previous_original_actions は**ポリシー空間の生値** (shim の
            # original_actions = gateway action.original_values) のみ使用可。
            # 以前は shim の actions (gateway 後処理済み = 度/グリッパ%) を控えて
            # いて、正規化空間に実単位が混入し行動が桁違いに壊れた (gripper 5000% /
            # 全関節セグメント拒否で立ち往生, 2026-08-07)。original が無い
            # (旧 shim / 旧 gateway) 場合は raw を空にする = RTC ブレンド無しの
            # 素の再推論 (安全側)。actions と本数が合わない場合も同様。
            if len(original_actions) != len(actions):
                if original_actions:
                    self.get_logger().warning(
                        "original_actions/actions length mismatch "
                        f"({len(original_actions)}/{len(actions)}); "
                        "disabling RTC blend for this chunk")
                raw_actions: list[list[float]] = []
            else:
                raw_actions = original_actions
            actions = [self._policy_action_to_ros_action(action) for action in actions]
            trajectories = {
                policy_id: [self._policy_action_to_ros_action(action) for action in policy_actions]
                for policy_id, policy_actions in trajectories.items()
            }
            with self._lock:
                # RTC のみ: チャンクの時間軸は観測時点起点なので、推論中に既に
                # 再生した本数 (submit時キュー長 − 現在キュー長) を新チャンクの
                # 先頭から捨てる。捨てないと毎リフィルで ~inference_delay 本
                # (≈0.43s) を巻き戻して再生し、関節ジャンプ/戻り動作になる
                # (実証済み実装 vlabor_inference/runtime.py は tick 比較で同じ
                # ことをしている)。非RTC (refill_threshold=0) は従来挙動のまま。
                if self._rtc_refill_threshold > 0 and self.replace_action_queue_on_infer:
                    consumed = max(0, self._infer_submit_queue_len - len(self._queue))
                    if consumed >= len(actions):
                        self.get_logger().warning(
                            f"chunk fully stale ({consumed} consumed >= "
                            f"{len(actions)} actions); dropping chunk")
                        actions = []
                        raw_actions = []
                    elif consumed:
                        actions = actions[consumed:]
                        if raw_actions:
                            raw_actions = raw_actions[consumed:]
                if self.replace_action_queue_on_infer:
                    self._queue.clear()
                self._queue_raw.clear()
                for action in actions:
                    self._queue.append(action)
                # 不変条件: _queue_raw は「空」か「_queue と完全に同長・同順」のみ。
                # 途中までの同期は許さない (pop の lockstep が崩れて RTC に
                # ズレた前チャンクを渡してしまう)。append モード (replace=False) で
                # 旧チャンクが残っている場合は空のまま = RTC ブレンド無し。
                if raw_actions and len(self._queue) == len(raw_actions):
                    for raw in raw_actions:
                        self._queue_raw.append(raw)
                self._last_error = ""
                self._last_infer_ms = infer_ms
                self._last_compare_ms = compare_ms
                self._last_infer_wall_ms = wall_ms
                self._last_trajectory_models = list(trajectories.keys())
                self._last_policy_infer_ms = dict(policy_infer_ms)
            self._publish_trajectory_markers(trajectories, policy_infer_ms)
        except Exception as exc:
            with self._lock:
                self._last_error = str(exc)
            self.get_logger().warning(f"infer failed: {exc}")
        self._publish_status("infer_done")

    def _publish_action(self, action: np.ndarray) -> None:
        action_values = [float(x) for x in action.tolist()]

        if self.policy_gripper_output_scale != 1.0 and self.action_joint_names:
            name = self.policy_gripper_joint_name
            for i, n in enumerate(self.action_joint_names):
                if n == name and i < len(action_values):
                    action_values[i] = action_values[i] * self.policy_gripper_output_scale
                    break

        msg = Float32MultiArray()
        msg.data = action_values
        self._action_pub.publish(msg)

        if self.action_joint_names:
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.name = list(self.action_joint_names)
            joint_msg.position = action_values
            self._joint_action_pub.publish(joint_msg)

    def _duration_msg(self, seconds: float):
        duration = Marker().lifetime
        duration.sec = int(seconds)
        duration.nanosec = int((seconds - duration.sec) * 1e9)
        return duration

    def _load_fk_solver(self) -> Optional[SimpleUrdfFk]:
        urdf_xml = ""
        if self.trajectory_fk_urdf_topic:
            urdf_xml = self._read_urdf_topic()
        if not urdf_xml and self.trajectory_fk_urdf_path:
            with open(self.trajectory_fk_urdf_path, "r", encoding="utf-8") as file:
                urdf_xml = file.read()
        if not urdf_xml:
            self.get_logger().warning(
                "trajectory_mode=fk but no URDF was loaded; falling back to action_xyz"
            )
            self.trajectory_mode = "action_xyz"
            return None
        try:
            solver = SimpleUrdfFk(urdf_xml, self.trajectory_fk_base_link, self.trajectory_fk_tip_link)
            chain_joints = [joint.name for joint in solver.chain]
            self.get_logger().info(
                f"trajectory FK enabled base={self.trajectory_fk_base_link} "
                f"tip={self.trajectory_fk_tip_link} joints={chain_joints}"
            )
            return solver
        except Exception as exc:
            self.get_logger().warning(f"trajectory FK setup failed: {exc}; falling back to action_xyz")
            self.trajectory_mode = "action_xyz"
            return None

    def _read_urdf_topic(self) -> str:
        received: list[str] = []

        def on_urdf(msg: String) -> None:
            if msg.data and not received:
                received.append(msg.data)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sub = self.create_subscription(String, self.trajectory_fk_urdf_topic, on_urdf, qos)
        deadline = time.time() + self.trajectory_fk_urdf_topic_timeout_sec
        try:
            while not received and time.time() < deadline and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
        finally:
            self.destroy_subscription(sub)
        if not received:
            self.get_logger().warning(f"URDF topic timed out: {self.trajectory_fk_urdf_topic}")
            return ""
        return received[0]

    def _action_to_point(self, action: np.ndarray) -> Point:
        values = np.asarray(action, dtype=np.float32).reshape(-1)
        if self.trajectory_mode == "fk" and self._fk_solver is not None:
            joint_positions = {
                name: float(values[index])
                for index, name in enumerate(self.trajectory_fk_joint_names)
                if index < values.size
            }
            xyz = self._fk_solver.position(joint_positions)
            xyz = xyz * self.trajectory_scale + self.trajectory_offset_xyz
            return Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))

        coords = []
        for index in self.trajectory_action_indices[:3]:
            coords.append(float(values[index]) if 0 <= index < values.size else 0.0)
        return Point(
            x=coords[0] * self.trajectory_scale + float(self.trajectory_offset_xyz[0]),
            y=coords[1] * self.trajectory_scale + float(self.trajectory_offset_xyz[1]),
            z=coords[2] * self.trajectory_scale + self.trajectory_z_offset + float(self.trajectory_offset_xyz[2]),
        )

    def _current_tip_point(self) -> Optional[Point]:
        if self.trajectory_mode != "fk" or self._fk_solver is None:
            return None
        with self._lock:
            if self._state_vector is None:
                return None
            names = list(self._state_names)
            values = np.asarray(self._state_vector, dtype=np.float32).copy()
        joint_positions = {
            name: float(values[index])
            for index, name in enumerate(names)
            if index < values.size
        }
        xyz = self._fk_solver.position(joint_positions)
        xyz = xyz * self.trajectory_scale + self.trajectory_offset_xyz
        return Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))

    @staticmethod
    def _shift_points(points: list[Point], offset: np.ndarray) -> list[Point]:
        return [
            Point(
                x=float(point.x + offset[0]),
                y=float(point.y + offset[1]),
                z=float(point.z + offset[2]),
            )
            for point in points
        ]

    def _anchor_actions_to_current_joints(self, actions: list[np.ndarray]) -> list[np.ndarray]:
        if not actions or self.trajectory_mode != "fk":
            return actions
        joint_names = self.trajectory_fk_joint_names
        with self._lock:
            if self._state_vector is None:
                return actions
            current = {name: float(value) for name, value in zip(self._state_names, self._state_vector)}
        first = np.asarray(actions[0], dtype=np.float32).reshape(-1)
        anchored: list[np.ndarray] = []
        for action in actions:
            values = np.asarray(action, dtype=np.float32).copy().reshape(-1)
            for index, name in enumerate(joint_names):
                if index >= values.size or index >= first.size or name not in current:
                    continue
                values[index] = float(current[name]) + (float(values[index]) - float(first[index]))
            anchored.append(values)
        return anchored

    @staticmethod
    def _marker_ns(policy_id: str) -> str:
        safe = "".join(ch if ch.isalnum() or ch in {"_", "-"} else "_" for ch in policy_id)
        return f"fv_policy_runner_trajectory_{safe or 'policy'}"

    def _palette_color(self, index: int) -> ColorRGBA:
        palette = [
            (0.0, 0.72, 1.0, 1.0),
            (1.0, 0.45, 0.05, 1.0),
            (0.2, 0.9, 0.35, 1.0),
            (1.0, 0.15, 0.35, 1.0),
            (0.95, 0.9, 0.1, 1.0),
        ]
        r, g, b, a = palette[index % len(palette)]
        return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))

    @staticmethod
    def _parse_hex_color(value: str) -> Optional[ColorRGBA]:
        """Parse "#RRGGBB" / "RRGGBB" / "#RRGGBBAA" → ColorRGBA. None if invalid."""
        s = (value or "").strip().lstrip("#")
        if len(s) not in (6, 8):
            return None
        try:
            r = int(s[0:2], 16) / 255.0
            g = int(s[2:4], 16) / 255.0
            b = int(s[4:6], 16) / 255.0
            a = int(s[6:8], 16) / 255.0 if len(s) == 8 else 1.0
        except ValueError:
            return None
        return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))

    def _marker_color(self, marker: Marker, index: int) -> ColorRGBA:
        # Operator-chosen colour is stable per policy; fall back to the
        # positional palette only when no colour is configured.
        color = self._traj_color if self._traj_color is not None \
            else self._palette_color(index)
        marker.color = color
        return color

    def _publish_trajectory_markers(
        self, trajectories: dict[str, list[np.ndarray]], policy_infer_ms: dict[str, float]
    ) -> None:
        if not self.trajectory_publish or not trajectories:
            return

        now = self.get_clock().now().to_msg()
        lifetime = self._duration_msg(self.trajectory_lifetime_sec)
        marker_array = MarkerArray()

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        current_tip = self._current_tip_point() if self.trajectory_include_current_tip else None
        for index, (policy_id, actions) in enumerate(trajectories.items()):
            if (
                current_tip is not None
                and self.trajectory_anchor_to_current_tip
                and self.trajectory_anchor_mode == "joint"
            ):
                actions = self._anchor_actions_to_current_joints(actions)
            points = [self._action_to_point(action) for action in actions]
            if (
                current_tip is not None
                and self.trajectory_anchor_to_current_tip
                and self.trajectory_anchor_mode == "cartesian"
                and points
            ):
                offset = np.asarray(
                    [
                        current_tip.x - points[0].x,
                        current_tip.y - points[0].y,
                        current_tip.z - points[0].z,
                    ],
                    dtype=np.float64,
                )
                points = self._shift_points(points, offset)
            if current_tip is not None:
                points.insert(0, current_tip)
            if not points:
                continue
            ns = self._marker_ns(policy_id)

            line = Marker()
            line.header.frame_id = self.trajectory_frame_id
            line.header.stamp = now
            line.ns = ns
            line.id = index * 3
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = self.trajectory_line_width
            line.points = points
            line.lifetime = lifetime
            color = self._marker_color(line, index)
            line.colors = [color for _ in points]
            marker_array.markers.append(line)

            dots = Marker()
            dots.header.frame_id = self.trajectory_frame_id
            dots.header.stamp = now
            dots.ns = f"{ns}_points"
            dots.id = index * 3 + 1
            dots.type = Marker.SPHERE_LIST
            dots.action = Marker.ADD
            dots.pose.orientation.w = 1.0
            dot_size = max(self.trajectory_line_width * 2.5, 0.01)
            dots.scale.x = dot_size
            dots.scale.y = dot_size
            dots.scale.z = dot_size
            dots.points = points
            dots.colors = [color for _ in points]
            dots.lifetime = lifetime
            self._marker_color(dots, index)
            marker_array.markers.append(dots)

            label = Marker()
            label.header.frame_id = self.trajectory_frame_id
            label.header.stamp = now
            label.ns = f"{ns}_label"
            label.id = index * 3 + 2
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = points[-1]
            label.pose.orientation.w = 1.0
            label.scale.z = self.trajectory_label_height
            elapsed_ms = policy_infer_ms.get(policy_id)
            label.text = f"{policy_id} ({elapsed_ms:.0f}ms)" if elapsed_ms is not None else policy_id
            label.lifetime = lifetime
            self._marker_color(label, index)
            marker_array.markers.append(label)

        if len(marker_array.markers) > 1:
            self._trajectory_pub.publish(marker_array)

    def _publish_status(self, reason: str) -> None:
        with self._lock:
            payload = {
                "reason": reason,
                "running": self._running,
                "policy_id": self._policy_id,
                "compare_policy_ids": self.compare_policy_ids,
                "queue_steps": len(self._queue),
                "inflight": self._infer_future is not None,
                "last_error": self._last_error,
                "last_infer_ms": self._last_infer_ms,
                "last_compare_ms": self._last_compare_ms,
                "last_infer_wall_ms": self._last_infer_wall_ms,
                "last_policy_infer_ms": self._last_policy_infer_ms,
                "last_trajectory_models": self._last_trajectory_models,
                "trajectory_mode": self.trajectory_mode,
                "policy_angle_unit": self.policy_angle_unit,
                "policy_gripper_state_override": self.policy_gripper_state_override,
                "policy_gripper_state_value": self.policy_gripper_state_value,
            }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=True)
        self._status_pub.publish(msg)

    def _on_tick(self) -> None:
        self._consume_future()

        with self._lock:
            running = self._running
            queue_len = len(self._queue)
            inflight = self._infer_future is not None

        if not running:
            return

        now = time.monotonic()
        if (
            self.trajectory_refresh_interval_sec > 0.0
            and not inflight
            and now >= self._next_trajectory_refresh_time
        ):
            self._submit_infer()
            self._next_trajectory_refresh_time = now + self.trajectory_refresh_interval_sec
            return

        if queue_len == 0 and not inflight:
            self._submit_infer()
            if self.trajectory_refresh_interval_sec > 0.0:
                self._next_trajectory_refresh_time = now + self.trajectory_refresh_interval_sec
            return

        with self._lock:
            action = self._queue.popleft() if self._queue else None
            if self._queue_raw:
                self._queue_raw.popleft()
            queue_len = len(self._queue)
            inflight = self._infer_future is not None

        if action is not None:
            self._publish_action(action)

        # RTC 有効時は shim が返した refill しきい値 (推論遅延 + 実行ホライズン) を
        # 使い、**再生中に**次を投げる。無効/未較正 (=0) のときだけ従来の
        # replenish_threshold_steps (既定=枯渇間際) にフォールバックする。
        # replace モード限定: RTC の合成は「新チャンクでキューを置き換える」前提。
        # append モード (実機ダイヘン既定) でこのしきい値を使うと、旧チャンクの
        # 後ろに新チャンクが積まれて定常ラグが増えるだけなので、従来の
        # replenish 挙動を1ビットも変えない。
        threshold = ((self._rtc_refill_threshold
                      if self.replace_action_queue_on_infer else 0)
                     or self.replenish_threshold_steps)
        if queue_len <= threshold and not inflight:
            self._submit_infer()

    def destroy_node(self) -> bool:
        self._executor.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FvPolicyRunnerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
