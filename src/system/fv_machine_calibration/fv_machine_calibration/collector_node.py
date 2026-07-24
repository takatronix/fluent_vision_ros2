"""Read-only ROS collector for stationary-bracket hand-eye session drafts.

This node has no actuator publishers or clients.  It never uses tf2 lookup,
so stamp rewriting, interpolation, and latest-transform fallback are absent.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Optional

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseArray
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage, Image, JointState
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from .collector_model import (
    CollectorMetadata,
    StationaryBracketEvidenceSynchronizer,
    selected_robot_id,
    session_draft_document,
)
from .draft import DRAFT_SUFFIX, write_new_session_draft
from .model import CalibrationError, Pose


def _stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def _pose(message) -> Pose:
    return Pose(
        (
            float(message.position.x),
            float(message.position.y),
            float(message.position.z),
        ),
        (
            float(message.orientation.x),
            float(message.orientation.y),
            float(message.orientation.z),
            float(message.orientation.w),
        ),
    )


def _transform_pose(message) -> Pose:
    return Pose(
        (
            float(message.translation.x),
            float(message.translation.y),
            float(message.translation.z),
        ),
        (
            float(message.rotation.x),
            float(message.rotation.y),
            float(message.rotation.z),
            float(message.rotation.w),
        ),
    )


class HandEyeCollector(Node):
    """Collect synchronized evidence in memory and write one new draft."""

    def __init__(self) -> None:
        super().__init__("fv_handeye_collector")
        robot_id = selected_robot_id()
        self.metadata = CollectorMetadata(
            session_id=self._required_string("session_id"),
            robot_id=robot_id,
            d405_serial=self._required_string("d405.serial"),
            d405_firmware=self._required_string("d405.firmware"),
            joint_zero_calibration_id=self._required_string(
                "dependencies.joint_zero_calibration_id"
            ),
            tool_center_point_calibration_id=self._required_string(
                "dependencies.tool_center_point_calibration_id"
            ),
            d405_intrinsics_calibration_id=self._required_string(
                "dependencies.d405_intrinsics_calibration_id"
            ),
            tag_registry_sha256=self._required_string(
                "dependencies.tag_registry_sha256"
            ),
            urdf_sha256=self._required_string("dependencies.urdf_sha256"),
        )
        self.metadata.validate()

        output_text = self._required_string("output.session_draft_path")
        self.output_path = Path(output_text)
        if not self.output_path.is_absolute():
            raise CalibrationError("output.session_draft_path must be absolute")
        if not self.output_path.name.endswith(DRAFT_SUFFIX):
            raise CalibrationError(
                f"output.session_draft_path must end with {DRAFT_SUFFIX}"
            )
        if self.output_path.exists() or self.output_path.is_symlink():
            raise CalibrationError(
                f"refusing existing session draft path: {self.output_path}"
            )
        if not self.output_path.parent.is_dir() \
                or self.output_path.parent.is_symlink():
            raise CalibrationError("session draft parent must be an existing real directory")
        try:
            resolved_output_parent = self.output_path.parent.resolve(strict=True)
        except OSError as exc:
            raise CalibrationError("cannot resolve session draft parent") from exc
        if resolved_output_parent != self.output_path.parent:
            raise CalibrationError("session draft parent path must not traverse symlinks")

        self.image_topic = self._required_absolute_topic("topics.image")
        self.detections_topic = self._required_absolute_topic("topics.detections")
        self.poses_topic = self._required_absolute_topic("topics.tag_poses")
        self.joint_states_topic = self._required_absolute_topic(
            "topics.joint_states"
        )
        self.tf_topic = self._required_absolute_topic("topics.tf")
        self.tf_static_topic = self._required_absolute_topic("topics.tf_static")
        self.source_topics = {
            "image": self.image_topic,
            "detections": self.detections_topic,
            "tag_poses": self.poses_topic,
            "joint_states": self.joint_states_topic,
            "tf": self.tf_topic,
            "tf_static": self.tf_static_topic,
        }
        source_topics = (
            self.image_topic,
            self.detections_topic,
            self.poses_topic,
            self.joint_states_topic,
            self.tf_topic,
            self.tf_static_topic,
        )
        if len(set(source_topics)) != len(source_topics):
            raise CalibrationError("collector source topics must be unique")

        image_transport = self._required_string("image.transport")
        if image_transport not in {"raw", "compressed"}:
            raise CalibrationError("image.transport must be raw or compressed")
        required_joints = self._required_string_array("joints.required_names")
        tf_frames = self._required_string_array("tf.frame_allowlist")
        required_frames = {"base_link", "gripper_tip"}
        if not required_frames <= set(tf_frames):
            raise CalibrationError(
                "tf.frame_allowlist must contain base_link and gripper_tip"
            )
        self.tf_frame_allowlist = frozenset(tf_frames)
        max_age_s = self._finite_parameter("safety.max_data_age_s", 0.5)
        max_bracket_gap_s = self._finite_parameter(
            "safety.max_bracket_gap_s", 0.05
        )
        max_sample_interval_s = self._finite_parameter(
            "safety.max_sample_interval_s", 0.03
        )
        min_samples_per_side = self._integer_parameter(
            "safety.min_samples_per_side", 2
        )
        velocity_limit = self._finite_parameter(
            "safety.stationary_velocity_limit_rad_s", 0.01
        )
        max_joint_delta_rad = self._finite_parameter(
            "safety.max_joint_delta_rad", 0.001
        )
        max_tf_translation_delta_m = self._finite_parameter(
            "safety.max_tf_translation_delta_m", 0.0005
        )
        max_tf_rotation_delta_rad = self._finite_parameter(
            "safety.max_tf_rotation_delta_rad", 0.001
        )
        self.synchronizer = StationaryBracketEvidenceSynchronizer(
            required_joint_names=required_joints,
            max_data_age_s=max_age_s,
            max_bracket_gap_s=max_bracket_gap_s,
            max_sample_interval_s=max_sample_interval_s,
            min_samples_per_side=min_samples_per_side,
            stationary_velocity_limit_rad_s=velocity_limit,
            max_joint_delta_rad=max_joint_delta_rad,
            max_tf_translation_delta_m=max_tf_translation_delta_m,
            max_tf_rotation_delta_rad=max_tf_rotation_delta_rad,
        )

        self.samples = []
        self._finalized = False
        self._last_image_stamp_ns: Optional[int] = None
        self._last_array_stamp: dict[str, int] = {}
        self._detection_messages: dict[int, AprilTagDetectionArray] = {}
        self._pose_messages: dict[int, PoseArray] = {}
        self._last_error = "no synchronized evidence received"

        self.create_subscription(
            CompressedImage if image_transport == "compressed" else Image,
            self.image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            AprilTagDetectionArray,
            self.detections_topic,
            self._on_detections,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseArray,
            self.poses_topic,
            self._on_tag_poses,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            self.joint_states_topic,
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        tf_static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            TFMessage, self.tf_topic, self._on_tf, tf_qos
        )
        self.create_subscription(
            TFMessage, self.tf_static_topic, self._on_tf_static, tf_static_qos
        )
        self.create_service(
            Trigger, "~/capture_training", self._capture_training
        )
        self.create_service(
            Trigger, "~/capture_holdout", self._capture_holdout
        )
        self.create_service(Trigger, "~/write_session_draft", self._write_draft)
        self.get_logger().warn(
            "read-only hand-eye collector started; it has no CAN/Piper command "
            "interfaces and writes only a new unapproved session draft"
        )

    def _parameter(self, name: str, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _required_string(self, name: str) -> str:
        value = str(self._parameter(name, "")).strip()
        if not value:
            raise CalibrationError(f"required parameter {name} is empty")
        return value

    def _required_absolute_topic(self, name: str) -> str:
        value = self._required_string(name)
        if not value.startswith("/") or value.endswith("/") or "//" in value:
            raise CalibrationError(f"{name} must be a normalized absolute topic")
        return value

    def _required_string_array(self, name: str) -> tuple[str, ...]:
        raw = self._parameter(name, [])
        values = tuple(str(item).strip() for item in raw)
        if not values or any(not item for item in values) \
                or len(values) != len(set(values)):
            raise CalibrationError(f"{name} must be a unique nonempty string array")
        return values

    def _finite_parameter(self, name: str, default: float) -> float:
        value = float(self._parameter(name, default))
        if not math.isfinite(value):
            raise CalibrationError(f"{name} must be finite")
        return value

    def _integer_parameter(self, name: str, default: int) -> int:
        value = self._parameter(name, default)
        if isinstance(value, bool) or not isinstance(value, int):
            raise CalibrationError(f"{name} must be an integer")
        return value

    def _reject(self, stamp_ns: int, reason: object) -> None:
        self._last_error = str(reason)
        self.synchronizer.block_stamp(stamp_ns, self._last_error)
        self.get_logger().error(f"collector evidence rejected: {self._last_error}")

    def _on_image(self, message) -> None:
        stamp = _stamp_ns(message)
        try:
            self.synchronizer.observe_image(
                stamp_ns=stamp, frame_id=message.header.frame_id
            )
            self._last_image_stamp_ns = stamp
        except CalibrationError as exc:
            self._reject(stamp, exc)

    def _strict_array_stamp(self, stream: str, stamp: int) -> None:
        previous = self._last_array_stamp.get(stream)
        if stamp <= 0 or (previous is not None and stamp <= previous):
            raise CalibrationError(
                f"{stream} stamp is duplicate/non-monotonic: {stamp} after {previous}"
            )
        self._last_array_stamp[stream] = stamp

    def _on_detections(self, message: AprilTagDetectionArray) -> None:
        stamp = _stamp_ns(message)
        try:
            self._strict_array_stamp("detections", stamp)
            self._detection_messages[stamp] = message
            self._try_pair_tag(stamp)
        except CalibrationError as exc:
            self._reject(stamp, exc)

    def _on_tag_poses(self, message: PoseArray) -> None:
        stamp = _stamp_ns(message)
        try:
            self._strict_array_stamp("tag_poses", stamp)
            self._pose_messages[stamp] = message
            self._try_pair_tag(stamp)
        except CalibrationError as exc:
            self._reject(stamp, exc)

    def _try_pair_tag(self, stamp: int) -> None:
        detections = self._detection_messages.get(stamp)
        poses = self._pose_messages.get(stamp)
        if detections is None or poses is None:
            return
        if detections.header.frame_id != poses.header.frame_id:
            raise CalibrationError("detection and tag-pose frames differ")
        if len(detections.detections) != len(poses.poses):
            raise CalibrationError("detection and tag-pose array lengths differ")
        indices = [
            index for index, detection in enumerate(detections.detections)
            if detection.family == "tag36h11" and int(detection.id) == 0
        ]
        if len(indices) != 1:
            raise CalibrationError("exactly one tag36h11 ID 0 detection is required")
        index = indices[0]
        detection = detections.detections[index]
        self.synchronizer.observe_tag(
            stamp_ns=stamp,
            frame_id=detections.header.frame_id,
            family=detection.family,
            tag_id=int(detection.id),
            hamming=int(detection.hamming),
            decision_margin=float(detection.decision_margin),
            tag_in_camera=_pose(poses.poses[index]),
        )
        self._detection_messages.pop(stamp, None)
        self._pose_messages.pop(stamp, None)

    def _on_joint_state(self, message: JointState) -> None:
        stamp = _stamp_ns(message)
        try:
            self.synchronizer.observe_joint_state(
                stamp_ns=stamp,
                names=message.name,
                positions=message.position,
                velocities=message.velocity,
            )
        except CalibrationError as exc:
            self._reject(stamp, exc)

    def _observe_tf_message(self, message: TFMessage, *, is_static: bool) -> None:
        for transform in message.transforms:
            parent = transform.header.frame_id
            child = transform.child_frame_id
            if parent not in self.tf_frame_allowlist \
                    or child not in self.tf_frame_allowlist:
                continue
            stamp = _stamp_ns(transform)
            try:
                self.synchronizer.observe_transform(
                    parent_frame=parent,
                    child_frame=child,
                    pose=_transform_pose(transform.transform),
                    stamp_ns=None if is_static else stamp,
                    is_static=is_static,
                )
            except CalibrationError as exc:
                self._reject(stamp, exc)

    def _on_tf(self, message: TFMessage) -> None:
        self._observe_tf_message(message, is_static=False)

    def _on_tf_static(self, message: TFMessage) -> None:
        self._observe_tf_message(message, is_static=True)

    def _source_authority_error(self) -> Optional[str]:
        for topic in (
            self.image_topic,
            self.detections_topic,
            self.poses_topic,
            self.joint_states_topic,
        ):
            count = len(self.get_publishers_info_by_topic(topic))
            if count != 1:
                return f"publisher count for {topic} is {count}, must be exactly 1"
        return None

    def _capture(self, kind: str, response):
        if self._finalized:
            response.success = False
            response.message = "session draft already finalized"
            return response
        authority_error = self._source_authority_error()
        if authority_error:
            response.success = False
            response.message = authority_error
            return response
        if self._last_image_stamp_ns is None:
            response.success = False
            response.message = self._last_error
            return response
        try:
            sample = self.synchronizer.capture(
                kind=kind,
                image_stamp_ns=self._last_image_stamp_ns,
                now_ns=self.get_clock().now().nanoseconds,
            )
            self.samples.append(sample)
        except CalibrationError as exc:
            response.success = False
            response.message = str(exc)
            return response
        response.success = True
        response.message = (
            f"captured {kind} image_stamp_ns={sample.image_stamp_ns}; "
            f"training={sum(item.kind == 'training' for item in self.samples)}, "
            f"holdout={sum(item.kind == 'holdout' for item in self.samples)}"
        )
        return response

    def _capture_training(self, _request, response):
        return self._capture("training", response)

    def _capture_holdout(self, _request, response):
        return self._capture("holdout", response)

    def _write_draft(self, _request, response):
        if self._finalized:
            response.success = False
            response.message = "session draft already finalized"
            return response
        try:
            document = session_draft_document(
                self.metadata,
                self.samples,
                source_topics=self.source_topics,
                require_complete=True,
            )
            write_new_session_draft(self.output_path, document)
            self._finalized = True
        except CalibrationError as exc:
            response.success = False
            response.message = str(exc)
            return response
        response.success = True
        response.message = str(self.output_path)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = HandEyeCollector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except CalibrationError as exc:
        print(f"fv_handeye_collector: FATAL: {exc}")
        raise SystemExit(2) from exc
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
