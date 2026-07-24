"""ROS-independent stationary-bracket evidence for hand-eye collection."""

from __future__ import annotations

from bisect import bisect_left
from dataclasses import dataclass
import math
import os
import re
import socket
from typing import Mapping, Sequence

import numpy as np

from .model import (
    CalibrationError,
    Pose,
    Sample,
    matrix_to_quaternion,
    parse_session,
)


NSEC_PER_SEC = 1_000_000_000
_ROBOT_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")


def selected_robot_id(
    environ: Mapping[str, str] | None = None,
    hostname: str | None = None,
) -> str:
    """Select ASPA_ROBOT_ID, otherwise the short hostname, without fallback."""
    environment = os.environ if environ is None else environ
    configured = environment.get("ASPA_ROBOT_ID", "").strip()
    candidate = configured or (
        socket.gethostname() if hostname is None else hostname
    ).strip().split(".", 1)[0]
    if not _ROBOT_ID_RE.fullmatch(candidate) or candidate in {".", ".."}:
        raise CalibrationError("ASPA_ROBOT_ID/short hostname is not fleet-safe")
    return candidate


def _normalized_frame(value: object, label: str) -> str:
    frame = str(value or "")
    if (
        not frame
        or frame.startswith("/")
        or frame.endswith("/")
        or "//" in frame
    ):
        raise CalibrationError(f"{label} is not a normalized frame id")
    return frame


def _positive_stamp(stamp_ns: object, label: str) -> int:
    if isinstance(stamp_ns, bool) or not isinstance(stamp_ns, int) \
            or stamp_ns <= 0:
        raise CalibrationError(f"{label} must be a positive integer timestamp")
    return stamp_ns


def _pose_from_matrix(matrix: np.ndarray) -> Pose:
    transform = np.asarray(matrix, dtype=np.float64)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise CalibrationError("resolved TF is not a finite 4x4 matrix")
    if not np.allclose(transform[3], (0.0, 0.0, 0.0, 1.0), atol=1e-9):
        raise CalibrationError("resolved TF is not homogeneous")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-3) \
            or abs(float(np.linalg.det(rotation)) - 1.0) > 1e-3:
        raise CalibrationError("resolved TF rotation is not rigid")
    return Pose(
        tuple(float(value) for value in transform[:3, 3]),
        matrix_to_quaternion(rotation),
    )


@dataclass(frozen=True)
class CollectorMetadata:
    session_id: str
    robot_id: str
    d405_serial: str
    d405_firmware: str
    joint_zero_calibration_id: str
    tool_center_point_calibration_id: str
    d405_intrinsics_calibration_id: str
    tag_registry_sha256: str
    urdf_sha256: str

    def base_document(self) -> dict[str, object]:
        return {
            "schema_version": 1,
            "session_id": self.session_id,
            "robot_id": self.robot_id,
            "d405_serial": self.d405_serial,
            "d405_firmware": self.d405_firmware,
            "joint_zero_calibration_id": self.joint_zero_calibration_id,
            "tool_center_point_calibration_id": (
                self.tool_center_point_calibration_id
            ),
            "d405_intrinsics_calibration_id": (
                self.d405_intrinsics_calibration_id
            ),
            "tag_family": "tag36h11",
            "tag_id": 0,
            "tag_black_edge_m": 0.040,
            "tag_registry_sha256": self.tag_registry_sha256,
            "urdf_sha256": self.urdf_sha256,
            "frames": {
                "base": "base_link",
                "gripper": "gripper_tip",
                "camera": "d405_optical_frame",
            },
            "samples": [],
        }

    def validate(self) -> None:
        parse_session(self.base_document(), require_complete=False)


@dataclass(frozen=True)
class ImageEvidence:
    stamp_ns: int
    frame_id: str


@dataclass(frozen=True)
class TagEvidence:
    stamp_ns: int
    frame_id: str
    family: str
    tag_id: int
    hamming: int
    decision_margin: float
    tag_in_camera: Pose


@dataclass(frozen=True)
class JointEvidence:
    stamp_ns: int
    names: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...]


@dataclass(frozen=True)
class TransformEvidence:
    parent_frame: str
    child_frame: str
    pose: Pose
    stamp_ns: int | None
    is_static: bool


@dataclass(frozen=True)
class StationaryBracketLimits:
    max_bracket_gap_ns: int
    max_sample_interval_ns: int
    min_samples_per_side: int
    stationary_velocity_limit_rad_s: float
    max_joint_delta_rad: float
    max_tf_translation_delta_m: float
    max_tf_rotation_delta_rad: float


@dataclass(frozen=True)
class ResolvedTransform:
    pose: Pose
    stamp_ns: int
    path: tuple[tuple[TransformEvidence, bool], ...]


@dataclass(frozen=True)
class SampleProvenance:
    image: ImageEvidence
    tag: TagEvidence
    joint_before: JointEvidence
    joint_after: JointEvidence
    joint_window: tuple[JointEvidence, ...]
    tf_before: ResolvedTransform
    tf_after: ResolvedTransform
    tf_window: tuple[ResolvedTransform, ...]
    required_joint_names: tuple[str, ...]
    limits: StationaryBracketLimits
    max_joint_delta_rad: float
    max_observed_velocity_rad_s: float
    velocity_span_estimate_rad: float
    tf_translation_delta_m: float
    tf_rotation_delta_rad: float
    max_joint_sample_interval_ns: int
    max_tf_sample_interval_ns: int


@dataclass(frozen=True)
class CapturedSample:
    sample: Sample
    provenance: SampleProvenance

    @property
    def kind(self) -> str:
        return self.sample.kind

    @property
    def image_stamp_ns(self) -> int:
        return self.sample.image_stamp_ns


def joint_window_metrics(
    window: Sequence[JointEvidence], required_joint_names: Sequence[str]
) -> tuple[float, float, float]:
    required = tuple(required_joint_names)
    if not window or not required:
        raise CalibrationError("joint observation window is incomplete")
    position_maps = [dict(zip(item.names, item.positions)) for item in window]
    velocity_maps = [dict(zip(item.names, item.velocities)) for item in window]
    if any(not set(required) <= set(values) for values in position_maps) \
            or any(not set(required) <= set(values) for values in velocity_maps):
        raise CalibrationError("joint observation window is missing required joints")
    max_delta = max(
        max(values) - min(values)
        for name in required
        for values in ([positions[name] for positions in position_maps],)
    )
    max_velocity = max(
        abs(velocities[name])
        for velocities in velocity_maps
        for name in required
    )
    span_s = (window[-1].stamp_ns - window[0].stamp_ns) / NSEC_PER_SEC
    return max_delta, max_velocity, max_velocity * span_s


def pose_window_metrics(poses: Sequence[Pose]) -> tuple[float, float]:
    matrices = [pose.matrix() for pose in poses]
    if len(matrices) < 2:
        raise CalibrationError("TF observation window is incomplete")
    translation_delta = 0.0
    rotation_delta = 0.0
    for index, left in enumerate(matrices):
        for right in matrices[index + 1:]:
            translation_delta = max(
                translation_delta,
                float(np.linalg.norm(right[:3, 3] - left[:3, 3])),
            )
            relative_rotation = left[:3, :3].T @ right[:3, :3]
            cosine = min(
                1.0,
                max(-1.0, (float(np.trace(relative_rotation)) - 1.0) / 2.0),
            )
            rotation_delta = max(rotation_delta, math.acos(cosine))
    return translation_delta, rotation_delta


def max_sample_interval_ns(stamps: Sequence[int], image_stamp_ns: int) -> int:
    ordered = sorted((*stamps, image_stamp_ns))
    if len(ordered) < 2:
        raise CalibrationError("observation window has no sample interval")
    return max(right - left for left, right in zip(ordered, ordered[1:]))


def _pose_document(pose: Pose) -> dict[str, object]:
    return {
        "translation_m": list(pose.translation_m),
        "quaternion_xyzw": list(pose.quaternion_xyzw),
    }


def _joint_document(evidence: JointEvidence) -> dict[str, object]:
    return {
        "stamp_ns": evidence.stamp_ns,
        "names": list(evidence.names),
        "positions_rad": list(evidence.positions),
        "velocities_rad_s": list(evidence.velocities),
    }


def _resolved_tf_document(evidence: ResolvedTransform) -> dict[str, object]:
    path = []
    for edge, traversed_inverse in evidence.path:
        path.append({
            "parent_frame": edge.parent_frame,
            "child_frame": edge.child_frame,
            "pose_parent_from_child": _pose_document(edge.pose),
            "source_stamp_ns": edge.stamp_ns,
            "is_static": edge.is_static,
            "traversed_inverse": traversed_inverse,
        })
    return {
        "snapshot_stamp_ns": evidence.stamp_ns,
        "resolved_gripper_in_base": _pose_document(evidence.pose),
        "contributing_path_edges": path,
    }


def _validate_capture(capture: CapturedSample) -> None:
    sample = capture.sample
    evidence = capture.provenance
    stamp = sample.image_stamp_ns
    if evidence.image.stamp_ns != stamp or evidence.tag.stamp_ns != stamp:
        raise CalibrationError("image/tag provenance stamp differs from sample")
    if sample.tag_in_camera != evidence.tag.tag_in_camera \
            or sample.decision_margin != evidence.tag.decision_margin:
        raise CalibrationError("tag provenance differs from solver sample")
    if sample.gripper_in_base != evidence.tf_before.pose:
        raise CalibrationError("solver pose is not the verified preceding TF pose")
    brackets = (
        (evidence.joint_before.stamp_ns, evidence.joint_after.stamp_ns, "JointState"),
        (evidence.tf_before.stamp_ns, evidence.tf_after.stamp_ns, "TF"),
    )
    for before, after, label in brackets:
        if not before < stamp < after:
            raise CalibrationError(f"{label} provenance does not strictly bracket image")
        if max(stamp - before, after - stamp) \
                > evidence.limits.max_bracket_gap_ns:
            raise CalibrationError(f"{label} provenance exceeds bracket gap limit")
    minimum = evidence.limits.min_samples_per_side
    if len(evidence.joint_window) < 2 * minimum \
            or len(evidence.tf_window) < 2 * minimum:
        raise CalibrationError("provenance observation window is incomplete")
    joint_stamps = tuple(item.stamp_ns for item in evidence.joint_window)
    tf_stamps = tuple(item.stamp_ns for item in evidence.tf_window)
    if tuple(sorted(joint_stamps)) != joint_stamps \
            or tuple(sorted(tf_stamps)) != tf_stamps:
        raise CalibrationError("provenance observation window is not ordered")
    joint_metrics = joint_window_metrics(
        evidence.joint_window, evidence.required_joint_names
    )
    tf_metrics = pose_window_metrics(
        tuple(item.pose for item in evidence.tf_window)
    )
    joint_interval = max_sample_interval_ns(joint_stamps, stamp)
    tf_interval = max_sample_interval_ns(tf_stamps, stamp)
    recomputed = (
        (*joint_metrics, *tf_metrics, joint_interval, tf_interval),
        (
            evidence.max_joint_delta_rad,
            evidence.max_observed_velocity_rad_s,
            evidence.velocity_span_estimate_rad,
            evidence.tf_translation_delta_m,
            evidence.tf_rotation_delta_rad,
            evidence.max_joint_sample_interval_ns,
            evidence.max_tf_sample_interval_ns,
        ),
    )
    if any(
        not math.isclose(float(actual), float(recorded), abs_tol=1e-12)
        for actual, recorded in zip(*recomputed)
    ):
        raise CalibrationError("saved stationarity metrics differ from raw provenance")
    if max(joint_interval, tf_interval) > evidence.limits.max_sample_interval_ns:
        raise CalibrationError("provenance sample interval violates its limit")
    bounds = (
        (
            evidence.max_joint_delta_rad,
            evidence.limits.max_joint_delta_rad,
            "joint delta",
        ),
        (
            evidence.max_observed_velocity_rad_s,
            evidence.limits.stationary_velocity_limit_rad_s,
            "joint velocity",
        ),
        (
            evidence.tf_translation_delta_m,
            evidence.limits.max_tf_translation_delta_m,
            "TF translation delta",
        ),
        (
            evidence.tf_rotation_delta_rad,
            evidence.limits.max_tf_rotation_delta_rad,
            "TF rotation delta",
        ),
    )
    for observed, limit, label in bounds:
        if not math.isfinite(observed) or observed < 0.0 or observed > limit:
            raise CalibrationError(f"{label} provenance violates its limit")
    if not math.isfinite(evidence.velocity_span_estimate_rad) \
            or evidence.velocity_span_estimate_rad < 0.0:
        raise CalibrationError("velocity-span estimate provenance is invalid")
    for resolved in evidence.tf_window:
        dynamic_edges = [edge for edge, _inverted in resolved.path if not edge.is_static]
        if not dynamic_edges or any(
            edge.stamp_ns != resolved.stamp_ns for edge in dynamic_edges
        ):
            raise CalibrationError("TF provenance path has inconsistent dynamic stamps")


def sample_document(capture: CapturedSample) -> dict[str, object]:
    _validate_capture(capture)
    sample = capture.sample
    evidence = capture.provenance
    stamp = sample.image_stamp_ns

    return {
        "kind": sample.kind,
        "image_stamp_ns": sample.image_stamp_ns,
        "gripper_in_base": _pose_document(sample.gripper_in_base),
        "tag_in_camera": _pose_document(sample.tag_in_camera),
        "decision_margin": sample.decision_margin,
        "raw_evidence_provenance": {
            "image": {
                "stamp_ns": evidence.image.stamp_ns,
                "frame_id": evidence.image.frame_id,
            },
            "tag_detection_and_pose": {
                "stamp_ns": evidence.tag.stamp_ns,
                "frame_id": evidence.tag.frame_id,
                "family": evidence.tag.family,
                "tag_id": evidence.tag.tag_id,
                "hamming": evidence.tag.hamming,
                "decision_margin": evidence.tag.decision_margin,
                "tag_in_camera": _pose_document(evidence.tag.tag_in_camera),
            },
            "joint_state_before": _joint_document(evidence.joint_before),
            "joint_state_after": _joint_document(evidence.joint_after),
            "joint_state_window": [
                _joint_document(item) for item in evidence.joint_window
            ],
            "tf_before": _resolved_tf_document(evidence.tf_before),
            "tf_after": _resolved_tf_document(evidence.tf_after),
            "tf_window": [
                _resolved_tf_document(item) for item in evidence.tf_window
            ],
            "bracket_offsets_ns": {
                "joint_before": stamp - evidence.joint_before.stamp_ns,
                "joint_after": evidence.joint_after.stamp_ns - stamp,
                "tf_before": stamp - evidence.tf_before.stamp_ns,
                "tf_after": evidence.tf_after.stamp_ns - stamp,
            },
            "observed_stationarity_bounds": {
                "max_joint_delta_rad": evidence.max_joint_delta_rad,
                "max_observed_velocity_rad_s": (
                    evidence.max_observed_velocity_rad_s
                ),
                "velocity_times_bracket_span_estimate_rad": (
                    evidence.velocity_span_estimate_rad
                ),
                "tf_translation_delta_m": evidence.tf_translation_delta_m,
                "tf_rotation_delta_rad": evidence.tf_rotation_delta_rad,
                "max_joint_sample_interval_ns": (
                    evidence.max_joint_sample_interval_ns
                ),
                "max_tf_sample_interval_ns": evidence.max_tf_sample_interval_ns,
                "interpretation": (
                    "bounded_observed_window_not_hidden_excursion_guarantee"
                ),
            },
        },
    }


def session_draft_document(
    metadata: CollectorMetadata,
    samples: Sequence[CapturedSample],
    *,
    source_topics: Mapping[str, str],
    require_complete: bool = True,
) -> dict[str, object]:
    metadata.validate()
    captures = tuple(samples)
    if not captures or not all(isinstance(item, CapturedSample) for item in captures):
        raise CalibrationError("session draft requires raw provenance for every sample")
    limits = captures[0].provenance.limits
    if any(item.provenance.limits != limits for item in captures):
        raise CalibrationError("stationary bracket limits changed within the session")
    required_joint_names = captures[0].provenance.required_joint_names
    if any(
        item.provenance.required_joint_names != required_joint_names
        for item in captures
    ):
        raise CalibrationError("required joint set changed within the session")
    topic_document = {str(key): str(value) for key, value in source_topics.items()}
    expected_topic_keys = {
        "image", "detections", "tag_poses", "joint_states", "tf", "tf_static"
    }
    if set(topic_document) != expected_topic_keys or any(
        not value.startswith("/")
        or value.endswith("/")
        or "//" in value
        for value in topic_document.values()
    ) or len(set(topic_document.values())) != len(topic_document):
        raise CalibrationError("source_topics must contain normalized absolute topics")
    document = metadata.base_document()
    document["status"] = "SESSION_DRAFT_UNAPPROVED"
    document["calibration_ready"] = False
    document["samples"] = [sample_document(capture) for capture in captures]
    document["source_topics"] = topic_document
    document["collector_contract"] = {
        "stationary_bracket_contract_version": 1,
        "time_alignment": "strict_before_after_stationary_brackets",
        "image_and_tag_stamp_equality_required": True,
        "tf_pose_selection": "preceding_snapshot_verified_by_following_snapshot",
        "stamp_rewrite": "forbidden",
        "tf_interpolation": "forbidden",
        "tf_latest_lookup": "forbidden",
        "required_joint_names": list(required_joint_names),
        "limits": {
            "max_bracket_gap_ns": limits.max_bracket_gap_ns,
            "max_sample_interval_ns": limits.max_sample_interval_ns,
            "min_samples_per_side": limits.min_samples_per_side,
            "stationary_velocity_limit_rad_s": (
                limits.stationary_velocity_limit_rad_s
            ),
            "max_joint_delta_rad": limits.max_joint_delta_rad,
            "max_tf_translation_delta_m": limits.max_tf_translation_delta_m,
            "max_tf_rotation_delta_rad": limits.max_tf_rotation_delta_rad,
        },
        "writes": "new_session_draft_only",
    }
    parse_session(document, require_complete=require_complete)
    return document


class StationaryBracketEvidenceSynchronizer:
    """Join image/tag exactly and bound observed JointState/TF stationarity."""

    def __init__(
        self,
        *,
        expected_camera_frame: str = "d405_optical_frame",
        base_frame: str = "base_link",
        gripper_frame: str = "gripper_tip",
        required_joint_names: Sequence[str],
        max_data_age_s: float = 0.5,
        max_bracket_gap_s: float = 0.05,
        max_sample_interval_s: float = 0.03,
        min_samples_per_side: int = 2,
        stationary_velocity_limit_rad_s: float = 0.01,
        max_joint_delta_rad: float = 0.001,
        max_tf_translation_delta_m: float = 0.0005,
        max_tf_rotation_delta_rad: float = 0.001,
    ) -> None:
        self.expected_camera_frame = _normalized_frame(
            expected_camera_frame, "expected_camera_frame"
        )
        self.base_frame = _normalized_frame(base_frame, "base_frame")
        self.gripper_frame = _normalized_frame(gripper_frame, "gripper_frame")
        joints = tuple(str(value) for value in required_joint_names)
        if not joints or any(not value for value in joints) \
                or len(set(joints)) != len(joints):
            raise CalibrationError("required_joint_names must be unique and nonempty")
        self.required_joint_names = joints
        if not math.isfinite(max_data_age_s) or max_data_age_s <= 0.0:
            raise CalibrationError("max_data_age_s must be finite and positive")
        if not math.isfinite(max_bracket_gap_s) or max_bracket_gap_s <= 0.0:
            raise CalibrationError("max_bracket_gap_s must be finite and positive")
        if not math.isfinite(max_sample_interval_s) \
                or max_sample_interval_s <= 0.0:
            raise CalibrationError(
                "max_sample_interval_s must be finite and positive"
            )
        if isinstance(min_samples_per_side, bool) \
                or not isinstance(min_samples_per_side, int) \
                or min_samples_per_side < 2:
            raise CalibrationError("min_samples_per_side must be an integer >= 2")
        if (
            not math.isfinite(stationary_velocity_limit_rad_s)
            or stationary_velocity_limit_rad_s < 0.0
        ):
            raise CalibrationError(
                "stationary_velocity_limit_rad_s must be finite and nonnegative"
            )
        nonnegative_limits = {
            "max_joint_delta_rad": max_joint_delta_rad,
            "max_tf_translation_delta_m": max_tf_translation_delta_m,
            "max_tf_rotation_delta_rad": max_tf_rotation_delta_rad,
        }
        for label, value in nonnegative_limits.items():
            if not math.isfinite(value) or value < 0.0:
                raise CalibrationError(f"{label} must be finite and nonnegative")
        self.max_data_age_ns = int(max_data_age_s * NSEC_PER_SEC)
        self.limits = StationaryBracketLimits(
            max_bracket_gap_ns=int(max_bracket_gap_s * NSEC_PER_SEC),
            max_sample_interval_ns=int(max_sample_interval_s * NSEC_PER_SEC),
            min_samples_per_side=min_samples_per_side,
            stationary_velocity_limit_rad_s=stationary_velocity_limit_rad_s,
            max_joint_delta_rad=max_joint_delta_rad,
            max_tf_translation_delta_m=max_tf_translation_delta_m,
            max_tf_rotation_delta_rad=max_tf_rotation_delta_rad,
        )
        self.images: dict[int, ImageEvidence] = {}
        self.tags: dict[int, TagEvidence] = {}
        self.joints: dict[int, JointEvidence] = {}
        self.dynamic_transforms: dict[
            int, dict[tuple[str, str], TransformEvidence]
        ] = {}
        self.static_transforms: dict[tuple[str, str], TransformEvidence] = {}
        self._last_stream_stamp: dict[str, int] = {}
        self._last_tf_stamp: dict[tuple[str, str], int] = {}
        self._blocked_stamps: dict[int, tuple[str, ...]] = {}
        self._global_block_reasons: list[str] = []
        self._captured_stamps: set[int] = set()

    def block_stamp(self, stamp_ns: int, reason: str) -> None:
        if isinstance(stamp_ns, int) and stamp_ns > 0:
            current = list(self._blocked_stamps.get(stamp_ns, ()))
            current.append(str(reason))
            self._blocked_stamps[stamp_ns] = tuple(dict.fromkeys(current))

    def _strict_stream_stamp(self, stream: str, stamp_ns: int) -> int:
        stamp = _positive_stamp(stamp_ns, stream + " stamp")
        previous = self._last_stream_stamp.get(stream)
        if previous is not None and stamp <= previous:
            reason = f"{stream} stamp {stamp} is duplicate/non-monotonic after {previous}"
            self.block_stamp(stamp, reason)
            raise CalibrationError(reason)
        self._last_stream_stamp[stream] = stamp
        return stamp

    def observe_image(self, *, stamp_ns: int, frame_id: str) -> None:
        stamp = self._strict_stream_stamp("image", stamp_ns)
        frame = _normalized_frame(frame_id, "image frame")
        if frame != self.expected_camera_frame:
            self.block_stamp(stamp, "image frame mismatch")
            raise CalibrationError("image frame does not match d405_optical_frame")
        self.images[stamp] = ImageEvidence(stamp, frame)

    def observe_tag(
        self,
        *,
        stamp_ns: int,
        frame_id: str,
        family: str,
        tag_id: int,
        hamming: int,
        decision_margin: float,
        tag_in_camera: Pose,
    ) -> None:
        stamp = self._strict_stream_stamp("tag", stamp_ns)
        frame = _normalized_frame(frame_id, "tag pose frame")
        margin = float(decision_margin)
        reasons = []
        if frame != self.expected_camera_frame:
            reasons.append("tag pose frame mismatch")
        if family != "tag36h11" or tag_id != 0:
            reasons.append("target must be tag36h11 ID 0")
        if isinstance(hamming, bool) or hamming != 0:
            reasons.append("tag detection hamming distance must be zero")
        if not math.isfinite(margin) or margin < 50.0:
            reasons.append("tag decision margin is below 50")
        try:
            tag_in_camera.matrix()
        except CalibrationError as exc:
            reasons.append(str(exc))
        if reasons:
            self.block_stamp(stamp, " | ".join(reasons))
            raise CalibrationError(" | ".join(reasons))
        self.tags[stamp] = TagEvidence(
            stamp, frame, family, tag_id, hamming, margin, tag_in_camera
        )

    def observe_joint_state(
        self,
        *,
        stamp_ns: int,
        names: Sequence[str],
        positions: Sequence[float],
        velocities: Sequence[float],
    ) -> None:
        stamp = self._strict_stream_stamp("joint_state", stamp_ns)
        joint_names = tuple(str(value) for value in names)
        try:
            joint_positions = tuple(float(value) for value in positions)
            joint_velocities = tuple(float(value) for value in velocities)
        except (TypeError, ValueError) as exc:
            self.block_stamp(stamp, "joint state contains nonnumeric values")
            raise CalibrationError("joint state contains nonnumeric values") from exc
        if (
            not joint_names
            or len(set(joint_names)) != len(joint_names)
            or len(joint_positions) != len(joint_names)
            or len(joint_velocities) != len(joint_names)
            or not all(math.isfinite(value) for value in joint_positions)
            or not all(math.isfinite(value) for value in joint_velocities)
        ):
            self.block_stamp(stamp, "joint state name/position/velocity is incomplete")
            raise CalibrationError(
                "joint state must have unique names and complete finite position/velocity"
            )
        if not set(self.required_joint_names) <= set(joint_names):
            self.block_stamp(stamp, "required joint state is missing")
            raise CalibrationError("joint state is missing a required arm joint")
        self.joints[stamp] = JointEvidence(
            stamp, joint_names, joint_positions, joint_velocities
        )

    def observe_transform(
        self,
        *,
        parent_frame: str,
        child_frame: str,
        pose: Pose,
        stamp_ns: int | None,
        is_static: bool = False,
    ) -> None:
        parent = _normalized_frame(parent_frame, "TF parent")
        child = _normalized_frame(child_frame, "TF child")
        if parent == child:
            raise CalibrationError("TF parent and child must differ")
        pose.matrix()
        edge = (parent, child)
        if is_static:
            if edge in self.static_transforms:
                reason = f"duplicate static TF edge {parent} -> {child}"
                self._global_block_reasons.append(reason)
                raise CalibrationError(reason)
            self.static_transforms[edge] = TransformEvidence(
                parent, child, pose, None, True
            )
            return

        stamp = _positive_stamp(stamp_ns, "dynamic TF stamp")
        previous = self._last_tf_stamp.get(edge)
        if previous is not None and stamp <= previous:
            reason = (
                f"dynamic TF {parent} -> {child} stamp {stamp} is "
                f"duplicate/non-monotonic after {previous}"
            )
            self.block_stamp(stamp, reason)
            raise CalibrationError(reason)
        self._last_tf_stamp[edge] = stamp
        transforms = self.dynamic_transforms.setdefault(stamp, {})
        if edge in transforms:
            reason = f"duplicate dynamic TF edge {parent} -> {child} at {stamp}"
            self.block_stamp(stamp, reason)
            raise CalibrationError(reason)
        transforms[edge] = TransformEvidence(parent, child, pose, stamp, False)

    def _resolve_gripper_in_base(self, stamp_ns: int) -> ResolvedTransform:
        dynamic_snapshot = self.dynamic_transforms.get(stamp_ns, {})
        if not dynamic_snapshot:
            raise CalibrationError(
                f"no dynamic TF snapshot at bracket stamp {stamp_ns}"
            )
        edges = [*self.static_transforms.values(), *dynamic_snapshot.values()]
        adjacency: dict[
            str, list[tuple[str, np.ndarray, bool, TransformEvidence]]
        ] = {}
        for edge in edges:
            matrix = edge.pose.matrix()
            adjacency.setdefault(edge.parent_frame, []).append(
                (edge.child_frame, matrix, False, edge)
            )
            adjacency.setdefault(edge.child_frame, []).append(
                (edge.parent_frame, np.linalg.inv(matrix), True, edge)
            )

        solutions: list[
            tuple[np.ndarray, tuple[tuple[TransformEvidence, bool], ...]]
        ] = []
        stack = [(
            self.base_frame,
            np.eye(4),
            False,
            (self.base_frame,),
            (),
        )]
        while stack:
            frame, base_from_frame, used_dynamic, visited, path = stack.pop()
            if frame == self.gripper_frame:
                if used_dynamic:
                    solutions.append((base_from_frame, path))
                continue
            for neighbor, frame_from_neighbor, inverted, edge in adjacency.get(
                frame, []
            ):
                if neighbor in visited:
                    continue
                stack.append((
                    neighbor,
                    base_from_frame @ frame_from_neighbor,
                    used_dynamic or not edge.is_static,
                    (*visited, neighbor),
                    (*path, (edge, inverted)),
                ))
        if not solutions:
            raise CalibrationError(
                "no base_link <- gripper_tip path using bracket-stamp dynamic TF"
            )
        if len(solutions) != 1:
            raise CalibrationError("TF graph has multiple base-to-gripper paths")
        matrix, path = solutions[0]
        return ResolvedTransform(_pose_from_matrix(matrix), stamp_ns, path)

    @staticmethod
    def _strict_window(
        stamps: Sequence[int],
        image_stamp_ns: int,
        label: str,
        min_samples_per_side: int,
    ) -> tuple[int, ...]:
        ordered = sorted(stamps)
        index = bisect_left(ordered, image_stamp_ns)
        after_index = index + (
            1 if index < len(ordered) and ordered[index] == image_stamp_ns else 0
        )
        before = ordered[:index][-min_samples_per_side:]
        after = ordered[after_index:][:min_samples_per_side]
        if len(before) != min_samples_per_side \
                or len(after) != min_samples_per_side:
            raise CalibrationError(
                f"missing {min_samples_per_side} strict {label} samples on each "
                "side of image timestamp"
            )
        if not all(value < image_stamp_ns for value in before) \
                or not all(value > image_stamp_ns for value in after):
            raise CalibrationError(f"invalid strict {label} timestamp window")
        return (*before, *after)

    def _check_bracket_gap(
        self, before: int, image_stamp_ns: int, after: int, label: str
    ) -> None:
        before_gap = image_stamp_ns - before
        after_gap = after - image_stamp_ns
        if max(before_gap, after_gap) > self.limits.max_bracket_gap_ns:
            raise CalibrationError(
                f"{label} bracket gap exceeds "
                f"{self.limits.max_bracket_gap_ns / NSEC_PER_SEC:.6f}s: "
                f"before={before_gap / NSEC_PER_SEC:.6f}s "
                f"after={after_gap / NSEC_PER_SEC:.6f}s"
            )

    def _joint_stationarity(
        self, window: Sequence[JointEvidence]
    ) -> tuple[float, float, float]:
        max_delta, max_velocity, velocity_span_estimate = joint_window_metrics(
            window, self.required_joint_names
        )
        if max_velocity > self.limits.stationary_velocity_limit_rad_s:
            raise CalibrationError(
                f"arm observed velocity {max_velocity:.9f} rad/s exceeds "
                f"{self.limits.stationary_velocity_limit_rad_s:.9f} rad/s"
            )
        if max_delta > self.limits.max_joint_delta_rad:
            raise CalibrationError(
                f"joint bracket delta {max_delta:.9f} rad exceeds "
                f"{self.limits.max_joint_delta_rad:.9f} rad"
            )
        return max_delta, max_velocity, velocity_span_estimate

    def _tf_stationarity(
        self, window: Sequence[ResolvedTransform]
    ) -> tuple[float, float]:
        translation_delta, rotation_delta = pose_window_metrics(
            tuple(item.pose for item in window)
        )
        if translation_delta > self.limits.max_tf_translation_delta_m:
            raise CalibrationError(
                f"TF bracket translation delta {translation_delta:.9f} m exceeds "
                f"{self.limits.max_tf_translation_delta_m:.9f} m"
            )
        if rotation_delta > self.limits.max_tf_rotation_delta_rad:
            raise CalibrationError(
                f"TF bracket rotation delta {rotation_delta:.9f} rad exceeds "
                f"{self.limits.max_tf_rotation_delta_rad:.9f} rad"
            )
        return translation_delta, rotation_delta

    def capture(
        self, *, kind: str, image_stamp_ns: int, now_ns: int
    ) -> CapturedSample:
        if kind not in {"training", "holdout"}:
            raise CalibrationError("capture kind must be explicit training or holdout")
        stamp = _positive_stamp(image_stamp_ns, "capture image stamp")
        now = _positive_stamp(now_ns, "collector clock")
        if self._global_block_reasons:
            raise CalibrationError(
                "collector is globally blocked: "
                + " | ".join(dict.fromkeys(self._global_block_reasons))
            )
        if stamp in self._blocked_stamps:
            raise CalibrationError(
                "capture stamp is blocked: " + " | ".join(self._blocked_stamps[stamp])
            )
        if stamp in self._captured_stamps:
            raise CalibrationError("image timestamp was already captured")
        age_ns = now - stamp
        if age_ns < 0:
            raise CalibrationError("image timestamp is in the future")
        if age_ns > self.max_data_age_ns:
            raise CalibrationError(
                f"image/TF evidence is stale by {age_ns / NSEC_PER_SEC:.6f}s"
            )
        image = self.images.get(stamp)
        tag = self.tags.get(stamp)
        missing = [
            name for name, evidence in (
                ("image", image), ("tag pose", tag)
            ) if evidence is None
        ]
        if missing:
            raise CalibrationError(
                "missing exact-stamp evidence: " + ", ".join(missing)
            )
        assert image is not None and tag is not None
        if image.stamp_ns != tag.stamp_ns:
            raise CalibrationError("internal evidence stamp mismatch")
        joint_stamps = self._strict_window(
            tuple(self.joints),
            stamp,
            "JointState",
            self.limits.min_samples_per_side,
        )
        tf_stamps = self._strict_window(
            tuple(self.dynamic_transforms),
            stamp,
            "TF",
            self.limits.min_samples_per_side,
        )
        if joint_stamps[-1] > now or tf_stamps[-1] > now:
            raise CalibrationError("stationary bracket evidence is in the future")
        side_count = self.limits.min_samples_per_side
        blocked_bracket_stamps = {
            value: self._blocked_stamps[value]
            for value in (*joint_stamps, *tf_stamps)
            if value in self._blocked_stamps
        }
        if blocked_bracket_stamps:
            raise CalibrationError(
                f"stationary bracket includes blocked evidence: "
                f"{blocked_bracket_stamps}"
            )
        self._check_bracket_gap(
            joint_stamps[0], stamp, joint_stamps[-1], "JointState window"
        )
        self._check_bracket_gap(
            tf_stamps[0], stamp, tf_stamps[-1], "TF window"
        )
        joint_window = tuple(self.joints[value] for value in joint_stamps)
        joint_before = joint_window[side_count - 1]
        joint_after = joint_window[side_count]
        max_joint_delta, max_velocity, velocity_span_estimate = (
            self._joint_stationarity(joint_window)
        )
        tf_window = tuple(
            self._resolve_gripper_in_base(value) for value in tf_stamps
        )
        tf_before = tf_window[side_count - 1]
        tf_after = tf_window[side_count]
        translation_delta, rotation_delta = self._tf_stationarity(
            tf_window
        )
        max_joint_interval = max_sample_interval_ns(joint_stamps, stamp)
        max_tf_interval = max_sample_interval_ns(tf_stamps, stamp)
        if max(max_joint_interval, max_tf_interval) \
                > self.limits.max_sample_interval_ns:
            raise CalibrationError(
                "observation sample interval exceeds "
                f"{self.limits.max_sample_interval_ns / NSEC_PER_SEC:.6f}s: "
                f"joint={max_joint_interval / NSEC_PER_SEC:.6f}s "
                f"tf={max_tf_interval / NSEC_PER_SEC:.6f}s"
            )
        sample = Sample(
            kind=kind,
            image_stamp_ns=stamp,
            gripper_in_base=tf_before.pose,
            tag_in_camera=tag.tag_in_camera,
            decision_margin=tag.decision_margin,
        )
        provenance = SampleProvenance(
            image=image,
            tag=tag,
            joint_before=joint_before,
            joint_after=joint_after,
            joint_window=joint_window,
            tf_before=tf_before,
            tf_after=tf_after,
            tf_window=tf_window,
            required_joint_names=self.required_joint_names,
            limits=self.limits,
            max_joint_delta_rad=max_joint_delta,
            max_observed_velocity_rad_s=max_velocity,
            velocity_span_estimate_rad=velocity_span_estimate,
            tf_translation_delta_m=translation_delta,
            tf_rotation_delta_rad=rotation_delta,
            max_joint_sample_interval_ns=max_joint_interval,
            max_tf_sample_interval_ns=max_tf_interval,
        )
        self._captured_stamps.add(stamp)
        return CapturedSample(sample, provenance)
