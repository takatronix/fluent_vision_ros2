"""Strict, ROS-independent hand-eye session model."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
from typing import Iterable

import numpy as np


class CalibrationError(ValueError):
    """Raised when calibration evidence is incomplete or ambiguous."""


@dataclass(frozen=True)
class Pose:
    translation_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]

    def matrix(self) -> np.ndarray:
        qx, qy, qz, qw = self.quaternion_xyzw
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if abs(norm - 1.0) > 1e-3:
            raise CalibrationError(f"quaternion norm {norm:.9f} is not one")
        rotation = np.array([
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ], dtype=np.float64)
        output = np.eye(4, dtype=np.float64)
        output[:3, :3] = rotation
        output[:3, 3] = self.translation_m
        return output


@dataclass(frozen=True)
class Sample:
    kind: str
    image_stamp_ns: int
    gripper_in_base: Pose
    tag_in_camera: Pose
    decision_margin: float


@dataclass(frozen=True)
class Session:
    session_id: str
    robot_id: str
    d405_serial: str
    d405_firmware: str
    joint_zero_calibration_id: str
    tool_center_point_calibration_id: str
    d405_intrinsics_calibration_id: str
    tag_registry_sha256: str
    urdf_sha256: str
    samples: tuple[Sample, ...]

    @property
    def training(self) -> tuple[Sample, ...]:
        return tuple(sample for sample in self.samples if sample.kind == "training")

    @property
    def holdout(self) -> tuple[Sample, ...]:
        return tuple(sample for sample in self.samples if sample.kind == "holdout")


def _finite_vector(value: object, size: int, label: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise CalibrationError(f"{label} must contain exactly {size} values")
    converted = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in converted):
        raise CalibrationError(f"{label} contains a non-finite value")
    return converted


def _pose(value: object, label: str) -> Pose:
    if not isinstance(value, dict):
        raise CalibrationError(f"{label} must be an object")
    pose = Pose(
        _finite_vector(value.get("translation_m"), 3, label + ".translation_m"),
        _finite_vector(value.get("quaternion_xyzw"), 4, label + ".quaternion_xyzw"),
    )
    pose.matrix()
    return pose


def _sha256(value: object, label: str) -> str:
    text = str(value or "")
    if not re.fullmatch(r"[0-9a-f]{64}", text):
        raise CalibrationError(f"{label} must be a lowercase SHA-256")
    return text


def _identity(value: object, label: str) -> str:
    text = str(value or "")
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.:-]*", text):
        raise CalibrationError(f"invalid {label}")
    if text.upper() in {"UNKNOWN", "UNIDENTIFIED", "UNAPPROVED"}:
        raise CalibrationError(f"{label} must identify reviewed evidence")
    return text


def parse_session(document: object, *, require_complete: bool = True) -> Session:
    if not isinstance(document, dict) or document.get("schema_version") != 1:
        raise CalibrationError("session schema_version must be 1")
    if document.get("tag_family") != "tag36h11":
        raise CalibrationError("hand-eye family must be tag36h11")
    if document.get("tag_id") != 0:
        raise CalibrationError("hand-eye target must be ID 0")
    if abs(float(document.get("tag_black_edge_m", 0.0)) - 0.040) > 1e-9:
        raise CalibrationError("ID 0 black edge must be exactly 0.040 m")
    frames = document.get("frames")
    if frames != {
        "base": "base_link",
        "gripper": "gripper_tip",
        "camera": "d405_optical_frame",
    }:
        raise CalibrationError("session frame contract is not exact")
    robot_id = str(document.get("robot_id") or "")
    serial = str(document.get("d405_serial") or "")
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", robot_id):
        raise CalibrationError("invalid robot_id")
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", serial):
        raise CalibrationError("invalid D405 serial")

    raw_samples = document.get("samples")
    if not isinstance(raw_samples, list):
        raise CalibrationError("samples must be a list")
    samples: list[Sample] = []
    stamps: set[int] = set()
    for index, raw in enumerate(raw_samples):
        if not isinstance(raw, dict):
            raise CalibrationError(f"samples[{index}] must be an object")
        kind = raw.get("kind")
        if kind not in {"training", "holdout"}:
            raise CalibrationError(f"samples[{index}].kind is invalid")
        stamp = raw.get("image_stamp_ns")
        if isinstance(stamp, bool) or not isinstance(stamp, int) or stamp <= 0:
            raise CalibrationError(f"samples[{index}].image_stamp_ns is invalid")
        if stamp in stamps:
            raise CalibrationError("duplicate image timestamp in session")
        stamps.add(stamp)
        margin = float(raw.get("decision_margin", 0.0))
        if not math.isfinite(margin) or margin < 50.0:
            raise CalibrationError(f"samples[{index}] decision margin is below 50")
        samples.append(Sample(
            kind=kind,
            image_stamp_ns=stamp,
            gripper_in_base=_pose(raw.get("gripper_in_base"), f"samples[{index}].gripper_in_base"),
            tag_in_camera=_pose(raw.get("tag_in_camera"), f"samples[{index}].tag_in_camera"),
            decision_margin=margin,
        ))
    session = Session(
        session_id=_identity(document.get("session_id"), "session_id"),
        robot_id=robot_id,
        d405_serial=serial,
        d405_firmware=_identity(document.get("d405_firmware"), "d405_firmware"),
        joint_zero_calibration_id=_identity(
            document.get("joint_zero_calibration_id"),
            "joint_zero_calibration_id",
        ),
        tool_center_point_calibration_id=_identity(
            document.get("tool_center_point_calibration_id"),
            "tool_center_point_calibration_id",
        ),
        d405_intrinsics_calibration_id=_identity(
            document.get("d405_intrinsics_calibration_id"),
            "d405_intrinsics_calibration_id",
        ),
        tag_registry_sha256=_sha256(document.get("tag_registry_sha256"), "tag_registry_sha256"),
        urdf_sha256=_sha256(document.get("urdf_sha256"), "urdf_sha256"),
        samples=tuple(samples),
    )
    if require_complete and (len(session.training) < 10 or len(session.holdout) < 3):
        raise CalibrationError(
            f"need at least 10 training and 3 holdout samples; have "
            f"{len(session.training)} and {len(session.holdout)}"
        )
    return session


def load_session(path: Path, *, require_complete: bool = True) -> Session:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CalibrationError(f"cannot read session {path}: {exc}") from exc
    return parse_session(document, require_complete=require_complete)


def rotation_angle(rotation: np.ndarray) -> float:
    cosine = min(1.0, max(-1.0, (float(np.trace(rotation)) - 1.0) / 2.0))
    return math.acos(cosine)


def validate_diversity(samples: Iterable[Sample]) -> dict[str, float]:
    samples = tuple(samples)
    if len(samples) < 3:
        raise CalibrationError("at least three samples are required for diversity")
    poses = [sample.gripper_in_base.matrix() for sample in samples]
    max_translation = 0.0
    max_rotation = 0.0
    for index, left in enumerate(poses):
        for right in poses[index + 1:]:
            max_translation = max(
                max_translation,
                float(np.linalg.norm(left[:3, 3] - right[:3, 3])),
            )
            max_rotation = max(
                max_rotation,
                rotation_angle(left[:3, :3].T @ right[:3, :3]),
            )
    if max_translation < 0.05:
        raise CalibrationError(f"translation diversity {max_translation:.6f} m is below 0.05 m")
    if max_rotation < math.radians(30.0):
        raise CalibrationError(
            f"rotation diversity {math.degrees(max_rotation):.3f} deg is below 30 deg"
        )
    return {
        "max_translation_m": max_translation,
        "max_rotation_deg": math.degrees(max_rotation),
    }


def matrix_to_quaternion(rotation: np.ndarray) -> tuple[float, float, float, float]:
    matrix = np.asarray(rotation, dtype=np.float64)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        root = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * root
        qx = (matrix[2, 1] - matrix[1, 2]) / root
        qy = (matrix[0, 2] - matrix[2, 0]) / root
        qz = (matrix[1, 0] - matrix[0, 1]) / root
    else:
        axis = int(np.argmax(np.diag(matrix)))
        if axis == 0:
            root = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            qx, qy, qz, qw = 0.25 * root, (matrix[0, 1] + matrix[1, 0]) / root, (matrix[0, 2] + matrix[2, 0]) / root, (matrix[2, 1] - matrix[1, 2]) / root
        elif axis == 1:
            root = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            qx, qy, qz, qw = (matrix[0, 1] + matrix[1, 0]) / root, 0.25 * root, (matrix[1, 2] + matrix[2, 1]) / root, (matrix[0, 2] - matrix[2, 0]) / root
        else:
            root = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            qx, qy, qz, qw = (matrix[0, 2] + matrix[2, 0]) / root, (matrix[1, 2] + matrix[2, 1]) / root, 0.25 * root, (matrix[1, 0] - matrix[0, 1]) / root
    quaternion = np.array([qx, qy, qz, qw], dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    if quaternion[3] < 0.0:
        quaternion *= -1.0
    return tuple(float(value) for value in quaternion)
