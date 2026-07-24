"""Multi-method hand-eye solve with explicit training/holdout validation."""

from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np

from .model import (
    CalibrationError,
    Sample,
    Session,
    matrix_to_quaternion,
    rotation_angle,
    validate_diversity,
)


@dataclass(frozen=True)
class MethodResult:
    name: str
    camera_in_gripper: np.ndarray


@dataclass(frozen=True)
class SolveResult:
    method: str
    camera_in_gripper: np.ndarray
    method_inliers: tuple[str, ...]
    method_translation_spread_m: float
    method_rotation_spread_deg: float
    diversity: dict[str, float]
    train_metrics: dict[str, float]
    holdout_metrics: dict[str, float]


METHODS = (
    ("TSAI", cv2.CALIB_HAND_EYE_TSAI),
    ("PARK", cv2.CALIB_HAND_EYE_PARK),
    ("HORAUD", cv2.CALIB_HAND_EYE_HORAUD),
    ("ANDREFF", cv2.CALIB_HAND_EYE_ANDREFF),
    ("DANIILIDIS", cv2.CALIB_HAND_EYE_DANIILIDIS),
)


def _solve_method(samples: tuple[Sample, ...], name: str, code: int) -> MethodResult:
    gripper = [sample.gripper_in_base.matrix() for sample in samples]
    target = [sample.tag_in_camera.matrix() for sample in samples]
    try:
        rotation, translation = cv2.calibrateHandEye(
            R_gripper2base=[pose[:3, :3] for pose in gripper],
            t_gripper2base=[pose[:3, 3].reshape(3, 1) for pose in gripper],
            R_target2cam=[pose[:3, :3] for pose in target],
            t_target2cam=[pose[:3, 3].reshape(3, 1) for pose in target],
            method=code,
        )
    except cv2.error as exc:
        raise CalibrationError(f"{name} solve failed: {exc}") from exc
    output = np.eye(4, dtype=np.float64)
    output[:3, :3] = np.asarray(rotation, dtype=np.float64)
    output[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(output)) or abs(np.linalg.det(output[:3, :3]) - 1.0) > 1e-3:
        raise CalibrationError(f"{name} returned an invalid rigid transform")
    return MethodResult(name, output)


def _select_consensus(results: tuple[MethodResult, ...]) -> tuple[MethodResult, tuple[MethodResult, ...], float, float]:
    if len(results) < 3:
        raise CalibrationError("fewer than three hand-eye methods succeeded")
    scores = []
    for candidate in results:
        distances = []
        for other in results:
            distances.append(
                float(np.linalg.norm(candidate.camera_in_gripper[:3, 3] - other.camera_in_gripper[:3, 3]))
                + rotation_angle(candidate.camera_in_gripper[:3, :3].T @ other.camera_in_gripper[:3, :3]) * 0.1
            )
        scores.append((float(np.median(distances)), candidate))
    medoid = min(scores, key=lambda item: item[0])[1]
    inliers = tuple(result for result in results if (
        np.linalg.norm(result.camera_in_gripper[:3, 3] - medoid.camera_in_gripper[:3, 3]) <= 0.015
        and rotation_angle(result.camera_in_gripper[:3, :3].T @ medoid.camera_in_gripper[:3, :3]) <= math.radians(3.0)
    ))
    if len(inliers) < 3:
        raise CalibrationError("hand-eye methods do not have a three-method consensus")
    translation_spread = max(
        float(np.linalg.norm(left.camera_in_gripper[:3, 3] - right.camera_in_gripper[:3, 3]))
        for index, left in enumerate(inliers) for right in inliers[index + 1:]
    )
    rotation_spread = max(
        rotation_angle(left.camera_in_gripper[:3, :3].T @ right.camera_in_gripper[:3, :3])
        for index, left in enumerate(inliers) for right in inliers[index + 1:]
    )
    return medoid, inliers, translation_spread, math.degrees(rotation_spread)


def _mean_rigid(transforms: list[np.ndarray]) -> np.ndarray:
    output = np.eye(4, dtype=np.float64)
    output[:3, 3] = np.mean([transform[:3, 3] for transform in transforms], axis=0)
    u, _, vt = np.linalg.svd(sum(transform[:3, :3] for transform in transforms))
    rotation = u @ vt
    if np.linalg.det(rotation) < 0.0:
        u[:, -1] *= -1.0
        rotation = u @ vt
    output[:3, :3] = rotation
    return output


def _target_transforms(samples: tuple[Sample, ...], camera_in_gripper: np.ndarray) -> list[np.ndarray]:
    return [
        sample.gripper_in_base.matrix() @ camera_in_gripper @ sample.tag_in_camera.matrix()
        for sample in samples
    ]


def _metrics(transforms: list[np.ndarray], reference: np.ndarray) -> dict[str, float]:
    translation = [float(np.linalg.norm(transform[:3, 3] - reference[:3, 3])) for transform in transforms]
    rotation = [math.degrees(rotation_angle(reference[:3, :3].T @ transform[:3, :3])) for transform in transforms]
    return {
        "count": len(transforms),
        "translation_rms_m": math.sqrt(sum(value * value for value in translation) / len(translation)),
        "translation_max_m": max(translation),
        "rotation_rms_deg": math.sqrt(sum(value * value for value in rotation) / len(rotation)),
        "rotation_max_deg": max(rotation),
    }


def solve_session(session: Session) -> SolveResult:
    training = session.training
    holdout = session.holdout
    if len(training) < 10 or len(holdout) < 3:
        raise CalibrationError("solve requires at least 10 training and 3 holdout samples")
    diversity = validate_diversity(training)
    method_results = []
    errors = []
    for name, code in METHODS:
        try:
            method_results.append(_solve_method(training, name, code))
        except CalibrationError as exc:
            errors.append(str(exc))
    if len(method_results) < 3:
        raise CalibrationError("too few solver methods succeeded: " + " | ".join(errors))
    medoid, inliers, translation_spread, rotation_spread = _select_consensus(tuple(method_results))
    training_targets = _target_transforms(training, medoid.camera_in_gripper)
    target_reference = _mean_rigid(training_targets)
    train_metrics = _metrics(training_targets, target_reference)
    holdout_metrics = _metrics(_target_transforms(holdout, medoid.camera_in_gripper), target_reference)
    if (
        train_metrics["translation_rms_m"] > 0.010
        or train_metrics["rotation_rms_deg"] > 2.0
        or holdout_metrics["translation_max_m"] > 0.020
        or holdout_metrics["rotation_max_deg"] > 3.0
    ):
        raise CalibrationError(
            "hand-eye residual/holdout thresholds failed: "
            f"train={train_metrics}, holdout={holdout_metrics}"
        )
    return SolveResult(
        method=medoid.name,
        camera_in_gripper=medoid.camera_in_gripper,
        method_inliers=tuple(result.name for result in inliers),
        method_translation_spread_m=translation_spread,
        method_rotation_spread_deg=rotation_spread,
        diversity=diversity,
        train_metrics=train_metrics,
        holdout_metrics=holdout_metrics,
    )


def result_document(session: Session, result: SolveResult) -> dict[str, object]:
    transform = result.camera_in_gripper
    return {
        "schema_version": 1,
        "status": "PROPOSED_UNAPPROVED",
        "calibration_ready": False,
        "session_id": session.session_id,
        "robot_id": session.robot_id,
        "d405_serial": session.d405_serial,
        "d405_firmware": session.d405_firmware,
        "dependency_calibration_ids": {
            "joint_zero": session.joint_zero_calibration_id,
            "tool_center_point": session.tool_center_point_calibration_id,
            "d405_intrinsics": session.d405_intrinsics_calibration_id,
        },
        "tag_family": "tag36h11",
        "tag_id": 0,
        "tag_black_edge_m": 0.040,
        "tag_registry_sha256": session.tag_registry_sha256,
        "urdf_sha256": session.urdf_sha256,
        "d405_hand_eye": {
            "parent": "gripper_tip",
            "child": "d405_optical_frame",
            "xyz_m": [float(value) for value in transform[:3, 3]],
            "quaternion_xyzw": list(matrix_to_quaternion(transform[:3, :3])),
        },
        "solver": {
            "selected_method": result.method,
            "method_inliers": list(result.method_inliers),
            "method_translation_spread_m": result.method_translation_spread_m,
            "method_rotation_spread_deg": result.method_rotation_spread_deg,
            "diversity": result.diversity,
            "training": result.train_metrics,
            "holdout": result.holdout_metrics,
        },
        "activation_note": (
            "Proposal only. Merge into the per-machine artifact only after "
            "physical review; this file never makes calibration READY."
        ),
    }
