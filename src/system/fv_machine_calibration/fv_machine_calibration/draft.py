"""Fail-closed creation of new unapproved session drafts."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import tempfile

import numpy as np

from .collector_model import (
    JointEvidence,
    joint_window_metrics,
    max_sample_interval_ns,
    pose_window_metrics,
)
from .model import CalibrationError, Pose, parse_session


DRAFT_SUFFIX = ".session-draft.json"


def _pose(value: object, label: str) -> Pose:
    if not isinstance(value, dict):
        raise CalibrationError(f"{label} must be an object")
    try:
        translation = tuple(float(item) for item in value["translation_m"])
        quaternion = tuple(float(item) for item in value["quaternion_xyzw"])
    except (KeyError, TypeError, ValueError) as exc:
        raise CalibrationError(f"{label} is incomplete") from exc
    if len(translation) != 3 or len(quaternion) != 4:
        raise CalibrationError(f"{label} has invalid dimensions")
    pose = Pose(translation, quaternion)
    pose.matrix()
    return pose


def _joint(value: object, label: str) -> JointEvidence:
    if not isinstance(value, dict):
        raise CalibrationError(f"{label} must be an object")
    try:
        stamp = int(value["stamp_ns"])
        names = tuple(str(item) for item in value["names"])
        positions = tuple(float(item) for item in value["positions_rad"])
        velocities = tuple(float(item) for item in value["velocities_rad_s"])
    except (KeyError, TypeError, ValueError) as exc:
        raise CalibrationError(f"{label} is incomplete") from exc
    if stamp <= 0 or not names or len(set(names)) != len(names) \
            or len(positions) != len(names) or len(velocities) != len(names) \
            or not all(math.isfinite(item) for item in (*positions, *velocities)):
        raise CalibrationError(f"{label} is invalid")
    return JointEvidence(stamp, names, positions, velocities)


def _resolved_tf_pose(value: object, label: str) -> Pose:
    if not isinstance(value, dict):
        raise CalibrationError(f"{label} must be an object")
    snapshot = _positive_int(value.get("snapshot_stamp_ns"), label + ".stamp")
    resolved = _pose(value.get("resolved_gripper_in_base"), label + ".resolved")
    path = value.get("contributing_path_edges")
    if not isinstance(path, list) or not path:
        raise CalibrationError(f"{label} has no contributing TF path")
    matrix = np.eye(4)
    current_frame = "base_link"
    used_dynamic = False
    for index, raw_edge in enumerate(path):
        if not isinstance(raw_edge, dict):
            raise CalibrationError(f"{label}.path[{index}] must be an object")
        parent = str(raw_edge.get("parent_frame") or "")
        child = str(raw_edge.get("child_frame") or "")
        inverted = raw_edge.get("traversed_inverse")
        is_static = raw_edge.get("is_static")
        if not parent or not child or not isinstance(inverted, bool) \
                or not isinstance(is_static, bool):
            raise CalibrationError(f"{label}.path[{index}] identity is invalid")
        source_stamp = raw_edge.get("source_stamp_ns")
        if (is_static and source_stamp is not None) \
                or (not is_static and source_stamp != snapshot):
            raise CalibrationError(f"{label}.path[{index}] stamp is inconsistent")
        edge_pose = _pose(
            raw_edge.get("pose_parent_from_child"),
            f"{label}.path[{index}].pose",
        )
        expected, next_frame = (child, parent) if inverted else (parent, child)
        if current_frame != expected:
            raise CalibrationError(f"{label}.path[{index}] is not contiguous")
        edge_matrix = edge_pose.matrix()
        matrix = matrix @ (np.linalg.inv(edge_matrix) if inverted else edge_matrix)
        current_frame = next_frame
        used_dynamic = used_dynamic or not is_static
    if current_frame != "gripper_tip" or not used_dynamic \
            or not np.allclose(matrix, resolved.matrix(), atol=1e-9):
        raise CalibrationError(f"{label} resolved pose differs from raw TF path")
    return resolved


def _recorded_number(value: object, label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise CalibrationError(f"{label} is not numeric") from exc
    if not math.isfinite(number) or number < 0.0:
        raise CalibrationError(f"{label} is not finite and nonnegative")
    return number


def _positive_int(value: object, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise CalibrationError(f"{label} must be a positive integer")
    return value


def _validate_raw_metrics(sample: dict[str, object], contract: dict[str, object]) -> None:
    raw = sample.get("raw_evidence_provenance")
    limits = contract.get("limits")
    required = contract.get("required_joint_names")
    if not isinstance(raw, dict) or not isinstance(limits, dict) \
            or not isinstance(required, list) or not required:
        raise CalibrationError("session draft raw provenance contract is incomplete")
    required_names = tuple(str(item) for item in required)
    if any(not item for item in required_names) \
            or len(set(required_names)) != len(required_names):
        raise CalibrationError("required joint provenance is invalid")
    joint_raw = raw.get("joint_state_window")
    tf_raw = raw.get("tf_window")
    bounds = raw.get("observed_stationarity_bounds")
    if not isinstance(joint_raw, list) or not isinstance(tf_raw, list) \
            or not isinstance(bounds, dict):
        raise CalibrationError("session draft raw observation windows are incomplete")
    minimum = _positive_int(limits.get("min_samples_per_side"), "min samples")
    if minimum < 2 or len(joint_raw) != 2 * minimum \
            or len(tf_raw) != 2 * minimum:
        raise CalibrationError("session draft observation window size is invalid")
    joints = tuple(
        _joint(value, f"joint_state_window[{index}]")
        for index, value in enumerate(joint_raw)
    )
    poses = tuple(
        _resolved_tf_pose(value, f"tf_window[{index}]")
        for index, value in enumerate(tf_raw)
    )
    stamp = _positive_int(sample.get("image_stamp_ns"), "sample image timestamp")
    tf_stamps = tuple(
        _positive_int(
            value.get("snapshot_stamp_ns") if isinstance(value, dict) else None,
            f"tf_window[{index}].snapshot_stamp_ns",
        )
        for index, value in enumerate(tf_raw)
    )
    joint_stamps = tuple(item.stamp_ns for item in joints)
    if tuple(sorted(joint_stamps)) != joint_stamps \
            or tuple(sorted(tf_stamps)) != tf_stamps \
            or not all(value < stamp for value in joint_stamps[:minimum]) \
            or not all(value > stamp for value in joint_stamps[minimum:]) \
            or not all(value < stamp for value in tf_stamps[:minimum]) \
            or not all(value > stamp for value in tf_stamps[minimum:]):
        raise CalibrationError("raw observation windows do not strictly bracket image")
    max_bracket_gap = _positive_int(
        limits.get("max_bracket_gap_ns"), "max bracket gap"
    )
    if max(
        stamp - joint_stamps[0],
        joint_stamps[-1] - stamp,
        stamp - tf_stamps[0],
        tf_stamps[-1] - stamp,
    ) > max_bracket_gap:
        raise CalibrationError("raw observation window exceeds max bracket gap")
    recomputed = (
        *joint_window_metrics(joints, required_names),
        *pose_window_metrics(poses),
        max_sample_interval_ns(joint_stamps, stamp),
        max_sample_interval_ns(tf_stamps, stamp),
    )
    fields = (
        "max_joint_delta_rad",
        "max_observed_velocity_rad_s",
        "velocity_times_bracket_span_estimate_rad",
        "tf_translation_delta_m",
        "tf_rotation_delta_rad",
        "max_joint_sample_interval_ns",
        "max_tf_sample_interval_ns",
    )
    recorded = tuple(_recorded_number(bounds.get(name), name) for name in fields)
    if any(
        not math.isclose(float(actual), saved, abs_tol=1e-12)
        for actual, saved in zip(recomputed, recorded)
    ):
        raise CalibrationError("saved max metrics differ from raw observation windows")
    limit_fields = (
        "max_joint_delta_rad",
        "stationary_velocity_limit_rad_s",
        None,
        "max_tf_translation_delta_m",
        "max_tf_rotation_delta_rad",
        "max_sample_interval_ns",
        "max_sample_interval_ns",
    )
    for observed, limit_name in zip(recomputed, limit_fields):
        if limit_name is not None and float(observed) > _recorded_number(
            limits.get(limit_name), limit_name
        ):
            raise CalibrationError(f"raw metric exceeds {limit_name}")
    if bounds.get("interpretation") != (
        "bounded_observed_window_not_hidden_excursion_guarantee"
    ):
        raise CalibrationError("raw evidence interpretation is missing")


def _validate_raw_provenance(document: dict[str, object]) -> None:
    contract = document.get("collector_contract")
    samples = document.get("samples")
    if not isinstance(contract, dict) \
            or contract.get("stationary_bracket_contract_version") != 1:
        raise CalibrationError("stationary bracket contract version must be 1")
    required_contract = {
        "stamp_rewrite": "forbidden",
        "tf_interpolation": "forbidden",
        "tf_latest_lookup": "forbidden",
        "tf_pose_selection": (
            "preceding_snapshot_verified_by_following_snapshot"
        ),
    }
    if any(contract.get(key) != value for key, value in required_contract.items()):
        raise CalibrationError("stationary bracket contract was weakened")
    topics = document.get("source_topics")
    topic_keys = {
        "image", "detections", "tag_poses", "joint_states", "tf", "tf_static"
    }
    if not isinstance(topics, dict) or set(topics) != topic_keys \
            or len(set(topics.values())) != len(topics) \
            or any(
                not isinstance(value, str)
                or not value.startswith("/")
                or value.endswith("/")
                or "//" in value
                for value in topics.values()
            ):
        raise CalibrationError("session draft source topics are invalid")
    if not isinstance(samples, list):
        raise CalibrationError("session draft samples must be a list")
    for sample in samples:
        if not isinstance(sample, dict):
            raise CalibrationError("session draft sample must be an object")
        _validate_raw_metrics(sample, contract)


def validate_session_draft(document: object) -> None:
    if not isinstance(document, dict):
        raise CalibrationError("session draft must be an object")
    if document.get("status") != "SESSION_DRAFT_UNAPPROVED":
        raise CalibrationError("output status must be SESSION_DRAFT_UNAPPROVED")
    if document.get("calibration_ready") is not False:
        raise CalibrationError("session draft calibration_ready must be false")
    parse_session(document, require_complete=True)
    _validate_raw_provenance(document)


def write_new_session_draft(path: Path, document: dict[str, object]) -> None:
    """Atomically create one mode-0600 draft without replacing any path."""
    validate_session_draft(document)
    requested = Path(path)
    if not requested.is_absolute():
        raise CalibrationError("session draft output path must be absolute")
    if not requested.name.endswith(DRAFT_SUFFIX):
        raise CalibrationError(f"session draft output must end with {DRAFT_SUFFIX}")
    parent = requested.parent
    if not parent.is_dir() or parent.is_symlink():
        raise CalibrationError("session draft parent must be an existing real directory")
    try:
        resolved_parent = parent.resolve(strict=True)
    except OSError as exc:
        raise CalibrationError("cannot resolve session draft parent") from exc
    if resolved_parent != parent:
        raise CalibrationError("session draft parent path must not traverse symlinks")
    if requested.exists() or requested.is_symlink():
        raise CalibrationError(f"refusing to overwrite existing session draft: {requested}")

    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{requested.name}.", dir=str(parent)
    )
    try:
        os.fchmod(descriptor, 0o600)
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(document, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        if requested.exists() or requested.is_symlink():
            raise CalibrationError(
                f"refusing to overwrite existing session draft: {requested}"
            )
        try:
            os.link(temporary_name, requested)
        except FileExistsError as exc:
            raise CalibrationError(
                f"refusing to overwrite existing session draft: {requested}"
            ) from exc
        os.unlink(temporary_name)
        directory_descriptor = os.open(str(parent), os.O_DIRECTORY)
        try:
            os.fsync(directory_descriptor)
        finally:
            os.close(directory_descriptor)
    except Exception:
        try:
            os.unlink(temporary_name)
        except OSError:
            pass
        raise
