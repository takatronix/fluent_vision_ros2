import hashlib
import json
import math

import cv2
import numpy as np
import pytest

from fv_machine_calibration.cli import main as cli_main, write_new_atomic
from fv_machine_calibration.model import (
    CalibrationError,
    parse_session,
    validate_diversity,
)
from fv_machine_calibration.solver import result_document, solve_session


HASH_A = "a" * 64
HASH_B = "b" * 64


def _pose_document(transform):
    from fv_machine_calibration.model import matrix_to_quaternion

    return {
        "translation_m": transform[:3, 3].tolist(),
        "quaternion_xyzw": list(matrix_to_quaternion(transform[:3, :3])),
    }


def _transform(axis, angle_deg, translation):
    axis = np.asarray(axis, dtype=np.float64)
    axis /= np.linalg.norm(axis)
    rotation, _ = cv2.Rodrigues(axis * math.radians(angle_deg))
    output = np.eye(4)
    output[:3, :3] = rotation
    output[:3, 3] = translation
    return output


KNOWN_CAMERA_IN_GRIPPER = _transform((1, 2, 1), 37.0, (0.03, -0.04, 0.08))
FIXED_TAG_IN_BASE = _transform((0, 0, 1), 12.0, (0.45, -0.20, 0.15))


def _document():
    poses = [
        _transform((1, 0, 0), -35 + index * 7, (
            0.20 + 0.025 * (index % 5),
            -0.25 + 0.03 * ((index * 2) % 5),
            0.25 + 0.02 * ((index * 3) % 4),
        )) @ _transform((0, 0, 1), index * 13, (0, 0, 0))
        for index in range(15)
    ]
    samples = []
    for index, gripper_in_base in enumerate(poses):
        tag_in_camera = np.linalg.inv(gripper_in_base @ KNOWN_CAMERA_IN_GRIPPER) @ FIXED_TAG_IN_BASE
        samples.append({
            "kind": "training" if index < 12 else "holdout",
            "image_stamp_ns": 1_000_000_000 + index * 100_000_000,
            "gripper_in_base": _pose_document(gripper_in_base),
            "tag_in_camera": _pose_document(tag_in_camera),
            "decision_margin": 100.0,
        })
    return {
        "schema_version": 1,
        "session_id": "aspa-test-handeye-001",
        "robot_id": "aspa-test",
        "d405_serial": "D405TEST001",
        "d405_firmware": "5.15.1.55",
        "joint_zero_calibration_id": "joint-zero-001",
        "tool_center_point_calibration_id": "tcp-001",
        "d405_intrinsics_calibration_id": "d405-intrinsics-001",
        "tag_family": "tag36h11",
        "tag_id": 0,
        "tag_black_edge_m": 0.040,
        "tag_registry_sha256": HASH_A,
        "urdf_sha256": HASH_B,
        "frames": {
            "base": "base_link",
            "gripper": "gripper_tip",
            "camera": "d405_optical_frame",
        },
        "samples": samples,
    }


def test_synthetic_handeye_solve_recovers_known_transform():
    session = parse_session(_document())
    result = solve_session(session)
    assert len(result.method_inliers) >= 3
    assert np.linalg.norm(
        result.camera_in_gripper[:3, 3] - KNOWN_CAMERA_IN_GRIPPER[:3, 3]
    ) < 1e-5
    rotation_error = result.camera_in_gripper[:3, :3].T @ KNOWN_CAMERA_IN_GRIPPER[:3, :3]
    from fv_machine_calibration.model import rotation_angle

    assert math.degrees(rotation_angle(rotation_error)) < 1e-4
    assert result.holdout_metrics["translation_max_m"] < 1e-5
    proposal = result_document(session, result)
    assert proposal["status"] == "PROPOSED_UNAPPROVED"
    assert proposal["calibration_ready"] is False
    assert proposal["dependency_calibration_ids"]["joint_zero"] == "joint-zero-001"


@pytest.mark.parametrize(
    "field,value,match",
    [
        ("tag_id", 20, "ID 0"),
        ("tag_black_edge_m", 0.05, "0.040"),
        ("robot_id", "../aspa", "robot_id"),
        ("d405_serial", "UN IDENTIFIED", "serial"),
        ("joint_zero_calibration_id", "UNIDENTIFIED", "reviewed evidence"),
    ],
)
def test_session_identity_and_tag_contract_is_strict(field, value, match):
    document = _document()
    document[field] = value
    with pytest.raises(CalibrationError, match=match):
        parse_session(document)


def test_duplicate_image_stamp_is_rejected():
    document = _document()
    document["samples"][1]["image_stamp_ns"] = document["samples"][0]["image_stamp_ns"]
    with pytest.raises(CalibrationError, match="duplicate"):
        parse_session(document)


def test_low_decision_margin_is_rejected():
    document = _document()
    document["samples"][0]["decision_margin"] = 49.9
    with pytest.raises(CalibrationError, match="margin"):
        parse_session(document)


def test_translation_only_set_is_rejected_as_degenerate():
    session = parse_session(_document())
    first = session.training[0]
    from dataclasses import replace
    from fv_machine_calibration.model import Pose

    samples = tuple(replace(
        sample,
        gripper_in_base=Pose(
            sample.gripper_in_base.translation_m,
            first.gripper_in_base.quaternion_xyzw,
        ),
    ) for sample in session.training)
    with pytest.raises(CalibrationError, match="rotation diversity"):
        validate_diversity(samples)


def test_proposal_write_is_atomic_and_non_overwriting(tmp_path):
    target = tmp_path / "proposal.json"
    write_new_atomic(target, {"status": "PROPOSED_UNAPPROVED"})
    assert target.stat().st_mode & 0o777 == 0o600
    with pytest.raises(CalibrationError, match="overwrite"):
        write_new_atomic(target, {"status": "different"})


def test_cli_records_exact_source_session_hash(tmp_path):
    source = tmp_path / "session.json"
    destination = tmp_path / "proposal.json"
    source.write_text(json.dumps(_document(), sort_keys=True), encoding="utf-8")
    assert cli_main([str(source), "--output", str(destination)]) == 0
    proposal = json.loads(destination.read_text(encoding="utf-8"))
    assert proposal["source_session_sha256"] == hashlib.sha256(
        source.read_bytes()
    ).hexdigest()
