from dataclasses import replace
import ast
import json
import math
import os
from pathlib import Path

import pytest

from fv_machine_calibration.collector_model import (
    CollectorMetadata,
    StationaryBracketEvidenceSynchronizer,
    selected_robot_id,
    session_draft_document,
)
from fv_machine_calibration.draft import write_new_session_draft
from fv_machine_calibration.model import CalibrationError, Pose


HASH_A = "a" * 64
HASH_B = "b" * 64
IDENTITY = Pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
TAG_POSE = Pose((0.0, 0.0, 0.3), (0.0, 0.0, 0.0, 1.0))
STAMP = 10_000_000_000
BRACKET_NS = 10_000_000
JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
SOURCE_TOPICS = {
    "image": "/d405/image",
    "detections": "/apriltag/detections",
    "tag_poses": "/apriltag/poses",
    "joint_states": "/joint_states",
    "tf": "/tf",
    "tf_static": "/tf_static",
}


def metadata(**changes):
    values = {
        "session_id": "aspa1-d405-handeye-001",
        "robot_id": "aspa1",
        "d405_serial": "D405-123456",
        "d405_firmware": "5.15.1.55",
        "joint_zero_calibration_id": "joint-zero-20260722",
        "tool_center_point_calibration_id": "tcp-20260722",
        "d405_intrinsics_calibration_id": "d405-intrinsics-20260722",
        "tag_registry_sha256": HASH_A,
        "urdf_sha256": HASH_B,
    }
    values.update(changes)
    return CollectorMetadata(**values)


def synchronizer(**changes):
    values = {
        "required_joint_names": JOINTS,
        "max_data_age_s": 0.5,
        "stationary_velocity_limit_rad_s": 0.01,
    }
    values.update(changes)
    return StationaryBracketEvidenceSynchronizer(**values)


def add_sample_evidence(
    sync,
    stamp=STAMP,
    *,
    velocities=None,
    positions_before=None,
    positions_after=None,
    joint_position_window=None,
    tf_pose_before=None,
    tf_pose_after=None,
    bracket_ns=BRACKET_NS,
):
    sync.observe_image(stamp_ns=stamp, frame_id="d405_optical_frame")
    sync.observe_tag(
        stamp_ns=stamp,
        frame_id="d405_optical_frame",
        family="tag36h11",
        tag_id=0,
        hamming=0,
        decision_margin=100.0,
        tag_in_camera=TAG_POSE,
    )
    offsets = (-2 * bracket_ns, -bracket_ns, bracket_ns, 2 * bracket_ns)
    default_positions = (
        (0.0,) * 6 if positions_before is None else positions_before,
        (0.0,) * 6 if positions_before is None else positions_before,
        (0.0,) * 6 if positions_after is None else positions_after,
        (0.0,) * 6 if positions_after is None else positions_after,
    )
    selected_positions = default_positions \
        if joint_position_window is None else joint_position_window
    for offset, joint_positions in zip(offsets, selected_positions):
        sync.observe_joint_state(
            stamp_ns=stamp + offset,
            names=JOINTS,
            positions=joint_positions,
            velocities=(0.0,) * 6 if velocities is None else velocities,
        )
    sync.observe_transform(
        parent_frame="base_link",
        child_frame="arm_mount",
        pose=IDENTITY,
        stamp_ns=None,
        is_static=True,
    )
    for offset in (-2 * bracket_ns, -bracket_ns):
        sync.observe_transform(
            parent_frame="arm_mount",
            child_frame="gripper_tip",
            pose=tf_pose_before or Pose(
                (0.2, -0.1, 0.4), (0.0, 0.0, 0.0, 1.0)
            ),
            stamp_ns=stamp + offset,
        )
    for offset in (bracket_ns, 2 * bracket_ns):
        sync.observe_transform(
            parent_frame="arm_mount",
            child_frame="gripper_tip",
            pose=tf_pose_after or Pose(
                (0.2, -0.1, 0.4), (0.0, 0.0, 0.0, 1.0)
            ),
            stamp_ns=stamp + offset,
        )


def test_stationary_joint_and_tf_brackets_capture_without_interpolation():
    sync = synchronizer()
    add_sample_evidence(sync)
    capture = sync.capture(
        kind="training", image_stamp_ns=STAMP, now_ns=STAMP + 100_000_000
    )
    assert capture.kind == "training"
    assert capture.image_stamp_ns == STAMP
    assert capture.sample.gripper_in_base.translation_m == pytest.approx(
        (0.2, -0.1, 0.4)
    )
    assert capture.sample.tag_in_camera == TAG_POSE
    assert capture.provenance.joint_before.stamp_ns == STAMP - BRACKET_NS
    assert capture.provenance.joint_after.stamp_ns == STAMP + BRACKET_NS
    assert capture.provenance.tf_before.stamp_ns == STAMP - BRACKET_NS
    assert capture.provenance.tf_after.stamp_ns == STAMP + BRACKET_NS


def test_tf_before_and_after_must_be_within_explicit_gap():
    sync = synchronizer()
    add_sample_evidence(sync, bracket_ns=50_000_001)
    with pytest.raises(CalibrationError, match="bracket gap"):
        sync.capture(
            kind="training", image_stamp_ns=STAMP, now_ns=STAMP + 100_000_002
        )


def test_observation_window_sample_interval_is_bounded():
    sync = synchronizer(max_sample_interval_s=0.005)
    add_sample_evidence(sync)
    with pytest.raises(CalibrationError, match="sample interval"):
        sync.capture(
            kind="training", image_stamp_ns=STAMP, now_ns=STAMP + 20_000_000
        )


def test_static_only_gripper_path_is_not_timestamp_evidence():
    sync = synchronizer()
    sync.observe_image(stamp_ns=STAMP, frame_id="d405_optical_frame")
    sync.observe_tag(
        stamp_ns=STAMP,
        frame_id="d405_optical_frame",
        family="tag36h11",
        tag_id=0,
        hamming=0,
        decision_margin=100.0,
        tag_in_camera=TAG_POSE,
    )
    for stamp in (
        STAMP - 2 * BRACKET_NS,
        STAMP - BRACKET_NS,
        STAMP + BRACKET_NS,
        STAMP + 2 * BRACKET_NS,
    ):
        sync.observe_joint_state(
            stamp_ns=stamp,
            names=JOINTS,
            positions=(0.0,) * 6,
            velocities=(0.0,) * 6,
        )
    sync.observe_transform(
        parent_frame="base_link",
        child_frame="gripper_tip",
        pose=IDENTITY,
        stamp_ns=None,
        is_static=True,
    )
    with pytest.raises(CalibrationError, match="strict TF"):
        sync.capture(
            kind="training", image_stamp_ns=STAMP, now_ns=STAMP + 20_000_000
        )


def test_ambiguous_exact_tf_graph_is_rejected():
    sync = synchronizer()
    add_sample_evidence(sync)
    for stamp in (STAMP - BRACKET_NS, STAMP + BRACKET_NS):
        sync.observe_transform(
            parent_frame="base_link",
            child_frame="gripper_tip",
            pose=IDENTITY,
            stamp_ns=stamp,
        )
    with pytest.raises(CalibrationError, match="multiple"):
        sync.capture(
            kind="training", image_stamp_ns=STAMP, now_ns=STAMP + 20_000_000
        )


def test_duplicate_image_stamp_poisoned_and_cannot_be_captured():
    sync = synchronizer()
    add_sample_evidence(sync)
    with pytest.raises(CalibrationError, match="duplicate/non-monotonic"):
        sync.observe_image(stamp_ns=STAMP, frame_id="d405_optical_frame")
    with pytest.raises(CalibrationError, match="blocked"):
        sync.capture(
            kind="training", image_stamp_ns=STAMP, now_ns=STAMP + 20_000_000
        )


def test_duplicate_dynamic_tf_bracket_stamp_poisoned():
    sync = synchronizer()
    add_sample_evidence(sync)
    with pytest.raises(CalibrationError, match="duplicate/non-monotonic"):
        sync.observe_transform(
            parent_frame="arm_mount",
            child_frame="gripper_tip",
            pose=IDENTITY,
            stamp_ns=STAMP - BRACKET_NS,
        )
    with pytest.raises(CalibrationError, match="blocked evidence"):
        sync.capture(
            kind="training", image_stamp_ns=STAMP, now_ns=STAMP + 20_000_000
        )


def test_moving_arm_missing_velocity_and_stale_data_fail_closed():
    moving = synchronizer()
    add_sample_evidence(moving, velocities=(0.0, 0.0, 0.02, 0.0, 0.0, 0.0))
    with pytest.raises(CalibrationError, match="observed velocity"):
        moving.capture(
            kind="holdout", image_stamp_ns=STAMP, now_ns=STAMP + 20_000_000
        )

    incomplete = synchronizer()
    with pytest.raises(CalibrationError, match="complete finite"):
        incomplete.observe_joint_state(
            stamp_ns=STAMP,
            names=JOINTS,
            positions=(0.0,) * 6,
            velocities=(),
        )

    stale = synchronizer(max_data_age_s=0.1)
    add_sample_evidence(stale)
    with pytest.raises(CalibrationError, match="stale"):
        stale.capture(
            kind="training", image_stamp_ns=STAMP, now_ns=STAMP + 100_000_001
        )

    future = synchronizer()
    add_sample_evidence(future)
    with pytest.raises(CalibrationError, match="future"):
        future.capture(
            kind="training", image_stamp_ns=STAMP, now_ns=STAMP - 1
        )


@pytest.mark.parametrize(
    "hamming,margin,match",
    [
        (1, 100.0, "hamming"),
        (0, 49.9, "margin"),
    ],
)
def test_tag_quality_failure_poisoned(hamming, margin, match):
    sync = synchronizer()
    with pytest.raises(CalibrationError, match=match):
        sync.observe_tag(
            stamp_ns=STAMP,
            frame_id="d405_optical_frame",
            family="tag36h11",
            tag_id=0,
            hamming=hamming,
            decision_margin=margin,
            tag_in_camera=TAG_POSE,
        )
    assert STAMP in sync._blocked_stamps


def test_missing_exact_stream_and_implicit_kind_are_rejected():
    sync = synchronizer()
    sync.observe_image(stamp_ns=STAMP, frame_id="d405_optical_frame")
    with pytest.raises(CalibrationError, match="explicit training or holdout"):
        sync.capture(kind="auto", image_stamp_ns=STAMP, now_ns=STAMP + 1)
    with pytest.raises(CalibrationError, match="missing exact-stamp"):
        sync.capture(kind="training", image_stamp_ns=STAMP, now_ns=STAMP + 1)


def test_missing_strict_after_bracket_is_rejected_without_latest_fallback():
    sync = synchronizer()
    sync.observe_image(stamp_ns=STAMP, frame_id="d405_optical_frame")
    sync.observe_tag(
        stamp_ns=STAMP,
        frame_id="d405_optical_frame",
        family="tag36h11",
        tag_id=0,
        hamming=0,
        decision_margin=100.0,
        tag_in_camera=TAG_POSE,
    )
    sync.observe_joint_state(
        stamp_ns=STAMP - BRACKET_NS,
        names=JOINTS,
        positions=(0.0,) * 6,
        velocities=(0.0,) * 6,
    )
    sync.observe_transform(
        parent_frame="base_link",
        child_frame="gripper_tip",
        pose=IDENTITY,
        stamp_ns=STAMP - BRACKET_NS,
    )
    with pytest.raises(CalibrationError, match="strict JointState samples"):
        sync.capture(kind="training", image_stamp_ns=STAMP, now_ns=STAMP + 1)


def test_joint_and_tf_motion_between_brackets_fail_closed():
    joint_motion = synchronizer()
    add_sample_evidence(
        joint_motion,
        positions_after=(0.0, 0.0, 0.0011, 0.0, 0.0, 0.0),
    )
    with pytest.raises(CalibrationError, match="joint bracket delta"):
        joint_motion.capture(
            kind="training", image_stamp_ns=STAMP, now_ns=STAMP + 20_000_000
        )

    tf_translation = synchronizer()
    add_sample_evidence(
        tf_translation,
        tf_pose_after=Pose(
            (0.2006, -0.1, 0.4), (0.0, 0.0, 0.0, 1.0)
        ),
    )
    with pytest.raises(CalibrationError, match="TF bracket translation"):
        tf_translation.capture(
            kind="training", image_stamp_ns=STAMP, now_ns=STAMP + 20_000_000
        )

    tf_rotation = synchronizer()
    add_sample_evidence(
        tf_rotation,
        tf_pose_after=Pose(
            (0.2, -0.1, 0.4),
            (0.0, 0.0, math.sin(0.0011 / 2.0), math.cos(0.0011 / 2.0)),
        ),
    )
    with pytest.raises(CalibrationError, match="TF bracket rotation"):
        tf_rotation.capture(
            kind="training", image_stamp_ns=STAMP, now_ns=STAMP + 20_000_000
        )


def test_multi_sample_window_detects_excursion_and_records_velocity_estimate():
    zero = (0.0,) * 6
    excursion = (0.0, 0.0, 0.0011, 0.0, 0.0, 0.0)
    sampled_excursion = synchronizer()
    add_sample_evidence(
        sampled_excursion,
        joint_position_window=(excursion, zero, zero, zero),
    )
    with pytest.raises(CalibrationError, match="joint bracket delta"):
        sampled_excursion.capture(
            kind="training", image_stamp_ns=STAMP, now_ns=STAMP + 20_000_000
        )

    estimated = synchronizer()
    add_sample_evidence(estimated, velocities=(0.005,) * 6)
    capture = estimated.capture(
        kind="training", image_stamp_ns=STAMP, now_ns=STAMP + 20_000_000
    )
    assert capture.provenance.velocity_span_estimate_rad == pytest.approx(0.0002)
    assert capture.provenance.max_joint_sample_interval_ns == BRACKET_NS


def test_nonfinite_ros_pose_evidence_is_rejected_before_capture():
    sync = synchronizer()
    with pytest.raises(CalibrationError, match="non-finite"):
        sync.observe_tag(
            stamp_ns=STAMP,
            frame_id="d405_optical_frame",
            family="tag36h11",
            tag_id=0,
            hamming=0,
            decision_margin=100.0,
            tag_in_camera=Pose(
                (float("nan"), 0.0, 0.3), (0.0, 0.0, 0.0, 1.0)
            ),
        )


def test_robot_identity_uses_env_then_short_hostname_without_aspa1_default():
    assert selected_robot_id(
        {"ASPA_ROBOT_ID": "fieldbot-07"}, "ignored.example"
    ) == "fieldbot-07"
    assert selected_robot_id({}, "fieldbot-08.example") == "fieldbot-08"
    with pytest.raises(CalibrationError, match="fleet-safe"):
        selected_robot_id({"ASPA_ROBOT_ID": "../aspa1"}, "fieldbot")


@pytest.mark.parametrize(
    "change,match",
    [
        ({"d405_serial": ""}, "serial"),
        ({"d405_firmware": "UNKNOWN"}, "reviewed evidence"),
        ({"joint_zero_calibration_id": ""}, "joint_zero"),
        ({"tool_center_point_calibration_id": "UNAPPROVED"}, "reviewed evidence"),
        ({"d405_intrinsics_calibration_id": ""}, "intrinsics"),
        ({"tag_registry_sha256": "abcd"}, "SHA-256"),
        ({"urdf_sha256": ""}, "SHA-256"),
    ],
)
def test_all_machine_identity_dependencies_are_mandatory(change, match):
    with pytest.raises(CalibrationError, match=match):
        metadata(**change).validate()


def complete_samples():
    captures = []
    for index in range(13):
        stamp = STAMP + index * 100_000_000
        sync = synchronizer()
        add_sample_evidence(sync, stamp=stamp)
        captures.append(sync.capture(
            kind="training" if index < 10 else "holdout",
            image_stamp_ns=stamp,
            now_ns=stamp + 20_000_000,
        ))
    return tuple(captures)


def test_session_draft_is_solver_schema_compatible_and_labels_split():
    document = session_draft_document(
        metadata(), complete_samples(), source_topics=SOURCE_TOPICS
    )
    assert document["status"] == "SESSION_DRAFT_UNAPPROVED"
    assert document["calibration_ready"] is False
    assert [sample["kind"] for sample in document["samples"]].count("training") == 10
    assert [sample["kind"] for sample in document["samples"]].count("holdout") == 3
    contract = document["collector_contract"]
    assert contract["tf_interpolation"] == "forbidden"
    assert contract["tf_latest_lookup"] == "forbidden"
    assert contract["stamp_rewrite"] == "forbidden"
    provenance = document["samples"][0]["raw_evidence_provenance"]
    assert provenance["joint_state_before"]["stamp_ns"] == STAMP - BRACKET_NS
    assert provenance["joint_state_after"]["stamp_ns"] == STAMP + BRACKET_NS
    assert provenance["tf_before"]["snapshot_stamp_ns"] == STAMP - BRACKET_NS
    assert provenance["tf_after"]["snapshot_stamp_ns"] == STAMP + BRACKET_NS
    bounds = provenance["observed_stationarity_bounds"]
    assert bounds["max_joint_delta_rad"] == 0.0
    assert bounds["max_observed_velocity_rad_s"] == 0.0
    assert bounds["velocity_times_bracket_span_estimate_rad"] == 0.0
    assert bounds["tf_translation_delta_m"] == 0.0
    assert bounds["tf_rotation_delta_rad"] == 0.0
    assert bounds["max_joint_sample_interval_ns"] == BRACKET_NS
    assert bounds["max_tf_sample_interval_ns"] == BRACKET_NS
    assert len(provenance["joint_state_window"]) == 4
    assert len(provenance["tf_window"]) == 4


def test_incomplete_or_nonmonotonic_draft_is_rejected():
    with pytest.raises(CalibrationError, match="10 training and 3 holdout"):
        session_draft_document(
            metadata(), complete_samples()[:-1], source_topics=SOURCE_TOPICS
        )
    samples = list(complete_samples())
    samples[3], samples[4] = samples[4], samples[3]
    with pytest.raises(CalibrationError, match="strictly increasing"):
        session_draft_document(
            metadata(), samples, source_topics=SOURCE_TOPICS
        )


def test_draft_rejects_missing_or_mismatched_raw_provenance():
    captures = list(complete_samples())
    first = captures[0]
    captures[0] = replace(
        first,
        provenance=replace(
            first.provenance,
            image=replace(first.provenance.image, stamp_ns=STAMP + 1),
        ),
    )
    with pytest.raises(CalibrationError, match="provenance stamp"):
        session_draft_document(
            metadata(), captures, source_topics=SOURCE_TOPICS
        )
    with pytest.raises(CalibrationError, match="source_topics"):
        session_draft_document(
            metadata(), complete_samples(), source_topics={"image": "relative"}
        )


def test_draft_write_is_atomic_mode_0600_and_never_overwrites(tmp_path):
    document = session_draft_document(
        metadata(), complete_samples(), source_topics=SOURCE_TOPICS
    )
    target = tmp_path / "capture.session-draft.json"
    write_new_session_draft(target, document)
    assert target.stat().st_mode & 0o777 == 0o600
    assert json.loads(target.read_text(encoding="utf-8"))["status"] \
        == "SESSION_DRAFT_UNAPPROVED"
    with pytest.raises(CalibrationError, match="overwrite"):
        write_new_session_draft(target, document)


def test_draft_writer_rejects_non_draft_paths_and_status(tmp_path):
    document = session_draft_document(
        metadata(), complete_samples(), source_topics=SOURCE_TOPICS
    )
    with pytest.raises(CalibrationError, match="must end"):
        write_new_session_draft(tmp_path / "calibration.json", document)
    approved = dict(document)
    approved["status"] = "APPROVED"
    with pytest.raises(CalibrationError, match="SESSION_DRAFT_UNAPPROVED"):
        write_new_session_draft(tmp_path / "bad.session-draft.json", approved)
    relative = os.path.relpath(tmp_path / "relative.session-draft.json")
    with pytest.raises(CalibrationError, match="absolute"):
        write_new_session_draft(relative, document)


def test_draft_writer_recomputes_metrics_from_raw_windows(tmp_path):
    document = session_draft_document(
        metadata(), complete_samples(), source_topics=SOURCE_TOPICS
    )
    tampered = json.loads(json.dumps(document))
    bounds = tampered["samples"][0]["raw_evidence_provenance"][
        "observed_stationarity_bounds"
    ]
    bounds["max_joint_delta_rad"] = 0.0001
    with pytest.raises(CalibrationError, match="differ from raw"):
        write_new_session_draft(
            tmp_path / "tampered.session-draft.json", tampered
        )

    tf_tampered = json.loads(json.dumps(document))
    tf_path_pose = tf_tampered["samples"][0]["raw_evidence_provenance"][
        "tf_window"
    ][0]["contributing_path_edges"][0]["pose_parent_from_child"]
    tf_path_pose["translation_m"][0] = 0.01
    with pytest.raises(CalibrationError, match="differs from raw TF path"):
        write_new_session_draft(
            tmp_path / "tf-tampered.session-draft.json", tf_tampered
        )

    missing = json.loads(json.dumps(document))
    missing["samples"][0].pop("raw_evidence_provenance")
    with pytest.raises(CalibrationError, match="raw provenance"):
        write_new_session_draft(
            tmp_path / "missing.session-draft.json", missing
        )


def test_collector_node_has_no_actuator_or_tf_lookup_interfaces():
    source = (
        Path(__file__).resolve().parents[1]
        / "fv_machine_calibration"
        / "collector_node.py"
    ).read_text(encoding="utf-8")
    tree = ast.parse(source)
    called_attributes = {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    assert not {
        "create_publisher",
        "create_client",
        "create_action_client",
        "lookup_transform",
        "lookup_transform_async",
    } & called_attributes
    imported_modules = {
        node.module if isinstance(node, ast.ImportFrom) else alias.name
        for node in ast.walk(tree)
        if isinstance(node, (ast.Import, ast.ImportFrom))
        for alias in node.names
    }
    assert not any(
        "piper" in module.lower() or "can" in module.lower()
        or module == "tf2_ros"
        for module in imported_modules
    )


def test_node_parameter_wiring_and_readme_defaults_match_contract():
    package_root = Path(__file__).resolve().parents[1]
    source = (package_root / "fv_machine_calibration" / "collector_node.py").read_text(
        encoding="utf-8"
    )
    readme = (package_root / "README.md").read_text(encoding="utf-8")
    expected = (
        ("safety.max_bracket_gap_s", "0.05"),
        ("safety.max_sample_interval_s", "0.03"),
        ("safety.min_samples_per_side", "2"),
        ("safety.stationary_velocity_limit_rad_s", "0.01"),
        ("safety.max_joint_delta_rad", "0.001"),
        ("safety.max_tf_translation_delta_m", "0.0005"),
        ("safety.max_tf_rotation_delta_rad", "0.001"),
    )
    for name, value in expected:
        assert f'"{name}", {value}' in source
        assert f"{name}: {value}" in readme
    limits = synchronizer().limits
    assert limits.min_samples_per_side == 2
    assert limits.max_bracket_gap_ns == 50_000_000
    assert limits.max_sample_interval_ns == 30_000_000
