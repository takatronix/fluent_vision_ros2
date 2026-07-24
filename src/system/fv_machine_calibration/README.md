# fv_machine_calibration

Review-first, per-machine calibration belongs to Fluent Vision. This package
contains a read-only, timestamp-synchronized ROS evidence collector and an
offline eye-in-hand solver for Piper's wrist D405.

The input session is strict evidence: `robot_id`, D405 serial/firmware,
URDF/tag-registry hashes, reviewed joint-zero/TCP/intrinsics calibration IDs,
`tag36h11` ID 0 with a **0.040 m black edge**, exact image timestamps, and
`base_link <- gripper_tip` plus `d405_optical_frame <- tag0` poses. It
requires at least 10 diverse training poses and 3 separately labelled holdout
poses. Five OpenCV solvers are compared; at least three must cluster. Target
pose consistency is checked on training and holdout samples.

```bash
fv_handeye_solve session.json --output handeye_proposal.json
```

The command is offline and never connects to CAN, enables Piper, sends motion,
calls `set_zero`, or edits an installed xacro. Output status is always
`PROPOSED_UNAPPROVED` with `calibration_ready: false`; it refuses to overwrite
an existing file. A reviewed proposal must later be merged with the same
machine's base mount, joint zero, TCP, intrinsics, identities, and hashes before
ASPA's calibration guardian can become READY.

The existing vlabor viewer hand-eye UI is therefore a future client of this
package, not the artifact authority. Legacy latest-TF samples must not be
imported as approved data.

## Stationary-bracket collector

`fv_handeye_collect` subscribes only to sensor/evidence topics. It does not
create CAN, Piper enable/motion/`set_zero`, URDF-write, or calibration-approval
interfaces. `ASPA_ROBOT_ID` is required when set; otherwise the short hostname
is the robot identity. There is no `aspa1` fallback.

All machine evidence parameters are mandatory: session ID, D405 serial and
firmware, joint-zero/TCP/D405-intrinsics calibration IDs, tag-registry SHA-256,
and URDF SHA-256. Image, detection, tag-pose, JointState, TF, and TF-static
topics must also be explicitly supplied. `tf.frame_allowlist` must list the
reviewed base-to-gripper chain and `joints.required_names` must list every arm
joint used to bound the observed motion during capture.

The image and its AprilTag detection/pose must retain the same exact camera
timestamp. JointState and dynamic TF normally use a different hardware clock
cadence, so equality with the camera timestamp is neither fabricated nor
required. Instead, the collector chooses the nearest strictly preceding and
following JointState samples and the nearest strictly preceding and following
complete TF snapshots. At least `safety.min_samples_per_side` samples are
required on each side for both streams, and the outer samples must remain
within `safety.max_bracket_gap_s`.
The largest observed interval (including each side's distance to the camera
instant) must also remain below `safety.max_sample_interval_s`.

No value is stamped as if it occurred at another time. There is no `tf2`
lookup, `Time(0)`, latest lookup, or pose interpolation. The preceding resolved
`base_link <- gripper_tip` TF is used as the solver pose only when the full
before/after observation window remains within configured translation/rotation
bounds. Joint positions must likewise remain within the joint-delta bound, and
every required joint velocity in the window must be below the stationary
limit. Missing either side, an incomplete/ambiguous TF
path, an excessive gap or delta, duplicate/non-monotonic evidence, stale/future
data, low-margin/nonzero-hamming ID 0, or non-unique primary source publishers
blocks capture.

The fail-closed limits are explicit ROS parameters. Current defaults are
provisional and must be reviewed against the machine's measured publication
cadence before an attended capture:

- `safety.max_bracket_gap_s`: `0.05`
- `safety.max_sample_interval_s`: `0.03`
- `safety.min_samples_per_side`: `2`
- `safety.stationary_velocity_limit_rad_s`: `0.01`
- `safety.max_joint_delta_rad`: `0.001`
- `safety.max_tf_translation_delta_m`: `0.0005`
- `safety.max_tf_rotation_delta_rad`: `0.001`

The current ASPA frame/topic contract is wired with the following parameter
shape. It is a fail-closed template for the Fluent Vision system launch, not a
standalone `ros2 run` command. Every `REPLACE_*` value must be populated from
reviewed evidence; the placeholders intentionally fail node validation.

```yaml
fv_handeye_collector:
  ros__parameters:
    session_id: "REPLACE_SESSION_ID"
    d405.serial: "REPLACE_MACHINE_D405_SERIAL"
    d405.firmware: "REPLACE_D405_FIRMWARE"
    dependencies.joint_zero_calibration_id: "REPLACE_JOINT_ZERO_ID"
    dependencies.tool_center_point_calibration_id: "REPLACE_TCP_ID"
    dependencies.d405_intrinsics_calibration_id: "REPLACE_INTRINSICS_ID"
    dependencies.tag_registry_sha256: "REPLACE_WITH_64_LOWERCASE_HEX"
    dependencies.urdf_sha256: "REPLACE_WITH_64_LOWERCASE_HEX"
    output.session_draft_path: "/absolute/existing/dir/capture.session-draft.json"
    image.transport: "raw"
    topics.image: "/d405_color/image_raw"
    topics.detections: "/d405_apriltag/apriltag_node/detections"
    topics.tag_poses: "/d405_apriltag/apriltag_node/poses"
    topics.joint_states: "/follower_arm/joint_states_single"
    topics.tf: "/tf"
    topics.tf_static: "/tf_static"
    joints.required_names: [joint1, joint2, joint3, joint4, joint5, joint6]
    tf.frame_allowlist:
      - base_link
      - piper_base_mount
      - piper_base_link
      - link1_1
      - link2_1
      - link3_1
      - link4_1
      - link5_1
      - gripper_base_1
      - gripper_tip
    safety.max_data_age_s: 0.5
    safety.max_bracket_gap_s: 0.05
    safety.max_sample_interval_s: 0.03
    safety.min_samples_per_side: 2
    safety.stationary_velocity_limit_rad_s: 0.01
    safety.max_joint_delta_rad: 0.001
    safety.max_tf_translation_delta_m: 0.0005
    safety.max_tf_rotation_delta_rad: 0.001
```

Do not substitute `/joint_states`: on current ASPA integration that is the
CuGo track state, not Piper. The final launch integration must retain
`/follower_arm/joint_states_single` and the reviewed Piper TF allowlist.

Each sample retains raw evidence provenance in the draft: image and tag
identity/stamps, the full JointState/TF observation windows, resolved TF poses
and their contributing source edges, four nearest bracket offsets, maximum
observed sample intervals, joint/TF excursions, observed velocity, and the
configured limits. It also stores `observed velocity × bracket span` as a
diagnostic displacement estimate. Because acceleration between observations
is not bounded, this estimate and the sampled window are explicitly **not** a
mathematical guarantee against a hidden move-and-return excursion. The root of
the draft also records every subscribed source topic and
`stationary_bracket_contract_version: 1`. Draft writing recomputes all saved
maximum deltas, observed velocity, velocity-times-span estimate, and sample
intervals from the raw windows; a mismatch blocks the write. This is bounded
stationary-bracket evidence across the camera instant, not an implicit
interpolation result or a proof of continuous-time stationarity.

Collection is explicitly split with read-only `std_srvs/Trigger` services:

```bash
ros2 service call /fv_handeye_collector/capture_training std_srvs/srv/Trigger
ros2 service call /fv_handeye_collector/capture_holdout std_srvs/srv/Trigger
ros2 service call /fv_handeye_collector/write_session_draft std_srvs/srv/Trigger
```

The final service requires at least 10 training and 3 holdout samples and
writes exactly one new absolute path ending in `.session-draft.json`. The file
is atomically created mode `0600`; existing files/symlinks are never replaced.
Its status is always `SESSION_DRAFT_UNAPPROVED` and `calibration_ready` is
always false. The collector never writes a solver proposal, approved artifact,
URDF, or xacro.

## Implementation boundary

This repository's current rules prefer C++ and disallow new Python production
surfaces. `fv_machine_calibration` already has a Python offline solver and ROS
collector, and this change deliberately keeps the bracket contract inside those
existing files instead of adding another runtime process or package. A future
C++ port should preserve the JSON evidence contract and fail-closed tests
exactly, including its version; the current safety boundary must not be
weakened merely to complete that migration.
