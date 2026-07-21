# fv_machine_calibration

Review-first, per-machine calibration belongs to Fluent Vision. The first
implemented stage is an offline eye-in-hand solver for Piper's wrist D405.

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
package, not the artifact authority. A timestamp-synchronized ROS collector is
the next step; legacy latest-TF samples must not be imported as approved data.
