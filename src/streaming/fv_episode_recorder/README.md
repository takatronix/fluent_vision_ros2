# fv_episode_recorder

VLA training data recorder — rosbag + camera mp4 + markers + retention + replay.

OSS, ROS2 distro-agnostic (Humble / Jazzy). Used by both vlabor and aspa-navigation.

## Status

**Phase 1 Step 1 (walking skeleton)** — minimal start/stop with meta.json output.
Bag recording, mp4 segments, frames sidecar, preflight, active_lock, markers,
retention, replay come in later steps.

## Quick test

```bash
colcon build --packages-select fv_episode_msgs fv_episode_recorder
source install/setup.bash
ros2 launch fv_episode_recorder fv_episode_recorder_launch.py output_dir:=/tmp/datasets

# in another shell
curl -X POST http://localhost:8083/api/v1/episodes \
  -H 'Content-Type: application/json' \
  -d '{"task_description":"Pick up the white cube","profile":"piper_single_teleop"}'
# → 201 after the bag/camera ready barrier
#   {"episode_id":"01J...","timeline_start_ros_ns":..., ...}

curl -X POST http://localhost:8083/api/v1/episodes/<id>/stop \
  -H 'Content-Type: application/json' \
  -d '{"outcome":"success"}'
# → 202 {"state":"finalizing","timeline_end_ros_ns":..., ...}
# Poll GET /api/v1/episodes/<id> until finished, failed, or discarded.

curl http://localhost:8083/api/v1/episodes
```

## Design

`meta.json` schema version 3 records the effective episode as the half-open ROS-clock interval
`[timeline_start_ros_ns, timeline_end_ros_ns)`. `timeline_start_ros_ns` is fixed
only after the standard rosbag2 writer and every color-camera writer have each
accepted a valid sample. The start response then waits until every required bag
topic has a writer-accepted sample at or after `T0 - min(0.1s, 3/fps)` and every
enabled color camera contains a frame at or after T0. Camera writers retain pre-T0 raw frames,
while dataset export excludes samples outside the canonical interval. A successful finalization makes the video tree
read-only after transferring file ownership to the dataset-root owner, so that
the host exporter can still create hardlinks with `fs.protected_hardlinks=1`.
Enabled depth cameras use their lossless compressedDepth bag topic as a required
bag topic, so start cannot succeed before rosbag2 has accepted depth data too.
If this ready barrier times out, `POST /api/v1/episodes` returns a typed `503`
payload with `code`, `message`, `missing_bag`, `missing_cameras`,
`bag_counts`, `camera_counts`, and the observed timestamp evidence. It does not
encode those fields into a `detail` string.
On startup, abandoned `finalizing` episodes become `failed/abort` without
success inference, and historical `finished/success` sources are migrated to
the same read-only ownership contract.
If the detached rosbag helper does not exit within 10 seconds, finalization terminates it and commits the episode as `failed/abort` instead of waiting indefinitely.

Lifecycle failures stored in `stale_input_events` always use
`{"code": "...", "detail": "..."}`; start and asynchronous-finalize errors do
not use different payload shapes.

`POST /api/v1/episodes/{episode_id}/tags` atomically appends the required
`tags` list and removes duplicates without replacing tags written by another
client.

- [FV EpisodeからLeRobot Datasetへの変換仕様](../fv_lerobot_exporter/docs/lerobot_dataset_export.md)
