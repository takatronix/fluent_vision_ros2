# fv_episode_recorder

VLA training data recorder — rosbag + camera mp4 + markers + retention + replay.

OSS, ROS2 distro-agnostic (Humble / Jazzy). Used by both vlabor and aspa-navigation.

## Status

The recorder writes rosbag, per-camera H.264 MP4, `frames.parquet`, and
`meta.json` for each episode. Marker, retention, replay, and disk preflight
APIs are included in the same service.

## Quick test

```bash
colcon build --packages-select fv_episode_msgs fv_episode_recorder
source install/setup.bash
ros2 launch fv_episode_recorder fv_episode_recorder_launch.py output_dir:=/tmp/datasets

# in another shell
curl -X POST http://localhost:8083/api/v1/episodes \
  -H 'Content-Type: application/json' \
  -d '{"task_description":"Pick up the white cube","profile":"piper_single_teleop"}'
# → 201 {"episode_id":"01J...","started_at":"...","bag_path":"...","meta_path":"..."}

curl -X POST http://localhost:8083/api/v1/episodes/<id>/stop \
  -H 'Content-Type: application/json' -d '{"outcome":"success"}'
# → 202 {"episode_id":"01J...","state":"finalizing","finalization_pending":true}

curl http://localhost:8083/api/v1/episodes
```

The profile defines the complete camera and bag-topic catalog. A start request
may narrow that catalog with typed `include` and `exclude` selectors; it does
not redefine camera topics or codecs.

```json
{
  "task_description": "Pick up the white cube",
  "profile": "piper_single_teleop",
  "exclude": {
    "cameras": [{"kind": "depth"}],
    "topics": [{"role": "annotation"}]
  }
}
```

Camera selectors match `name`, `topic`, and `kind`. Topic selectors match
`topic` and `role`. Conditions within one selector are ANDed; selectors in a
list are ORed. Excluding a camera also prevents camera-derived bag topics and
per-episode workers from being created. A selector that matches no resolved
profile resource rejects the start request.

Stopping an episode closes its ROS subscriptions and releases the active slot
before rosbag and camera encoders finish flushing. The episode remains
`finalizing` until its per-episode background job commits `finished`, `failed`,
or `discarded`. A new episode never reuses resources owned by an older
finalizer.

## Storage contract

`meta.json` is the authoritative episode document. Its `schema_version` is
validated strictly; migrations advance exactly one registered version at a
time, and unknown versions or fields stop loading. The SQLite index is a
disposable projection rebuilt from `meta.json` at startup.

Color camera messages are encoded as VFR H.264. Each MP4 PTS is derived from
the ROS header timestamp relative to that camera's first frame, using a
`1/1000000` time base. `frames.parquet` stores both the original
`ros_stamp_ns` and the resulting `video_pts`, so consumers can map the video
timeline back to ROS time without retiming the recording.

After a successful finalization, bag and video payload files are owned by the
output root owner and made read-only. Consumers running as that owner can
hardlink the source files on the same filesystem without copying or mutating
them. `meta.json` remains mutable for episode annotations and is not part of
this hardlink source contract.

## Design

See `vlabor_ros2/docs/調整中/20260531_VLAbor_エピソード保存再生機能_設計書.md`.
