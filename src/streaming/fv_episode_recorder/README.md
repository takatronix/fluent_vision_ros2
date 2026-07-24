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
# → 201 {"episode_id":"01J...","started_at":"...","bag_path":"...","meta_path":"..."}

curl -X POST http://localhost:8083/api/v1/episodes/<id>/stop \
  -H 'Content-Type: application/json' -d '{"outcome":"success"}'

curl http://localhost:8083/api/v1/episodes
```

## Event Bus → markers (Event Bus Contract v1)

The recorder subscribes to `/fv/event/active` (`std_msgs/String`, param
`event_active_topic`) — the situation-event stream fv_soundboard re-publishes.
While an episode is recording, every event becomes a point-in-time marker
(`kind="event"`) on that episode's timeline, so the *same* Event Bus that
drives sound + speech also produces the learning-data annotations.

Asparagus harvest cycle `detect → approach → grasp → harvest/cut → place →
success|failure` lands as ordered markers:

- phase events (`detect/approach/grasp/harvest/cut/place`) → tagged `phase`
- boundaries/outcomes (`auto_start/goal_reached/success/failure/stop`) →
  tagged `boundary`; `success`/`failure` carry a marker `outcome`
  (`success`/`abort`).

Payloads are accepted either bare (`"detect"`) or structured
(`{"event":"detect","variant":"grade_A"}`). Structured keys (variant / grade /
class / confidence / …) are stored as marker `attributes`; distinct detect
variants are rolled up to `meta.detected_variants` at stop. NOTE: fv_soundboard
currently forwards only the bare marker name — the structured payload requires
the producer to publish JSON on `/fv/event/active` (see `event_bridge.py`).

The bridge does **not** start/stop recording; episode lifecycle stays on the
REST/batch path. Auto-segmentation (auto_start → start, place/success → stop)
is a documented follow-up. Disable the bridge with `event_marker_enabled:=false`.

ASPA bring-up: `ros2 launch aspa_launch fv_episode_recorder.launch.py`.

## Design

See `vlabor_ros2/docs/調整中/20260531_VLAbor_エピソード保存再生機能_設計書.md`.
