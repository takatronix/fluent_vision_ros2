# fv_episode_recorder

The package also owns the dialogue perception path defined by the ASPA
architecture:

- `video_anomaly_detector` publishes deterministic anomaly episode
  `started` / `ended` lifecycle on `/environment/change`.
- `moss_realtime_adapter` continuously sends timestamped frames at 1 FPS to
  the official MOSS-VL-Realtime `/v1/realtime` WebSocket service. It filters
  control tokens, commits complete non-silence rounds, and publishes the first
  semantic annotation on `/environment/event`.
- After episode end, later complete non-silence rounds can dynamically correct
  the same annotation. Corrections do not publish another event; ownership
  closes when a new anomaly episode starts.
- `episode_search` provides `/episode/search` using SQLite FTS5 `trigram` over
  MOSS annotation text only. It does not index raw anomaly lifecycle events.

The ASPA defaults subscribe directly to the D555 compressed relay
`/aspa/restamped/color_compressed` once, in the detector. It decodes the
message for ONNX inference and relays the original bytes at 1 FPS on the
package-internal `/perception/moss/image/compressed` topic. The MOSS adapter
forwards those JPEG/PNG bytes without a duplicate high-rate camera subscription
or raw-image stream. The configured detector model is
`/home/aspa/.aspa/models/visual-condition-multihead.onnx`; the canonical node
fails startup when the model or ONNX runtime cannot be loaded.

Annotations default to the writable `~/.aspa/episodes/annotations.db` (or
`FV_EPISODE_OUTPUT_DIR`). All recorder/search processes use SQLite WAL,
`busy_timeout`, and bounded write retry against that one database.

Run the complete package path through the system launcher, using
`perception_episode_recorder.launch.py` as the package-level launch include.

The MOSS model server remains a standalone application. The checked launcher
executes the official OpenMOSS `realtime_inference/run_online_inference.py`
service and fails before model load when the repository, required runtime
versions, CUDA 12.8, WebSocket dependencies, checkpoint, or port are missing:

```bash
MOSS_VL_REPO=/opt/MOSS-VL ./scripts/prepare_moss_vl_realtime

MOSS_VL_REPO=/opt/MOSS-VL \
MOSS_VL_CHECKPOINT=/models/MOSS-VL-Realtime \
./scripts/start_moss_vl_realtime --preflight-only

MOSS_VL_REPO=/opt/MOSS-VL \
MOSS_VL_CHECKPOINT=/models/MOSS-VL-Realtime \
./scripts/start_moss_vl_realtime
```

It serves `ws://127.0.0.1:18081/v1/realtime` by default. The equivalent launch
contract is `moss_vl_realtime_runtime.launch.py`; it uses `ExecuteProcess`, not
a ROS node. A Hugging Face model ID must already be cached unless download is
explicitly enabled.

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
