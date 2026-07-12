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
# → 201 after the configured writers have started
#   {"episode_id":"01J...","bag_path":"...","meta_path":"...", ...}

curl -X POST http://localhost:8083/api/v1/episodes/<id>/stop \
  -H 'Content-Type: application/json' \
  -d '{"outcome":"success"}'
# → 202 {"state":"finalizing", ...}
# Poll GET /api/v1/episodes/<id> until finished, failed, or discarded.

curl http://localhost:8083/api/v1/episodes
```

## Design

The recorder starts the configured bag and camera writers and reports only
whether those operations succeeded. It does not diagnose ROS input health or
decide whether missing inputs should block recording. VLAbor health/status owns
input warnings and errors so every recorder caller observes the same system
state without a second recorder-specific health contract.

The recorder does not persist an exporter-specific canonical start or end
timestamp. Dataset export derives its fixed-FPS timeline from the camera
sidecars and required bag samples.

A successful finalization makes the video tree
read-only after transferring file ownership to the dataset-root owner, so that
the host exporter can still create hardlinks with `fs.protected_hardlinks=1`.
Enabled depth cameras add their lossless compressedDepth topic to the bag topic
list. Missing depth input is reported by VLAbor health/status; it does not alter
the recorder start API contract.
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

### Episode metadata schema versions

Every `meta.json` declares `schema_version`. Before opening the sqlite index or
serving requests, the recorder applies registered forward-only migrations to
every older metadata file. Normal recorder code reads and writes only the
current version 2 schema.

Version 1 migrates to version 2. The migration preserves unrelated metadata and
removes the obsolete canonical timeline and stop-count snapshot fields. It does
not rewrite MP4 files or synthesize a missing `video_pts` sidecar column.

| Contract | Version 1 | Native version 2 | Migrated version 1 |
| --- | --- | --- | --- |
| Effective episode interval | Not recorded | Derived during export from recorded samples | Derived only when the recorded data satisfies the version 2 export contract |
| Start handling | API request time | Writers start immediately | Metadata migration does not alter recording data |
| Stop camera counts | Camera summaries | Camera summaries | Camera summaries |
| MP4 timing | No stable declared contract | ROS header timestamps mapped to packet PTS | Existing MP4 is not rewritten |
| Camera timing metadata | Absent or incomplete | No duplicated mode, origin, or time-base fields | Obsolete fields are removed |
| Frame sidecar | No `video_pts` column | Includes the MP4 packet `video_pts` | Existing sidecar is not rewritten |

A migrated version 1 episode can be listed, edited, replayed, and previewed under
the version 2 metadata model. Export remains unavailable when the recorded
camera sidecar and MP4 do not satisfy the version 2 timing contract.

A missing version, a version without a forward migration, or a failed migration
stops recorder startup. Metadata files are replaced atomically only after their
migrated document passes current-schema validation. `index.db` is then rebuilt
from the migrated `meta.json` files.
