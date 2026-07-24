# fv_video_event_describer

Rust ROS 2 node that samples one compressed camera at 1 FPS, keeps a
32-second ring, publishes typed event start/end boundaries, and asks a
persistent Qwen3-VL server for one natural-language description after the
interval ends.

```text
/aspa/restamped/color_compressed
  -> /environment/change
  -> Qwen3-VL
  -> /environment/annotation
```

The node uses the existing `fv_episode_recorder` REST API. If an episode is
already active, it only adds the interval marker and leaves that recording
running. Otherwise it starts an episode with `recorder_profile`, owns that
episode, and stops it after the event marker ends. The completed Qwen
description patches the same marker after finalization.
`aspa_dialogue` sends start, end, and the completed description as application
`additionalContext`.
