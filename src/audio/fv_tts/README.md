# fv_tts

`fv_tts` is an `rclcpp` node that calls a pinned VOICEVOX Core 0.16.4 C API
extension in-process. The extension preserves the source Unicode-character
range that produced each mora. It subscribes to `/aspa/tts/say` and publishes
`fv_audio_interfaces/msg/SynthesizedSpeech` to `/audio/agent/frame` or
`/audio/system/frame`; no Python runtime and no VOICEVOX Engine HTTP process
are involved.

Each `SynthesizedSpeech` atomically contains PCM16LE and `SpeechMark[]`.
The marks carry source-character, mora, and synthesized-frame ranges. The
AudioQuery duration must exactly match the resulting PCM length; a missing or
inconsistent mapping fails synthesis instead of estimating a character
position.

Native synthesis is serialized. Pending SYSTEM requests overtake pending agent
requests. Typed `fv_audio_interfaces/msg/PlaybackControl` messages on
`/audio/playback/control` advance the agent floor on `DISCARD`, cancel queued
stale work, and suppress PCM from native work that became stale while it was
running. A targeted SYSTEM abort is remembered briefly so cross-topic
reordering cannot resurrect already-aborted speech.

Every in-process VOICEVOX Core synthesis call has a whole-call deadline,
including the startup smoke synthesis and every accepted request. The
`synthesis_timeout_seconds` parameter defaults to `60.0` and must be finite and
greater than zero. If the native C API has not returned by that deadline, the
node prints the failing context (startup, or request kind and `utterance_id`)
and calls `std::_Exit(EXIT_FAILURE)`. This deliberately bypasses scheduler
destruction, whose worker join could otherwise wait forever; the launch file
then shuts down its failure domain because the native node exited.

Every accepted request terminates on `/aspa/tts/result` with strict JSON fields
`kind`, `utterance_id`, and `status` (`completed`, `failed`, or `cancelled`). A
`failed` result also carries `error`. `completed` means synthesis finished and
the PCM frame was published; playback completion remains the typed
`/audio/playback/event` authority.

The request is a `std_msgs/String` containing exactly:

```json
{"kind":"agent","utterance_id":"agent-0-example","text":"こんにちは"}
```

The selected style is updated through TRANSIENT_LOCAL `/aspa/tts/settings`:

```json
{"version":1,"style_id":30}
```

The current selection and all installed talk styles are published
TRANSIENT_LOCAL on `/aspa/tts/voices`:

```json
{"version":1,"current_style_id":30,"voices":[{"id":30,"speaker":"No.7","style":"アナウンス","label":"No.7 / アナウンス"}]}
```

Build the pinned Linux arm64 C API extension and install the accepted ONNX
Runtime, Open JTalk dictionary, and talk models before building:

```bash
scripts/setup-voicevox-core
export VOICEVOX_CORE_ROOT="$HOME/.aspa/voicevox_core/current/core"
source /opt/ros/jazzy/setup.bash
colcon build --packages-select fv_tts
```

`VOICEVOX_CORE_ROOT` is optional at the default install location. For runtime,
export the asset paths printed by the setup script (the normal stack launcher
derives the same defaults), then launch:

```bash
export VOICEVOX_ONNXRUNTIME_PATH=/path/to/libvoicevox_onnxruntime.so
export VOICEVOX_OPEN_JTALK_DICT_DIR=/path/to/open_jtalk_dic_utf_8-1.11
export VOICEVOX_VOICE_MODEL_PATH='/path/to/vvms/[0-9]*.vvm'
ros2 launch fv_tts fv_tts.launch.py \
  style_id:=30 acceleration_mode:=auto synthesis_timeout_seconds:=60.0
```
