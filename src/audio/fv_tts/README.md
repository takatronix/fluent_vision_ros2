# fv_tts

`fv_tts` subscribes to `/aspa/tts/say`, invokes `voicevox_core` natively in the
ROS 2 process, and publishes PCM16LE to `/audio/agent/frame` or
`/audio/system/frame`. There is no `Speak` service and no VOICEVOX Engine HTTP
process.

Synthesis is serialized because the native backend is blocking. Pending SYSTEM
requests take priority over pending agent requests. `/audio/playback/control`
advances the accepted agent floor on `discard`, removes queued stale agent work,
and suppresses an obsolete result if native synthesis was already running.

Every syntactically valid request terminates on `/aspa/tts/result` with strict JSON fields
`kind`, `utterance_id`, and `status` (`completed`, `failed`, or `cancelled`). A
`failed` result also carries `error`. `completed` means synthesis finished and
the PCM frame was published; it does not mean that the speaker played the frame.
Actual playback completion remains `/audio/playback/event` authority.

The request is a `std_msgs/String` containing exactly:

```json
{"kind":"agent","utterance_id":"agent-0-example","text":"こんにちは"}
```

Install the `voicevox_core` Python package and obtain its ONNX Runtime library,
Open JTalk dictionary, and `.vvm` voice model. Export their paths, then launch:

```bash
export VOICEVOX_ONNXRUNTIME_PATH=/path/to/libvoicevox_onnxruntime.so
export VOICEVOX_OPEN_JTALK_DICT_DIR=/path/to/open_jtalk_dic_utf_8-1.11
export VOICEVOX_VOICE_MODEL_PATH=/path/to/voice.vvm
ros2 launch fv_tts fv_tts.launch.py style_id:=3 acceleration_mode:=auto
```
