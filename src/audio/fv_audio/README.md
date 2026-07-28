# fv_audio

ROS 2 owns both physical audio devices and the robot's single playback stream.

- `audio_device_registry` owns no stream and remains available when capture or
  output cannot open a configured device. Its `/audio/devices/list` service
  returns input or output devices with the CPAL stable ID, display metadata,
  and default-role flag. `capture_device` and `output_device` accept this exact
  ID or the explicit `default` role; a display name is never substituted for a
  missing or mismatched ID.
- `audio_capture` uses Rust/CPAL only to acquire the device's native PCM. Every
  input buffer then passes through the required persistent GStreamer
  `audioconvert ! audioresample ! audiobuffersplit` pipeline. Converted PCM is
  copied into a lock-free SPSC byte ring; the ROS publisher thread prebuffers
  four 10 ms chunks and then removes exactly one chunk per monotonic-clock
  deadline. This absorbs the device's observed 40 ms callback bursts without
  publishing four ROS messages at once. The node publishes 16 kHz mono PCM16LE
  to `/audio/mic/raw` with `source_id=fv_audio_capture` and
  `stream_id=audio/mic/raw`. There is no custom microphone resampler or
  conversion fallback; a GStreamer construction, negotiation, conversion,
  streaming, ring overflow, or 500 ms capture stall stops the process. The ring
  holds 64 chunks by default.
- `audio_aec` is the continuous boundary between raw capture and speech
  processing. While ASPA is silent it forwards `/audio/mic/raw` unchanged.
  During playback it consumes the volume-scaled,
  callback-timed `/audio/output/render_reference`, converts the render reference
  through persistent GStreamer `audioconvert ! audioresample`, applies the
  accepted 40 ms render delay, and publishes 16 kHz mono PCM16LE to
  `/audio/mic/frame` with
  `source_id=fv_audio_aec` and `stream_id=audio/mic/main`. AEC remains active for
  500 ms after playback completes to cover room reverberation. If the render
  reference is unavailable, output falls back to raw microphone PCM and
  `/audio/aec/status` reports `degraded`; it automatically returns to AEC when
  the reference recovers. Sequence/sample gaps flush DSP state and resynchronize
  without resetting the output sequence. Identity, format, and malformed-payload
  violations remain fatal.
- `playback_controller` is a Rust/GStreamer node and the only long playback
  queue. It subscribes to
  `/audio/agent/frame`, `/audio/system/frame`, and `/audio/cue/frame`, performs
  GStreamer resampling/channel conversion, and publishes unscaled typed
  `fv_audio_interfaces/msg/PlaybackFrame` messages only on `/audio/play`.
  The same stream is the browser playback source; there is no JSON projection
  or compatibility output topic.
- `audio_output` is a Rust/CPAL device node. It subscribes only to
  `/audio/play`, subscribes to TRANSIENT_LOCAL `/audio/output/volume`
  (`std_msgs/msg/Float32`, clamped to 0.0 through 1.0), and provides the typed
  keyed `/audio/output/flush` service. Volume is applied to samples in the CPAL
  device callback, so published PCM stays unscaled and a live volume change
  affects samples that have not yet reached the device.
  The same callback publishes the exact scaled PCM handed to CPAL as the AEC
  render reference. A device-buffer flush terminates that reference generation,
  causing `audio_aec` to clear its queued reference and reset WebRTC AEC3 before
  accepting the replacement stream.
  Keys not observed before a flush become one-shot tombstones, so a service
  request that overtakes its DDS frame still rejects the late stale PCM. On
  `/audio/output/drained`, `accepted` is only a flow-control acknowledgement;
  `drained` or `flushed` reports the frames that reached CPAL's predicted
  device playback timestamp. A flush closes and recreates the CPAL stream so
  samples already handed to the host buffer are discarded.

The launch file keeps capture, AEC, controller, and output as separate recovery
domains. Each node exits on an unrecoverable contract failure and the launch
owner respawns that exact executable after two seconds. Recoverable transport
gaps are handled in-process so `/audio/mic/frame` remains available. Live UI reports both the process
and ROS-node state and can request the same exact-process restart. If the launch
owner itself is absent, an individual restart fails instead of starting an
unowned process.

An unavailable or failed flush remains fatal to the playback controller. During
its restart window, playback readiness is false and no other component assumes
that stale physical playback was invalidated.
The controller also exits non-zero if a chunk is not accepted within 5 seconds,
if all acknowledgement/drain progress stops for 5 seconds, if a flush call does
not return within 5 seconds, or if GStreamer conversion does not finish within
10 seconds. Drain health is global rather than measured from each accepted
chunk, so a healthy long utterance queued ahead of later chunks does not cause a
false timeout.

Launch with the default CPAL devices:

```bash
ros2 launch fv_audio fv_audio.launch.py
```

Select exact CPAL device names when needed:

```bash
ros2 launch fv_audio fv_audio.launch.py \
  capture_device:='USB Microphone' output_device:='USB Speaker' \
  aec_stream_delay_ms:=40
```

The production capture contract is 16 kHz, mono, PCM16LE. The output defaults
to 48 kHz stereo PCM16LE. `audio_output` fails loudly when the selected device
does not expose that exact CPAL configuration; all output resampling, channel
mapping remains in the GStreamer playback controller, while device gain belongs
only to `audio_output`.
Build prerequisites are Rust/Cargo, Meson, the CPAL system backend
(for Linux, ALSA development headers), GStreamer 1.0 base plugins, ROS 2, and
`fv_audio_interfaces` plus `fv_speech_interfaces` in the same
colcon workspace or an underlay.

Playback coordination uses typed messages throughout:

- `/audio/agent/frame`: `fv_audio_interfaces/msg/SynthesizedSpeech`
- `/audio/system/frame`: `fv_audio_interfaces/msg/SynthesizedSpeech`
- `/audio/cue/frame`: `fv_speech_interfaces/msg/AudioFrame`
- `/audio/playback/control`: `fv_audio_interfaces/msg/PlaybackControl`
- `/audio/playback/event`: `fv_audio_interfaces/msg/PlaybackEvent`
- `/audio/play`: `fv_audio_interfaces/msg/PlaybackFrame`
- `/audio/mic/raw`: `fv_speech_interfaces/msg/AudioFrame`
- `/audio/output/render_reference`: `fv_speech_interfaces/msg/AudioFrame`
- `/audio/mic/frame`: `fv_speech_interfaces/msg/AudioFrame`
- `/audio/aec/status`: `std_msgs/msg/String` (`bypass|active|tail|degraded`)
- `/audio/output/drained`: `fv_audio_interfaces/msg/OutputDrained`
- `/audio/output/flush`: `fv_audio_interfaces/srv/FlushAudio`
- `/audio/devices/list`: `fv_audio_interfaces/srv/ListAudioDevices`

List input and output devices without opening either stream:

```bash
ros2 service call /audio/devices/list \
  fv_audio_interfaces/srv/ListAudioDevices '{direction: 1}'
ros2 service call /audio/devices/list \
  fv_audio_interfaces/srv/ListAudioDevices '{direction: 2}'
```

Every playback frame and event carries the controller's current
`playout_generation` for its playback kind. Agent pause/discard advances only
the Agent generation before publishing the event. Targeted SYSTEM abort uses
the utterance ID as a bounded tombstone, so it cannot invalidate a different
SYSTEM utterance when DDS topics are reordered. Initial values are wall-clock
milliseconds, kept within JavaScript's safe integer range, so a controller-only
restart cannot make a still-connected browser reject new PCM as stale.

Agent and SYSTEM speech arrive as one complete PCM payload with source
alignment marks. GStreamer performs sample-rate and channel conversion; the
controller scales mark frame offsets with the same rate ratio and ties-to-even
rounding. Agent `PlaybackEvent` reports only source spans whose ending frame
has physically drained. Cue audio has no source-text position.
