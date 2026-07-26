# aspa_audio

ROS 2 owns both physical audio devices and the robot's single playback stream.

- `audio_capture` uses Rust/CPAL only to acquire the device's native PCM. Every
  input buffer then passes through the required persistent GStreamer
  `audioconvert ! audioresample` pipeline before the node publishes 16 kHz mono
  PCM16LE to `/audio/mic/raw` with `source_id=aspa_audio_capture` and
  `stream_id=audio/mic/raw`. There is no custom microphone resampler or conversion
  fallback; a GStreamer construction, negotiation, conversion, or streaming
  failure stops the capture process. The bounded 64-chunk handoff queue absorbs
  the observed GStreamer startup burst; exhausting it during steady operation is
  still fatal.
- `audio_aec` is the mandatory WebRTC AEC3 boundary between raw capture and
  speech processing. It consumes `/audio/mic/raw` and the volume-scaled,
  callback-timed `/audio/output/render_reference`, converts the render reference
  through persistent GStreamer `audioconvert ! audioresample`, applies the
  accepted 40 ms render delay, and publishes 16 kHz mono PCM16LE to
  `/audio/mic/frame` with
  `source_id=aspa_audio_aec` and `stream_id=audio/mic/main`. A missing,
  discontinuous, malformed, or overrun render reference is fatal; raw microphone
  PCM is never passed through as an implicit fallback.
- `playback_controller` is a Rust/GStreamer node and the only long playback
  queue. It subscribes to
  `/audio/agent/frame`, `/audio/system/frame`, and `/audio/cue/frame`, performs
  GStreamer resampling/channel conversion, and publishes unscaled typed
  `aspa_audio_interfaces/msg/PlaybackFrame` messages only on `/audio/play`.
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

The launch file treats capture, AEC, controller, and output as one failure domain.
If any node exits, the launch shuts down all four. In particular, an
unavailable or failed flush is fatal to the playback controller; the stack does
not continue the audio path after it can no longer invalidate stale physical
playback.
The controller also exits non-zero if a chunk is not accepted within 5 seconds,
if all acknowledgement/drain progress stops for 5 seconds, if a flush call does
not return within 5 seconds, or if GStreamer conversion does not finish within
10 seconds. Drain health is global rather than measured from each accepted
chunk, so a healthy long utterance queued ahead of later chunks does not cause a
false timeout.

Launch with the default CPAL devices:

```bash
ros2 launch aspa_audio aspa_audio.launch.py
```

Select exact CPAL device names when needed:

```bash
ros2 launch aspa_audio aspa_audio.launch.py \
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
`aspa_audio_interfaces` plus `fv_speech_interfaces` in the same
colcon workspace or an underlay.

Playback coordination uses typed messages throughout:

- `/audio/agent/frame`: `aspa_audio_interfaces/msg/SynthesizedSpeech`
- `/audio/system/frame`: `aspa_audio_interfaces/msg/SynthesizedSpeech`
- `/audio/cue/frame`: `fv_speech_interfaces/msg/AudioFrame`
- `/audio/playback/control`: `aspa_audio_interfaces/msg/PlaybackControl`
- `/audio/playback/event`: `aspa_audio_interfaces/msg/PlaybackEvent`
- `/audio/play`: `aspa_audio_interfaces/msg/PlaybackFrame`
- `/audio/mic/raw`: `fv_speech_interfaces/msg/AudioFrame`
- `/audio/output/render_reference`: `fv_speech_interfaces/msg/AudioFrame`
- `/audio/mic/frame`: `fv_speech_interfaces/msg/AudioFrame`
- `/audio/output/drained`: `aspa_audio_interfaces/msg/OutputDrained`
- `/audio/output/flush`: `aspa_audio_interfaces/srv/FlushAudio`

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
