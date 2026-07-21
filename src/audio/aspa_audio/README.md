# aspa_audio

ROS 2 owns both physical audio devices and the robot's single playback stream.

- `audio_capture` is a Rust/CPAL input node. It publishes 16 kHz mono PCM16LE to
  `/audio/mic/frame` with `source_id=aspa_audio_capture` and
  `stream_id=audio/mic/main`.
- `playback_controller` is the only long playback queue. It subscribes to
  `/audio/agent/frame`, `/audio/system/frame`, and `/audio/cue/frame`, performs
  GStreamer resampling/volume conversion, and publishes only
  `/audio/output/frame`.
- `audio_output` is a Rust/CPAL device node. It subscribes only to
  `/audio/output/frame` and provides `/audio/output/flush`. On
  `/audio/output/drained`, `accepted` is only a flow-control acknowledgement;
  `drained` or `flushed` reports the frames that reached CPAL's predicted
  device playback timestamp. A flush closes and recreates the CPAL stream so
  samples already handed to the host buffer are discarded.

Launch with the default CPAL devices:

```bash
ros2 launch aspa_audio aspa_audio.launch.py
```

Select exact CPAL device names when needed:

```bash
ros2 launch aspa_audio aspa_audio.launch.py \
  capture_device:='USB Microphone' output_device:='USB Speaker'
```

The production capture contract is 16 kHz, mono, PCM16LE. The output defaults
to 48 kHz stereo PCM16LE. `audio_output` fails loudly when the selected device
does not expose that exact CPAL configuration; all output resampling, channel
mapping, and volume processing remains in the GStreamer playback controller.
Build prerequisites are Rust/Cargo, the CPAL system backend
(for Linux, ALSA development headers), GStreamer 1.0 base plugins, ROS 2, and
`fluent_dialogue_dora_interfaces` in the same colcon workspace or an underlay.
