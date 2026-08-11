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
  Agent and SYSTEM Magpie streams publish exactly the first converted playout
  chunk as soon as it is available, then publish one chunk on each fixed
  monotonic deadline (`chunk_ms`, 20 ms in the production configuration).
  They do not wait for or burst a three-chunk TTS prefill. Fully assembled cues
  retain their existing three-chunk initial device prefill.
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
- `/aspa/tts/timing` carries one versioned, host-local
  `fv_audio_interfaces/msg/TtsTimingEvent` chain. The controller samples
  `FIRST_PLAYBACK_FRAME_PUBLISHED` immediately before its first `/audio/play`
  publish and emits the event only after that publish succeeds. A seq-zero
  replay after flush does not repeat this first-publication stage.
  `audio_output` emits `PHYSICAL_PLAYBACK_STARTED` once per exact TTS identity
  at the scheduled DAC boundary: callback `CLOCK_MONOTONIC` plus CPAL's
  playback-minus-callback duration plus the frame offset inside the device
  buffer. This is CPAL's scheduled physical start, not callback-entry time and
  not a microphone measurement of audibility.
  It emits `PHYSICAL_PLAYBACK_ENDED` only after the final TTS frame's scheduled
  device-drain deadline. A flush discards that pending end marker, so cancelled
  PCM cannot produce a false complete timing chain.
  `PLAYBACK_UNDERRUN` identifies the same request/generation/utterance and the
  exact frame count filled with silence only while that TTS stream is active
  and has not supplied a final frame. Normal idle silence emits no underrun.
  Timing queue overflow, identity change before final, and cue interruption of
  an active non-final TTS stream stop the output process.
- `tts_timing_collector` is the only timing-chain owner and runs outside the
  CPAL output process. Multiple DDS writers may deliver valid stages out of
  arrival order; the collector joins them by exact
  request/generation/utterance identity and publishes
  `/aspa/tts/timing/receipt` only when all eight required timestamps are
  present and strictly increasing. Missing stages fail after a bounded
  deadline. Duplicate stages, identity drift, reversal, active-chain overflow,
  or receipt persistence failure exit the collector.
  The receipt file is an absolute-path, owner-only `0600` regular file in a
  same-owner, non-writable-by-others canonical directory. Symlinks, hard links,
  owner/mode drift, and a changed receipt bound are rejected. Updates use an
  owner-only temporary file, file and directory `fsync`, and atomic rename.
  The bounded disk history and TRANSIENT_LOCAL receipt topic contain only
  complete `TtsTimingReceipt` records. The topic and Live UI subscriber both
  retain the latest 64 receipts, matching the Live UI's exact-request lookup
  window instead of retaining only the most recent concurrent completion.
  A complete receipt passes streaming
  playback acceptance only when `total_underrun_frames == 0`. A non-zero count
  is still persisted and published for diagnosis, and the collector logs the
  failed acceptance; it is never hidden by treating inserted silence as
  successful streaming. This acceptance covers scheduled CPAL/device delivery.
  It does not turn the scheduled DAC timestamps into a microphone measurement
  of acoustic audibility.

The launch file keeps capture, AEC, controller, and output as separate recovery
domains. Those nodes exit on an unrecoverable contract failure and the launch
owner respawns the exact executable after two seconds. The timing collector is
different: it is not respawned alone, and its exit shuts down the complete
`fv_audio` launch group so a broken or unpersisted measurement cannot be hidden
by continuing voice output. The enclosing voice supervisor then leaves voice
admission closed during replacement. Recoverable transport gaps are handled
in-process so `/audio/mic/frame` remains available. Live UI reports both the
process and ROS-node state and can request supported exact-process restarts. If
the launch owner itself is absent, an individual restart fails instead of
starting an unowned process.

An unavailable or failed flush remains fatal to the playback controller. During
its restart window, playback readiness is false and no other component assumes
that stale physical playback was invalidated.
The controller also exits non-zero if a chunk is not accepted within 5 seconds,
if all acknowledgement/drain progress stops for 5 seconds, if a flush call does
not return within 5 seconds, or if GStreamer conversion does not finish within
10 seconds. Drain health is global rather than measured from each accepted
chunk, so a healthy long utterance queued ahead of later chunks does not cause a
false timeout.

A user PAUSE is correlated end to end by its canonical UUID. The controller
publishes `AGENT_PAUSED` as soon as it installs the logical hold, then starts the
keyed device flush. It publishes `AGENT_PAUSE_COMMITTED` only after both the
flush service and every terminal output acknowledgement have completed. Only
that commit contains the exact physically played frame/source-character
prefix. If there was no active Agent output, the controller emits an immediate
commit with the same UUID and a strictly empty TTS identity.

While that physical pause transaction is pending, RESUME, DISCARD, and targeted
abort controls are retained in arrival order. They run only after the commit
event has been published, so a racing DISCARD cannot erase the prefix used for
interruption context. The FIFO is limited to eight controls and a second PAUSE
is rejected; overflow or overlap is an explicit contract error rather than an
unbounded queue or guessed ordering.

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
The r2r build generates bindings only for `fv_audio_interfaces`,
`fv_speech_interfaces`, and `std_msgs`; unrelated or stale IDL packages in the
surrounding workspace cannot alter this package's generated bindings.

Playback coordination uses typed messages throughout:

- `/audio/agent/frame`: `fv_audio_interfaces/msg/SynthesizedSpeechChunk`
- `/audio/system/frame`: `fv_audio_interfaces/msg/SynthesizedSpeechChunk`
- `/audio/cue/frame`: `fv_speech_interfaces/msg/AudioFrame`
- `/audio/playback/control`: `fv_audio_interfaces/msg/PlaybackControl`
- `/audio/playback/event`: `fv_audio_interfaces/msg/PlaybackEvent`
- `/audio/play`: `fv_audio_interfaces/msg/PlaybackFrame`
- `/audio/mic/raw`: `fv_speech_interfaces/msg/AudioFrame`
- `/audio/output/render_reference`: `fv_speech_interfaces/msg/AudioFrame`
- `/audio/mic/frame`: `fv_speech_interfaces/msg/AudioFrame`
- `/audio/aec/status`: `std_msgs/msg/String` (`bypass|active|tail|degraded`)
- `/audio/output/drained`: `fv_audio_interfaces/msg/OutputDrained`
- `/aspa/tts/timing`: `fv_audio_interfaces/msg/TtsTimingEvent`
- `/aspa/tts/timing/receipt`: `fv_audio_interfaces/msg/TtsTimingReceipt`
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
`playout_generation` for its playback kind. Agent pause/discard/abort advances
only the Agent generation before publishing the event. Targeted SYSTEM abort
uses the utterance ID as a bounded tombstone, so it cannot invalidate a
different SYSTEM utterance when DDS topics are reordered. Initial values are
wall-clock milliseconds, kept within JavaScript's safe integer range, so a
controller-only restart cannot make a still-connected browser reject new PCM
as stale.

Agent and SYSTEM speech arrive incrementally as 22,050 Hz mono float32 PCM.
The first MagpieTTS-RT chunk is exactly four codec frames, each non-final
steady chunk is exactly eight, and the explicit final AUDIO is zero through
eight frames. A zero-frame final is a non-first control marker: it advances
sequence without writing PCM or changing sample/token progress. A zero- or
eight-frame final is therefore distinguished only by `final_chunk=true`; the
controller never guesses termination from payload length. One persistent
GStreamer pipeline per utterance performs sample-rate and channel conversion
without resetting resampler state between chunks.
If a recognizable Agent or SYSTEM stream violates this contract, the
controller tombstones and flushes it locally and publishes a targeted
`ABORT_AGENT` or `ABORT_SYSTEM` control. `fv_tts` uses that same utterance ID to
cancel the queued or active native request, so rejected PCM cannot leave an
unobserved GPU generation running.

The always-on controller admits at most 16 retained utterances and 120 seconds
of converted output PCM across the active, paused, queued, and drain-waiting
states. Cue assembly is separately bounded to 16 streams and 8 MiB total.
Crossing a bound rejects that identifiable utterance; speech rejection also
uses the targeted abort path above so generation stops instead of accumulating
an unseen backlog. Cue conversion must return the exact rationally scaled frame
count. Its GStreamer resampler is flushed with an explicit 64-frame zero
boundary and only that known flush tail is cropped; output shorter than the
source duration or longer than the source-plus-boundary envelope is rejected.
Missing output is never padded with guessed silence.

The first chunk also carries the exact prepared-token-to-source map. Native
alignment sample boundaries are converted with an end-exclusive ceiling ratio,
and Agent `PlaybackEvent` reports only source spans whose converted boundary
has physically drained. `COMPLETE` is accepted only after an explicit final
audio chunk and full source alignment. Sequence gaps, reordered or duplicate
chunks, format drift, audio after final, and malformed COMPLETE/ABORT messages
abort only that utterance and flush any partial playback. Cue audio has no
source-text position.

An accepted speech stream must make input progress at least once every 35
seconds. This receiver deadline is deliberately wider than `fv_tts`'s
30-second native progress deadline plus its 2-second cancellation-terminal
deadline. If the TTS process disappears before publishing `COMPLETE` or
`ABORT`, the controller uses the retained request/generation/kind identity to
target-abort and flush only that stalled utterance; unrelated streams remain
admitted.
