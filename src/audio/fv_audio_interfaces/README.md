# fv_audio_interfaces

Interface-only ROS 2 package for the FV audio path. It defines the typed
boundary among physical device discovery, audio producers, dialogue control,
playback coordination, and direct audio output. It does not contain nodes or
compatibility adapters.

`ListAudioDevices` accepts exactly `INPUT` or `OUTPUT`. Each successful result
contains the CPAL stable device ID used by `fv_audio`, the display metadata
reported by CPAL, and whether the device owns the selected direction's default
role. Optional CPAL metadata is represented by an empty string or empty
`details` array. If enumeration, the stable ID, or required device metadata
cannot be read, the service returns `success=false` and no partial list.

`PlaybackFrame.pcm_s16le` is interleaved signed 16-bit little-endian PCM. Its
byte length must equal `frame_count * channels * 2`. A flush identifies frames
by parallel `(utterance_ids[i], seqs[i])` keys; both arrays must have the same
length. A controller must not reuse one of these keys during its lifetime.

The playback controller owns one `playout_generation` sequence per playback
kind. Agent pause/discard increments only the Agent sequence; targeted SYSTEM
abort is ordered by its utterance-ID tombstone and does not invalidate unrelated
SYSTEM audio. Playback projections reject older frames and events within the
same kind; physical output invalidation uses the keyed flush service. A
controller seeds all three sequences from wall-clock milliseconds so they
remain ordered across controller-only restarts while still fitting JavaScript's
safe integer range.

Except for `PlaybackControl.NONE`, zero-valued enum fields are invalid. Empty
or otherwise unused control fields are interpreted according to `action`:

| Action | Additional fields |
| --- | --- |
| `PAUSE`, `RESUME` | `release_hold=NONE`; epoch and ID unused |
| `DISCARD` | release `USER` or `SYSTEM`; minimum agent epoch; SYSTEM hold ID |
| `ABORT_SYSTEM` | target ID; `SYSTEM` releases its hold, `NONE` preserves it |

```bash
colcon build --packages-select fv_audio_interfaces
pytest -q src/audio/fv_audio_interfaces/test/test_interfaces.py
```
