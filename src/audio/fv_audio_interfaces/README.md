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
AGENT and SYSTEM frames preserve the originating TTS request and generation
UUIDs through the physical output boundary. CUE frames carry both fields
empty because they do not originate in TTS.

`SynthesizedSpeechChunk` is one ordered TTS stream. `AUDIO` messages normally
contain non-empty float32 PCM and use contiguous `sequence` and
`first_sample_index` values. The only zero-frame `AUDIO` is a non-first
`final_chunk=true` control marker after a non-empty logical stream; it advances
`sequence` but not the sample index or committed-token progress.
`final_chunk` otherwise preserves the native final-lease flag explicitly. It
cannot be inferred from size because both a steady chunk and a final tail may
contain eight codec frames. Sequence zero also carries the exact source text
and the complete prepared-token-to-source progress table. Later `AUDIO`
messages must not repeat that table. A stream ends exactly once with a
zero-payload `COMPLETE` or `ABORT`; its sequence and sample index are the next
expected values. Format changes, gaps, duplicate terminals, or an unterminated
stream are errors.

`TtsRequest` is the external request boundary owned by Dialogue.
`TtsSay` is the admitted request boundary from Dialogue to TTS. Both carry a
canonical request UUID; `TtsSay` additionally carries the exact current TTS
generation UUID. Dialogue is the sole publisher of `TtsSay`. A producer must
not bypass admission by publishing directly to TTS.

`TtsResult` is the typed terminal projection. `COMPLETED`, `FAILED`, and
`CANCELLED` always identify the accepted request and generation.
`REJECTED` makes a closed-admission or invalid request observable without
pretending it entered synthesis. `TtsRuntimeState` identifies one process
generation and carries a strictly increasing heartbeat sequence. Freshness is
measured by the receiver's monotonic clock; the message does not provide a
wall-clock fallback.

`TtsAdmissionState` is Dialogue's TRANSIENT_LOCAL projection of its TTS
admission decision. It carries a process-lifetime Dialogue controller UUID, a
strictly increasing heartbeat sequence, the observed TTS generation, OPEN or
CLOSED, and the exact CLOSED reason. A consumer may call voice response ready
only when both heartbeats are fresh, TTS runtime state is READY, admission is
OPEN, and both messages name the exact same TTS generation.

`TtsTimingEvent.TTS_TIMING_SCHEMA_VERSION` is the only accepted schema version,
and its `AGENT`/`SYSTEM` constants make the timing stream self-describing.
`TtsTimingEvent` uses Linux `CLOCK_MONOTONIC` nanoseconds and one exact
request/generation/utterance identity across the complete host-local path. It
records request admission, Japanese frontend completion, native request
start, first native PCM, first ROS PCM publication, first playback-frame
publication, CPAL's physical playback-start timestamp, and physical playback
end after the final device-drain deadline. Data-publication stages are sampled
immediately before their publish call and emitted only after that call
succeeds. `PHYSICAL_PLAYBACK_STARTED` and `PHYSICAL_PLAYBACK_ENDED` are
scheduled DAC boundaries derived from CPAL callback timing; neither is
callback-entry time or an audibility estimate. `PLAYBACK_UNDERRUN` reports only
the exact silence frames inserted while a non-final TTS utterance remains
active; ordinary idle-device silence is not an underrun.

The stages have multiple DDS writers, so arrival order is not a synchronization
contract. The dedicated collector retains the eight required stage timestamps
by exact identity and publishes `TtsTimingReceipt` only after all eight are
present and strictly increasing. A repeated required stage, identity drift,
timestamp reversal, bounded-chain timeout, or receipt-store failure is fatal.
The receipt includes the complete timestamps and accumulated underrun frames;
partial receipts and estimated timestamps do not exist.

`SetTtsAdmission` closes or opens exactly the named generation. Closing waits
for Dialogue's active turn interruption, targeted playback aborts, and
outstanding request maps to drain. `QuiesceTts` closes the native TTS recovery
domain and succeeds only after accepted requests reach native terminal events.
A timeout is a failure, not a successful degraded restart.

`VoiceMaintenanceState` is the process-independent admission gate used while
the complete voice group is being replaced. Scene Viewer owns and persists the
state, publishes it with transient-local durability, and changes it only
through `SetVoiceMaintenance`. `ENTER` must be reflected on the state topic
before any voice process is stopped. `EXIT` requires the exact operation UUID
that entered maintenance and is permitted only after the replacement Dialogue
is ready. TTS request producers reject locally while maintenance is active or
the state has not been observed; a volatile `TtsRequest` must never be
published into an absent Dialogue subscriber.

`TtsAlignmentEvent.sample_index` is an end-exclusive boundary in the native
TTS sample rate. `committed_text_tokens` indexes `TtsSourceProgress` directly;
consumers must not estimate source characters from elapsed time or token
counts. `PlaybackEvent.total_frames_valid=false` means streaming has not yet
established the final converted frame count; `total_frames` is zero in that
case.

The playback controller owns one `playout_generation` sequence per playback
kind. Agent pause/discard/abort increments only the Agent sequence; targeted
SYSTEM abort is ordered by its utterance-ID tombstone and does not invalidate
unrelated SYSTEM audio. Playback projections reject older frames and events
within the same kind; physical output invalidation uses the keyed flush service.
A controller seeds all three sequences from wall-clock milliseconds so they
remain ordered across controller-only restarts while still fitting JavaScript's
safe integer range.

Every user `PAUSE` is one transaction identified by a canonical lowercase UUID
in `pause_id`. `AGENT_PAUSED` acknowledges the logical hold immediately so UI
and browser playback stop without waiting for the device. It carries that same
UUID. An internal SYSTEM hold is not a user pause transaction and may therefore
emit `AGENT_PAUSED` with an empty `pause_id`.

`AGENT_PAUSE_COMMITTED` is the physical boundary. The controller publishes it
only after `/audio/output/flush` has succeeded and every keyed output chunk has
returned its terminal acknowledgement. Its frame and source-character
positions are the exact prefix accounted by those acknowledgements; consumers
must not treat `AGENT_PAUSED` as an exact heard position. If no Agent speech was
active, the commit still acknowledges the requested `pause_id`, with empty TTS
identity, zero frames, and invalid total/source positions. No other empty
identity form is valid.

Except for `PlaybackControl.NONE`, zero-valued enum fields are invalid. Empty
or otherwise unused control fields are interpreted according to `action`:

| Action | Additional fields |
| --- | --- |
| `PAUSE` | canonical lowercase UUID `pause_id`; `release_hold=NONE`; epoch and utterance ID unused |
| `RESUME` | empty `pause_id`; `release_hold=NONE`; epoch and ID unused |
| `DISCARD` | release `USER` or `SYSTEM`; minimum agent epoch; SYSTEM hold ID |
| `ABORT_SYSTEM` | target ID; `SYSTEM` releases its hold, `NONE` preserves it |
| `ABORT_AGENT` | target ID; `release_hold=NONE`; epoch unused |

`DISCARD`, `ABORT_SYSTEM`, and `ABORT_AGENT` also require an empty `pause_id`.

`ABORT_AGENT` and `ABORT_SYSTEM` are also the fail-closed feedback path from
the playback controller to the TTS scheduler. If an otherwise identifiable
utterance violates the streaming PCM contract, playback tombstones and flushes
that utterance locally, then publishes the matching targeted abort so native
generation cannot continue unseen on the GPU.

```bash
colcon build --packages-select fv_audio_interfaces
pytest -q src/audio/fv_audio_interfaces/test/test_interfaces.py
```
