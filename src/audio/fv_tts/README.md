# fv_tts

`fv_tts` is the Rust/r2r streaming synthesis node for ASPA voice dialogue. It
loads one authenticated MagpieTTS-RT bundle whose baked public voice is Sofia.
There is no mutable speaker/style API and no VOICEVOX or HTTP fallback.
The r2r build generates bindings only for `fv_audio_interfaces` and
`std_srvs`; unrelated or stale IDL packages in the surrounding workspace
cannot alter this package's generated bindings.

## Startup gate

The node becomes ready only after all of the following succeed:

1. the explicit MagpieTTS-RT shared library exposes C ABI v1;
2. the bundle manifest matches the configured authenticated SHA-256;
3. MagpieTTS-RT creates its session and passes the mandatory bundle golden
   generation (this also warms the CUDA context, TensorRT execution contexts,
   and CUDA streams);
4. the persistent Japanese frontend verifies its Open JTalk dictionary and
   tokenizer assets;
5. the frontend ready record exactly equals `model_get_info`: tokenizer
   identity SHA-256, `tokenizer_vocabulary_size=3357`,
   `text_embedding_rows=3359`, `bos_token_id=3357`, `eos_token_id=3358`, and
   `japanese_global_pad_token_id=1015`;
6. the `/aspa/tts/timing` publisher has discovered at least one inter-process
   subscriber for the dedicated timing collector.

Any mismatch stops this recovery domain. The node does not guess a tokenizer,
select a different model, or fall back to another synthesizer.

`/aspa/tts/ready` is not advertised during startup. It appears only after that
gate has passed and then returns the admitted generation identity.
`/aspa/tts/status` is a TRANSIENT_LOCAL
`fv_audio_interfaces/msg/TtsRuntimeState`. It carries a process-lifetime UUID,
strictly increasing heartbeat sequence, fixed voice, bundle and tokenizer
identities, CUDA device, authenticated frame schedule, process ID, and the
last typed request result. Consumers measure heartbeat freshness with their own
monotonic receive clock. A cached sample or a sample from a previous generation
cannot satisfy readiness.
The collector dependency is checked again after native startup and before
READY is published. At runtime it is polled on a fixed 20 ms maximum interval,
including while SAY messages are being drained, so DDS graph introspection is
not performed in every 2 ms ROS loop. Once the DDS API reports that every
inter-process timing subscriber is gone, the process detects that state within
the next 20 ms poll, publishes FAILED, disables both playback streams, rejects
further admission, and terminates the TTS recovery process. A fixed startup
delay is not used as a substitute for DDS discovery;
`timing_collector_discovery_timeout_seconds` is only the explicit fail-closed
deadline when the required subscriber never appears.

## Request and streaming output

External speech requests enter Dialogue through
`/aspa/dialogue/tts/request` as
`fv_audio_interfaces/msg/TtsRequest`. Dialogue is the sole publisher of
`/aspa/tts/say`, whose type is `fv_audio_interfaces/msg/TtsSay`. Each admitted
message contains its speech kind, canonical request UUID, current TTS
generation UUID, utterance ID, and exact text. Agent IDs are
`agent-<floor_epoch>-<id>` and every ID is at most 256 UTF-8 bytes. SAY and
frontend JSONL are bounded to one MiB before a request can enter the scheduler.
`fv_tts` rejects a request UUID while it is live or in a 1,024-entry terminal
reorder window; the bounded receiver window detects DDS duplicates without
retaining every historical ID for the lifetime of the process.

Dialogue closes admission with
`fv_audio_interfaces/srv/SetTtsAdmission` before a restart. The TTS
`fv_audio_interfaces/srv/QuiesceTts` service then disables both playback
streams, requests native cancellation, and returns success only after every
accepted request reaches its native terminal event and the request ledger is
empty. The process remains non-ready after quiesce. Dialogue may open only the
new, fresh generation that passed the complete startup gate.
The exact source text, including surrounding whitespace, is sent to one
persistent strict-JSONL Japanese frontend process. The frontend returns
prepared token IDs and the complete end-exclusive prepared-token to
source-character/UTF-8 map as one validated response. Its typed
`independent_sentence_segments_v1` segments contain at most 80 OpenJTalk NJD
words and end only at a certified source boundary plus sentence boundary. A
longer single sentence fails closed. Each segment starts fresh native model
state; this is not the training-time Magpie chunk-history path. ROS still
exposes one logical SAY with continuous sequence/sample/token/source offsets,
one final AUDIO (a one-through-eight-frame tail or a zero-frame control
marker), and one cancel/result.

Every prepared segment must end in exactly the authenticated EOS token. All
earlier positions must be normal tokenizer rows below `3357`. BOS, an interior
EOS, or any other embedding-only row is rejected before a native synthesis
request can be created; the node does not rewrite or drop an invalid token.

One persistent MagpieTTS-RT session serializes synthesis. Pending SYSTEM
requests overtake pending agent requests. `/audio/playback/control` advances
the agent floor and cancels stale native requests, or aborts one targeted
SYSTEM request. A bounded tombstone handles an abort that arrives before its
SYSTEM SAY message. The node also checks both playback subscriptions; losing
the subscriber for one stream cancels its active and queued requests instead
of generating audio that cannot be consumed.

Audio is published without buffering the utterance:

- Agent: `/audio/agent/frame`
- SYSTEM: `/audio/system/frame`
- Type: `fv_audio_interfaces/msg/SynthesizedSpeechChunk`
- Format: 22,050 Hz mono float32

Native segment chunks are rechunked into one logical ROS sequence while
preserving exact sample, token, and source progress. The initial four codec
frames are published immediately. Every complete steady batch of eight frames
is also published by the same scheduler call that receives it; it does not wait
for another frame or end-of-stream. Only an incomplete residual waits for the
logical terminal. The final flag is not inferred from chunk size because a
zero-frame control marker and an eight-frame native tail are both valid final
AUDIO. A zero-frame final is non-first, carries no alignment advance, advances
only sequence, and never enters the PCM converter.
AUDIO sequence zero carries the exact source text and complete progress table;
later AUDIO chunks do not repeat it. Normal completion is a zero-payload
COMPLETE only after the explicitly marked native final lease and request
completion. Failure or cancellation is one zero-payload ABORT. Every valid
accepted request produces exactly one of these terminal messages and one typed
`fv_audio_interfaces/msg/TtsResult` on `/aspa/tts/result`. The terminal result
repeats request UUID, generation UUID, utterance ID, and speech kind so
Dialogue can reject stale or cross-wired completion.

## Timing telemetry

Every accepted request is measured on `/aspa/tts/timing` with
`fv_audio_interfaces/msg/TtsTimingEvent.TTS_TIMING_SCHEMA_VERSION`. All stages carry the same
speech kind, request UUID, generation UUID, and utterance ID and use Linux
`CLOCK_MONOTONIC` nanoseconds, so host-local durations can be subtracted
without a wall-clock conversion. `REQUEST_ACCEPTED` is emitted only after the
bounded scheduler accepts the request. `FRONTEND_COMPLETED` follows the exact
Japanese frontend result, `NATIVE_REQUEST_STARTED` is sampled immediately
before the native request call, and `FIRST_NATIVE_AUDIO` is sampled when its
first valid PCM event reaches the scheduler. `FIRST_ROS_AUDIO_PUBLISHED` is
sampled immediately before the first ROS PCM publish and emitted only if that
publish succeeds. The data boundary is sampled before publishing so a
downstream process cannot produce a causally later stage with an earlier
timestamp.

The playback controller and physical output append first playback-frame
publication, scheduled physical start, and physical end after final device
drain; their exact semantics are documented by `fv_audio`. These stages have
multiple DDS writers, so their arrival order is not used as an ordering
guarantee. The dedicated `tts_timing_collector` joins all eight required
timestamps by exact identity and publishes one immutable
`TtsTimingReceipt` only after the complete strictly increasing chain exists.
A missing stage is not filled with an estimate. Missing-stage timeout,
duplicate stage, invalid identity, timestamp reversal, bounded-state overflow,
or durable receipt failure terminates the audio launch group and leaves voice
admission closed.

The frontend response slot and worker-to-ROS channel each have capacity one.
If ROS publishing stops, synthesis backpressure remains bounded while cancel
and shutdown commands continue to be handled by the dedicated Magpie inference
thread. Shutdown drains pending AUDIO and the single ABORT/result for accepted
active or queued requests before publishing the final process status. If the
native teardown misses its explicit deadline, the final status is `failed`;
the node does not publish a misleading `stopped` state.

The deterministic native seed is the little-endian first 32 bits of SHA-256
over the domain `fv_tts/magpie-seed/v1`, the authenticated manifest digest, and
length-framed utterance ID and source text. It is stable without sharing a
mutable random generator across requests.

## Runtime configuration

All asset locations are explicit absolute paths. The launch file reads them
from:

```bash
export MAGPIE_TTS_RT_NATIVE_LIBRARY=/opt/magpie-tts-rt/lib/libmagpie_tts_rt.so
export MAGPIE_TTS_RT_BUNDLE=/opt/magpie-tts-rt/bundles/sofia
export MAGPIE_TTS_RT_MANIFEST_SHA256=<64-hex authenticated digest>
export MAGPIE_TTS_RT_FRONTEND_PYTHON=/opt/magpie-tts-rt/frontend-venv/bin/python
export MAGPIE_TTS_RT_FRONTEND_SERVER=/opt/magpie-tts-rt/tools/frontend/japanese_frontend_server.py
export MAGPIE_TTS_RT_FRONTEND_LOCK=/opt/magpie-tts-rt/reference/oracle-lock.json
export MAGPIE_TTS_RT_FRONTEND_CONTRACT=/opt/magpie-tts-rt/receipts/frontend-contract.json
```

Then:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select fv_audio_interfaces fv_tts
ros2 launch fv_tts fv_tts.launch.py
```

`frontend_timeout_seconds`, `request_progress_timeout_seconds`,
`cancellation_timeout_seconds`, `startup_timeout_seconds`,
`timing_collector_discovery_timeout_seconds`, and
`scheduler_watchdog_seconds` are explicit positive finite deadlines.
Cancellation must reach one native terminal event before its shorter deadline.
A blocked scheduler watchdog terminates the process without joining the blocked
native call; the launch owner then recreates this single recovery domain.
