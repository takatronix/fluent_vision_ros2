# fv_kws

Rust/r2r keyword spotting node for the fixed wake phrase `アスパ`.

The node consumes the existing `/audio/mic/frame` PCM stream and
`/dialogue/vad/activity`. `silero_vad` is unchanged and has no KWS-specific
publisher. `fv_kws` buffers the raw PCM by absolute sample index, extracts the
exact 512-sample spans described by `VoiceActivity`, and owns candidate
segmentation and the shorter endpoint. It decodes only while
`/aspa/dialogue/session_state` is `sleeping`.

The Japanese Vosk model and one grammar recognizer are loaded once. Candidate
utterances use a KWS-only 192 ms silence endpoint. After every candidate,
`final_result()` is read and the same recognizer is reset. The accepted result
must contain the consecutive Vosk tokens `アス パ`. Acoustic `[unk]` tokens may
surround the pair; partial forms such as `アス` and different words such as
`アス パラ` are rejected.

Install and verify the pinned Vosk runtime and model before building:

```bash
scripts/setup-vosk-kws --install
scripts/setup-vosk-kws --check
```

The node waits for the transient-local dialogue session state before publishing
`/dialogue/kws/ready=true`. Unknown session state, input discontinuity, runtime
receipt mismatch, or stream termination fails closed.
