# fv_speech

Rust ROS 2 speech processing for FluentVision. The package owns three nodes and
does not own dialogue state, an LLM client, TTS, or physical audio I/O.

## Nodes

### `silero_vad`

- subscribes: `/audio/mic/frame` (`fv_speech_interfaces/msg/AudioFrame`)
- publishes: `/dialogue/vad/activity` (`fv_speech_interfaces/msg/VoiceActivity`)
- accepts only WebRTC AEC3-cleaned 16 kHz mono PCM16LE from `aspa_audio_aec`
- loads the packaged `models/silero_vad_16k_op15.onnx` and verifies its SHA-256

### `turn_detector`

- subscribes: `/dialogue/vad/activity`
- publishes: `/dialogue/turn/event` (`fv_speech_interfaces/msg/TurnEvent`)
- emits deterministic `started` and `ended` boundaries from sample indices

### `parakeet_asr`

- subscribes: `/audio/mic/frame`
- accepts only WebRTC AEC3-cleaned PCM from `aspa_audio_aec`; raw capture is not
  a valid ASR input
- subscribes: `/dialogue/asr/control` (`fv_speech_interfaces/msg/AsrControl`)
- publishes: `/dialogue/asr/transcript` (`fv_speech_interfaces/msg/Transcript`)
- publishes transient-local readiness on `/dialogue/asr/ready`
- validates and loads the pinned CUDA runtime manifest supplied through
  `ASPA_PARAKEET_RUNTIME_MANIFEST`
- requires a schema 7 verification receipt that binds the driver, CUDA toolkit,
  nvcc build, model, CUDA runtime,
  exact Japanese golden WAV/transcript, probe log, and Nsight CUDA-kernel report
  by SHA-256; older receipts are rejected by the production path
- rechecks the receipt log for CUDA EP, TF32-off, exact cold parity, and warm
  parity instead of trusting a boolean in the manifest
- disables TF32 so the production CUDA computation matches the FP32 contract

All stream identities, sequence numbers, frame counts, and sample spans are
validated fail-closed. The ASR runtime and model assets are external because
they are machine-specific; the Silero ONNX model is part of this package.

## Silero model provenance

`models/silero_vad_16k_op15.onnx` is the pinned 16 kHz, ONNX opset 15 model
from the [upstream Silero VAD project](https://github.com/snakers4/silero-vad).
It was carried into this package from
`Escenda/fluent-dialogue-dora@2a9090041a149d7966a65eb4d55125138914c3c9`.

- Upstream copyright: `Copyright (c) 2020-present Silero Team`
- Upstream license: MIT; see `LICENSE.SILERO-VAD`
- SHA-256: `7ed98ddbad84ccac4cd0aeb3099049280713df825c610a8ed34543318f1b2c49`
- Size: `1,289,603` bytes

The runtime verifies this hash before loading the model. Replacing the model
requires updating the hash in `rust/src/vad.rs`, this README, the third-party
notice, and the package contract test.

```bash
scripts/setup-parakeet-rs-cuda --install \
  --model-dir /path/to/model-parakeet-rs-320ms \
  --wav /path/to/ja.wav
scripts/setup-parakeet-rs-cuda --check
ros2 launch fv_speech fv_speech.launch.py \
  parakeet_runtime_manifest:="${XDG_DATA_HOME:-$HOME/.local/share}/aspa-navigation/parakeet-cuda/runtime.json"
```

The launch file starts `parakeet_asr` and `silero_vad` through
`fv_speech_runtime_exec`. That Rust wrapper verifies the manifest and every
bound artifact before setting `ORT_DYLIB_PATH` and `LD_LIBRARY_PATH` only for
the selected child process. Direct launch therefore uses the same CUDA/cuDNN
runtime as `start-stack` without polluting the rest of the process tree.
Nsight and benchmark working profiles remain under `/tmp`; `--verify` copies
the verified runtime, model, golden WAV, and receipt into the persistent data
root before atomically publishing `runtime.json`.
