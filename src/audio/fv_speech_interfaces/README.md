# fv_speech_interfaces

Typed ROS 2 boundary for the FluentVision speech pipeline. This package contains
exactly five messages and no nodes or services:

- `AudioFrame`: timestamped PCM frames shared by capture, TTS, and playback.
- `VoiceActivity`: frame-aligned Silero speech/silence decisions.
- `TurnEvent`: user-turn start/end events derived from voice activity.
- `Transcript`: partial or final ASR text for a user turn.
- `AsrControl`: start, stop, or cancel commands for streaming ASR.

The messages preserve stream identity, sequence, and sample-index fields so
nodes can reject gaps or mismatched streams instead of guessing.
