from __future__ import annotations

from dataclasses import dataclass
import wave


@dataclass(frozen=True)
class WavPcm:
    data: bytes
    sample_rate_hz: int
    channels: int


def load_wav(path: str) -> WavPcm:
    with wave.open(path, "rb") as wav:
        if wav.getcomptype() != "NONE":
            raise ValueError(f"cue WAV must be uncompressed PCM: {path}")
        if wav.getsampwidth() != 2:
            raise ValueError(f"cue WAV must be PCM16: {path}")
        channels = wav.getnchannels()
        sample_rate_hz = wav.getframerate()
        data = wav.readframes(wav.getnframes())
    if channels <= 0 or sample_rate_hz <= 0 or not data:
        raise ValueError(f"cue WAV is empty or has invalid format: {path}")
    return WavPcm(data=data, sample_rate_hz=sample_rate_hz, channels=channels)
