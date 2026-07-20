from __future__ import annotations

from dataclasses import dataclass
from io import BytesIO
from pathlib import Path
import wave


@dataclass(frozen=True)
class SynthesizedAudio:
    pcm: bytes
    sample_rate_hz: int
    channels: int
    bit_depth: int = 16


def decode_pcm16_wav(data: bytes) -> SynthesizedAudio:
    try:
        with wave.open(BytesIO(data), "rb") as wav:
            if wav.getcomptype() != "NONE" or wav.getsampwidth() != 2:
                raise ValueError("VOICEVOX output must be uncompressed PCM16 WAV")
            channels = wav.getnchannels()
            sample_rate = wav.getframerate()
            pcm = wav.readframes(wav.getnframes())
    except (EOFError, wave.Error) as exc:
        raise ValueError("VOICEVOX returned an invalid WAV payload") from exc
    if channels <= 0 or sample_rate <= 0 or not pcm:
        raise ValueError("VOICEVOX returned empty or malformed audio")
    return SynthesizedAudio(pcm, sample_rate, channels)


class VoicevoxCoreBackend:
    """Blocking native voicevox_core backend; no HTTP Engine is involved."""

    def __init__(
        self,
        *,
        onnxruntime_filename: str,
        open_jtalk_dict_dir: str,
        voice_model_path: str,
        acceleration_mode: str,
        cpu_num_threads: int,
        style_id: int,
    ) -> None:
        required = {
            "onnxruntime_filename": onnxruntime_filename,
            "open_jtalk_dict_dir": open_jtalk_dict_dir,
            "voice_model_path": voice_model_path,
        }
        missing = [name for name, value in required.items() if not value.strip()]
        if missing:
            raise ValueError(f"missing VOICEVOX path parameter(s): {', '.join(missing)}")
        if cpu_num_threads < 0 or style_id < 0:
            raise ValueError("cpu_num_threads and style_id must be non-negative")
        try:
            from voicevox_core import AccelerationMode
            from voicevox_core.blocking import (
                Onnxruntime,
                OpenJtalk,
                Synthesizer,
                VoiceModelFile,
            )
        except ImportError as exc:
            raise RuntimeError("voicevox_core Python package is not installed") from exc

        modes = {
            "auto": AccelerationMode.AUTO,
            "cpu": AccelerationMode.CPU,
            "gpu": AccelerationMode.GPU,
        }
        try:
            mode = modes[acceleration_mode.lower()]
        except KeyError as exc:
            raise ValueError("acceleration_mode must be auto, cpu, or gpu") from exc

        runtime_path = Path(onnxruntime_filename).expanduser().resolve()
        dictionary_path = Path(open_jtalk_dict_dir).expanduser().resolve()
        model_path = Path(voice_model_path).expanduser().resolve()
        if not runtime_path.is_file():
            raise FileNotFoundError(f"VOICEVOX ONNX Runtime was not found: {runtime_path}")
        if not dictionary_path.is_dir():
            raise FileNotFoundError(f"Open JTalk dictionary was not found: {dictionary_path}")
        if not model_path.is_file():
            raise FileNotFoundError(f"VOICEVOX voice model was not found: {model_path}")

        onnxruntime = Onnxruntime.load_once(filename=runtime_path)
        self._synthesizer = Synthesizer(
            onnxruntime,
            OpenJtalk(dictionary_path),
            acceleration_mode=mode,
            cpu_num_threads=cpu_num_threads,
        )
        with VoiceModelFile.open(model_path) as model:
            self._synthesizer.load_voice_model(model)
        self._style_id = style_id

    def synthesize(self, text: str) -> SynthesizedAudio:
        query = self._synthesizer.create_audio_query(text, self._style_id)
        wav = self._synthesizer.synthesis(query, self._style_id)
        return decode_pcm16_wav(wav)
