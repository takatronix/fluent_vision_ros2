from pathlib import Path
import wave

import pytest

from fv_soundboard.event_registry import EventRegistry, parse_event_name
from fv_soundboard.soundboard_node import load_wav


def test_event_contract_accepts_bare_json_and_ignores_legacy_speech_override():
    assert parse_event_name("ready") == "ready"
    assert parse_event_name('{"event":"ready"}') == "ready"
    assert parse_event_name(
        '{"event":"ready","say":"override","speak":true}'
    ) == "ready"
    with pytest.raises(ValueError, match="unknown"):
        parse_event_name('{"event":"ready","priority":99}')


def test_registry_keeps_only_fixed_speech(tmp_path: Path):
    sounds = tmp_path / "sounds"
    sounds.mkdir()
    cue = sounds / "ready.wav"
    with wave.open(str(cue), "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(48_000)
        wav.writeframes(b"\x00\x00" * 16)
    registry_path = tmp_path / "events.yaml"
    registry_path.write_text(
        "version: 1\nevents:\n"
        "  ready:\n    sound: ready\n    say: '準備完了'\n    speak: true\n"
        "    record: system_ready\n"
        "  cue_only:\n    sound: ready\n    say: '喋らない'\n    speak: false\n",
        encoding="utf-8",
    )
    registry = EventRegistry.load(registry_path, sounds)
    assert registry.resolve("ready").say == "準備完了"
    assert registry.resolve("ready").record == "system_ready"
    assert registry.resolve("cue_only").say == ""
    assert registry.resolve("cue_only").record == ""
    with pytest.raises(ValueError, match="unregistered"):
        registry.resolve("unknown")


def test_load_wav_preserves_pcm_format(tmp_path: Path):
    path = tmp_path / "cue.wav"
    with wave.open(str(path), "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(48_000)
        wav.writeframes(b"\x01\x00" * 480)
    audio = load_wav(str(path))
    assert audio.sample_rate_hz == 48_000
    assert audio.channels == 1
    assert len(audio.data) == 960
