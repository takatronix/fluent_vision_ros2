from pathlib import Path
from types import SimpleNamespace
import wave

import pytest

from fv_soundboard.contracts import SoundSettings
from fv_soundboard.event_registry import EventRegistry, parse_event_name
from fv_soundboard.wav_pcm import load_wav


def test_sound_settings_contract_is_versioned_and_exact():
    settings = SoundSettings.from_json(
        '{"version":1,"robot_enabled":false,'
        '"disabled_events":["ready","warning","ready"]}'
    )
    assert not settings.robot_enabled
    assert settings.disabled_events == frozenset({"ready", "warning"})

    with pytest.raises(ValueError, match="fields"):
        SoundSettings.from_json(
            '{"version":1,"robot_enabled":true,"disabled_events":[],"volume":1}'
        )
    with pytest.raises(ValueError, match="version"):
        SoundSettings.from_json(
            '{"version":2,"robot_enabled":true,"disabled_events":[]}'
        )
    with pytest.raises(ValueError, match="event names"):
        SoundSettings.from_json(
            '{"version":1,"robot_enabled":true,"disabled_events":["Bad Name"]}'
        )


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


def test_normal_event_marks_before_gate_while_preview_only_plays():
    pytest.importorskip("ament_index_python")
    pytest.importorskip("rclpy")
    pytest.importorskip("fv_speech_interfaces")
    from fv_soundboard.event_registry import ResolvedEvent
    from fv_soundboard.soundboard_node import SoundboardNode

    actions = []

    class Publisher:
        def __init__(self, name):
            self.name = name

        def publish(self, message):
            actions.append((self.name, message.data))

    event = ResolvedEvent("ready", "準備完了", None, "system_ready")
    controller = SimpleNamespace(
        _registry=SimpleNamespace(resolve=lambda _payload: event),
        _settings=SoundSettings(True, frozenset()),
        _marker_pub=Publisher("marker"),
        _say_pub=Publisher("say"),
        get_logger=lambda: SimpleNamespace(error=lambda _message: None),
    )

    SoundboardNode._handle_event(controller, "ready", preview=False)
    assert [name for name, _payload in actions] == ["marker", "say"]

    actions.clear()
    controller._settings = SoundSettings(True, frozenset({"ready"}))
    SoundboardNode._handle_event(controller, "ready", preview=False)
    assert actions == [("marker", "system_ready")]

    actions.clear()
    controller._settings = SoundSettings(False, frozenset({"ready"}))
    SoundboardNode._handle_event(controller, "ready", preview=True)
    assert [name for name, _payload in actions] == ["say"]
