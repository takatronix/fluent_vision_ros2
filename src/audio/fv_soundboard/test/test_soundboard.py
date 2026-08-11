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
        "  cue_only:\n    sound: ready\n    say: '喋らない'\n    speak: false\n"
        "  marker_only:\n    sound: ''\n    say: ''\n    speak: false\n"
        "    record: person_lost\n",
        encoding="utf-8",
    )
    registry = EventRegistry.load(registry_path, sounds)
    assert registry.resolve("ready").say == "準備完了"
    assert registry.resolve("ready").record == "system_ready"
    assert registry.resolve("cue_only").say == ""
    assert registry.resolve("cue_only").record == ""
    assert registry.resolve("marker_only").sound_path is None
    assert registry.resolve("marker_only").say == ""
    assert registry.resolve("marker_only").record == "person_lost"
    with pytest.raises(ValueError, match="unregistered"):
        registry.resolve("unknown")


def test_registry_rejects_an_event_without_audio_speech_or_marker(tmp_path: Path):
    sounds = tmp_path / "sounds"
    sounds.mkdir()
    registry_path = tmp_path / "events.yaml"
    registry_path.write_text(
        "version: 1\nevents:\n"
        "  empty:\n    sound: ''\n    say: ''\n    speak: false\n    record: ''\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="nor record marker"):
        EventRegistry.load(registry_path, sounds)


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
    from fv_audio_interfaces.msg import (
        TtsRequest,
        TtsResult,
        VoiceMaintenanceState,
    )
    from fv_soundboard.event_registry import ResolvedEvent
    from fv_soundboard.soundboard_node import SoundboardNode

    actions = []

    class Publisher:
        def __init__(self, name):
            self.name = name

        def publish(self, message):
            if self.name == "marker":
                actions.append((self.name, message.data))
                return
            if self.name == "say":
                assert isinstance(message, TtsRequest)
                actions.append((self.name, message.text))
                return
            assert isinstance(message, TtsResult)
            actions.append((self.name, message.error))

    event = ResolvedEvent("ready", "準備完了", None, "system_ready")

    class Controller:
        _handle_event = SoundboardNode._handle_event
        _reject_tts_request = SoundboardNode._reject_tts_request

    controller = Controller()
    controller._registry = SimpleNamespace(resolve=lambda _payload: event)
    controller._settings = SoundSettings(True, frozenset())
    controller._marker_pub = Publisher("marker")
    controller._say_pub = Publisher("say")
    controller._tts_result_pub = Publisher("result")
    controller._maintenance_state = VoiceMaintenanceState(
        version=1, active=False, operation_id="", reason="")
    controller.get_logger = lambda: SimpleNamespace(
        error=lambda _message: None,
        warning=lambda _message: None,
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


def test_soundboard_rejects_speech_until_maintenance_state_is_known():
    pytest.importorskip("ament_index_python")
    pytest.importorskip("rclpy")
    pytest.importorskip("fv_speech_interfaces")
    from fv_audio_interfaces.msg import TtsResult
    from fv_soundboard.event_registry import ResolvedEvent
    from fv_soundboard.soundboard_node import SoundboardNode

    requests = []
    results = []

    class Publisher:
        def __init__(self, target):
            self._target = target

        def publish(self, message):
            self._target.append(message)

    class Controller:
        _handle_event = SoundboardNode._handle_event
        _reject_tts_request = SoundboardNode._reject_tts_request

    controller = Controller()
    controller._registry = SimpleNamespace(
        resolve=lambda _payload: ResolvedEvent(
            "ready", "準備完了", None, ""))
    controller._settings = SoundSettings(True, frozenset())
    controller._marker_pub = Publisher([])
    controller._say_pub = Publisher(requests)
    controller._tts_result_pub = Publisher(results)
    controller._maintenance_state = None
    controller.get_logger = lambda: SimpleNamespace(
        error=lambda _message: None,
        warning=lambda _message: None,
    )

    controller._handle_event("ready", preview=False)

    assert requests == []
    assert len(results) == 1
    result = results[0]
    assert result.kind == TtsResult.SYSTEM
    assert result.status == TtsResult.REJECTED
    assert result.generation_valid is False
    assert result.generation_id == ""
    assert result.request_id
    assert result.utterance_id.startswith("system-ready-")
    assert result.error == "voice maintenance state has not been observed"


def test_soundboard_active_maintenance_is_acknowledged_and_rejected():
    pytest.importorskip("ament_index_python")
    pytest.importorskip("rclpy")
    pytest.importorskip("fv_speech_interfaces")
    from fv_audio_interfaces.msg import (
        TtsResult,
        VoiceMaintenanceState,
    )
    from fv_soundboard.event_registry import ResolvedEvent
    from fv_soundboard.soundboard_node import SoundboardNode

    requests = []
    results = []
    acknowledgements = []

    class Publisher:
        def __init__(self, target):
            self._target = target

        def publish(self, message):
            self._target.append(message)

    class Controller:
        _handle_event = SoundboardNode._handle_event
        _reject_tts_request = SoundboardNode._reject_tts_request
        _on_voice_maintenance = SoundboardNode._on_voice_maintenance

    controller = Controller()
    controller._registry = SimpleNamespace(
        resolve=lambda _payload: ResolvedEvent(
            "ready", "準備完了", None, ""))
    controller._settings = SoundSettings(True, frozenset())
    controller._marker_pub = Publisher([])
    controller._say_pub = Publisher(requests)
    controller._tts_result_pub = Publisher(results)
    controller._maintenance_ack_pub = Publisher(acknowledgements)
    controller._maintenance_state = None
    controller.get_logger = lambda: SimpleNamespace(
        error=lambda _message: None,
        warning=lambda _message: None,
    )
    state = VoiceMaintenanceState(
        version=1,
        active=True,
        operation_id="6a5717aa-18ef-4d1c-a308-9ac5d2139582",
        reason="restart-voice",
    )

    controller._on_voice_maintenance(state)
    controller._handle_event("ready", preview=False)

    assert acknowledgements == [state]
    assert requests == []
    assert len(results) == 1
    result = results[0]
    assert result.status == TtsResult.REJECTED
    assert result.generation_valid is False
    assert state.operation_id in result.error
