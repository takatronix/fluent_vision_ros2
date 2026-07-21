from io import BytesIO
import json
from threading import Event, Thread
import time
import wave

import pytest

from fv_tts_py.contracts import SayRequest, TtsResult
from fv_tts_py.synthesis_scheduler import SynthesisScheduler, agent_floor_epoch
from fv_tts_py.voicevox_backend import SynthesizedAudio, decode_pcm16_wav


def test_say_contract_is_exactly_three_fields():
    request = SayRequest.from_json(
        '{"kind":"system","utterance_id":"system-ready-1","text":"準備完了"}'
    )
    assert request.kind == "system"
    with pytest.raises(ValueError, match="fields"):
        SayRequest.from_json(json.dumps({
            "kind": "system",
            "utterance_id": "system-ready-1",
            "text": "準備完了",
            "voice": 3,
        }))


def test_voicevox_wav_is_decoded_without_sample_conversion():
    target = BytesIO()
    with wave.open(target, "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(24_000)
        wav.writeframes(b"\x01\x00\x02\x00")
    audio = decode_pcm16_wav(target.getvalue())
    assert audio.pcm == b"\x01\x00\x02\x00"
    assert (audio.sample_rate_hz, audio.channels, audio.bit_depth) == (24_000, 1, 16)


def test_tts_result_contract_distinguishes_synthesis_from_playback():
    completed = TtsResult("agent", "agent-1-a", "completed")
    assert TtsResult.from_json(completed.to_json()) == completed
    assert json.loads(completed.to_json()) == {
        "kind": "agent",
        "utterance_id": "agent-1-a",
        "status": "completed",
    }

    failed = TtsResult("system", "system-a", "failed", "native error")
    assert TtsResult.from_json(failed.to_json()) == failed
    with pytest.raises(ValueError, match="requires error"):
        TtsResult("agent", "agent-1-b", "failed")
    with pytest.raises(ValueError, match="fields do not match status"):
        TtsResult.from_json(
            '{"kind":"agent","utterance_id":"agent-1-b",'
            '"status":"completed","error":"unexpected"}'
        )


def _request(kind: str, utterance_id: str) -> SayRequest:
    return SayRequest(kind, utterance_id, utterance_id)


def _wait_until(predicate, timeout: float = 1.0) -> None:
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise AssertionError("timed out waiting for synthesis scheduler")
        time.sleep(0.005)


def _audio() -> SynthesizedAudio:
    return SynthesizedAudio(b"\x00\x00", 24_000, 1)


def test_system_overtakes_queued_agent_synthesis():
    active = Event()
    release = Event()
    delivered = []

    def synthesize(text):
        if text == "agent-0-active":
            active.set()
            assert release.wait(1.0)
        return _audio()

    scheduler = SynthesisScheduler(
        synthesize,
        lambda request, _audio: delivered.append(request.utterance_id),
        lambda _request, error: (_ for _ in ()).throw(error),
        lambda _request: None,
    )
    try:
        assert scheduler.submit(_request("agent", "agent-0-active")) == "accepted"
        assert active.wait(1.0)
        assert scheduler.submit(_request("agent", "agent-0-queued")) == "accepted"
        assert scheduler.submit(_request("system", "system-warning")) == "accepted"
        release.set()
        _wait_until(lambda: len(delivered) == 3)
        assert delivered == ["agent-0-active", "system-warning", "agent-0-queued"]
    finally:
        release.set()
        scheduler.close()


def test_discard_floor_cancels_queued_and_suppresses_active_stale_agent():
    active = Event()
    release = Event()
    delivered = []
    cancelled = []

    def synthesize(text):
        if text == "agent-0-active":
            active.set()
            assert release.wait(1.0)
        return _audio()

    scheduler = SynthesisScheduler(
        synthesize,
        lambda request, _audio: delivered.append(request.utterance_id),
        lambda _request, error: (_ for _ in ()).throw(error),
        lambda request: cancelled.append(request.utterance_id),
    )
    try:
        scheduler.submit(_request("agent", "agent-0-active"))
        assert active.wait(1.0)
        scheduler.submit(_request("agent", "agent-0-queued"))
        scheduler.submit(_request("system", "system-warning"))

        assert scheduler.advance_agent_floor(1) == ("agent-0-queued",)
        assert scheduler.submit(_request("agent", "agent-0-late")) == "stale"
        assert scheduler.submit(_request("agent", "agent-1-new")) == "accepted"
        release.set()

        _wait_until(lambda: len(delivered) == 2)
        assert delivered == ["system-warning", "agent-1-new"]
        assert cancelled == ["agent-0-queued", "agent-0-late", "agent-0-active"]
    finally:
        release.set()
        scheduler.close()


def test_system_abort_cancels_queued_synthesis():
    active = Event()
    release = Event()
    delivered = []
    cancelled = []

    def synthesize(text):
        if text == "agent-0-active":
            active.set()
            assert release.wait(1.0)
        return _audio()

    scheduler = SynthesisScheduler(
        synthesize,
        lambda request, _audio: delivered.append(request.utterance_id),
        lambda _request, error: (_ for _ in ()).throw(error),
        lambda request: cancelled.append(request.utterance_id),
    )
    try:
        scheduler.submit(_request("agent", "agent-0-active"))
        assert active.wait(1.0)
        scheduler.submit(_request("system", "system-warning"))
        assert scheduler.cancel_system("system-warning")
        release.set()
        _wait_until(lambda: delivered == ["agent-0-active"])
        time.sleep(0.02)
        assert delivered == ["agent-0-active"]
        assert cancelled == ["system-warning"]
    finally:
        release.set()
        scheduler.close()


def test_system_abort_suppresses_active_synthesis_and_worker_continues():
    active = Event()
    release = Event()
    delivered = []
    cancelled = []

    def synthesize(text):
        if text == "system-warning":
            active.set()
            assert release.wait(1.0)
        return _audio()

    scheduler = SynthesisScheduler(
        synthesize,
        lambda request, _audio: delivered.append(request.utterance_id),
        lambda _request, error: (_ for _ in ()).throw(error),
        lambda request: cancelled.append(request.utterance_id),
    )
    try:
        scheduler.submit(_request("system", "system-warning"))
        assert active.wait(1.0)
        assert scheduler.cancel_system("system-warning")
        scheduler.submit(_request("agent", "agent-0-next"))
        release.set()
        _wait_until(lambda: delivered == ["agent-0-next"])
        assert cancelled == ["system-warning"]
    finally:
        release.set()
        scheduler.close()


def test_agent_floor_advance_cannot_return_inside_pcm_publish_commit():
    publishing = Event()
    release_publish = Event()
    floor_returned = Event()
    delivered = []

    def completed(request, _audio):
        publishing.set()
        assert release_publish.wait(1.0)
        delivered.append(request.utterance_id)

    scheduler = SynthesisScheduler(
        lambda _text: _audio(),
        completed,
        lambda _request, error: (_ for _ in ()).throw(error),
        lambda _request: None,
    )
    try:
        scheduler.submit(_request("agent", "agent-0-race"))
        assert publishing.wait(1.0)

        def advance_floor():
            scheduler.advance_agent_floor(1)
            floor_returned.set()

        thread = Thread(target=advance_floor)
        thread.start()
        assert not floor_returned.wait(0.05)
        release_publish.set()
        thread.join(timeout=1.0)

        assert floor_returned.is_set()
        assert delivered == ["agent-0-race"]
    finally:
        release_publish.set()
        scheduler.close()


def test_system_abort_cannot_return_inside_pcm_publish_commit():
    publishing = Event()
    release_publish = Event()
    abort_returned = Event()
    delivered = []

    def completed(request, _audio):
        publishing.set()
        assert release_publish.wait(1.0)
        delivered.append(request.utterance_id)

    scheduler = SynthesisScheduler(
        lambda _text: _audio(),
        completed,
        lambda _request, error: (_ for _ in ()).throw(error),
        lambda _request: None,
    )
    try:
        scheduler.submit(_request("system", "system-race"))
        assert publishing.wait(1.0)

        def abort_system():
            scheduler.cancel_system("system-race")
            abort_returned.set()

        thread = Thread(target=abort_system)
        thread.start()
        assert not abort_returned.wait(0.05)
        release_publish.set()
        thread.join(timeout=1.0)

        assert abort_returned.is_set()
        assert delivered == ["system-race"]
    finally:
        release_publish.set()
        scheduler.close()


def test_system_abort_tombstone_handles_cross_topic_reordering():
    delivered = []
    cancelled = []
    scheduler = SynthesisScheduler(
        lambda _text: _audio(),
        lambda request, _audio: delivered.append(request.utterance_id),
        lambda _request, error: (_ for _ in ()).throw(error),
        lambda request: cancelled.append(request.utterance_id),
    )
    try:
        assert not scheduler.cancel_system("system-delayed")
        assert scheduler.submit(_request("system", "system-delayed")) == "cancelled"
        time.sleep(0.02)
        assert delivered == []
        assert cancelled == ["system-delayed"]
    finally:
        scheduler.close()


def test_synthesis_failure_reports_terminal_result_and_worker_continues():
    delivered = []
    failed = []

    def synthesize(text):
        if text == "agent-0-broken":
            raise RuntimeError("native failure")
        return _audio()

    scheduler = SynthesisScheduler(
        synthesize,
        lambda request, _audio: delivered.append(request.utterance_id),
        lambda request, error: failed.append((request.utterance_id, str(error))),
        lambda _request: None,
    )
    try:
        scheduler.submit(_request("agent", "agent-0-broken"))
        scheduler.submit(_request("agent", "agent-0-next"))
        _wait_until(lambda: delivered == ["agent-0-next"])
        assert failed == [("agent-0-broken", "native failure")]
    finally:
        scheduler.close()


@pytest.mark.parametrize("utterance_id", ["agent-x-id", "agent--1-id", "other-0-id"])
def test_agent_floor_epoch_rejects_invalid_id(utterance_id):
    with pytest.raises(ValueError):
        agent_floor_epoch(utterance_id)
