from io import BytesIO
import json
import wave

import pytest

from fv_tts_py.contracts import SayRequest
from fv_tts_py.voicevox_backend import decode_pcm16_wav


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
