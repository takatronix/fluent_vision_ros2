from __future__ import annotations

from dataclasses import dataclass
import json
from typing import Literal


@dataclass(frozen=True)
class SayRequest:
    kind: Literal["agent", "system"]
    utterance_id: str
    text: str

    @classmethod
    def from_json(cls, payload: str) -> "SayRequest":
        try:
            value = json.loads(payload)
        except json.JSONDecodeError as exc:
            raise ValueError("TTS request must be valid JSON") from exc
        expected = {"kind", "utterance_id", "text"}
        if not isinstance(value, dict) or set(value) != expected:
            raise ValueError("TTS request fields must be kind, utterance_id, and text")
        if value["kind"] not in {"agent", "system"}:
            raise ValueError("TTS kind must be agent or system")
        for field in ("utterance_id", "text"):
            if not isinstance(value[field], str) or not value[field].strip():
                raise ValueError(f"TTS {field} must not be empty")
        return cls(
            kind=value["kind"],
            utterance_id=value["utterance_id"],
            text=value["text"].strip(),
        )
