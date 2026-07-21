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


@dataclass(frozen=True)
class TtsResult:
    kind: Literal["agent", "system"]
    utterance_id: str
    status: Literal["completed", "failed", "cancelled"]
    error: str | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.kind, str) or self.kind not in {"agent", "system"}:
            raise ValueError("TTS result kind must be agent or system")
        if not isinstance(self.utterance_id, str) or not self.utterance_id.strip():
            raise ValueError("TTS result utterance_id must not be empty")
        if not isinstance(self.status, str) or self.status not in {
            "completed",
            "failed",
            "cancelled",
        }:
            raise ValueError("invalid TTS result status")
        if self.status == "failed":
            if not isinstance(self.error, str) or not self.error.strip():
                raise ValueError("failed TTS result requires error")
        elif self.error is not None:
            raise ValueError("only failed TTS result accepts error")

    @classmethod
    def from_json(cls, payload: str) -> "TtsResult":
        try:
            value = json.loads(payload)
        except json.JSONDecodeError as exc:
            raise ValueError("TTS result must be valid JSON") from exc
        if not isinstance(value, dict):
            raise ValueError("TTS result must be an object")
        expected = {"kind", "utterance_id", "status"}
        if value.get("status") == "failed":
            expected.add("error")
        if set(value) != expected:
            raise ValueError("TTS result fields do not match status")
        return cls(**value)

    def to_json(self) -> str:
        value = {
            "kind": self.kind,
            "utterance_id": self.utterance_id,
            "status": self.status,
        }
        if self.error is not None:
            value["error"] = self.error
        return json.dumps(value, ensure_ascii=False, separators=(",", ":"))
