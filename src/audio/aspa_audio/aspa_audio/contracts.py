"""Strict JSON contracts for playback control and event topics."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from typing import Literal


PlaybackAction = Literal["pause", "resume", "discard", "system_abort"]
PlaybackKind = Literal["agent", "system", "cue"]
PlaybackHold = Literal["user", "system"]


@dataclass(frozen=True)
class PlaybackControl:
    action: PlaybackAction
    floor_epoch: int | None = None
    release_hold: PlaybackHold | None = None
    utterance_id: str | None = None

    @classmethod
    def from_json(cls, payload: str) -> "PlaybackControl":
        try:
            value = json.loads(payload)
        except json.JSONDecodeError as exc:
            raise ValueError("playback control must be valid JSON") from exc
        if not isinstance(value, dict) or not set(value).issubset(
            {"action", "floor_epoch", "release_hold", "utterance_id"}
        ):
            raise ValueError("playback control contains unsupported fields")
        if "action" not in value:
            raise ValueError("playback control requires action")
        action = value["action"]
        if action not in {"pause", "resume", "discard", "system_abort"}:
            raise ValueError(f"unsupported playback action: {action!r}")
        floor_epoch = value.get("floor_epoch")
        release_hold = value.get("release_hold")
        utterance_id = value.get("utterance_id")
        if action == "discard":
            if not isinstance(floor_epoch, int) or isinstance(floor_epoch, bool) or floor_epoch < 0:
                raise ValueError("discard requires a non-negative floor_epoch")
            if release_hold not in {"user", "system"}:
                raise ValueError("discard requires release_hold=user or system")
            if release_hold == "system":
                if not isinstance(utterance_id, str) or not utterance_id:
                    raise ValueError("system discard requires utterance_id")
            elif utterance_id is not None:
                raise ValueError("user discard must not specify utterance_id")
        elif action == "system_abort":
            if not isinstance(utterance_id, str) or not utterance_id:
                raise ValueError("system_abort requires utterance_id")
            if floor_epoch is not None:
                raise ValueError("system_abort does not accept floor_epoch")
            if release_hold not in {None, "system"}:
                raise ValueError("system_abort release_hold must be system when provided")
        elif floor_epoch is not None:
            raise ValueError("floor_epoch is only valid for discard")
        elif release_hold is not None:
            raise ValueError("release_hold is only valid for discard")
        elif utterance_id is not None:
            raise ValueError("utterance_id is only valid for SYSTEM control")
        return cls(
            action=action,
            floor_epoch=floor_epoch,
            release_hold=release_hold,
            utterance_id=utterance_id,
        )

    def to_json(self) -> str:
        return json.dumps(asdict(self), separators=(",", ":"), sort_keys=True)


@dataclass(frozen=True)
class PlaybackEvent:
    event: str
    kind: PlaybackKind
    utterance_id: str
    played_frames: int
    total_frames: int

    def to_json(self) -> str:
        return json.dumps(asdict(self), separators=(",", ":"), sort_keys=True)

    @classmethod
    def from_json(cls, payload: str) -> "PlaybackEvent":
        try:
            value = json.loads(payload)
        except json.JSONDecodeError as exc:
            raise ValueError("playback event must be valid JSON") from exc
        expected = {"event", "kind", "utterance_id", "played_frames", "total_frames"}
        if not isinstance(value, dict) or set(value) != expected:
            raise ValueError("playback event fields do not match the contract")
        if value["kind"] not in {"agent", "system", "cue"}:
            raise ValueError("invalid playback event kind")
        if not isinstance(value["utterance_id"], str) or not value["utterance_id"]:
            raise ValueError("utterance_id must not be empty")
        for field in ("played_frames", "total_frames"):
            if not isinstance(value[field], int) or value[field] < 0:
                raise ValueError(f"{field} must be a non-negative integer")
        return cls(**value)


@dataclass(frozen=True)
class OutputDrained:
    utterance_id: str
    seq: int
    frame_count: int
    final: bool
    status: Literal["accepted", "drained", "flushed"] = "drained"

    def to_json(self) -> str:
        return json.dumps(asdict(self), separators=(",", ":"), sort_keys=True)

    @classmethod
    def from_json(cls, payload: str) -> "OutputDrained":
        try:
            value = json.loads(payload)
        except json.JSONDecodeError as exc:
            raise ValueError("output drained ack must be valid JSON") from exc
        expected = {"utterance_id", "seq", "frame_count", "final", "status"}
        if not isinstance(value, dict) or set(value) != expected:
            raise ValueError("output drained ack fields do not match the contract")
        if not isinstance(value["utterance_id"], str) or not value["utterance_id"]:
            raise ValueError("output drained utterance_id must not be empty")
        for field in ("seq", "frame_count"):
            if not isinstance(value[field], int) or isinstance(value[field], bool) or value[field] < 0:
                raise ValueError(f"output drained {field} must be non-negative")
        if not isinstance(value["final"], bool):
            raise ValueError("output drained final must be bool")
        if value["status"] not in {"accepted", "drained", "flushed"}:
            raise ValueError(
                "output drained status must be accepted, drained, or flushed"
            )
        if value["status"] == "flushed" and value["final"]:
            raise ValueError("a flushed output chunk cannot be final")
        return cls(**value)
