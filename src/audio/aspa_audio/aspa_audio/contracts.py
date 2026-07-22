"""Strict domain conversion for the typed ASPA playback interfaces."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal


PlaybackAction = Literal["pause", "resume", "discard", "system_abort"]
PlaybackKind = Literal["agent", "system", "cue"]
PlaybackHold = Literal["user", "system"]
PlaybackStatus = Literal["accepted", "drained", "flushed"]

ACTION_FROM_CODE: dict[int, PlaybackAction] = {
    1: "pause",
    2: "resume",
    3: "discard",
    4: "system_abort",
}
ACTION_TO_CODE = {value: key for key, value in ACTION_FROM_CODE.items()}
HOLD_FROM_CODE: dict[int, PlaybackHold | None] = {
    0: None,
    1: "user",
    2: "system",
}
HOLD_TO_CODE = {value: key for key, value in HOLD_FROM_CODE.items()}
KIND_FROM_CODE: dict[int, PlaybackKind] = {
    1: "agent",
    2: "system",
    3: "cue",
}
KIND_TO_CODE = {value: key for key, value in KIND_FROM_CODE.items()}
EVENT_FROM_CODE = {
    1: "agent_started",
    2: "system_started",
    3: "cue_started",
    4: "agent_paused",
    5: "agent_resumed",
    6: "agent_discarded",
    7: "agent_completed",
    8: "system_completed",
    9: "cue_completed",
    10: "system_aborted",
}
EVENT_TO_CODE = {value: key for key, value in EVENT_FROM_CODE.items()}
EVENT_KINDS: dict[str, PlaybackKind] = {
    name: "agent" if name.startswith("agent_")
    else "system" if name.startswith("system_")
    else "cue"
    for name in EVENT_TO_CODE
}
STATUS_FROM_CODE: dict[int, PlaybackStatus] = {
    1: "accepted",
    2: "drained",
    3: "flushed",
}
STATUS_TO_CODE = {value: key for key, value in STATUS_FROM_CODE.items()}
MAX_SAFE_PLAYOUT_GENERATION = (1 << 53) - 1


def _enum(value, mapping: dict[int, object], field: str):
    if type(value) is not int or value not in mapping:
        raise ValueError(f"{field} contains an unknown numeric enum: {value!r}")
    return mapping[value]


def _uint(value, field: str) -> int:
    if type(value) is not int or value < 0:
        raise ValueError(f"{field} must be a non-negative integer")
    return value


def playout_generation_seed(wall_time_ns: int) -> int:
    milliseconds = _uint(wall_time_ns, "wall_time_ns") // 1_000_000
    if milliseconds > MAX_SAFE_PLAYOUT_GENERATION:
        raise ValueError("playout generation exceeds the JavaScript safe integer range")
    return milliseconds


def advance_playout_generation(current: int) -> int:
    current = _uint(current, "playout_generation")
    if current >= MAX_SAFE_PLAYOUT_GENERATION:
        raise ValueError("playout generation exceeds the JavaScript safe integer range")
    return current + 1


@dataclass(frozen=True)
class PlaybackControl:
    action: PlaybackAction
    minimum_agent_epoch: int = 0
    release_hold: PlaybackHold | None = None
    utterance_id: str = ""

    def __post_init__(self) -> None:
        if self.action not in ACTION_TO_CODE:
            raise ValueError(f"unsupported playback action: {self.action!r}")
        _uint(self.minimum_agent_epoch, "minimum_agent_epoch")
        if self.release_hold not in HOLD_TO_CODE:
            raise ValueError("release_hold must be user, system, or None")
        if not isinstance(self.utterance_id, str):
            raise ValueError("utterance_id must be a string")
        if self.action in {"pause", "resume"}:
            if self.minimum_agent_epoch or self.release_hold or self.utterance_id:
                raise ValueError(f"{self.action} contains fields that must be empty")
        elif self.action == "discard":
            if self.release_hold not in {"user", "system"}:
                raise ValueError("discard requires release_hold=user or system")
            if self.release_hold == "system" and not self.utterance_id:
                raise ValueError("system discard requires utterance_id")
            if self.release_hold == "user" and self.utterance_id:
                raise ValueError("user discard must not specify utterance_id")
        elif (
            not self.utterance_id
            or self.minimum_agent_epoch
            or self.release_hold not in {None, "system"}
        ):
            raise ValueError("system_abort fields are invalid")

    @classmethod
    def from_message(cls, message) -> "PlaybackControl":
        return cls(
            action=_enum(message.action, ACTION_FROM_CODE, "playback action"),
            minimum_agent_epoch=_uint(
                message.minimum_agent_epoch, "minimum_agent_epoch"
            ),
            release_hold=_enum(
                message.release_hold, HOLD_FROM_CODE, "playback release_hold"
            ),
            utterance_id=message.utterance_id,
        )

    def to_message(self, message_type):
        message = message_type()
        message.action = ACTION_TO_CODE[self.action]
        message.release_hold = HOLD_TO_CODE[self.release_hold]
        message.minimum_agent_epoch = self.minimum_agent_epoch
        message.utterance_id = self.utterance_id
        return message


@dataclass(frozen=True)
class PlaybackEvent:
    event: str
    kind: PlaybackKind
    utterance_id: str
    played_frames: int
    total_frames: int
    playout_generation: int = 0

    def __post_init__(self) -> None:
        if self.event not in EVENT_TO_CODE:
            raise ValueError(f"unsupported playback event: {self.event!r}")
        if self.kind != EVENT_KINDS[self.event]:
            raise ValueError(f"{self.event} cannot use kind={self.kind!r}")
        if not isinstance(self.utterance_id, str):
            raise ValueError("utterance_id must be a string")
        played = _uint(self.played_frames, "played_frames")
        total = _uint(self.total_frames, "total_frames")
        _uint(self.playout_generation, "playout_generation")
        if played > total:
            raise ValueError("played_frames cannot exceed total_frames")

    @classmethod
    def from_message(cls, message) -> "PlaybackEvent":
        return cls(
            event=_enum(message.event, EVENT_FROM_CODE, "playback event"),
            kind=_enum(message.kind, KIND_FROM_CODE, "playback kind"),
            utterance_id=message.utterance_id,
            played_frames=_uint(message.played_frames, "played_frames"),
            total_frames=_uint(message.total_frames, "total_frames"),
            playout_generation=_uint(
                message.playout_generation, "playout_generation"
            ),
        )

    def to_message(self, message_type, playout_generation: int | None = None):
        generation = (
            self.playout_generation
            if playout_generation is None
            else _uint(playout_generation, "playout_generation")
        )
        message = message_type()
        message.event = EVENT_TO_CODE[self.event]
        message.kind = KIND_TO_CODE[self.kind]
        message.utterance_id = self.utterance_id
        message.played_frames = self.played_frames
        message.total_frames = self.total_frames
        message.playout_generation = generation
        return message


@dataclass(frozen=True)
class OutputDrained:
    utterance_id: str
    seq: int
    frame_count: int
    final: bool
    status: PlaybackStatus = "drained"

    def __post_init__(self) -> None:
        if not isinstance(self.utterance_id, str) or not self.utterance_id:
            raise ValueError("output drained utterance_id must not be empty")
        _uint(self.seq, "output drained seq")
        _uint(self.frame_count, "output drained frame_count")
        if not isinstance(self.final, bool):
            raise ValueError("output drained final must be bool")
        if self.status not in STATUS_TO_CODE:
            raise ValueError("unknown output drained status")
        if self.status == "flushed" and self.final:
            raise ValueError("a flushed output chunk cannot be final")

    @classmethod
    def from_message(cls, message) -> "OutputDrained":
        return cls(
            utterance_id=message.utterance_id,
            seq=_uint(message.seq, "output drained seq"),
            frame_count=_uint(message.frame_count, "output drained frame_count"),
            final=message.final,
            status=_enum(message.status, STATUS_FROM_CODE, "output drained status"),
        )
