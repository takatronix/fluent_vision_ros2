from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import re

import yaml


_EVENT_NAME = re.compile(r"^[a-z][a-z0-9_]*$")
_LEGACY_EVENT_FIELDS = {"event", "say", "speak"}
_EVENT_FIELDS = {
    "sound",
    "say",
    "speak",
    "interrupt",
    "record",
    "category",
    "variants",
}


@dataclass(frozen=True)
class ResolvedEvent:
    name: str
    say: str
    sound_path: str | None
    record: str


def parse_event_name(payload: str) -> str:
    value = payload.strip()
    if value.startswith("{"):
        try:
            parsed = json.loads(value)
        except json.JSONDecodeError as exc:
            raise ValueError("/fv/event JSON is invalid") from exc
        if (
            not isinstance(parsed, dict)
            or "event" not in parsed
            or set(parsed) - _LEGACY_EVENT_FIELDS
        ):
            raise ValueError('/fv/event JSON must contain "event" and no unknown fields')
        if "say" in parsed and not isinstance(parsed["say"], str):
            raise ValueError('/fv/event legacy "say" must be a string')
        if "speak" in parsed and not isinstance(parsed["speak"], bool):
            raise ValueError('/fv/event legacy "speak" must be a boolean')
        value = parsed["event"]
    if not isinstance(value, str) or not _EVENT_NAME.fullmatch(value):
        raise ValueError("/fv/event name must be lowercase ASCII snake_case")
    return value


class EventRegistry:
    def __init__(self, events: dict[str, ResolvedEvent]) -> None:
        if not events:
            raise ValueError("soundboard registry contains no events")
        self._events = events

    def __len__(self) -> int:
        return len(self._events)

    @classmethod
    def load(cls, path: str | Path, sounds_dir: str | Path) -> "EventRegistry":
        registry_path = Path(path).expanduser().resolve()
        sound_root = Path(sounds_dir).expanduser().resolve()
        try:
            value = yaml.safe_load(registry_path.read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError) as exc:
            raise ValueError(f"failed to load soundboard registry: {registry_path}") from exc
        if not isinstance(value, dict) or set(value) - {"version", "events"}:
            raise ValueError('soundboard registry root must contain only "version" and "events"')
        if value.get("version", 1) != 1:
            raise ValueError("unsupported soundboard registry version")
        raw_events = value.get("events")
        if not isinstance(raw_events, dict):
            raise ValueError("soundboard events must be a mapping")

        events: dict[str, ResolvedEvent] = {}
        for name, raw in raw_events.items():
            if not isinstance(name, str) or not _EVENT_NAME.fullmatch(name):
                raise ValueError(f"invalid soundboard event name: {name!r}")
            if not isinstance(raw, dict) or set(raw) - _EVENT_FIELDS:
                raise ValueError(f"invalid soundboard event definition: {name}")
            say = raw.get("say", "")
            speak = raw.get("speak", False)
            sound = raw.get("sound", "")
            record = raw.get("record", "")
            if not all(isinstance(item, str) for item in (say, sound, record)):
                raise ValueError(
                    f"soundboard {name} say, sound, and record must be strings"
                )
            if not isinstance(speak, bool):
                raise ValueError(f"soundboard {name}.speak must be a boolean")
            sound_path = None
            if sound.strip():
                candidate = (sound_root / f"{sound.strip()}.wav").resolve()
                if candidate.parent != sound_root or not candidate.is_file():
                    raise ValueError(f"soundboard cue does not exist: {candidate}")
                sound_path = str(candidate)
            fixed_say = say.strip() if speak else ""
            if not sound_path and not fixed_say:
                raise ValueError(f"soundboard {name} has neither fixed speech nor cue")
            events[name] = ResolvedEvent(
                name=name,
                say=fixed_say,
                sound_path=sound_path,
                record=record.strip(),
            )
        return cls(events)

    def resolve(self, payload: str) -> ResolvedEvent:
        name = parse_event_name(payload)
        try:
            return self._events[name]
        except KeyError as exc:
            raise ValueError(f"unregistered /fv/event: {name}") from exc
