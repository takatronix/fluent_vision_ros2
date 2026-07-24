from __future__ import annotations

from dataclasses import dataclass
import json
import re


_EVENT_NAME = re.compile(r"^[a-z][a-z0-9_]*$")


@dataclass(frozen=True)
class SoundSettings:
    robot_enabled: bool
    disabled_events: frozenset[str]

    @classmethod
    def from_json(cls, payload: str) -> "SoundSettings":
        try:
            value = json.loads(payload)
        except json.JSONDecodeError as exc:
            raise ValueError("sound settings must be valid JSON") from exc
        expected = {"version", "robot_enabled", "disabled_events"}
        if not isinstance(value, dict) or set(value) != expected:
            raise ValueError(
                "sound settings fields must be version, robot_enabled, and disabled_events"
            )
        if type(value["version"]) is not int or value["version"] != 1:
            raise ValueError("sound settings version must be 1")
        if not isinstance(value["robot_enabled"], bool):
            raise ValueError("sound settings robot_enabled must be boolean")
        disabled = value["disabled_events"]
        if not isinstance(disabled, list) or any(
            not isinstance(name, str) or not _EVENT_NAME.fullmatch(name)
            for name in disabled
        ):
            raise ValueError(
                "sound settings disabled_events must contain event names"
            )
        return cls(value["robot_enabled"], frozenset(disabled))
