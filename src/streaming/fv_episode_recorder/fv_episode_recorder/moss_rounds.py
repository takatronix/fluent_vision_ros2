"""Incremental MOSS control-token parser."""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Optional


ROUND_END = "<|round_end|>"
SILENCE = "<|silence|>"
_CONTROL_TOKEN = re.compile(r"<\|[^|>]+\|>")
_UNSET = object()


@dataclass(frozen=True)
class RoutedRound:
    episode_id: Optional[str]
    text: Optional[str]


class EpisodeRoundOwnership:
    """Route an ended episode's dynamic corrections until a new episode starts."""

    def __init__(self) -> None:
        self._active_episode_id: Optional[str] = None
        self._finalizing_episode_id: Optional[str] = None

    def start(self, episode_id: str, now: Optional[float] = None) -> None:
        del now
        self._active_episode_id = episode_id
        self._finalizing_episode_id = None

    def end(self, episode_id: str, now: Optional[float] = None) -> bool:
        del now
        if self._active_episode_id != episode_id:
            return False
        self._active_episode_id = None
        self._finalizing_episode_id = episode_id
        return True

    def route_episode(self, now: Optional[float] = None) -> Optional[str]:
        del now
        return self._active_episode_id or self._finalizing_episode_id

    def is_active(self, episode_id: str) -> bool:
        return self._active_episode_id == episode_id

    def is_finalizing(self, episode_id: str, now: Optional[float] = None) -> bool:
        del now
        return self._finalizing_episode_id == episode_id

    def claim_completion(self, episode_id: str, now: Optional[float] = None) -> bool:
        """Accept semantic rounds while the episode owns active/finalizing output."""
        del now
        if self._active_episode_id == episode_id:
            return True
        return self._finalizing_episode_id == episode_id


class MossRoundParser:
    """Collect raw chunks and release semantic text only at round boundaries."""

    def __init__(self) -> None:
        self._buffer = ""
        self._episode_id: object | Optional[str] = _UNSET

    def push(self, chunk: str) -> list[str]:
        return [
            result.text
            for result in self.push_routed(chunk, episode_id=None)
            if result.text is not None
        ]

    def push_routed(self, chunk: str, episode_id: Optional[str]) -> list[RoutedRound]:
        if not self._buffer:
            self._episode_id = episode_id
        self._buffer += chunk
        completed: list[RoutedRound] = []
        while ROUND_END in self._buffer:
            raw_round, self._buffer = self._buffer.split(ROUND_END, 1)
            routed_episode = (
                None if self._episode_id is _UNSET else self._episode_id
            )
            if SILENCE in raw_round:
                semantic = None
            else:
                filtered = _CONTROL_TOKEN.sub("", raw_round).strip()
                semantic = filtered or None
            completed.append(RoutedRound(routed_episode, semantic))
            self._episode_id = episode_id if self._buffer else _UNSET
        return completed

    @property
    def pending(self) -> str:
        return self._buffer
