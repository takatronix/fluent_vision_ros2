from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class VadEvent:
    action: str
    bad: bool


class VadState:
    """Time-based min-on/min-off state machine."""

    def __init__(self, threshold: float, min_on_sec: float, min_off_sec: float) -> None:
        self.threshold = threshold
        self.min_on_sec = max(0.0, min_on_sec)
        self.min_off_sec = max(0.0, min_off_sec)
        self.state = False
        self.candidate = False
        self.candidate_since: Optional[float] = None

    def update(self, score: float, now_sec: float) -> Optional[VadEvent]:
        raw_bad = score >= self.threshold

        if self.candidate_since is None or raw_bad != self.candidate:
            self.candidate = raw_bad
            self.candidate_since = now_sec

        if self.candidate == self.state:
            return None

        required_sec = self.min_on_sec if self.candidate else self.min_off_sec
        if now_sec - float(self.candidate_since) < required_sec:
            return None

        self.state = self.candidate
        return VadEvent(action="start" if self.state else "stop", bad=self.state)
