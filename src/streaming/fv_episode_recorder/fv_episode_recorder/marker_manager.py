"""Marker manager — sub-task / event / note annotations inside an episode.

Phase 2 minimum: in-memory store, flushed to meta.json on episode stop.
canonical vs audit log split (bag /episode/markers publish) comes later.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field, asdict
from typing import Optional

from .episode_store import new_episode_id, utc_now_iso


@dataclass
class Marker:
    marker_id: str
    episode_id: str
    kind: str = "subtask"        # subtask | event | note
    task_description: str = ""
    started_at: str = ""
    stopped_at: Optional[str] = None
    outcome: Optional[str] = None
    tags: list = field(default_factory=list)
    rev: int = 0

    def to_meta_dict(self) -> dict:
        return asdict(self)


class MarkerManager:
    """Thread-safe in-memory marker store, scoped per episode_id."""

    def __init__(self):
        self._lock = threading.Lock()
        self._by_episode: dict[str, dict[str, Marker]] = {}

    # ---- mutators ----

    def start(self, episode_id: str, task_description: str,
              kind: str = "subtask", tags: Optional[list] = None) -> Marker:
        if kind not in ("subtask", "event", "note"):
            raise ValueError(f"invalid kind: {kind}")
        mid = new_episode_id()
        m = Marker(
            marker_id=mid,
            episode_id=episode_id,
            kind=kind,
            task_description=task_description,
            started_at=utc_now_iso(),
            tags=list(tags or []),
        )
        # event / note are point-in-time → auto-stop immediately
        if kind in ("event", "note"):
            m.stopped_at = m.started_at
        with self._lock:
            self._by_episode.setdefault(episode_id, {})[mid] = m
        return m

    def stop(self, marker_id: str, outcome: str = "success") -> Optional[Marker]:
        with self._lock:
            for eps in self._by_episode.values():
                if marker_id in eps:
                    m = eps[marker_id]
                    if m.stopped_at is None:
                        m.stopped_at = utc_now_iso()
                    m.outcome = outcome
                    m.rev += 1
                    return m
            return None

    def patch(self, marker_id: str, **changes) -> Optional[Marker]:
        """Time shift / label edit / outcome change (Phase 2 minimum)."""
        allowed = {"started_at", "stopped_at", "task_description", "tags", "outcome", "kind"}
        with self._lock:
            for eps in self._by_episode.values():
                if marker_id in eps:
                    m = eps[marker_id]
                    for k, v in changes.items():
                        if k in allowed and v is not None:
                            setattr(m, k, v)
                    m.rev += 1
                    return m
            return None

    def delete(self, marker_id: str) -> bool:
        with self._lock:
            for eps in self._by_episode.values():
                if marker_id in eps:
                    del eps[marker_id]
                    return True
            return False

    # ---- queries ----

    def list(self, episode_id: str) -> list[Marker]:
        with self._lock:
            return list(self._by_episode.get(episode_id, {}).values())

    def active(self, episode_id: str) -> list[Marker]:
        """Currently in-progress subtask markers (kind=subtask + stopped_at None)."""
        return [m for m in self.list(episode_id)
                if m.kind == "subtask" and m.stopped_at is None]

    def flush(self, episode_id: str) -> list[dict]:
        """Return marker dicts for meta.json, clear from memory."""
        with self._lock:
            eps = self._by_episode.pop(episode_id, {})
        return [m.to_meta_dict() for m in eps.values()]
