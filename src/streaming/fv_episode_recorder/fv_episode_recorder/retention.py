"""Automatic episode retention (Phase 2).

Evaluates three independent rules — `max_age_days`, `max_episodes`,
`free_min_pct` — and unions the candidate set. `pinned` episodes are
always protected. Each rule fires deterministically (oldest-first) so
the operator can reason about what will be deleted.

The runner walks the filesystem each tick; a sqlite index would make
this cheaper at 10k+ episodes but for the current scale (hundreds)
filesystem walk is fine and avoids index-staleness bugs.
"""

from __future__ import annotations

import logging
import shutil
import time
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Optional

from .episode_store import (
    DuplicateEpisodeIdError,
    EpisodeMeta,
    EpisodeStore,
    read_episode_meta,
)

LOG = logging.getLogger("fv_episode_recorder.retention")


@dataclass
class RetentionPolicy:
    enabled: bool = False                   # off by default — opt-in per profile
    max_age_days: Optional[int] = None
    max_episodes: Optional[int] = None
    free_min_pct: Optional[float] = None    # delete oldest until free >= this
    grace_period_s: int = 60                # wait this long before actually deleting (operator can pin)
    interval_s: int = 300                   # planner tick interval

    @classmethod
    def from_profile(cls, profile_dict: dict) -> "RetentionPolicy":
        er = (profile_dict or {}).get("episode_recorder") or {}
        ret = er.get("retention") or {}
        if not ret:
            return cls()
        return cls(
            enabled=True,
            max_age_days=ret.get("max_age_days"),
            max_episodes=ret.get("max_episodes"),
            free_min_pct=ret.get("free_min_pct"),
            grace_period_s=int(ret.get("grace_period_s", 60)),
            interval_s=int(ret.get("interval_s", 300)),
        )

    @classmethod
    def from_dict(cls, d: dict) -> "RetentionPolicy":
        """Operator-supplied policy (from the dashboard settings popover)."""
        return cls(
            enabled=bool(d.get("enabled", False)),
            max_age_days=d.get("max_age_days"),
            max_episodes=d.get("max_episodes"),
            free_min_pct=d.get("free_min_pct"),
            grace_period_s=int(d.get("grace_period_s", 60)),
            interval_s=int(d.get("interval_s", 300)),
        )

    def to_dict(self) -> dict:
        return {
            "enabled": self.enabled,
            "max_age_days": self.max_age_days,
            "max_episodes": self.max_episodes,
            "free_min_pct": self.free_min_pct,
            "grace_period_s": self.grace_period_s,
            "interval_s": self.interval_s,
        }


@dataclass
class Candidate:
    meta: EpisodeMeta
    ep_dir: Path
    size_bytes: int
    reasons: list[str]      # one or more of "age", "count", "disk"


class RetentionPlanner:
    """Reads all episodes, applies the policy, returns candidates."""

    def __init__(self, store: EpisodeStore):
        self.store = store

    def _list_all(self) -> list[tuple[EpisodeMeta, Path, int]]:
        """All episodes (unbounded), with their dir paths + sizes."""
        out: list[tuple[EpisodeMeta, Path, int]] = []
        episodes_root = self.store.output_dir / "episodes"
        if not episodes_root.exists():
            return out
        seen: dict[str, Path] = {}
        for meta_path in episodes_root.glob("*/*/*/meta.json"):
            meta, _data = read_episode_meta(meta_path)
            previous = seen.get(meta.episode_id)
            if previous is not None and previous != meta_path.parent:
                raise DuplicateEpisodeIdError(
                    f"duplicate episode ID {meta.episode_id}: "
                    f"{previous} and {meta_path.parent}"
                )
            seen[meta.episode_id] = meta_path.parent
            ep_dir = meta_path.parent
            size = 0
            try:
                for f in ep_dir.rglob("*"):
                    if f.is_file():
                        size += f.stat().st_size
            except OSError:
                pass
            out.append((meta, ep_dir, size))
        return out

    def plan(self, policy: RetentionPolicy) -> list[Candidate]:
        """Return the list of episodes to delete. Pinned + currently-active
        recording are always excluded."""
        if not policy.enabled:
            return []
        all_eps = self._list_all()
        # Drop pinned + active.
        active_id = self.store.active.episode_id if self.store.active else None
        unpinned = [(m, d, s) for m, d, s in all_eps
                    if not m.pinned and m.episode_id != active_id]

        marks: dict[str, list[str]] = {}   # episode_id -> reasons

        # Rule 1: age
        if policy.max_age_days is not None:
            cutoff = datetime.now(timezone.utc) - timedelta(days=policy.max_age_days)
            cutoff_iso = cutoff.strftime("%Y-%m-%dT%H:%M:%S.%fZ")
            for m, _d, _s in unpinned:
                if m.started_at and m.started_at < cutoff_iso:
                    marks.setdefault(m.episode_id, []).append("age")

        # Rule 2: count overflow — drop the oldest N - max
        if policy.max_episodes is not None and len(unpinned) > policy.max_episodes:
            by_age = sorted(unpinned, key=lambda x: x[0].started_at or "")
            overflow = by_age[: len(unpinned) - policy.max_episodes]
            for m, _d, _s in overflow:
                marks.setdefault(m.episode_id, []).append("count")

        # Rule 3: disk pressure — delete oldest until free_min_pct satisfied
        if policy.free_min_pct is not None:
            try:
                usage = shutil.disk_usage(self.store.output_dir)
                pct_free = 100.0 * usage.free / usage.total if usage.total else 100
            except OSError:
                pct_free = 100.0
            if pct_free < policy.free_min_pct:
                by_age = sorted(unpinned, key=lambda x: x[0].started_at or "")
                # Approximate: assume reclaiming size bytes frees that much (single FS).
                bytes_to_free = (policy.free_min_pct - pct_free) / 100.0 * usage.total
                freed = 0
                for m, _d, s in by_age:
                    if freed >= bytes_to_free:
                        break
                    marks.setdefault(m.episode_id, []).append("disk")
                    freed += s

        # Build Candidate list preserving original order info
        out: list[Candidate] = []
        by_id = {m.episode_id: (m, d, s) for m, d, s in unpinned}
        for eid, reasons in marks.items():
            m, d, s = by_id[eid]
            out.append(Candidate(meta=m, ep_dir=d, size_bytes=s, reasons=reasons))
        return out


class RetentionRunner:
    """Background loop. Schedules candidates with a grace period, deletes
    them on the next tick if they're still candidates and still unpinned.

    State files (survive restarts):
      `<output_dir>/.retention_scheduled.json` — grace-period schedule
      `<output_dir>/.retention_policy.json`    — operator-supplied policy
                                                  (set via UI / API)"""

    SCHEDULE_FILE = ".retention_scheduled.json"
    POLICY_FILE = ".retention_policy.json"

    def __init__(self, store: EpisodeStore, profile_policy_loader=None):
        """profile_policy_loader: optional callable returning the
        profile-driven default RetentionPolicy. Operator overrides via
        set_policy() take precedence over the profile default."""
        self.store = store
        self._profile_policy_loader = profile_policy_loader
        self._planner = RetentionPlanner(store)
        self._scheduled: dict[str, float] = {}    # episode_id -> ready_at_epoch
        self._policy_override: Optional[RetentionPolicy] = None
        self._load_schedule()
        self._load_policy()
        self._last_tick = 0.0

    # ---- policy ----

    def current_policy(self) -> RetentionPolicy:
        if self._policy_override is not None:
            return self._policy_override
        if self._profile_policy_loader is not None:
            try:
                return self._profile_policy_loader()
            except Exception:
                return RetentionPolicy()
        return RetentionPolicy()

    def set_policy(self, policy: RetentionPolicy) -> None:
        self._policy_override = policy
        self._save_policy()

    def clear_override(self) -> None:
        self._policy_override = None
        self._save_policy()

    def _load_policy(self) -> None:
        path = self.store.output_dir / self.POLICY_FILE
        if not path.exists():
            return
        try:
            import json
            with path.open() as f:
                self._policy_override = RetentionPolicy.from_dict(json.load(f))
        except Exception as exc:
            LOG.warning("retention: failed to load policy: %s", exc)

    def _save_policy(self) -> None:
        path = self.store.output_dir / self.POLICY_FILE
        try:
            import json
            if self._policy_override is None:
                path.unlink(missing_ok=True)
                return
            tmp = path.with_suffix(".json.tmp")
            with tmp.open("w") as f:
                json.dump(self._policy_override.to_dict(), f)
            tmp.replace(path)
        except Exception as exc:
            LOG.warning("retention: failed to save policy: %s", exc)

    def _load_schedule(self) -> None:
        path = self.store.output_dir / self.SCHEDULE_FILE
        if not path.exists():
            return
        try:
            import json
            with path.open() as f:
                self._scheduled = {str(k): float(v) for k, v in json.load(f).items()}
        except Exception as exc:
            LOG.warning("retention: failed to load schedule: %s", exc)
            self._scheduled = {}

    def _save_schedule(self) -> None:
        path = self.store.output_dir / self.SCHEDULE_FILE
        try:
            import json
            tmp = path.with_suffix(".json.tmp")
            with tmp.open("w") as f:
                json.dump(self._scheduled, f)
            tmp.replace(path)
        except Exception as exc:
            LOG.warning("retention: failed to save schedule: %s", exc)

    def tick(self, dry_run: bool = False, policy: Optional[RetentionPolicy] = None) -> dict:
        """One pass: plan → schedule new candidates → delete ripe ones.
        Returns a summary dict (also suitable as POST response).
        Pass `policy` to override the stored policy for this single tick
        (used by the dashboard's "プレビュー" button)."""
        if policy is None:
            policy = self.current_policy()
        if not policy.enabled:
            return {"enabled": False, "scheduled": 0, "deleted": 0, "candidates": [], "policy": policy.to_dict()}

        candidates = self._planner.plan(policy)
        cand_ids = {c.meta.episode_id for c in candidates}
        now = time.time()

        # Schedule new candidates.
        newly_scheduled: list[dict] = []
        for c in candidates:
            if c.meta.episode_id not in self._scheduled:
                ready_at = now + policy.grace_period_s
                self._scheduled[c.meta.episode_id] = ready_at
                newly_scheduled.append({
                    "episode_id": c.meta.episode_id,
                    "task": c.meta.task_description,
                    "reasons": c.reasons,
                    "size_bytes": c.size_bytes,
                    "ready_in_s": policy.grace_period_s,
                })

        # Drop schedule entries that are no longer candidates (e.g. pinned).
        for eid in list(self._scheduled.keys()):
            if eid not in cand_ids:
                del self._scheduled[eid]

        # Delete ripe ones.
        deleted: list[dict] = []
        if not dry_run:
            ripe = [c for c in candidates
                    if c.meta.episode_id in self._scheduled
                    and self._scheduled[c.meta.episode_id] <= now]
            for c in ripe:
                try:
                    self.store.delete_episode(c.meta.episode_id)
                    deleted.append({
                        "episode_id": c.meta.episode_id,
                        "task": c.meta.task_description,
                        "reasons": c.reasons,
                        "size_bytes": c.size_bytes,
                    })
                    del self._scheduled[c.meta.episode_id]
                    LOG.info("retention: deleted %s (%s, %d bytes) reasons=%s",
                             c.meta.episode_id, c.meta.task_description,
                             c.size_bytes, ",".join(c.reasons))
                except OSError as exc:
                    LOG.warning("retention: delete failed for %s: %s",
                                c.meta.episode_id, exc)

        self._save_schedule()
        self._last_tick = now

        return {
            "enabled": True,
            "policy": {
                "max_age_days": policy.max_age_days,
                "max_episodes": policy.max_episodes,
                "free_min_pct": policy.free_min_pct,
                "grace_period_s": policy.grace_period_s,
            },
            "candidates": [
                {
                    "episode_id": c.meta.episode_id,
                    "task": c.meta.task_description,
                    "reasons": c.reasons,
                    "size_bytes": c.size_bytes,
                    "ready_at": self._scheduled.get(c.meta.episode_id),
                    "ready_in_s": max(0, self._scheduled.get(c.meta.episode_id, now) - now),
                }
                for c in candidates
            ],
            "newly_scheduled": newly_scheduled,
            "deleted": deleted,
            "dry_run": dry_run,
        }
