"""Retention tag-hold regression tests (pure pytest, no ROS runtime needed).

A 1500-EP synthetic generation run under `max_episodes: 500` lost 1168
not-yet-converted episodes overnight to the count rule. Episodes whose
tags appear in `.retention_hold.json` must be protected like `pinned`.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from fv_episode_recorder.episode_store import EpisodeStore
from fv_episode_recorder.retention import (
    HOLD_FILE, RetentionPlanner, RetentionPolicy, RetentionRunner,
    load_hold_tags)


def _write_ep(root: Path, episode_id: str, *, started_at: str,
              tags: list[str] | None = None, pinned: bool = False) -> Path:
    ep_dir = root / "episodes" / "sim_profile" / "2026-10-20" / episode_id
    ep_dir.mkdir(parents=True)
    (ep_dir / "meta.json").write_text(json.dumps({
        "episode_id": episode_id, "state": "finished",
        "started_at": started_at, "tags": tags or [], "pinned": pinned,
        "outcome": "success",
    }))
    return ep_dir


def _hold(root: Path, tags: list[str]) -> None:
    (root / HOLD_FILE).write_text(json.dumps({"tags": tags}))


def _count_policy(n: int) -> RetentionPolicy:
    return RetentionPolicy(enabled=True, max_episodes=n, grace_period_s=0)


@pytest.fixture
def store(tmp_path: Path) -> EpisodeStore:
    return EpisodeStore(tmp_path)


def _ts(i: int) -> str:
    return f"2026-10-20T00:{i // 60:02d}:{i % 60:02d}.000000Z"


def test_held_tag_episodes_are_not_count_candidates(store, tmp_path):
    # 6 datagen EPs of a running job (oldest) + 2 teleop EPs, max 3
    for i in range(6):
        _write_ep(tmp_path, f"gen{i}", started_at=_ts(i),
                  tags=["datagen", "job_a", "variant_x"])
    _write_ep(tmp_path, "tele0", started_at=_ts(10))
    _write_ep(tmp_path, "tele1", started_at=_ts(11))
    _hold(tmp_path, ["job_a"])

    planner = RetentionPlanner(store)
    cands = planner.plan(_count_policy(3))

    # held EPs neither get deleted nor count toward max_episodes
    assert cands == []
    assert planner.last_held == 6


def test_without_hold_the_oldest_datagen_episodes_are_deleted(store, tmp_path):
    """The incident: no hold → count rule eats the unconverted batch."""
    for i in range(6):
        _write_ep(tmp_path, f"gen{i}", started_at=_ts(i),
                  tags=["datagen", "job_a"])
    cands = RetentionPlanner(store).plan(_count_policy(3))
    assert sorted(c.meta.episode_id for c in cands) == ["gen0", "gen1", "gen2"]


def test_hold_only_protects_matching_tags(store, tmp_path):
    for i in range(3):
        _write_ep(tmp_path, f"old{i}", started_at=_ts(i),
                  tags=["datagen", "job_done"])
    for i in range(3):
        _write_ep(tmp_path, f"new{i}", started_at=_ts(10 + i),
                  tags=["datagen", "job_a"])
    _hold(tmp_path, ["job_a"])
    cands = RetentionPlanner(store).plan(_count_policy(1))
    assert sorted(c.meta.episode_id for c in cands) == ["old0", "old1"]


def test_held_episodes_survive_age_rule(store, tmp_path):
    _write_ep(tmp_path, "gen0", started_at="2000-01-01T00:00:00.000000Z",
              tags=["datagen", "job_a"])
    _write_ep(tmp_path, "tele0", started_at="2000-01-01T00:00:00.000000Z")
    _hold(tmp_path, ["job_a"])
    policy = RetentionPolicy(enabled=True, max_age_days=1, grace_period_s=0)
    cands = RetentionPlanner(store).plan(policy)
    assert [c.meta.episode_id for c in cands] == ["tele0"]


@pytest.mark.parametrize("content", ["{not json", '["job_a"]',
                                     '{"tags": "job_a"}'])
def test_unreadable_hold_file_plans_nothing(store, tmp_path, content):
    """Fail closed: a broken hold file means we can't tell what's protected."""
    for i in range(4):
        _write_ep(tmp_path, f"gen{i}", started_at=_ts(i),
                  tags=["datagen", "job_a"])
    (tmp_path / HOLD_FILE).write_text(content)
    planner = RetentionPlanner(store)
    assert planner.plan(_count_policy(1)) == []
    assert planner.last_held is None


def test_missing_hold_file_is_empty(tmp_path):
    assert load_hold_tags(tmp_path) == set()


def test_tick_deletes_nothing_held_and_reports_held(store, tmp_path):
    for i in range(4):
        _write_ep(tmp_path, f"gen{i}", started_at=_ts(i),
                  tags=["datagen", "job_a"])
    _write_ep(tmp_path, "tele0", started_at=_ts(10))
    _write_ep(tmp_path, "tele1", started_at=_ts(11))
    _hold(tmp_path, ["job_a"])
    runner = RetentionRunner(store, profile_policy_loader=lambda: _count_policy(1))

    summary = runner.tick()       # grace 0: scheduled and ripe at once

    assert [d["episode_id"] for d in summary["deleted"]] == ["tele0"]
    assert summary["held"] == 4
    assert runner.tick()["deleted"] == []
    for i in range(4):
        assert (tmp_path / "episodes" / "sim_profile" / "2026-10-20"
                / f"gen{i}").is_dir()


def test_releasing_the_hold_makes_episodes_eligible_again(store, tmp_path):
    for i in range(3):
        _write_ep(tmp_path, f"gen{i}", started_at=_ts(i),
                  tags=["datagen", "job_a"])
    _hold(tmp_path, ["job_a"])
    planner = RetentionPlanner(store)
    assert planner.plan(_count_policy(1)) == []
    _hold(tmp_path, [])            # job converted → producer drops the tag
    assert len(planner.plan(_count_policy(1))) == 2


def test_policy_view_advertises_hold_support(store, tmp_path):
    _hold(tmp_path, ["job_b", "job_a"])
    runner = RetentionRunner(store, profile_policy_loader=lambda: _count_policy(500))
    view = runner.policy_view()
    assert view["supports_hold_tags"] is True
    assert view["hold_tags"] == ["job_a", "job_b"]
    assert view["max_episodes"] == 500
    # the persisted override must stay a plain policy (no hold fields)
    runner.set_policy(_count_policy(7))
    saved = json.loads((tmp_path / RetentionRunner.POLICY_FILE).read_text())
    assert "supports_hold_tags" not in saved and "hold_tags" not in saved
