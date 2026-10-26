"""Episode API when the ROS executor stops spinning (#33 review): new episodes
are refused (503), the running one is not stored as a success, and healthz /
the stop reply say so. Needs aiohttp / pydantic and the recorder's ROS-side
imports (rclpy, cv2, pyarrow)."""

from __future__ import annotations

import asyncio
import json
import threading

import pytest

pytest.importorskip("aiohttp")
pytest.importorskip("pydantic")
pytest.importorskip("ulid")
pytest.importorskip("cv2")
pytest.importorskip("pyarrow.parquet")
pytest.importorskip("rclpy")
from aiohttp.test_utils import TestClient, TestServer  # noqa: E402

from fv_episode_recorder import api_server  # noqa: E402
from fv_episode_recorder.bag_recorder import BagRecorder  # noqa: E402
from fv_episode_recorder.episode_store import EpisodeStore  # noqa: E402
from fv_episode_recorder.executor_guard import SpinState  # noqa: E402


class _Pool:
    """CameraWriterPool stand-in (no cameras in these episodes)."""
    teardown_pending: list = []

    def start_all(self, *a, **k):
        return []

    def stop_all(self):
        return []

    def frame_counts(self):
        return {}

    def apply_depth_frame_counts(self, counts):
        pass

    def is_active(self):
        return False


@pytest.fixture
def spin():
    """A SpinState whose 'spin thread' is alive until the test kills it."""
    done = threading.Event()
    t = threading.Thread(target=done.wait, daemon=True)
    t.start()
    st = SpinState(); st.thread = t; st.alive = True

    def kill(gave_up=True):
        st.gave_up = gave_up
        st.exit_reason = "gave_up" if gave_up else "shutdown"
        st.alive = False
        done.set(); t.join(timeout=2)
    st.kill = kill
    yield st
    done.set()


def _run(tmp_path, monkeypatch, spin, scenario):
    monkeypatch.setattr(api_server, "run_preflight", lambda **kw: {"passed": True, "skipped": True})
    store = EpisodeStore(tmp_path)
    app = api_server.build_app(store, BagRecorder(), _Pool())
    app["spin_state"] = spin

    async def main():
        async with TestClient(TestServer(app)) as c:
            return await scenario(c, store)
    return asyncio.run(main())


_START = {"task_description": "t", "profile": "p", "record_bag": False}


def test_spin_dead_mid_episode_fails_it_and_refuses_the_next(tmp_path, monkeypatch, spin):
    async def scenario(c, store):
        r = await c.post("/api/v1/episodes", json=_START)
        assert r.status == 201, await r.text()
        ep = (await r.json())["episode_id"]
        h = await (await c.get("/api/v1/healthz")).json()
        assert h["status"] == "ok" and h["spin"]["healthy"] is True

        spin.kill(gave_up=True)               # spin_forever gave up mid-episode
        h = await (await c.get("/api/v1/healthz")).json()
        assert h["status"] == "degraded" and h["spin"]["healthy"] is False and h["spin"]["gave_up"]

        r = await c.post(f"/api/v1/episodes/{ep}/stop", json={"outcome": "success"})
        assert r.status == 200
        body = await r.json()
        assert body["state"] == "failed" and body["outcome"] == "abort"
        assert body["recorder_error"]["kind"] == "spin_dead"
        assert body["recorder_error"]["requested_outcome"] == "success"
        metas = [json.loads(p.read_text()) for p in store.output_dir.rglob("meta.json")]
        meta = next(m for m in metas if m["episode_id"] == ep)
        assert meta["state"] == "failed" and meta["outcome"] == "abort"
        assert meta["recorder_error"]["kind"] == "spin_dead"

        r = await c.post("/api/v1/episodes", json=_START)
        assert r.status == 503
        b = await r.json()
        assert b["error"] == "recorder_spin_dead" and b["spin"]["healthy"] is False
        r = await c.post("/api/v1/episodes/batch_start",
                         json={"task_description": "t", "profile": "p", "count": 2, "record_s": 1})
        assert r.status == 503
    _run(tmp_path, monkeypatch, spin, scenario)


def test_healthy_spin_keeps_a_success_and_discard_stays_a_discard(tmp_path, monkeypatch, spin):
    async def scenario(c, store):
        r = await c.post("/api/v1/episodes", json=_START)
        ep = (await r.json())["episode_id"]
        body = await (await c.post(f"/api/v1/episodes/{ep}/stop", json={"outcome": "success"})).json()
        assert body["state"] == "finished" and body["outcome"] == "success"
        assert body["recorder_error"] is None and body["teardown_pending"] == []

        r = await c.post("/api/v1/episodes", json=_START)
        ep2 = (await r.json())["episode_id"]
        spin.kill(gave_up=False)              # the spin thread ended (not a give-up)
        body = await (await c.post(f"/api/v1/episodes/{ep2}/stop", json={"outcome": "discard"})).json()
        assert body["state"] == "discarded" and body["recorder_error"] is None
        assert (await c.post("/api/v1/episodes", json=_START)).status == 503
    _run(tmp_path, monkeypatch, spin, scenario)
