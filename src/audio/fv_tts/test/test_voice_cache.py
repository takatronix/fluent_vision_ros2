"""声キャッシュの単体テスト (ROS 不要。numpy と pytest だけ)。

    python3 -m pytest src/audio/fv_tts/test -q
"""
import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
import pytest

_PATH = Path(__file__).resolve().parents[1] / "fv_tts_py" / "voice_cache.py"
_spec = importlib.util.spec_from_file_location("voice_cache", _PATH)
vc = importlib.util.module_from_spec(_spec)
sys.modules["voice_cache"] = vc
_spec.loader.exec_module(vc)


# ── key ────────────────────────────────────────────────
def test_key_normalizes_text():
    a = vc.make_key("aivis", 135718976, {}, 8.0, "テストです。")
    b = vc.make_key("aivis", 135718976, {}, 8.0, "  テストです。\n")
    c = vc.make_key("aivis", 135718976, {}, 8.0, "ﾃｽﾄです。")  # 半角カナ → NFKC
    assert a == b == c


@pytest.mark.parametrize("change", [
    dict(kind="other"), dict(speaker=1), dict(params={"speedScale": 1.1}),
    dict(volume_db=0.0), dict(text="テストでした。"),
])
def test_key_changes_with_audio_affecting_inputs(change):
    base = dict(kind="aivis", speaker=135718976, params={}, volume_db=8.0, text="テストです。")
    assert vc.make_key(**base) != vc.make_key(**{**base, **change})


# ── store / LRU ────────────────────────────────────────
def _pcm(seconds=0.1, sr=48000, amp=0.3):
    t = np.arange(int(sr * seconds)) / sr
    return vc.float_to_pcm16(amp * np.sin(2 * np.pi * 440 * t)), sr


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        self.t += 1.0
        return self.t


def test_store_roundtrip_and_index(tmp_path):
    s = vc.VoiceCacheStore(tmp_path, 10 * 1024 * 1024)
    pcm, sr = _pcm()
    key = "a" * 40
    s.put(key, pcm, sr, {"text": "テストです。"})
    assert s.get(key) == (pcm, sr)
    idx = json.loads((tmp_path / "index.json").read_text(encoding="utf-8"))
    assert idx["entries"][key]["text"] == "テストです。"
    assert idx["entries"][key]["hits"] == 1
    # 再起動 (読み直し) しても残る
    assert vc.VoiceCacheStore(tmp_path, 10 * 1024 * 1024).get(key) == (pcm, sr)
    assert not list(tmp_path.glob(".tmp-*"))


def test_store_lru_eviction(tmp_path):
    pcm, sr = _pcm(0.1)  # ~9.6KB
    size = len(vc.wav_bytes(pcm, sr))
    s = vc.VoiceCacheStore(tmp_path, size * 2 + 10, clock=Clock())
    a, b, c = "a" * 40, "b" * 40, "c" * 40
    s.put(a, pcm, sr, {})
    s.put(b, pcm, sr, {})
    assert s.get(a) is not None      # a を新しくする → b が最古
    s.put(c, pcm, sr, {})
    assert a in s and c in s and b not in s
    assert not (tmp_path / f"{b}.wav").exists()
    assert s.total_bytes() <= size * 2 + 10


def test_store_drops_corrupt_and_orphans(tmp_path):
    s = vc.VoiceCacheStore(tmp_path, 10 * 1024 * 1024)
    pcm, sr = _pcm()
    key = "d" * 40
    s.put(key, pcm, sr, {})
    (tmp_path / f"{key}.wav").write_bytes(b"broken")
    assert s.get(key) is None and key not in s
    orphan = tmp_path / ("e" * 40 + ".wav")
    orphan.write_bytes(b"x")
    other = tmp_path / "0100082194c7e181.fvtc"  # 他人のファイルは触らない
    other.write_bytes(b"x")
    vc.VoiceCacheStore(tmp_path, 10 * 1024 * 1024)
    assert not orphan.exists() and other.exists()


# ── loudness / format ──────────────────────────────────
def test_match_loudness_hits_target_and_keeps_format():
    sr_in = 44100
    t = np.arange(sr_in) / sr_in
    quiet = (0.05 * np.sin(2 * np.pi * 220 * t)).astype(np.float32)
    quiet = np.concatenate([np.zeros(sr_in // 2, np.float32), quiet])  # 前の無音は無視される
    up = vc.resample(quiet, sr_in, 48000)
    assert abs(up.size - int(round(quiet.size * 48000 / sr_in))) <= 1
    out, gain_db = vc.match_loudness(up, 48000, target_rms=0.2)
    assert gain_db > 10
    assert abs(vc.active_rms(out, 48000) - 0.2) < 0.01
    loud, _ = vc.match_loudness(up, 48000, target_rms=0.9)  # ピークは 1.0 未満に抑える
    assert np.max(np.abs(loud)) < 1.0


# ── worker / queue ─────────────────────────────────────
class FakeRender:
    def __init__(self, outcomes=None):
        self.calls = []
        self.outcomes = list(outcomes or [])

    def __call__(self, key, text, ref, job):
        self.calls.append((text, ref))
        self.jobs = getattr(self, 'jobs', []) + [job]
        if self.outcomes:
            o = self.outcomes.pop(0)
            if isinstance(o, Exception):
                raise o
        return _pcm()


def _worker(tmp_path, render, **kw):
    sleeps = []
    w = vc.RenderWorker(vc.VoiceCacheStore(tmp_path, 10 ** 8), render,
                        sleep=sleeps.append, nice=0, **kw)
    return w, sleeps


def _drain(w, limit=20):
    for _ in range(limit):
        if not (w._live or w._warm):
            return
        if w._live:
            w.step(w._live.popleft(), True)
        else:
            w.step(w._warm.popleft(), False)


def test_queue_dedupes_and_skips_cached(tmp_path):
    r = FakeRender()
    w, _ = _worker(tmp_path, r)
    assert w.submit("k" * 40, "テストです。", 0.1)
    assert not w.submit("k" * 40, "テストです。", 0.1)  # 重複
    assert w.queued() == 1
    _drain(w)
    assert len(r.calls) == 1 and ("k" * 40) in w.store
    assert not w.submit("k" * 40, "テストです。", 0.1)  # もうある


def test_live_requests_jump_ahead_of_warmup(tmp_path):
    r = FakeRender()
    w, _ = _worker(tmp_path, r)
    w.submit("1" * 40, "warm1", None, priority=False)
    w.submit("2" * 40, "warm2", None, priority=False)
    w.submit("3" * 40, "live", 0.2, priority=True)
    w.submit("2" * 40, "warm2", 0.2, priority=True)  # 待ち中の文がライブで来たら前へ
    _drain(w)
    assert [c[0] for c in r.calls] == ["live", "warm2", "warm1"]


def test_engine_down_backs_off_and_retries(tmp_path):
    r = FakeRender([vc.EngineDown("refused"), vc.EngineDown("refused")])
    w, sleeps = _worker(tmp_path, r, backoff_min=5, backoff_max=8, gap_s=0)
    w.submit("k" * 40, "テスト", 0.1)
    _drain(w)
    assert ("k" * 40) in w.store
    assert sleeps[:2] == [5, 8]       # 指数バックオフ (上限 8)
    assert w.backoff == 0.0           # 成功で解除


def test_text_failure_gives_up_after_max_attempts(tmp_path):
    r = FakeRender([RuntimeError("422")] * 5)
    w, _ = _worker(tmp_path, r, max_attempts=3, gap_s=0)
    w.submit("k" * 40, "だめな文", 0.1)
    _drain(w)
    assert len(r.calls) == 3 and w.queued() == 0
    assert not w.submit("k" * 40, "だめな文", 0.1)


def test_queue_is_bounded(tmp_path):
    w, _ = _worker(tmp_path, FakeRender(), max_queue=2)
    assert w.submit("1" * 40, "a") and w.submit("2" * 40, "b")
    assert not w.submit("3" * 40, "c")


def test_busy_guard_delays_warmup_only(tmp_path):
    busy = {"v": True}
    r = FakeRender()
    w, sleeps = _worker(tmp_path, r, busy=lambda: busy["v"])
    w.submit("1" * 40, "warm", None, priority=False)
    w.step(w._warm.popleft(), False)
    assert r.calls == [] and w.queued() == 1 and sleeps == [10.0]
    w.submit("2" * 40, "live", 0.1, priority=True)
    w.step(w._live.popleft(), True)  # ライブは負荷中でも作る
    assert [c[0] for c in r.calls] == ["live"]


def test_thread_renders_in_background(tmp_path):
    import time
    r = FakeRender()
    w = vc.RenderWorker(vc.VoiceCacheStore(tmp_path, 10 ** 8), r, gap_s=0, nice=0)
    w.start()
    w.submit("k" * 40, "テスト", 0.1)
    for _ in range(100):
        if ("k" * 40) in w.store:
            break
        time.sleep(0.02)
    w.stop()
    assert ("k" * 40) in w.store


def test_load_warmup_yaml_and_txt(tmp_path):
    y = tmp_path / "w.yaml"
    y.write_text("texts:\n  - 収穫を開始します。\n  - 収穫を開始します。\n  - ' 準備ができました '\n",
                 encoding="utf-8")
    assert vc.load_warmup(y) == ["収穫を開始します。", "準備ができました"]
    t = tmp_path / "w.txt"
    t.write_text("# comment\nこんにちは\n\n", encoding="utf-8")
    assert vc.load_warmup(t) == ["こんにちは"]
