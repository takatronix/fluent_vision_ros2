"""tts_node の声キャッシュ経路 (ROS 環境が必要。無ければ skip)。

ノードは起動せず、__init__ を飛ばした素のインスタンスに部品を差し込む。
"""
import concurrent.futures
import json
import threading
import time

import numpy as np
import pytest

tts_node = pytest.importorskip("fv_tts_py.tts_node")
vc = pytest.importorskip("fv_tts_py.voice_cache")
AivisUnreachable = tts_node.AivisUnreachable


class Log:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(m)

    warn = warning = error = debug = info


def _ojt_audio(amp=0.3):
    t = np.arange(48000) / 48000
    pcm = vc.float_to_pcm16(amp * np.sin(2 * np.pi * 200 * t))
    return tts_node.CachedAudio(pcm, 48000, 1, 16, 0.0, 0.0)


@pytest.fixture
def node(tmp_path, monkeypatch):
    n = tts_node.FvTtsNode.__new__(tts_node.FvTtsNode)
    log = Log()
    monkeypatch.setattr(n, "get_logger", lambda: log, raising=False)
    n.log = log
    n.engine = "pyopenjtalk"
    n.default_volume_db = 8.0
    n.output_sample_rate = 48000
    n.vc_kind, n.vc_speaker, n.vc_query, n.vc_max_chars = "aivis", 135718976, {}, 120
    n.voice_store = vc.VoiceCacheStore(tmp_path, 10 ** 8)
    n.vc_active = True
    n.vc_live_budget = 0.0  # その場合成は個別テストで ON
    n.vc_volume_db = 0.0
    n.vc_match_loudness = False
    n.vc_down_skip = 30.0
    n._vc_pool = None
    n._vc_inflight = set()
    n._vc_lock = threading.Lock()
    n._vc_skip_until = 0.0
    n._warmup_path = None
    n._warmup_due = False
    n.submitted = []

    class W:
        backoff = 0.0

        def submit(self, key, text, ref, priority=True, job=None):
            n.submitted.append((key, text, ref, priority, job))
            return True

        def queued(self):
            return len(n.submitted)

    n.voice_worker = W()
    n.ojt_calls = []

    def fake_get_cached(text, voice_id, volume_db, cache_key):
        n.ojt_calls.append(text)
        return _ojt_audio()

    n.get_cached = fake_get_cached
    return n


def test_miss_speaks_open_jtalk_and_enqueues(node):
    audio = node.get_audio("テストです。", "", 8.0, "k")
    assert node.ojt_calls == ["テストです。"]
    assert audio.sample_rate == 48000
    key, text, ref, prio, job = node.submitted[0]
    assert text == "テストです。" and prio and ref == pytest.approx(0.3 / np.sqrt(2), rel=0.02)
    assert job == {"speaker": 135718976}


def test_hit_plays_cached_voice_without_open_jtalk(node):
    key = node._voice_key("テストです。", 8.0)
    pcm = vc.float_to_pcm16(np.full(4800, 0.1, np.float32))
    node.voice_store.put(key, pcm, 48000, {"text": "テストです。"})
    audio = node.get_audio("テストです。", "", 8.0, "k")
    assert node.ojt_calls == [] and audio.audio_bytes == pcm
    assert node.submitted == []


def test_voice_id_or_long_text_bypasses_cache(node):
    node.get_audio("テストです。", "3", 8.0, "k")
    node.get_audio("あ" * 200, "", 8.0, "k")
    assert len(node.ojt_calls) == 2 and node.submitted == []


def test_broken_store_still_speaks(node):
    class Broken:
        def get(self, key):
            raise OSError("disk gone")
    node.voice_store = Broken()
    audio = node.get_audio("テストです。", "", 8.0, "k")
    assert node.ojt_calls == ["テストです。"] and audio is not None


def test_disabled_cache_is_todays_behavior(node):
    node.voice_store = None
    node.voice_worker = None
    node.get_audio("テストです。", "", 8.0, "k")
    assert node.ojt_calls == ["テストです。"]


def test_render_matches_open_jtalk_loudness_and_rate(node):
    class Backend:
        def synthesize(self, text, speaker, query):
            assert speaker == 135718976
            t = np.arange(44100) / 44100
            return vc.float_to_pcm16(0.05 * np.sin(2 * np.pi * 200 * t)), 44100, 1, 16

    node.vc_backend = Backend()
    node.vc_match_loudness = True
    pcm, sr = node._render_voice("k", "テスト", 0.2)
    assert sr == 48000
    assert vc.active_rms(vc.pcm16_to_float(pcm), sr) == pytest.approx(0.2, rel=0.03)


def test_render_default_keeps_engine_level_and_uses_job_speaker(node):
    seen = []

    class Backend:
        def synthesize(self, text, speaker, query):
            seen.append(speaker)
            t = np.arange(44100) / 44100
            return vc.float_to_pcm16(0.25 * np.sin(2 * np.pi * 200 * t)), 44100, 1, 16

    node.vc_backend = Backend()
    pcm, sr = node._render_voice("k", "テスト", None, {"speaker": 1310138977})
    assert seen == [1310138977] and sr == 48000
    # 0 dB: AivisSpeech の素の音量のまま (Open JTalk を呼ばない)
    assert np.max(np.abs(vc.pcm16_to_float(pcm))) == pytest.approx(0.25, rel=0.03)
    assert node.ojt_calls == []


class _Backend:
    def __init__(self, delay=0.0, exc=None):
        self.delay, self.exc, self.calls = delay, exc, []

    def synthesize(self, text, speaker, query):
        self.calls.append((text, speaker))
        time.sleep(self.delay)
        if self.exc is not None:
            raise self.exc
        return vc.float_to_pcm16(np.full(4410, 0.1, np.float32)), 44100, 1, 16


@pytest.fixture
def live(node):
    node.vc_live_budget = 0.5
    node._vc_pool = concurrent.futures.ThreadPoolExecutor(max_workers=2)
    yield node
    node._vc_pool.shutdown(wait=True)


def test_live_aivis_within_budget_plays_and_caches(live):
    live.vc_backend = _Backend()
    audio = live.get_audio("テストです。", "", 8.0, "k")
    assert live.ojt_calls == [] and live.submitted == []
    assert audio.sample_rate == 48000
    assert live._voice_key("テストです。", 8.0) in live.voice_store
    live.get_audio("テストです。", "", 8.0, "k")  # 2 回目はキャッシュ (エンジンを呼ばない)
    assert len(live.vc_backend.calls) == 1


def test_live_too_slow_falls_back_and_saves_later(live):
    live.vc_live_budget = 0.05
    live.vc_backend = _Backend(delay=0.3)
    t0 = time.monotonic()
    live.get_audio("テストです。", "", 8.0, "k")
    assert time.monotonic() - t0 < 0.25
    assert live.ojt_calls == ["テストです。"]
    assert live.submitted == []  # 続きはその場合成のスレッドが保存する (二重に作らない)
    for _ in range(50):
        if live._voice_key("テストです。", 8.0) in live.voice_store:
            break
        time.sleep(0.02)
    assert live._voice_key("テストです。", 8.0) in live.voice_store


def test_engine_down_speaks_open_jtalk_and_stops_trying_for_a_while(live):
    live.vc_backend = _Backend(exc=AivisUnreachable("connection refused"))
    live.get_audio("一つ目。", "", 8.0, "k1")
    live.get_audio("二つ目。", "", 8.0, "k2")
    assert live.ojt_calls == ["一つ目。", "二つ目。"]  # 無音にしない
    assert len(live.vc_backend.calls) == 1             # 2 回目は待たずに Open JTalk
    assert [s[1] for s in live.submitted] == ["一つ目。", "二つ目。"]  # 裏の作業者が後で作る


def test_male_and_female_are_cached_separately(node):
    female = node._voice_key("収穫を始めます。", 8.0, 135718976)
    male = node._voice_key("収穫を始めます。", 8.0, 1310138977)
    assert female != male
    node.voice_store.put(female, vc.float_to_pcm16(np.full(480, 0.1, np.float32)), 48000, {})
    node._apply_voice_settings({"tts_engine": "aivis", "aivis_speaker": 1310138977})
    node.get_audio("収穫を始めます。", "", 8.0, "k")
    assert node.ojt_calls == ["収穫を始めます。"]  # 男はまだ無い
    assert node.submitted[0][0] == male and node.submitted[0][4] == {"speaker": 1310138977}


def test_settings_toggle_engine_and_voice(node):
    msg = type("M", (), {})()
    msg.data = json.dumps({"tts_engine": "pyopenjtalk"})
    node.on_settings(msg)
    assert node.vc_active is False
    node.get_audio("テストです。", "", 8.0, "k")
    assert node.submitted == []  # Open JTalk のみ
    msg.data = json.dumps({"tts_engine": "aivis", "voice": "male", "aivis_speaker": 1310138977})
    node.on_settings(msg)
    assert node.vc_active is True and node.vc_speaker == 1310138977
    msg.data = json.dumps({"aivis_speaker": -1})
    node.on_settings(msg)  # 壊れた値は無視
    assert node.vc_speaker == 1310138977


def test_voice_change_rewarms_after_startup(node, tmp_path):
    w = tmp_path / "w.yaml"
    w.write_text("texts:\n  - 準備ができました\n  - 停止します\n", encoding="utf-8")
    node._warmup_path = w
    node._apply_voice_settings({"tts_engine": "aivis", "aivis_speaker": 1310138977})
    assert node.submitted == []  # 起動直後の待ち時間中は作らない
    node._warmup_fire()
    assert [s[4] for s in node.submitted] == [{"speaker": 1310138977}] * 2
    node._apply_voice_settings({"aivis_speaker": 135718976})
    assert [s[4]["speaker"] for s in node.submitted[2:]] == [135718976] * 2
    assert all(not s[3] for s in node.submitted)  # ウォームアップは低優先


def test_render_maps_unreachable_to_engine_down(node):
    class Backend:
        def synthesize(self, *a):
            raise AivisUnreachable("connection refused")

    node.vc_backend = Backend()
    with pytest.raises(vc.EngineDown):
        node._render_voice("k", "テスト", 0.2)



def test_legacy_voicevox_setting_turns_aivis_on(node):
    node._apply_voice_settings({"tts_engine": "pyopenjtalk"})
    assert node.vc_active is False
    node._apply_voice_settings({"tts_engine": "voicevox"})  # 旧 viewer の保存値
    assert node.vc_active is True


@pytest.mark.parametrize("engine,url,expected,warns", [
    ("pyopenjtalk", "", "pyopenjtalk", False),
    ("", "", "pyopenjtalk", False),
    ("AIVIS", "", "aivis", False),
    ("voicevox", "http://127.0.0.1:10101", "aivis", True),   # 旧設定: URL があれば aivis
    ("voicevox", "", "pyopenjtalk", True),                     # 無ければ Open JTalk
])
def test_resolve_engine(engine, url, expected, warns):
    got, warning = tts_node.resolve_engine(engine, url)
    assert got == expected and bool(warning) == warns


def test_resolve_engine_rejects_unknown():
    with pytest.raises(RuntimeError):
        tts_node.resolve_engine("espeak", "")


def test_open_jtalk_cache_key_is_stable(node):
    import hashlib
    digest = hashlib.sha1()
    for part in (b"pyopenjtalk", "テスト".encode(), b"", b"8.0"):
        digest.update(part)
    # 既存の Open JTalk ディスクキャッシュ (~/.fluent_voice_cache) をそのまま使える
    assert node.make_cache_key("テスト", "", 8.0) == digest.hexdigest()
    node.engine = "aivis"
    assert node.make_cache_key("テスト", "", 8.0) == digest.hexdigest()
