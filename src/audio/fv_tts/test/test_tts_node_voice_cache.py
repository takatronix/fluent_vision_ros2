"""tts_node の発話経路: 選んだエンジンだけで喋る (代わりの声は出さない)、文ごとに流す、温める。

ノードは起動せず、__init__ を飛ばした素のインスタンスに部品を差し込む (ROS が無ければ conftest の代用で import)。
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
SpeakFailed = tts_node.SpeakFailed


class Log:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(m)

    warn = warning = error = debug = info

    def errors(self):
        return [m for m in self.lines if "読み上げできませんでした" in m]


def _ojt_audio(amp=0.3, seconds=1.0):
    t = np.arange(int(48000 * seconds)) / 48000
    pcm = vc.float_to_pcm16(amp * np.sin(2 * np.pi * 200 * t))
    return tts_node.CachedAudio(pcm, 48000, 1, 16, 0.0, 0.0)


class _Pub:
    def __init__(self):
        self.frames = []

    def publish(self, frame):
        self.frames.append(frame)


class _Clock:
    def now(self):
        return self

    def to_msg(self):
        return 0


@pytest.fixture
def node(tmp_path, monkeypatch):
    n = tts_node.FvTtsNode.__new__(tts_node.FvTtsNode)
    log = Log()
    monkeypatch.setattr(n, "get_logger", lambda: log, raising=False)
    monkeypatch.setattr(n, "get_clock", lambda: _Clock(), raising=False)
    n.log = log
    n.engine = "aivis"
    n.default_voice = ""
    n.default_volume_db = 8.0
    n.output_sample_rate = 48000
    n.publish_playback = True
    n.playback_epoch = 1
    n.playback_backlog_s = 0.0
    n._published_ends = []
    n._speaking = False
    n._speaking_end = 0.0
    n._speaking_timer = None
    n._speaking_lock = threading.Lock()
    n.tts_pub, n.play_pub, n.speaking_pub = _Pub(), _Pub(), _Pub()
    n.cache, n.cache_dir = {}, None
    n.vc_kind, n.vc_speaker, n.vc_query, n.vc_max_chars = "aivis", 1310138977, {"speedScale": 1.3}, 200
    n.vc_speed = 1.3
    n.voice_store = vc.VoiceCacheStore(tmp_path, 10 ** 8)
    n.vc_active = True
    n.vc_timeout = 0.5
    n.vc_volume_db = 0.0
    n.vc_match_loudness = False
    n.vc_down_skip = 10.0
    n.vc_keepalive_s = 0.0
    n._vc_pool = concurrent.futures.ThreadPoolExecutor(max_workers=2)
    n._vc_inflight = set()
    n._vc_lock = threading.Lock()
    n._vc_skip_until = 0.0
    n._vc_skip_reason = ""
    n._aivis_last_use = 0.0
    n._warmup_path = None
    n._warmup_timer = None
    n._keepalive_timer = None
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
    yield n
    n._vc_pool.shutdown(wait=True)


class _Backend:
    def __init__(self, delay=0.0, exc=None):
        self.delay, self.exc, self.calls = delay, exc, []

    def synthesize(self, text, speaker, query):
        self.calls.append((text, speaker, dict(query)))
        time.sleep(self.delay)
        if self.exc is not None:
            raise self.exc
        return vc.float_to_pcm16(np.full(44100, 0.1, np.float32)), 44100, 1, 16


# ── 文の切り方 ─────────────────────────────────────────
def test_split_sentences_keeps_short_fragments_together():
    split = tts_node.split_sentences
    assert split("") == []
    assert split("収穫を始めます。") == ["収穫を始めます。"]
    assert split("2本目、失敗。空把持なので中止。戻った所で止めました。") == [
        "2本目、失敗。", "空把持なので中止。", "戻った所で止めました。"]
    assert split("はい。終わり。") == ["はい。終わり。"]          # 「終わり。」(4 字) は前につなぐ
    assert split("はい。わかりました。") == ["はい。わかりました。"]  # 前が短い時も 1 つに
    long = "、".join(["あいうえおかきくけこさしすせそ"] * 12) + "。"
    parts = split(long)
    assert len(parts) >= 2 and all(len(p) <= tts_node._SENTENCE_MAX for p in parts)
    assert "".join(parts) == long


# ── AivisSpeech: 代わりの声は出さない ─────────────────────
def test_cached_sentence_plays_without_engine(node):
    key = node._voice_key("テストです。", 8.0)
    pcm = vc.float_to_pcm16(np.full(4800, 0.1, np.float32))
    node.voice_store.put(key, pcm, 48000, {"text": "テストです。"})
    node.vc_backend = _Backend()
    clips = node.speak_text("テストです。", "", 8.0)
    assert len(clips) == 1 and clips[0].audio_bytes == pcm
    assert node.vc_backend.calls == [] and node.ojt_calls == []
    assert len(node.play_pub.frames) == 1


def test_live_sentence_is_synthesized_with_speed_and_cached(node):
    node.vc_backend = _Backend()
    clips = node.speak_text("テストです。", "", 8.0)
    assert len(clips) == 1 and clips[0].sample_rate == 48000
    assert node.vc_backend.calls[0][1] == 1310138977
    assert node.vc_backend.calls[0][2]["speedScale"] == 1.3
    key = node._voice_key("テストです。", 8.0)
    assert key in node.voice_store
    assert node.voice_store.index[key]["speed"] == 1.3 and node.voice_store.index[key]["speaker"] == 1310138977
    node.speak_text("テストです。", "", 8.0)  # 2 回目はキャッシュ
    assert len(node.vc_backend.calls) == 1
    assert node.ojt_calls == []


def test_long_text_is_spoken_sentence_by_sentence_in_order(node):
    node.vc_backend = _Backend(delay=0.02)
    text = "2本目、失敗。把持を確認できません。過負荷で停止しました。硬い物や茎の根元を掴んだ可能性があります。"
    clips = node.speak_text(text, "", 8.0)
    sentences = tts_node.split_sentences(text)
    assert len(clips) == len(sentences) >= 3
    assert [c[0] for c in node.vc_backend.calls] == sentences
    assert len(node.play_pub.frames) == len(sentences)
    assert node.ojt_calls == []  # 長い文でも Open JTalk にはしない


def test_timeout_skips_the_utterance_and_never_substitutes(node):
    node.vc_timeout = 0.05
    node.vc_backend = _Backend(delay=0.3)
    t0 = time.monotonic()
    clips = node.speak_text("一つ目の文です。そして二つ目の文です。", "", 8.0)
    assert clips == [] and node.ojt_calls == [] and node.play_pub.frames == []
    assert time.monotonic() - t0 < 0.25
    errs = node.log.errors()
    assert len(errs) == 1 and "aivis" in errs[0] and "残り 1 文も飛ばす" in errs[0]
    # 続きは裏で保存される (次回はキャッシュから)
    for _ in range(50):
        if node._voice_key("一つ目の文です。", 8.0) in node.voice_store:
            break
        time.sleep(0.02)
    assert node._voice_key("一つ目の文です。", 8.0) in node.voice_store


def test_engine_down_logs_and_skips_quietly_for_a_while(node):
    node.vc_backend = _Backend(exc=AivisUnreachable("connection refused"))
    assert node.speak_text("一つ目。", "", 8.0) == []
    assert node.speak_text("二つ目。", "", 8.0) == []
    assert len(node.vc_backend.calls) == 1          # 2 回目は試さずに飛ばす (待たせない)
    assert node.ojt_calls == [] and node.play_pub.frames == []
    errs = node.log.errors()
    assert len(errs) == 2 and "不通" in errs[0] and "秒前まで不通" in errs[1]
    assert node.submitted == []                      # 裏で別の声を作る予約もしない


def test_engine_error_on_one_sentence_continues_with_the_next(node):
    class Flaky(_Backend):
        def synthesize(self, text, speaker, query):
            if text.startswith("二"):
                self.calls.append((text, speaker, dict(query)))
                raise RuntimeError("synthesis HTTP 500")
            return super().synthesize(text, speaker, query)

    node.vc_backend = Flaky()
    clips = node.speak_text("一つ目の文です。二つ目の文です。三つ目の文です。", "", 8.0)
    assert len(clips) == 2
    assert [c[0] for c in node.vc_backend.calls] == ["一つ目の文です。", "二つ目の文です。", "三つ目の文です。"]
    assert len(node.log.errors()) == 1 and "HTTP 500" in node.log.errors()[0]


def test_stop_drops_the_remaining_sentences(node):
    class Stopper(_Backend):
        def synthesize(self, text, speaker, query):
            out = super().synthesize(text, speaker, query)
            if text.startswith("一"):
                node.on_stop(None)       # 1 文目の合成中に停止が来た
            return out

    node.vc_backend = Stopper()
    clips = node.speak_text("一つ目の文です。二つ目の文です。", "", 8.0)
    assert clips == [] and node.play_pub.frames == []
    assert [c[0] for c in node.vc_backend.calls] == ["一つ目の文です。"]


def test_backlog_waits_before_publishing_more(node):
    node.playback_backlog_s = 1.5
    node.vc_backend = _Backend()
    for _ in range(3):
        node.voice_store.put(node._voice_key(f"{_}番目の文です。", 8.0),
                             vc.float_to_pcm16(np.full(48000, 0.1, np.float32)), 48000, {})
    t0 = time.monotonic()
    node.speak_text("0番目の文です。1番目の文です。2番目の文です。", "", 8.0)
    # 3 文 × 1 秒: 2 文目までは即 (backlog 2 s > 1.5 s なので 3 文目は 0.5 秒待つ)
    assert 0.4 < time.monotonic() - t0 < 1.2
    assert len(node.play_pub.frames) == 3


def test_not_prepared_aivis_fails_the_utterance(node):
    node.voice_store = None
    assert node.speak_text("テストです。", "", 8.0) == []
    assert node.ojt_calls == [] and "準備ができていません" in node.log.errors()[0]


# ── Open JTalk: 選んだ時だけ、失敗は飛ばす ─────────────────
def test_open_jtalk_when_selected(node):
    node._apply_voice_settings({"tts_engine": "pyopenjtalk"})
    assert node.vc_active is False
    node.vc_backend = _Backend()
    clips = node.speak_text("テストです。二つ目。", "", 8.0)
    assert node.ojt_calls == ["テストです。二つ目。"] and len(clips) == 1
    assert node.vc_backend.calls == [] and node.submitted == []


def test_open_jtalk_failure_is_logged_not_replaced(node):
    node.vc_active = False

    def boom(text, voice_id, volume_db, cache_key):
        raise RuntimeError("pyopenjtalk is not available")

    node.get_cached = boom
    node.vc_backend = _Backend()
    assert node.speak_text("テストです。", "", 8.0) == []
    assert node.vc_backend.calls == [] and node.play_pub.frames == []
    assert "読み上げできませんでした（pyopenjtalk: pyopenjtalk is not available）" in node.log.errors()[0]


# ── 設定 ────────────────────────────────────────────────
def test_settings_select_engine_openai_means_aivis_for_announcements(node):
    msg = type("M", (), {})()
    msg.data = json.dumps({"tts_engine": "pyopenjtalk"})
    node.on_settings(msg)
    assert node.vc_active is False
    msg.data = json.dumps({"tts_engine": "openai"})
    node.on_settings(msg)
    assert node.vc_active is True               # アナウンスは Aivis
    msg.data = json.dumps({"tts_engine": "aivis", "voice": "female", "aivis_speaker": 135718976})
    node.on_settings(msg)
    assert node.vc_active is True and node.vc_speaker == 135718976
    msg.data = json.dumps({"aivis_speaker": -1})
    node.on_settings(msg)  # 壊れた値は無視
    assert node.vc_speaker == 135718976


def test_legacy_voicevox_setting_turns_aivis_on(node):
    node._apply_voice_settings({"tts_engine": "pyopenjtalk"})
    node._apply_voice_settings({"tts_engine": "voicevox"})  # 旧 viewer の保存値
    assert node.vc_active is True


def test_speed_setting_changes_query_and_cache_key(node):
    key_before = node._voice_key("収穫を始めます。", 8.0)
    node._apply_voice_settings({"aivis_speed": 1.0})
    assert node.vc_speed == 1.0 and node.vc_query["speedScale"] == 1.0
    assert node._voice_key("収穫を始めます。", 8.0) != key_before
    node._apply_voice_settings({"aivis_speed": "9"})    # 範囲外は丸める
    assert node.vc_speed == tts_node.AIVIS_SPEED_RANGE[1]
    node._apply_voice_settings({"aivis_speed": "abc"})  # 壊れた値は無視
    assert node.vc_speed == tts_node.AIVIS_SPEED_RANGE[1]


def test_male_and_female_are_cached_separately(node):
    female = node._voice_key("収穫を始めます。", 8.0, 135718976)
    male = node._voice_key("収穫を始めます。", 8.0, 1310138977)
    assert female != male
    node.voice_store.put(female, vc.float_to_pcm16(np.full(480, 0.1, np.float32)), 48000, {})
    node.vc_backend = _Backend()
    node.speak_text("収穫を始めます。", "", 8.0)
    assert len(node.vc_backend.calls) == 1          # 男はまだ無い → その場で合成
    node._apply_voice_settings({"aivis_speaker": 135718976})
    node.speak_text("収穫を始めます。", "", 8.0)
    assert len(node.vc_backend.calls) == 1          # 女はキャッシュ


def test_voice_change_rewarms_after_startup(node, tmp_path):
    w = tmp_path / "w.yaml"
    w.write_text("texts:\n  - 準備ができました\n  - 停止します\n", encoding="utf-8")
    node._warmup_path = w
    node._apply_voice_settings({"tts_engine": "aivis", "aivis_speaker": 135718976})
    assert node.submitted == []  # 起動直後の待ち時間中は作らない
    node._warmup_fire()
    assert [s[4] for s in node.submitted] == [{"speaker": 135718976, "speed": 1.3}] * 2
    node._apply_voice_settings({"aivis_speed": 1.0})
    assert len(node.submitted) == 4               # 速さが変わっても作り直す
    assert all(not s[3] for s in node.submitted)  # ウォームアップは低優先


# ── 温め ────────────────────────────────────────────────
def test_keepalive_renders_a_short_text_when_idle(node):
    node.vc_backend = _Backend()
    node.vc_keepalive_s = 60.0
    node._aivis_last_use = time.monotonic() - 120
    node._keepalive_tick()
    node._keepalive_timer.cancel()
    for _ in range(50):
        if any("温め" in m for m in node.log.lines):
            break
        time.sleep(0.02)
    assert [c[0] for c in node.vc_backend.calls] == [tts_node.KEEPALIVE_TEXT]
    assert node.voice_store.index == {}           # 温めの音は保存しない
    assert any("温め" in m for m in node.log.lines)
    # 直前に喋っていれば温めない
    node._aivis_last_use = time.monotonic()
    node._keepalive_tick()
    node._keepalive_timer.cancel()
    time.sleep(0.05)
    assert len(node.vc_backend.calls) == 1


def test_keepalive_is_off_while_open_jtalk_is_selected(node):
    node.vc_backend = _Backend()
    node.vc_keepalive_s = 60.0
    node.vc_active = False
    node._aivis_last_use = 0.0
    node._keepalive_tick()
    node._keepalive_timer.cancel()
    time.sleep(0.05)
    assert node.vc_backend.calls == []


# ── 合成の形 ───────────────────────────────────────────
def test_render_matches_open_jtalk_loudness_and_rate(node):
    class Backend:
        def synthesize(self, text, speaker, query):
            assert speaker == 1310138977
            t = np.arange(44100) / 44100
            return vc.float_to_pcm16(0.05 * np.sin(2 * np.pi * 200 * t)), 44100, 1, 16

    node.vc_backend = Backend()
    node.vc_match_loudness = True
    node._synthesize_pyopenjtalk = lambda text, voice_id, volume_db: _ojt_audio(0.2 * np.sqrt(2))
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
    pcm, sr = node._render_voice("k", "テスト", None, {"speaker": 135718976})
    assert seen == [135718976] and sr == 48000
    # 0 dB: AivisSpeech の素の音量のまま。48 kHz へリサンプル (fv_audio_output と同じ)
    assert np.max(np.abs(vc.pcm16_to_float(pcm))) == pytest.approx(0.25, rel=0.03)
    assert len(pcm) // 2 == pytest.approx(48000, rel=0.01)


def test_render_maps_unreachable_to_engine_down(node):
    class Backend:
        def synthesize(self, *a):
            raise AivisUnreachable("connection refused")

    node.vc_backend = Backend()
    with pytest.raises(vc.EngineDown):
        node._render_voice("k", "テスト", 0.2)


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


def test_speak_service_returns_one_frame_for_all_sentences(node):
    node.vc_backend = _Backend()
    req = tts_node.Speak.Request()
    req.text, req.play, req.voice_id, req.volume_db, req.cache_key = "一つ目の文です。二つ目の文です。", True, "", 0.0, ""
    res = node.handle_speak(req, tts_node.Speak.Response())
    assert res.success and res.played
    assert len(node.play_pub.frames) == 2
    total = sum(len(f.data) for f in node.play_pub.frames)
    assert len(res.frame.data) == total
    node.vc_backend = _Backend(exc=AivisUnreachable("down"))
    node._vc_skip_until = 0.0
    req.text = "三つ目。"
    res = node.handle_speak(req, tts_node.Speak.Response())
    assert not res.success and "読み上げできませんでした" in res.message
