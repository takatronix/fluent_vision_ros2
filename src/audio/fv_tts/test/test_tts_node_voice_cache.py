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
    n._speaking_lock = threading.RLock()
    n._closing = False
    n.tts_pub, n.play_pub, n.speaking_pub, n.spoken_pub = _Pub(), _Pub(), _Pub(), _Pub()
    n.cache, n.cache_dir = {}, None
    n.vc_kind, n.vc_speaker, n.vc_query, n.vc_max_chars = "aivis", 1310138977, {"speedScale": 1.3}, 200
    n.vc_speed = 1.3
    n.voice_store = vc.VoiceCacheStore(tmp_path, 10 ** 8)
    n.engine_selected = "aivis"
    n.vc_timeout = 0.5
    n.vc_volume_db = 0.0
    n.vc_match_loudness = False
    n.vc_down_skip = 10.0
    n.vc_keepalive_s = 0.0
    n._vc_pool = concurrent.futures.ThreadPoolExecutor(max_workers=2)
    n._vc_inflight = set()
    n._vc_futures = {}
    n._vc_lock = threading.Lock()
    n._cfg_lock = threading.Lock()
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


def _spoken(backend):
    """温め (「はい。」) を除いた合成の呼び出し。"""
    return [c for c in backend.calls if c[0] != tts_node.KEEPALIVE_TEXT]


def _wait_cached(node, key, timeout=2.0):
    """保存は合成の done callback の中 (speak_text が戻った後に終わることがある): 期限付きで待つ。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if key in node.voice_store and not node._vc_futures:
            return True
        time.sleep(0.01)
    return key in node.voice_store


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
    assert _wait_cached(node, key)
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
    assert node.vc_active is False and node.engine_selected == "aivis"
    assert node.speak_text("テストです。", "", 8.0) == []
    assert node.ojt_calls == [] and "準備ができていません" in node.log.errors()[0]
    # 準備が無いままでも、設定の選択は保つ (Open JTalk に流れない)
    node._apply_voice_settings({"tts_engine": "aivis"})
    assert node.engine_selected == "aivis"
    assert node.speak_text("もう一度。", "", 8.0) == [] and node.ojt_calls == []
    node._apply_voice_settings({"tts_engine": "pyopenjtalk"})
    assert node.engine_selected == "pyopenjtalk"
    node.speak_text("今度は。", "", 8.0)
    assert node.ojt_calls == ["今度は。"]


# ── Open JTalk: 選んだ時だけ、失敗は飛ばす ─────────────────
def test_open_jtalk_when_selected(node):
    node._apply_voice_settings({"tts_engine": "pyopenjtalk"})
    assert node.vc_active is False
    node.vc_backend = _Backend()
    clips = node.speak_text("テストです。二つ目。", "", 8.0)
    assert node.ojt_calls == ["テストです。二つ目。"] and len(clips) == 1
    assert node.vc_backend.calls == [] and node.submitted == []


def test_open_jtalk_failure_is_logged_not_replaced(node):
    node.engine_selected = "pyopenjtalk"

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
    assert len(_spoken(node.vc_backend)) == 1       # 女はキャッシュ (声を変えた時の温めは数えない)


def test_voice_change_rewarms_after_startup(node, tmp_path):
    w = tmp_path / "w.yaml"
    w.write_text("texts:\n  - 準備ができました\n  - 停止します\n", encoding="utf-8")
    node._warmup_path = w
    node._apply_voice_settings({"tts_engine": "aivis", "aivis_speaker": 135718976})
    assert node.submitted == []  # 起動直後の待ち時間中は作らない
    node._warmup_fire()
    assert [(s[4]["speaker"], s[4]["speed"]) for s in node.submitted] == [(135718976, 1.3)] * 2
    node._apply_voice_settings({"aivis_speed": 1.0})
    assert len(node.submitted) == 4               # 速さが変わっても作り直す
    assert [(s[4]["speaker"], s[4]["speed"], s[4]["query"]["speedScale"]) for s in node.submitted[2:]] == [(135718976, 1.0, 1.0)] * 2
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
    node.engine_selected = "pyopenjtalk"
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


# ── レビュー (2026-10-24) で足した試験 ───────────────────────
def test_stop_during_backlog_wait_publishes_nothing_more(node):
    node.playback_backlog_s = 0.3
    node.vc_backend = _Backend()
    for i in range(3):
        node.voice_store.put(node._voice_key(f"{i}番目の文です。", 8.0),
                             vc.float_to_pcm16(np.full(48000, 0.1, np.float32)), 48000, {})
    # 2 文目で待ちに入ったところで停止
    threading.Timer(0.1, node.on_stop, args=(None,)).start()
    clips = node.speak_text("0番目の文です。1番目の文です。2番目の文です。", "", 8.0)
    assert len(clips) == 1 and len(node.play_pub.frames) == 1   # 待った後にもう 1 文出さない


def test_timeout_is_not_waited_twice_and_the_same_sentence_is_not_rendered_twice(node):
    node.vc_timeout = 0.05
    node.vc_down_skip = 10.0
    node.vc_backend = _Backend(delay=0.3)
    t0 = time.monotonic()
    assert node.speak_text("一つ目の文です。", "", 8.0) == []
    assert node.speak_text("一つ目の文です。", "", 8.0) == []    # 2 回目は待たずに飛ばす
    assert time.monotonic() - t0 < 0.25
    assert "秒前まで不通" in node.log.errors()[1]
    assert _wait_cached(node, node._voice_key("一つ目の文です。", 8.0))
    assert len(node.vc_backend.calls) == 1                     # 同じ文を二重に投入していない
    assert node._vc_futures == {} and node._vc_inflight == set()


def test_two_requests_for_the_same_sentence_share_one_synthesis(node):
    node.vc_timeout = 2.0
    node.vc_backend = _Backend(delay=0.2)
    results = []

    def speak():
        results.append(node.speak_text("同じ文です。", "", 8.0))

    threads = [threading.Thread(target=speak) for _ in range(2)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert len(results) == 2 and all(len(r) == 1 for r in results)
    assert len(node.vc_backend.calls) == 1
    assert any("合成中だった声を待った" in m for m in node.log.lines)
    assert _wait_cached(node, node._voice_key("同じ文です。", 8.0))


def test_speed_change_during_synthesis_keeps_key_query_and_meta_consistent(node):
    class Slow(_Backend):
        def synthesize(self, text, speaker, query):
            node._apply_voice_settings({"aivis_speed": 1.0})   # 合成中に設定が変わる
            return super().synthesize(text, speaker, query)

    node.vc_backend = Slow()
    key_before = node._voice_key("テストです。", 8.0)             # speed 1.3 のキー
    node.speak_text("テストです。", "", 8.0)
    spoken = _spoken(node.vc_backend)
    assert len(spoken) == 1 and spoken[0][2]["speedScale"] == 1.3   # 合成は作り始めた時の速さ
    assert _wait_cached(node, key_before)                        # その速さのキーに保存
    assert node.voice_store.index[key_before]["speed"] == 1.3    # メタも同じ
    assert node._voice_key("テストです。", 8.0) != key_before       # いまの設定 (1.0) のキーとは別


def test_destroy_stops_the_say_thread_and_further_output(node, monkeypatch):
    import queue as queue_mod
    # 親 (rclpy の Node) の destroy_node は、__init__ を飛ばした素のインスタンスでは呼べない: 呼ばれたことだけ見る
    monkeypatch.setattr(tts_node.Node, "destroy_node", lambda self: True)
    node._say_queue = queue_mod.Queue()
    node._say_thread = threading.Thread(target=node._say_loop, daemon=True)
    node._say_thread.start()
    node.voice_worker.stop = lambda: None
    node.vc_backend = _Backend(delay=1.0)
    node.vc_timeout = 15.0
    node.vc_keepalive_s = 60.0
    node._arm_keepalive()
    # 15 秒待つ合成の途中で終了 → 待ちをやめてスレッドが終わる
    node._say_queue.put(("長い合成の途中です。", 0, node.playback_epoch))
    time.sleep(0.1)
    t0 = time.monotonic()
    assert node.destroy_node() is True                            # 代用の Node.destroy_node まで届く
    assert time.monotonic() - t0 < 3.0
    assert not node._say_thread.is_alive()
    assert node._keepalive_timer is None
    node._arm_keepalive()                                         # 終了後は登録し直さない
    assert node._keepalive_timer is None
    node.on_say(type("M", (), {"data": "もう受けない"})())
    assert node._say_queue.empty()
    assert node.speak_text("テストです。", "", 8.0) == []           # 閉じた後は出さない
    node._keepalive_now("x")                                      # 閉じた後は温めない
    assert [c for c in node.vc_backend.calls if c[0] == tts_node.KEEPALIVE_TEXT] == []


def test_switching_to_aivis_warms_with_the_new_voice_after_settings_apply(node):
    node.vc_backend = _Backend()
    node.engine_selected = "pyopenjtalk"
    node._apply_voice_settings({"tts_engine": "aivis", "aivis_speaker": 135718976, "aivis_speed": 1.0})
    for _ in range(50):
        if node.vc_backend.calls:
            break
        time.sleep(0.02)
    assert node.vc_backend.calls[0][0] == tts_node.KEEPALIVE_TEXT
    assert node.vc_backend.calls[0][1] == 135718976 and node.vc_backend.calls[0][2]["speedScale"] == 1.0


# ── レビュー 2 回目 (2026-10-24) で足した試験 ──────────────────
class _DonePool:
    """submit した時点で完了している Future を返す executor の代用 (コールバック登録前に完了するケース)。"""
    def submit(self, fn, *args):
        fut = concurrent.futures.Future()
        try:
            fut.set_result(fn(*args))
        except BaseException as exc:  # noqa: BLE001
            fut.set_exception(exc)
        return fut

    def shutdown(self, **_k):
        pass


def test_completed_future_callback_does_not_deadlock(node):
    """合成が一瞬で終わる (完了済みの Future にコールバックを付ける) 時に止まらない。"""
    node.vc_backend = _Backend(delay=0.0)
    node._vc_pool = _DonePool()
    done = []

    def run():
        done.append(node.speak_text("一瞬です。", "", 8.0))

    t = threading.Thread(target=run, daemon=True)
    t.start()
    t.join(timeout=3.0)
    assert not t.is_alive() and len(done) == 1 and len(done[0]) == 1
    assert _wait_cached(node, node._voice_key("一瞬です。", 8.0))
    assert node._vc_futures == {} and node._vc_inflight == set()


def test_long_sentence_is_not_rendered_twice_even_if_not_cached(node):
    node.vc_max_chars = 10
    node.vc_timeout = 2.0
    node.vc_backend = _Backend(delay=0.2)
    text = "保存しないほど長い文を同時に二つの経路から頼みます。"
    results = []
    threads = [threading.Thread(target=lambda: results.append(node.speak_text(text, "", 8.0))) for _ in range(2)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert all(len(r) == 1 for r in results)
    assert len(_spoken(node.vc_backend)) == 1
    for _ in range(100):
        if not node._vc_futures:
            break
        time.sleep(0.01)
    assert node.voice_store.index == {}                             # 長文は保存しない
    assert node._vc_futures == {} and node._vc_inflight == set()


def test_backlog_limit_holds_when_two_speakers_race(node):
    node.playback_backlog_s = 1.0
    node.vc_backend = _Backend()
    for i in range(6):
        node.voice_store.put(node._voice_key(f"{i}番目の文です。", 8.0),
                             vc.float_to_pcm16(np.full(48000, 0.1, np.float32)), 48000, {})
    seen_backlog = []
    real_publish = node.play_pub.publish

    def spy(frame):
        with node._speaking_lock:
            now = time.monotonic()
            seen_backlog.append(sum(1 for t in node._published_ends if t > now))
        real_publish(frame)

    node.play_pub.publish = spy
    texts = ["0番目の文です。1番目の文です。2番目の文です。", "3番目の文です。4番目の文です。5番目の文です。"]
    threads = [threading.Thread(target=node.speak_text, args=(t, "", 8.0)) for t in texts]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert len(node.play_pub.frames) == 6
    assert max(seen_backlog) <= 1                                   # 残り 1 秒 (=1 枠) を超えて同時に出さない


def test_spoken_topic_reports_engine_and_result(node):
    node.vc_backend = _Backend()
    node.speak_text("一つ目の文です。二つ目の文です。", "", 8.0)
    report = json.loads(node.spoken_pub.frames[-1].data)
    assert report["engine"] == "aivis" and report["ok"] is True and report["clips"] == 2
    assert report["text"] == "一つ目の文です。二つ目の文です。" and report["seconds"] > 0
    node.vc_backend = _Backend(exc=AivisUnreachable("down"))
    node._vc_skip_until = 0.0
    node.speak_text("だめな文です。", "", 8.0)
    report = json.loads(node.spoken_pub.frames[-1].data)
    assert report["ok"] is False and report["clips"] == 0 and "不通" in report["error"]
    node.engine_selected = "pyopenjtalk"
    node.speak_text("軽い声です。", "", 8.0)
    assert json.loads(node.spoken_pub.frames[-1].data)["engine"] == "pyopenjtalk"


def test_settings_snapshot_is_atomic_with_updates(node):
    """写しを取る間に設定が変わっても、話者と速さがちぐはぐな写しにならない。"""
    stop = threading.Event()
    bad = []

    def flip():
        i = 0
        while not stop.is_set():
            node._apply_voice_settings({"aivis_speaker": [135718976, 1310138977][i % 2],
                                        "aivis_speed": [1.0, 1.3][i % 2]})
            i += 1

    t = threading.Thread(target=flip, daemon=True)
    t.start()
    for _ in range(300):
        snap = node._aivis_snapshot()
        if (snap["speaker"], snap["speed"]) not in ((135718976, 1.0), (1310138977, 1.3)):
            bad.append(snap)
        if snap["query"]["speedScale"] != snap["speed"]:
            bad.append(snap)
    stop.set()
    t.join(timeout=2.0)
    assert bad == []


def test_dedupe_mark_stays_until_the_cache_write_is_done(node):
    """合成が終わってから保存が終わるまでの間に同じ文が来ても、再合成しない。"""
    node.vc_backend = _Backend(delay=0.1)
    slow_put = threading.Event()
    real_put = node.voice_store.put

    def put(key, pcm, sr, meta):
        slow_put.wait(0.5)            # 保存に時間がかかる
        real_put(key, pcm, sr, meta)

    node.voice_store.put = put
    first = threading.Thread(target=node.speak_text, args=("同じ文です。", "", 8.0))
    first.start()
    time.sleep(0.15)                  # 合成は済み、保存の途中
    second = node.speak_text("同じ文です。", "", 8.0)
    slow_put.set()
    first.join()
    assert len(second) == 1
    assert len(_spoken(node.vc_backend)) == 1


# ── 発話ごとの id とクリップの終わり方 (/aspa/tts/playback、playback_track.py) ─────────
pt = pytest.importorskip("fv_tts_py.playback_track")


def _with_tracking(node, outputs=1, latency=0.05, sound_unknown=False, drained=False):
    node.playback_pub = _Pub()
    node.tts_speaking_pub = _Pub()
    node.outputs = outputs
    node.drains = drained                                 # 偽の出力の done が「鳴らし切った」と言うか
    node.output_gid = b"\x01\x0f"                         # graph で見える出力の publisher の gid
    node.done_gid = None                                  # done の送り手 (None = 出力と同じ)
    node._output_state = lambda: node.output_gid if node.outputs == 1 else None
    node._sound_unknown = lambda: sound_unknown
    node._track = pt.PlaybackTracker(node._emit_playback, latency_s=latency, grace_s=1.0, boot="t35t",
                                     output_key=node._output_state)
    return node


def _notices(node, kind=None):
    out = [json.loads(m.data) for m in node.playback_pub.frames]
    return [e for e in out if kind is None or e["event"] == kind]


def _done(node, frame, epoch=None):
    gid = node.done_gid if node.done_gid is not None else getattr(node, "output_gid", b"")
    msg = tts_node.PlaybackDone(request_id=frame.header.frame_id, epoch=frame.epoch if epoch is None else epoch,
                                drained=getattr(node, "drains", False), publisher_gid=list(gid or b""))
    node.on_playback_done(msg)


def _settle(node, seconds=0.12):
    time.sleep(seconds)
    node._track_tick()


def test_utterance_id_frame_ids_and_natural_completion(node):
    _with_tracking(node)
    node.vc_backend = _Backend()
    clips = node.speak_text("実際に収穫するか確認してください。", "", 8.0)
    assert len(clips) == 1
    frame = node.play_pub.frames[0]
    start, = _notices(node, "start")
    assert frame.header.frame_id == f"fv_tts:{start['id']}:0" and start["of"] == 1 and start["engine"] == "aivis"
    spoken = json.loads(node.spoken_pub.frames[-1].data)
    assert spoken["id"] == start["id"] and spoken["clips"] == 1 and spoken["of"] == 1   # 今の知らせに足しただけ
    assert {"text", "engine", "ok", "clips", "seconds", "ms", "error"} <= set(spoken)
    assert _notices(node, "end") == []                   # 出しただけ (鳴り終わりはまだ)
    _done(node, frame)
    _settle(node)
    end, = _notices(node, "end")
    assert end["status"] == "done" and end["id"] == start["id"] and end["clips"] == 1 and end["sound"] is False


def test_say_topic_gets_an_id_when_queued(node):
    import queue as queue_mod
    _with_tracking(node)
    node._say_queue = queue_mod.Queue()
    node.on_say(tts_node.String(data="  走行を再開できます\x1f\x1e "))
    queued, = _notices(node, "queued")
    item = node._say_queue.get_nowait()
    assert item[0] == "走行を再開できます" and item[3] == queued["id"] and queued["text"] == "走行を再開できます"


def test_between_sentences_speaking_goes_off_but_the_utterance_is_not_done(node):
    """AivisSpeech は文ごと: 2 文目の合成が遅れると speaking は文の間で OFF (タイマー)。発話の end は 2 文とも鳴り終えてから、of=2。"""
    _with_tracking(node)

    class SlowSecond(_Backend):
        def synthesize(self, text, speaker, query):
            if text.startswith("二"):
                time.sleep(2.0)                           # 1 文目 (1 秒) の再生 + 尾が終わってから
            return super().synthesize(text, speaker, query)

    node.vc_backend = SlowSecond()
    node.vc_timeout = 5.0
    done_first = []

    def first_done():
        # 1 文目が鳴り終えた (出力の playback_done)
        while not node.play_pub.frames:
            time.sleep(0.01)
        time.sleep(1.3)
        _done(node, node.play_pub.frames[0])
        _settle(node)
        done_first.append([m.data for m in node.tts_speaking_pub.frames])

    t = threading.Thread(target=first_done)
    t.start()
    clips = node.speak_text("一つ目の文です。二つ目の文です。", "", 8.0)
    t.join()
    assert len(clips) == 2
    assert done_first[0] == [True, False]               # 文の間で読み上げの speaking が OFF になった
    assert [c["status"] for c in _notices(node, "clip")] == ["done"] and _notices(node, "end") == []
    _done(node, node.play_pub.frames[1])
    _settle(node)
    end, = _notices(node, "end")
    assert end["status"] == "done" and end["of"] == 2 and end["clips"] == 2


def test_stop_while_playing_cuts_the_clip(node):
    _with_tracking(node)
    node.vc_backend = _Backend()
    node.speak_text("止められる文です。", "", 8.0)
    frame = node.play_pub.frames[0]
    node.on_stop(None)
    _done(node, frame)                                   # 止めた後の done (別の世代) は数えない
    _settle(node)
    clip, = _notices(node, "clip")
    end, = _notices(node, "end")
    assert clip["status"] == "cut" and end["status"] == "cut"


def test_stop_during_the_second_sentence_is_cut(node):
    _with_tracking(node)

    class Stopper(_Backend):
        def synthesize(self, text, speaker, query):
            out = super().synthesize(text, speaker, query)
            if text.startswith("二"):
                node.on_stop(None)
            return out

    node.vc_backend = Stopper()
    clips = node.speak_text("一つ目の文です。二つ目の文です。", "", 8.0)
    assert len(clips) == 1
    end, = _notices(node, "end")
    assert end["status"] == "cut" and end["of"] == 2
    # 文ごとに 1 つずつ結果がある: 1 文目は鳴っている途中で cut、2 文目は止められて出さなかった cut (GPT P1)
    assert sorted((c["index"], c["status"]) for c in _notices(node, "clip")) == [(0, "cut"), (1, "cut")]
    assert end["statuses"] == ["cut", "cut"] or sorted(end["statuses"]) == ["cut", "cut"]


def test_sound_effect_during_the_utterance_is_reported(node):
    _with_tracking(node)
    node.vc_backend = _Backend()
    node.speak_text("効果音と重なる文です。", "", 8.0)
    node.on_sound_speaking(tts_node.Bool(data=True))
    node.on_sound_speaking(tts_node.Bool(data=False))
    _done(node, node.play_pub.frames[0])
    _settle(node)
    end, = _notices(node, "end")
    assert end["status"] == "done" and end["sound"] is True


def test_open_jtalk_utterance_is_tracked_as_one_clip(node):
    _with_tracking(node)
    node.engine_selected = "pyopenjtalk"
    node.speak_text("テストです。二つ目。", "", 8.0)
    start, = _notices(node, "start")
    assert start["of"] == 1 and start["engine"] == "pyopenjtalk"
    _done(node, node.play_pub.frames[0])
    _settle(node)
    assert _notices(node, "end")[0]["status"] == "done"


def test_unknown_when_outputs_are_not_exactly_one(node):
    _with_tracking(node, outputs=0)
    node.vc_backend = _Backend()
    node.speak_text("出力が無い文です。", "", 8.0)
    _done(node, node.play_pub.frames[0])
    _settle(node)
    assert _notices(node, "end")[0]["status"] == "unknown"


def test_tts_only_speaking_topic_mirrors_the_shared_one(node):
    _with_tracking(node)
    node.vc_backend = _Backend()
    node.speak_text("短い文です。", "", 8.0)
    time.sleep(1.3)
    assert [m.data for m in node.speaking_pub.frames] == [True, False]
    assert [m.data for m in node.tts_speaking_pub.frames] == [True, False]


def test_nothing_is_published_after_destroy(node):
    _with_tracking(node)
    node.vc_backend = _Backend()
    node.speak_text("終わる前の文です。", "", 8.0)
    n = len(node.playback_pub.frames)
    node._closing = True
    _done(node, node.play_pub.frames[0])
    _settle(node)
    node.on_stop(None)
    assert len(node.playback_pub.frames) == n


def test_pause_cuts_the_clip(node):
    _with_tracking(node)
    node.vc_backend = _Backend()
    node.speak_text("一時停止される文です。", "", 8.0)
    node.on_pause(None)                                  # fv_audio_output は鳴っている音を捨てる (done は出さない)
    _done(node, node.play_pub.frames[0])
    _settle(node)
    assert _notices(node, "end")[0]["status"] == "cut"


def test_a_late_check_after_a_blocked_executor_is_unknown(node):
    """done を受けた後、Speak サービスなどで実行器が塞がり tick が遅れた: その間の stop の callback がまだかもしれない (GPT P0)。"""
    _with_tracking(node)
    node.vc_backend = _Backend()
    node.speak_text("確かめが遅れる文です。", "", 8.0)
    _done(node, node.play_pub.frames[0])
    time.sleep(0.05 + node._track.late_s + 0.1)
    node._track_tick()
    end, = _notices(node, "end")
    assert end["status"] == "unknown"
    assert _notices(node, "clip")[0]["reason"] == "late check"


def test_a_second_output_appearing_before_the_check_is_unknown(node):
    _with_tracking(node, outputs=1)
    node.vc_backend = _Backend()
    node.speak_text("出力が増える文です。", "", 8.0)
    node.outputs = 2                                     # 合成中・再生中に 2 つ目の fv_audio_output が上がった
    _done(node, node.play_pub.frames[0])
    _settle(node)
    assert _notices(node, "end")[0]["status"] == "unknown"


def test_sound_unknown_at_start_marks_the_utterance(node):
    _with_tracking(node, sound_unknown=True)             # 起動直後・古い soundboard (出どころの分からない speaking)
    node.vc_backend = _Backend()
    node.speak_text("効果音が分からない文です。", "", 8.0)
    _done(node, node.play_pub.frames[0])
    _settle(node)
    assert _notices(node, "end")[0]["sound"] is True


def test_sound_unknown_rules(node):
    node.speaking_topic, node.sound_speaking_topic = "/aspa/audio/speaking", "/aspa/soundboard/speaking"
    counts = {"/aspa/audio/speaking": 2, "/aspa/soundboard/speaking": 1}
    node.count_publishers = lambda topic: counts[topic]
    node._t_boot = time.monotonic() - 10.0
    assert tts_node.FvTtsNode._sound_unknown(node) is False      # soundboard (新しい) がいる
    counts["/aspa/soundboard/speaking"] = 0
    assert tts_node.FvTtsNode._sound_unknown(node) is True       # 古い soundboard: 効果音だけの印が無い
    counts["/aspa/audio/speaking"] = 1
    assert tts_node.FvTtsNode._sound_unknown(node) is False      # 自分だけ (効果音を出すノードが無い)
    node._t_boot = time.monotonic()
    assert tts_node.FvTtsNode._sound_unknown(node) is True       # 起動直後


def test_extra_speaking_publish_failure_keeps_speaking(node):
    _with_tracking(node)

    class Broken:
        frames = []

        def publish(self, _msg):
            raise RuntimeError("publisher gone")

    node.tts_speaking_pub = Broken()
    node.vc_backend = _Backend()
    clips = node.speak_text("一つ目の文です。二つ目の文です。", "", 8.0)
    assert len(clips) == 2 and len(node.play_pub.frames) == 2      # 足した印が壊れても残りの文を出す
    assert [m.data for m in node.speaking_pub.frames][:1] == [True]
    starts = [c for c in _notices(node, "start")]
    assert starts and node._track.in_flight() == 1                 # 2 クリップとも数えている


def test_destroy_ends_every_pending_utterance_as_cut(node, monkeypatch):
    import queue as queue_mod
    monkeypatch.setattr(tts_node.Node, "destroy_node", lambda self: True)
    _with_tracking(node)
    node._say_queue = queue_mod.Queue()
    node.voice_worker.stop = lambda: None
    node.vc_backend = _Backend()
    node.speak_text("鳴っている間に終了する文です。", "", 8.0)
    node.on_say(tts_node.String(data="受付だけされた文です。"))
    node.destroy_node()
    ends = _notices(node, "end")
    assert sorted(e["text"] for e in ends) == sorted(["鳴っている間に終了する文です。", "受付だけされた文です。"])
    assert {e["status"] for e in ends} == {"cut"}
    n = len(node.playback_pub.frames)
    node.on_stop(None)
    node._track_tick()
    assert len(node.playback_pub.frames) == n                       # 終了の後は何も出さない


def test_drained_output_reports_drained_and_needs_no_latency(node):
    """鳴らし切ってから done を出す出力 (aspa: drain_after_frame=true): done の時にはもう音は残っていない。"""
    _with_tracking(node, drained=True, latency=0.4)
    node.vc_backend = _Backend()
    node.speak_text("鳴らし切る出力の文です。", "", 8.0)
    _done(node, node.play_pub.frames[0])
    node._track_tick()                                     # 待たずに確定 (出力の遅れを待つ必要が無い)
    end, = _notices(node, "end")
    assert end["status"] == "done" and end["drained"] is True
    assert _notices(node, "clip")[0]["drained"] is True


def test_not_drained_output_reports_drained_false(node):
    _with_tracking(node, drained=False)
    node.vc_backend = _Backend()
    node.speak_text("鳴らし切らない出力の文です。", "", 8.0)
    _done(node, node.play_pub.frames[0])
    _settle(node)
    end, = _notices(node, "end")
    assert end["status"] == "done" and end["drained"] is False      # 聞く側は証拠にしない


def test_output_restarted_between_publish_and_done_is_unknown(node):
    _with_tracking(node, drained=True)
    node.vc_backend = _Backend()
    node.speak_text("出力が入れ替わる文です。", "", 8.0)
    node.output_gid = b"\x02"                               # fv_audio_output が再起動した (gid が変わる)
    _done(node, node.play_pub.frames[0])
    _settle(node)
    assert _notices(node, "end")[0]["status"] == "unknown"


def test_output_state_is_the_single_done_publisher_that_plays(node):
    """_output_state: playback_done の出し手がちょうど 1 つで、再生の topic の購読者でもある時だけ、その publisher の gid。"""
    from types import SimpleNamespace as NS
    node.playback_done_topic, node.playback_topic = "audio/output/playback_done", "audio/output/frame"
    node.resolve_topic_name = lambda t: "/" + t
    pubs = [NS(node_name="fv_audio_output", node_namespace="/", endpoint_gid=[7, 7])]
    subs = [NS(node_name="fv_audio_output", node_namespace="/"), NS(node_name="bridge", node_namespace="/")]
    node.get_publishers_info_by_topic = lambda topic: list(pubs)
    node.get_subscriptions_info_by_topic = lambda topic: list(subs)
    assert tts_node.FvTtsNode._output_state(node) == bytes([7, 7])
    pubs.append(NS(node_name="fv_audio_output2", node_namespace="/", endpoint_gid=[8]))
    assert tts_node.FvTtsNode._output_state(node) is None                   # 出力が 2 つ
    pubs.pop()
    subs.pop(0)
    assert tts_node.FvTtsNode._output_state(node) is None                   # done を出すが再生の購読者でない


def test_done_from_another_output_or_an_old_output_is_unknown(node):
    """done の送り手 (PlaybackDone.publisher_gid) がクリップを出した時の出力でなければ unknown (GPT: 一時的な 2 つ目の出力)。"""
    for gid in (b"\x09", b""):                            # 別の出力 / 送り手の分からない古い出力
        _with_tracking(node, drained=True)
        node.done_gid = gid
        node.vc_backend = _Backend()
        node.speak_text("送り手が違う文です。", "", 8.0)
        _done(node, node.play_pub.frames[-1])
        _settle(node, 1.2)
        end = _notices(node, "end")[-1]
        assert end["status"] == "unknown" and end["drained"] is False, (gid, end)
