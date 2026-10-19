"""tts_node の声キャッシュ経路 (ROS 環境が必要。無ければ skip)。

ノードは起動せず、__init__ を飛ばした素のインスタンスに部品を差し込む。
"""
import numpy as np
import pytest

tts_node = pytest.importorskip("fv_tts_py.tts_node")
vc = pytest.importorskip("fv_tts_py.voice_cache")
VoicevoxUnreachable = tts_node.VoicevoxUnreachable


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
    n.vc_kind, n.vc_speaker, n.vc_query, n.vc_max_chars = "voicevox", 135718976, {}, 120
    n.voice_store = vc.VoiceCacheStore(tmp_path, 10 ** 8)
    n.submitted = []

    class W:
        def submit(self, key, text, ref, priority=True):
            n.submitted.append((key, text, ref, priority))
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
    key, text, ref, prio = node.submitted[0]
    assert text == "テストです。" and prio and ref == pytest.approx(0.3 / np.sqrt(2), rel=0.02)


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
    pcm, sr = node._render_voice("k", "テスト", 0.2)
    assert sr == 48000
    assert vc.active_rms(vc.pcm16_to_float(pcm), sr) == pytest.approx(0.2, rel=0.03)


def test_render_maps_unreachable_to_engine_down(node):
    class Backend:
        def synthesize(self, *a):
            raise VoicevoxUnreachable("connection refused")

    node.vc_backend = Backend()
    with pytest.raises(vc.EngineDown):
        node._render_voice("k", "テスト", 0.2)

