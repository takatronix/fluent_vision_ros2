#!/usr/bin/env python3
import concurrent.futures
import hashlib
import importlib
import json
import logging
import os
import sys
import threading
import time
from array import array
from pathlib import Path
from typing import Dict, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Bool, Empty, String

from fv_audio.msg import AudioFrame
from fv_tts.srv import Speak

from fv_tts_py.aivis_backend import DEFAULT_URL as AIVIS_DEFAULT_URL, AivisBackend, AivisUnreachable
from fv_tts_py import voice_cache as vc

# Disable tqdm progress bar for pyopenjtalk downloads
os.environ["TQDM_DISABLE"] = "1"

pyopenjtalk = None
_IMPORT_ERROR = None
_PYOPENJTALK_LOADED = False
# pyopenjtalk は内部状態を持つので、声キャッシュの裏スレッドと同時に呼ばない
_PYOPENJTALK_LOCK = threading.Lock()

ENGINES = ("pyopenjtalk", "aivis")
AIVIS_FEMALE = 135718976   # 楽町音穏_T3モデル ノーマル (女)
AIVIS_MALE = 1310138977    # 阿井田 茂 Calm (男)


def resolve_engine(engine: str, aivis_url: str):
    """engine パラメータを pyopenjtalk | aivis に正規化する。(engine, 警告文 or None)。

    旧設定の engine=voicevox (VOICEVOX 対応は削除済み) では止めずに、AivisSpeech の
    URL が設定されていれば aivis、無ければ pyopenjtalk として動かす。
    """
    name = (engine or "").strip().lower() or "pyopenjtalk"
    if name in ENGINES:
        return name, None
    if name == "voicevox":
        new = "aivis" if (aivis_url or "").strip() else "pyopenjtalk"
        return new, (f"engine=voicevox は廃止 (VOICEVOX 対応は削除) → engine={new} として動かします。"
                     f"設定を engine:={new} に直してください")
    raise RuntimeError(f"未知の engine='{engine}' (pyopenjtalk | aivis のいずれか)")


def load_pyopenjtalk():
    """Load Open JTalk lazily (first use).

    The bundled ARM64 wheel also contains the dictionary and voice, so
    importing it eagerly would use otherwise avoidable memory.
    """
    global pyopenjtalk, _IMPORT_ERROR, _PYOPENJTALK_LOADED
    if _PYOPENJTALK_LOADED:
        return pyopenjtalk
    _PYOPENJTALK_LOADED = True
    original_stdout = sys.stdout
    original_stderr = sys.stderr
    quiet_stream = None
    try:
        # Suppress dictionary/download progress output during the first import.
        quiet_stream = open(os.devnull, "w", encoding="utf-8")
        sys.stdout = quiet_stream
        sys.stderr = quiet_stream
        pyopenjtalk = importlib.import_module("pyopenjtalk")
    except ImportError as exc:  # pragma: no cover - deployment guard
        _IMPORT_ERROR = exc
        pyopenjtalk = None
    finally:
        sys.stdout = original_stdout
        sys.stderr = original_stderr
        if quiet_stream is not None:
            quiet_stream.close()
    return pyopenjtalk


class CachedAudio:
    __slots__ = ("audio_bytes", "sample_rate", "channels", "bit_depth", "rms", "peak")

    def __init__(self, audio_bytes: bytes, sample_rate: int, channels: int, bit_depth: int,
                 rms: float, peak: float) -> None:
        self.audio_bytes = audio_bytes
        self.sample_rate = sample_rate
        self.channels = channels
        self.bit_depth = bit_depth
        self.rms = rms
        self.peak = peak


class FvTtsNode(Node):
    def __init__(self) -> None:
        super().__init__("fv_tts")

        self.declare_parameter("default_voice", "")
        self.declare_parameter("output_topic", "audio/tts/frame")
        self.declare_parameter("playback_topic", "audio/output/frame")
        self.declare_parameter("use_playback_topic", True)
        self.declare_parameter("stop_topic", "audio/output/stop")
        self.declare_parameter("cache_dir", "")
        # Open JTalk gain (it is quiet): +8 dB.  The AivisSpeech voice has its
        # own gain (voice_cache_volume_db, 0 dB).
        self.declare_parameter("default_volume_db", 8.0)
        # エンジン: "pyopenjtalk"(軽量・既定) | "aivis" (AivisSpeech。Open JTalk が保険)
        self.declare_parameter("engine", "pyopenjtalk")
        # AivisSpeech Engine の URL。"" = 未設定 (engine=aivis なら既定 :10101)。
        # engine=pyopenjtalk でも設定されていれば、viewer の設定 (tts_engine=aivis)
        # で再起動なしに AivisSpeech へ切り替えられるよう準備しておく。
        self.declare_parameter("aivis_url", "")
        self.declare_parameter("aivis_speaker", AIVIS_FEMALE)  # 女。男は 1310138977
        # 出力パイプライン (fv_audio_output) のサンプルレート。AivisSpeech は 44.1kHz を
        # 返すので、このレート(既定 48kHz)へリサンプルしてから publish する。
        self.declare_parameter("output_sample_rate", 48000)
        # Event Bus 契約: 任意テキストを喋らせる topic I/F と再生中フラグ
        self.declare_parameter("say_topic", "/aspa/tts/say")
        self.declare_parameter("speaking_topic", "/aspa/audio/speaking")
        self.declare_parameter("settings_topic", "/aspa/tts/settings")

        self.default_voice = self.get_parameter("default_voice").get_parameter_value().string_value
        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.playback_topic = self.get_parameter("playback_topic").get_parameter_value().string_value
        self.publish_playback = self.get_parameter("use_playback_topic").get_parameter_value().bool_value
        self.stop_topic = self.get_parameter("stop_topic").get_parameter_value().string_value
        self.default_volume_db = self.get_parameter("default_volume_db").get_parameter_value().double_value
        self.aivis_url = str(self.get_parameter("aivis_url").value or "").strip()
        self.engine, warning = resolve_engine(
            self.get_parameter("engine").get_parameter_value().string_value, self.aivis_url)
        if warning:
            self.get_logger().warn(warning)
        self.output_sample_rate = self.get_parameter("output_sample_rate").get_parameter_value().integer_value
        self.say_topic = self.get_parameter("say_topic").get_parameter_value().string_value
        self.speaking_topic = self.get_parameter("speaking_topic").get_parameter_value().string_value
        self.settings_topic = self.get_parameter("settings_topic").value

        # AivisSpeech の声 + 声キャッシュ: 保存済みの文はその WAV、無ければその場で
        # AivisSpeech (budget 内)、間に合わない・不通なら Open JTalk で即答し裏で作る。
        self.declare_parameter("aivis_query", "{}")  # audio_query 上書き JSON
        self.declare_parameter("voice_cache_dir", "~/.aspa/tts_cache/voice")
        self.declare_parameter("voice_cache_max_mb", 300)
        self.declare_parameter("voice_cache_max_chars", 120)
        self.declare_parameter("voice_cache_timeout_s", 60.0)
        self.declare_parameter("voice_cache_warmup_file", "")
        self.declare_parameter("voice_cache_warmup_delay_s", 90.0)
        self.declare_parameter("voice_cache_max_load", 0.0)  # 1分 loadavg がこれ以上ならウォームアップ待ち (0=無効)
        self.declare_parameter("voice_cache_nice", 10)
        # キャッシュに無い文: エンジンが元気ならこの秒数までその場で高品質合成を待つ
        # (0 = 待たない。常に Open JTalk で即答し裏で作る)。超えたら Open JTalk。
        self.declare_parameter("voice_cache_live_budget_s", 2.0)
        # 高品質の声のゲイン。AivisSpeech は素の 0dB で Open JTalk +8dB とほぼ同じ音量。
        self.declare_parameter("voice_cache_volume_db", 0.0)
        # True なら文ごとに Open JTalk 版の発話 RMS へ合わせる (旧挙動)。
        self.declare_parameter("voice_cache_match_loudness", False)
        # エンジン不通を検知したら、この秒数はその場合成を試さない (毎回待たせない)。
        self.declare_parameter("voice_cache_down_skip_s", 30.0)

        self.get_logger().info(f"Starting FV TTS node (engine={self.engine})")

        # Open JTalk はどちらのエンジンでも要る (aivis の保険)。無ければ止める。
        if load_pyopenjtalk() is None:
            raise RuntimeError(
                f"pyopenjtalk が使えません: {_IMPORT_ERROR}"
            ) from _IMPORT_ERROR

        cache_dir_param = self.get_parameter("cache_dir").get_parameter_value().string_value
        self.cache_dir = Path(cache_dir_param).expanduser() if cache_dir_param else None
        if self.cache_dir:
            self.cache_dir.mkdir(parents=True, exist_ok=True)

        # RELIABLE QoS でメッセージドロップを防ぐ
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        qos.history = HistoryPolicy.KEEP_LAST

        self.tts_pub = self.create_publisher(AudioFrame, self.output_topic, qos)
        self.play_pub = self.create_publisher(AudioFrame, self.playback_topic, qos)
        self.speaking_pub = self.create_publisher(Bool, self.speaking_topic, 10)
        self.srv = self.create_service(Speak, "speak", self.handle_speak)

        # Event Bus 契約: soundboard 等が /aspa/tts/say に流す任意テキストを喋る
        self.say_sub = self.create_subscription(String, self.say_topic, self.on_say, 10)
        # Scene Viewerの永続設定。TRANSIENT_LOCALなのでTTS再起動時にも最新話者を受信。
        settings_qos = QoSProfile(depth=1)
        settings_qos.reliability = ReliabilityPolicy.RELIABLE
        settings_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        settings_qos.history = HistoryPolicy.KEEP_LAST
        self.settings_sub = self.create_subscription(
            String, self.settings_topic, self.on_settings, settings_qos)

        # Barge-in/stop 後に古い音声が鳴らないよう、再生世代(epoch)を管理する。
        # epoch は AudioFrame.epoch に埋め込み、audio_output 側で stop 時に世代を進めて破棄する。
        self.playback_epoch = 1
        self.stop_sub = self.create_subscription(Empty, self.stop_topic, self.on_stop, qos)

        # /aspa/audio/speaking (再生中フラグ) を再生尺ベースで上げ下げする。
        # 実再生は fv_audio_output が非同期で行うため、再生尺 + tail からタイマで戻す。
        self._speaking = False
        self._speaking_end = 0.0
        self._speaking_timer: Optional[threading.Timer] = None
        self._speaking_lock = threading.Lock()

        self.cache: Dict[str, CachedAudio] = {}

        self.voice_store: Optional[vc.VoiceCacheStore] = None
        self.voice_worker: Optional[vc.RenderWorker] = None
        self._warmup_timer: Optional[threading.Timer] = None
        self._warmup_path: Optional[Path] = None
        self._warmup_due = False
        # AivisSpeech の声を使うか。起動時は engine=aivis なら ON、viewer の設定
        # (tts_engine=aivis | pyopenjtalk) で再起動なしに ON/OFF を切り替える。
        self.vc_active = False
        self._vc_pool: Optional[concurrent.futures.ThreadPoolExecutor] = None
        self._vc_inflight: set = set()
        self._vc_lock = threading.Lock()
        self._vc_skip_until = 0.0
        try:
            self._setup_voice_cache()
        except Exception as exc:  # pylint: disable=broad-except
            # AivisSpeech は付加機能。壊れていても Open JTalk はそのまま喋る。
            self.get_logger().error(f"AivisSpeech voice disabled: {exc}")
            self.voice_store = None
            self.voice_worker = None

    # ── 声キャッシュ ───────────────────────────────────
    def _setup_voice_cache(self) -> None:
        if self.engine != "aivis" and not self.aivis_url:
            return  # Open JTalk だけ (AivisSpeech は使わない)
        self.vc_speaker = int(self.get_parameter("aivis_speaker").value)
        if self.vc_speaker < 0:
            raise RuntimeError("aivis_speaker が負")
        self.vc_query = json.loads(str(self.get_parameter("aivis_query").value or "{}"))
        if not isinstance(self.vc_query, dict):
            raise RuntimeError("aivis_query は JSON object")
        # キャッシュキーに入るエンジン名。旧 VOICEVOX 経路の保存音とは混ざらない。
        kind = "aivis"
        self.vc_kind = kind
        self.vc_max_chars = int(self.get_parameter("voice_cache_max_chars").value)
        url = self.aivis_url or AIVIS_DEFAULT_URL
        self.vc_backend = AivisBackend(
            url, timeout=float(self.get_parameter("voice_cache_timeout_s").value))
        max_mb = int(self.get_parameter("voice_cache_max_mb").value)
        cache_dir = str(self.get_parameter("voice_cache_dir").value)
        self.voice_store = vc.VoiceCacheStore(cache_dir, max_mb * 1024 * 1024)
        max_load = float(self.get_parameter("voice_cache_max_load").value)
        busy = (lambda: os.getloadavg()[0] >= max_load) if max_load > 0 else None
        self.vc_live_budget = float(self.get_parameter("voice_cache_live_budget_s").value)
        self.vc_volume_db = float(self.get_parameter("voice_cache_volume_db").value)
        self.vc_match_loudness = bool(self.get_parameter("voice_cache_match_loudness").value)
        self.vc_down_skip = float(self.get_parameter("voice_cache_down_skip_s").value)
        self.voice_worker = vc.RenderWorker(
            self.voice_store, self._render_voice,
            meta={"engine": kind},
            log=lambda m: self.get_logger().info(m),
            busy=busy, nice=int(self.get_parameter("voice_cache_nice").value))
        self.voice_worker.start()
        self._vc_pool = concurrent.futures.ThreadPoolExecutor(
            max_workers=2, thread_name_prefix="voice_live")
        self.vc_active = self.engine == "aivis"
        self.get_logger().info(
            f"AivisSpeech {'ON' if self.vc_active else '準備のみ (viewer の tts_engine=aivis で ON)'}: {url} live<={self.vc_live_budget:.1f}s speaker={self.vc_speaker} dir={self.voice_store.dir} "
            f"({len(self.voice_store.index)} 件, {self.voice_store.total_bytes() / 1e6:.1f}MB / {max_mb}MB)")

        warmup = str(self.get_parameter("voice_cache_warmup_file").value or "").strip()
        if warmup:
            path = Path(warmup).expanduser()
            if not path.is_absolute():
                try:
                    from ament_index_python.packages import get_package_share_directory
                    path = Path(get_package_share_directory("fv_tts")) / "config" / warmup
                except Exception:  # pylint: disable=broad-except
                    pass
            self._warmup_path = path
            delay = float(self.get_parameter("voice_cache_warmup_delay_s").value)
            self._warmup_timer = threading.Timer(delay, self._warmup_fire)
            self._warmup_timer.daemon = True
            self._warmup_timer.start()

    def _voice_key(self, text: str, volume_db: float, speaker: Optional[int] = None) -> str:
        spk = self.vc_speaker if speaker is None else int(speaker)
        return vc.make_key(self.vc_kind, spk, self.vc_query, volume_db, text)

    def _warmup_fire(self) -> None:
        self._warmup_due = True
        if self.vc_active and self._warmup_path is not None:
            self._enqueue_warmup(self._warmup_path)

    def _enqueue_warmup(self, path: Path) -> None:
        try:
            texts = vc.load_warmup(path)
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warn(f"voice cache warmup 読込失敗 {path}: {exc}")
            return
        speaker = self.vc_speaker
        n = 0
        for text in texts:
            if self.voice_worker and len(text) <= self.vc_max_chars:
                n += self.voice_worker.submit(
                    self._voice_key(text, self.default_volume_db, speaker), text, None,
                    priority=False, job={"speaker": speaker})
        self.get_logger().info(
            f"voice cache warmup: {len(texts)} 文中 {n} 文を裏で作成予定 "
            f"(speaker={speaker}, {path.name})")

    def _render_voice(self, _key: str, text: str, ref_rms: Optional[float],
                      job: Optional[Dict] = None):
        """高品質エンジンで合成→出力レートへ→音量 (固定ゲイン or Open JTalk 版に合わせる)。"""
        speaker = int((job or {}).get("speaker", self.vc_speaker))
        try:
            pcm, sr, channels, bit_depth = self.vc_backend.synthesize(text, speaker, self.vc_query)
        except AivisUnreachable as exc:
            raise vc.EngineDown(str(exc)) from exc
        if channels != 1 or bit_depth != 16:
            raise RuntimeError(f"想定外フォーマット channels={channels} bit_depth={bit_depth}")
        out_sr = int(self.output_sample_rate) if self.output_sample_rate > 0 else sr
        wave_f = vc.resample(vc.pcm16_to_float(pcm), sr, out_sr)
        if getattr(self, "vc_match_loudness", False):
            if ref_rms is None:
                ref = self._synthesize_pyopenjtalk(text, "", self.default_volume_db)
                ref_rms = vc.active_rms(vc.pcm16_to_float(ref.audio_bytes), ref.sample_rate)
            wave_f, gain_db = vc.match_loudness(wave_f, out_sr, ref_rms)
            self.get_logger().debug(f"voice cache gain {gain_db:+.1f}dB")
        else:
            gain_db = float(getattr(self, "vc_volume_db", 0.0))
            if gain_db != 0.0:
                wave_f = np.clip(wave_f * float(10.0 ** (gain_db / 20.0)), -1.0, 1.0)
        return vc.float_to_pcm16(wave_f), out_sr

    @staticmethod
    def _to_cached_audio(pcm: bytes, sr: int) -> CachedAudio:
        samples = vc.pcm16_to_float(pcm)
        rms = float(np.sqrt(np.mean(np.square(samples)))) if samples.size else 0.0
        peak = float(np.max(np.abs(samples))) if samples.size else 0.0
        return CachedAudio(pcm, sr, 1, 16, rms, peak)

    def _voice_cache_lookup(self, text: str, voice_id: str, volume_db: float):
        """(CachedAudio or None, key or None, speaker)。key None なら声キャッシュ対象外。"""
        speaker = getattr(self, "vc_speaker", -1)
        if self.voice_store is None or not getattr(self, "vc_active", True) or voice_id \
                or len(vc.normalize_text(text)) > self.vc_max_chars:
            return None, None, speaker
        key = self._voice_key(text, volume_db, speaker)
        try:
            hit = self.voice_store.get(key)
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warn(f"voice cache read failed: {exc}")
            return None, key, speaker
        if hit is None:
            return None, key, speaker
        self.get_logger().info(f"voice: キャッシュの声 ({self.vc_kind}:{speaker})")
        return self._to_cached_audio(*hit), key, speaker

    def _vc_put(self, key: str, text: str, speaker: int, pcm: bytes, sr: int, render_s: float) -> None:
        try:
            self.voice_store.put(key, pcm, sr, {
                "engine": self.vc_kind, "speaker": int(speaker),
                "text": vc.normalize_text(text), "render_s": round(render_s, 2)})
        except OSError as exc:
            self.get_logger().warn(f"voice cache: 保存失敗 {exc}")

    def _voice_live(self, key: str, text: str, speaker: int) -> Optional[CachedAudio]:
        """キャッシュに無い文を、エンジンが元気なら budget 秒までその場で合成する。

        間に合わなければ None (呼び出し側が Open JTalk で即答)。合成はそのまま続け、
        できたらキャッシュに入れる (次回から高品質)。エンジン不通なら暫く試さない。
        """
        budget = float(getattr(self, "vc_live_budget", 0.0))
        pool = getattr(self, "_vc_pool", None)
        if budget <= 0 or pool is None:
            return None
        now = time.monotonic()
        worker = self.voice_worker
        if now < self._vc_skip_until or (worker is not None and getattr(worker, "backoff", 0.0) > 0):
            return None
        with self._vc_lock:
            if key in self._vc_inflight:
                return None
            self._vc_inflight.add(key)
        job = {"speaker": speaker}
        fut = pool.submit(self._render_voice, key, text, None, job)
        try:
            pcm, sr = fut.result(timeout=budget)
        except concurrent.futures.TimeoutError:
            # 遅い (未読込の声・高負荷)。今回は Open JTalk、続きは裏で保存。
            self._vc_skip_until = time.monotonic() + min(10.0, self.vc_down_skip)
            self.get_logger().warn(
                f"voice: 高品質の声が {budget:.1f}s で間に合わない → Open JTalk (続きは裏で保存)")
            fut.add_done_callback(lambda f: self._vc_late_done(key, text, speaker, now, f))
            return None
        except vc.EngineDown as exc:
            self._vc_skip_until = time.monotonic() + self.vc_down_skip
            self.get_logger().warn(f"voice: 高品質エンジン不通 → Open JTalk ({exc})")
            self._vc_discard(key)
            return None
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warn(f"voice: 高品質の合成失敗 → Open JTalk ({exc})")
            self._vc_discard(key)
            return None
        dt = time.monotonic() - now
        self._vc_put(key, text, speaker, pcm, sr, dt)
        self._vc_discard(key)
        self.get_logger().info(f"voice: その場で高品質合成 {dt:.2f}s (speaker={speaker})")
        return self._to_cached_audio(pcm, sr)

    def _vc_discard(self, key: str) -> None:
        with self._vc_lock:
            self._vc_inflight.discard(key)

    def _vc_late_done(self, key: str, text: str, speaker: int, t0: float, fut) -> None:
        exc = fut.exception()
        self._vc_discard(key)
        if exc is None:
            pcm, sr = fut.result()
            self._vc_put(key, text, speaker, pcm, sr, time.monotonic() - t0)
            self.get_logger().info(f"voice cache: 遅れて保存 '{text[:30]}'")
            return
        if isinstance(exc, vc.EngineDown):
            self._vc_skip_until = time.monotonic() + self.vc_down_skip
        if self.voice_worker is not None:  # 裏の作業者に任せる (バックオフ・再試行あり)
            self.voice_worker.submit(key, text, None, priority=True, job={"speaker": speaker})

    def _voice_cache_submit(self, key: Optional[str], text: str, cached: CachedAudio,
                            speaker: Optional[int] = None) -> None:
        if key is None or self.voice_worker is None:
            return
        with self._vc_lock:
            if key in self._vc_inflight:  # その場合成の続きが保存する
                return
        spk = self.vc_speaker if speaker is None else speaker
        ref = vc.active_rms(vc.pcm16_to_float(cached.audio_bytes), cached.sample_rate)
        if self.voice_worker.submit(key, text, ref, priority=True, job={"speaker": spk}):
            self.get_logger().info(f"voice cache: 裏で作成を予約 (待ち {self.voice_worker.queued()})")

    def get_audio(self, text: str, voice_id: str, volume_db: float, cache_key: str) -> CachedAudio:
        """1) 声キャッシュ 2) その場で高品質 (budget 内) 3) Open JTalk + 裏で作成予約。"""
        voiced, vkey, speaker = self._voice_cache_lookup(text, voice_id, volume_db)
        if voiced is not None:
            return voiced
        if vkey is not None:
            live = self._voice_live(vkey, text, speaker)
            if live is not None:
                return live
        cached = self.get_cached(text, voice_id, volume_db, cache_key)
        self._voice_cache_submit(vkey, text, cached, speaker)
        return cached

    def destroy_node(self):
        if self._warmup_timer is not None:
            self._warmup_timer.cancel()
        if self.voice_worker is not None:
            self.voice_worker.stop()
        if self._vc_pool is not None:
            self._vc_pool.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()

    def on_settings(self, msg: String) -> None:
        try:
            obj = json.loads(msg.data or "{}")
            if not isinstance(obj, dict):
                raise ValueError("settings must be a JSON object")
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f"TTS settings parse failed: {exc}")
            return
        self._apply_voice_settings(obj)

    def _apply_voice_settings(self, obj: Dict) -> None:
        """viewer の設定: tts_engine=aivis で AivisSpeech ON、aivis_speaker で男/女。"""
        if self.voice_store is None:
            return
        engine = str(obj.get("tts_engine", "") or "").strip().lower()
        if engine:
            # 旧 viewer の保存値 voicevox は aivis 扱い (VOICEVOX 対応は削除済み)
            active = engine in ("aivis", "voicevox")
            if active != self.vc_active:
                self.vc_active = active
                self.get_logger().info(
                    f"高品質の声 (AivisSpeech): {'ON' if active else 'OFF (Open JTalk のみ)'}")
        if "aivis_speaker" not in obj:
            return
        try:
            speaker = int(obj["aivis_speaker"])
            if speaker < 0:
                raise ValueError("aivis_speaker must be >= 0")
        except (TypeError, ValueError) as exc:
            self.get_logger().warn(f"aivis_speaker parse failed: {exc}")
            return
        if speaker == self.vc_speaker:
            return
        self.get_logger().info(
            f"声の切替: speaker {self.vc_speaker} -> {speaker} ({obj.get('voice', '')})")
        self.vc_speaker = speaker
        self._vc_skip_until = 0.0
        # 起動直後の待ち時間が過ぎていれば、新しい声の決まり文句を裏で作り直す
        if self.vc_active and self._warmup_due and self._warmup_path is not None:
            self._enqueue_warmup(self._warmup_path)

    def on_stop(self, _msg: Empty) -> None:
        """音声停止を受けたら再生世代を進める。"""
        self.playback_epoch += 1

    def handle_speak(self, request: Speak.Request, response: Speak.Response) -> Speak.Response:
        text = request.text.strip()
        self.get_logger().info(f"TTS request: text={text[:50]}... play={request.play}")

        if not text:
            response.success = False
            response.message = "text is empty"
            self.get_logger().warn("TTS rejected: empty text")
            return response

        # stop 後に遅れて publish される旧音声を確実に drop できるよう、
        # stamp/epoch はリクエスト受領時点で確定させて全フレームに適用する。
        request_stamp = self.get_clock().now().to_msg()
        request_epoch = self.playback_epoch

        voice_id = request.voice_id or self.default_voice
        volume_db = request.volume_db if request.volume_db != 0.0 else self.default_volume_db
        cache_key = request.cache_key or self.make_cache_key(text, voice_id, volume_db)

        try:
            cached = self.get_audio(text, voice_id, volume_db, cache_key)
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f"TTS failed: {exc}")
            response.success = False
            response.message = f"TTS failed: {exc}"
            return response

        frame = self.build_frame(cached, stamp=request_stamp, epoch=request_epoch)
        response.frame = frame
        response.success = True
        response.message = "ok"

        self.tts_pub.publish(frame)
        response.played = False
        if request.play and self.publish_playback:
            self.play_pub.publish(frame)
            response.played = True
            self.begin_speaking(cached)
        else:
            self.get_logger().warn(f"TTS NOT published to playback: play={request.play}, publish_playback={self.publish_playback}")

        return response

    def on_say(self, msg: String) -> None:
        """Event Bus: 任意テキストを喋る (/aspa/tts/say)。常に再生する。"""
        text = (msg.data or "").strip()
        if not text:
            return
        self.get_logger().info(f"say topic: text={text[:50]}")
        voice_id = self.default_voice
        volume_db = self.default_volume_db
        cache_key = self.make_cache_key(text, voice_id, volume_db)
        try:
            cached = self.get_audio(text, voice_id, volume_db, cache_key)
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f"say TTS failed: {exc}")
            return
        stamp = self.get_clock().now().to_msg()
        frame = self.build_frame(cached, stamp=stamp, epoch=self.playback_epoch)
        self.tts_pub.publish(frame)
        if self.publish_playback:
            self.play_pub.publish(frame)
            self.begin_speaking(cached)

    def get_cached(self, text: str, voice_id: str, volume_db: float, cache_key: str) -> CachedAudio:
        """キャッシュ参照→無ければ合成して格納。合成失敗は例外送出 (fallback 禁止)。"""
        cached = self.cache.get(cache_key)
        if cached is None and self.cache_dir:
            cached = self.load_cache_from_disk(cache_key)
        if cached is not None:
            self.get_logger().info("TTS cache hit")
            return cached
        self.get_logger().info("TTS cache miss, synthesizing...")
        cached = self.synthesize(text, voice_id, volume_db)
        self.cache[cache_key] = cached
        if self.cache_dir:
            self.write_cache_to_disk(cache_key, cached)
        self.get_logger().info(
            f"TTS synthesized: {len(cached.audio_bytes)} bytes, {cached.sample_rate}Hz")
        return cached

    def begin_speaking(self, cached: CachedAudio) -> None:
        """再生尺ぶん /aspa/audio/speaking=True を立て、tail 経過後 False に戻す。"""
        bytes_per_sample = cached.channels * (cached.bit_depth // 8)
        duration = len(cached.audio_bytes) / bytes_per_sample / cached.sample_rate if bytes_per_sample else 0.0
        self.get_logger().info(f"TTS published to playback: {duration:.2f}s")
        with self._speaking_lock:
            if not self._speaking:
                self._speaking = True
                self.speaking_pub.publish(Bool(data=True))
            # tail 0.15s: 再生完了とフラグ降下のずれを吸収
            end = time.monotonic() + duration + 0.15
            self._speaking_end = max(self._speaking_end, end)
            if self._speaking_timer is not None:
                self._speaking_timer.cancel()
            delay = max(0.05, self._speaking_end - time.monotonic())
            self._speaking_timer = threading.Timer(delay, self._end_speaking)
            self._speaking_timer.daemon = True
            self._speaking_timer.start()

    def _end_speaking(self) -> None:
        with self._speaking_lock:
            # 後続発話でタイマが更新された場合はまだ喋っているので降ろさない
            if time.monotonic() + 0.02 < self._speaking_end:
                return
            self._speaking = False
            self._speaking_timer = None
            self.speaking_pub.publish(Bool(data=False))

    def synthesize(self, text: str, voice_id: str, volume_db: float) -> CachedAudio:
        """Open JTalk で合成。CachedAudio(16bit mono)を返す (AivisSpeech は get_audio 側)。"""
        return self._synthesize_pyopenjtalk(text, voice_id, volume_db)

    def _synthesize_pyopenjtalk(self, text: str, voice_id: str, volume_db: float) -> CachedAudio:
        if load_pyopenjtalk() is None:
            raise RuntimeError(f"pyopenjtalk is not available: {_IMPORT_ERROR}")
        options = {"predict_nani": False}
        if voice_id:
            self.get_logger().warn(
                f"pyopenjtalk: voice_id='{voice_id}' is ignored (single voice engine)")
        with _PYOPENJTALK_LOCK:
            wav, sample_rate = pyopenjtalk.tts(text, **options)
        waveform = np.asarray(wav, dtype=np.float32)

        # pyopenjtalk が返す波形は float64 だが、環境によっては
        # -32768〜32767 相当のスケールで返ってくることがある。
        # そのまま[-1,1]にクリップすると全区間が潰れてノイズ化するため、
        # 振幅が1.0を超える場合は16bitスケールとして正規化する。
        if waveform.size:
            abs_max = float(np.max(np.abs(waveform)))
            if abs_max > 1.0:
                waveform /= 32768.0
        else:
            abs_max = 1.0
        if volume_db != 0.0:
            gain = float(10.0 ** (volume_db / 20.0))
            waveform *= gain
        waveform = np.clip(waveform, -1.0, 1.0)
        pcm = (waveform * 32767.0).astype(np.int16)
        audio_bytes = pcm.tobytes()
        float_samples = pcm.astype(np.float32) / 32768.0
        rms = float(np.sqrt(np.mean(np.square(float_samples)))) if float_samples.size else 0.0
        peak = float(np.max(np.abs(float_samples))) if float_samples.size else 0.0
        return CachedAudio(audio_bytes, int(sample_rate), 1, 16, rms, peak)

    def build_frame(self, cached: CachedAudio, *, stamp=None, epoch: int | None = None) -> AudioFrame:
        frame = AudioFrame()
        frame.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        frame.encoding = "PCM16LE"
        frame.sample_rate = cached.sample_rate
        frame.channels = cached.channels
        frame.bit_depth = cached.bit_depth
        frame.rms = cached.rms
        frame.peak = cached.peak
        frame.vad = False
        frame.data = array('B', cached.audio_bytes)
        frame.epoch = int(epoch) if epoch is not None else 0
        return frame

    def make_cache_key(self, text: str, voice_id: str, volume_db: float) -> str:
        digest = hashlib.sha1()  # nosec B303
        # Open JTalk 版のキャッシュ (AivisSpeech の声は voice_cache 側、別キー)
        digest.update(b"pyopenjtalk")
        digest.update(text.encode("utf-8"))
        digest.update(voice_id.encode("utf-8"))
        digest.update(str(volume_db).encode("utf-8"))
        return digest.hexdigest()

    def cache_file_path(self, cache_key: str) -> Optional[Path]:
        if not self.cache_dir:
            return None
        return self.cache_dir / f"{cache_key}.pcm"

    def load_cache_from_disk(self, cache_key: str) -> Optional[CachedAudio]:
        path = self.cache_file_path(cache_key)
        if not path or not path.exists():
            return None
        try:
            data = path.read_bytes()
        except OSError as exc:
            self.get_logger().warning("Failed to read cache file %s: %s", path, exc)
            return None
        meta_path = path.with_suffix(".meta")
        sample_rate = 48000
        channels = 1
        bit_depth = 16
        rms = 0.0
        peak = 0.0
        if meta_path.exists():
            try:
                content = meta_path.read_text().strip().split("\n")
                for line in content:
                    if not line:
                        continue
                    key, _, value = line.partition(":")
                    key = key.strip()
                    value = value.strip()
                    if key == "sample_rate":
                        sample_rate = int(value)
                    elif key == "channels":
                        channels = int(value)
                    elif key == "bit_depth":
                        bit_depth = int(value)
                    elif key == "rms":
                        rms = float(value)
                    elif key == "peak":
                        peak = float(value)
            except OSError as exc:
                self.get_logger().warning("Failed to parse cache metadata %s: %s", meta_path, exc)
        return CachedAudio(data, sample_rate, channels, bit_depth, rms, peak)

    def write_cache_to_disk(self, cache_key: str, cached: CachedAudio) -> None:
        path = self.cache_file_path(cache_key)
        if not path:
            return
        try:
            path.write_bytes(cached.audio_bytes)
            meta = "\n".join([
                f"sample_rate:{cached.sample_rate}",
                f"channels:{cached.channels}",
                f"bit_depth:{cached.bit_depth}",
                f"rms:{cached.rms}",
                f"peak:{cached.peak}",
            ])
            path.with_suffix(".meta").write_text(meta)
        except OSError as exc:
            self.get_logger().warning("Failed to write cache file %s: %s", path, exc)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FvTtsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # SIGINT 時は rclpy が既に shutdown 済み (二重呼びで exit code 1 になっていた)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
