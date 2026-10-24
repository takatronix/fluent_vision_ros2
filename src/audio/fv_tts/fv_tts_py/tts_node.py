#!/usr/bin/env python3
"""fv_tts: 選んだエンジンだけで喋る読み上げノード。

エンジンは viewer の設定 (/aspa/tts/settings の tts_engine) で選ぶ:
  aivis       AivisSpeech (機体内 :10101)。文ごとに合成して、できた文から順に流す
  pyopenjtalk Open JTalk (軽量)。選んだ時だけ使う
  openai      会話の返事だけ OpenAI (aspa_agent 側)。アナウンス (このノード) は aivis で喋る

選んだエンジンが失敗した時に別の声で代わりに喋ることはしない (2026-10-24 オーナー:
「設定で aivis, openjtalk, openai 選べればいいだけ」「勝手に切り替えない」)。失敗した
発話は飛ばして「読み上げできませんでした（<engine>: 理由）」とログに残す。
"""
import concurrent.futures
import hashlib
import importlib
import json
import logging
import os
import queue
import re
import sys
import threading
import time
from array import array
from pathlib import Path
from typing import Dict, List, Optional

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
# viewer の設定値のうち、このノードが AivisSpeech で喋るもの (openai は会話の返事だけ OpenAI。
# アナウンスは Aivis)。voicevox は旧 viewer の保存値。
AIVIS_SETTING_VALUES = ("aivis", "openai", "voicevox")
AIVIS_FEMALE = 135718976   # 楽町音穏_T3モデル ノーマル (女)
AIVIS_MALE = 1310138977    # 阿井田 茂 Calm (男)
# 「少し速め」(オーナー 2026-10-24)。viewer の設定 aivis_speed で変えられる
AIVIS_DEFAULT_SPEED = 1.3
AIVIS_SPEED_RANGE = (0.5, 2.0)
# 温め: 起動直後と、暇が続いた時に短い文を 1 つ合成して、声の読込を保つ (結果は捨てる)
KEEPALIVE_TEXT = "はい。"

# 文の切れ目 (。！？!? と改行)。AivisSpeech は文ごとに合成して、できた順に流す
_SENTENCE_SPLIT = re.compile(r"(?<=[。！？!?\n])")
_CLAUSE_SPLIT = re.compile(r"(?<=[、,])")
_SENTENCE_MIN = 6      # これより短い切れ端は前の文につなぐ
_SENTENCE_MAX = 120    # これより長い文は読点で分ける (合成の待ち時間を揃える)


def split_sentences(text: str, minimum: int = _SENTENCE_MIN, maximum: int = _SENTENCE_MAX) -> List[str]:
    """読み上げる文の列。空は []。1 文なら [text]。"""
    parts = [p.strip() for p in _SENTENCE_SPLIT.split(text or "") if p and p.strip()]
    out: List[str] = []
    for part in parts:
        # 短い切れ端 (「はい。」「終わり。」) は前の文につなぐ。前が短くても同じ
        if out and (len(part) < minimum or len(out[-1]) < minimum) and len(out[-1]) + len(part) <= maximum:
            out[-1] = out[-1] + part
            continue
        if len(part) > maximum:
            # 長い 1 文は読点で分けて、maximum を超えない塊にまとめる
            chunk = ""
            for clause in (c for c in _CLAUSE_SPLIT.split(part) if c):
                if chunk and len(chunk) + len(clause) > maximum:
                    out.append(chunk.strip())
                    chunk = clause
                else:
                    chunk += clause
            if chunk.strip():
                out.append(chunk.strip())
            continue
        out.append(part)
    return out


class SpeakFailed(RuntimeError):
    """選んだエンジンで喋れなかった (代わりの声は出さない)。"""

    def __init__(self, engine: str, reason: str, *, engine_down: bool = False) -> None:
        super().__init__(f"読み上げできませんでした（{engine}: {reason}）")
        self.engine = engine
        self.reason = reason
        self.engine_down = engine_down


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
        # 起動時のエンジン: "pyopenjtalk"(軽量) | "aivis" (AivisSpeech)。viewer の設定
        # (/aspa/tts/settings の tts_engine) が届けばそちらに従う。
        self.declare_parameter("engine", "pyopenjtalk")
        # AivisSpeech Engine の URL。"" = 未設定 (engine=aivis なら既定 :10101)。
        # engine=pyopenjtalk でも設定されていれば、viewer の設定 (tts_engine=aivis)
        # で再起動なしに AivisSpeech へ切り替えられるよう準備しておく。
        self.declare_parameter("aivis_url", "")
        self.declare_parameter("aivis_speaker", AIVIS_MALE)  # 男 (既定)。女は 135718976
        # 話す速さ (AivisSpeech の speedScale)。viewer の設定 aivis_speed で変わる
        self.declare_parameter("aivis_speed", AIVIS_DEFAULT_SPEED)
        # 1 文の合成をこの秒数まで待つ。超えたら、その発話は飛ばす (別の声では喋らない)
        self.declare_parameter("aivis_timeout_s", 15.0)
        # 暇がこの秒数続いたら短い文を 1 つ合成して声の読込を保つ (0 = しない)
        self.declare_parameter("aivis_keepalive_s", 180.0)
        # 出力パイプライン (fv_audio_output) のサンプルレート。AivisSpeech は 44.1kHz を
        # 返すので、このレート(既定 48kHz)へリサンプルしてから publish する。
        self.declare_parameter("output_sample_rate", 48000)
        # Event Bus 契約: 任意テキストを喋らせる topic I/F と再生中フラグ
        self.declare_parameter("say_topic", "/aspa/tts/say")
        self.declare_parameter("speaking_topic", "/aspa/audio/speaking")
        self.declare_parameter("settings_topic", "/aspa/tts/settings")
        # 実際に喋った (または飛ばした) 発話の知らせ: JSON {text, engine, ok, clips, ms, error}
        self.declare_parameter("spoken_topic", "/aspa/tts/spoken")

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
        self.spoken_topic = str(self.get_parameter("spoken_topic").value or "")

        # AivisSpeech の声 + 声キャッシュ: 保存済みの文はその WAV、無ければその場で合成して保存。
        self.declare_parameter("aivis_query", "{}")  # audio_query 上書き JSON (speedScale は aivis_speed が入る)
        self.declare_parameter("voice_cache_dir", "~/.aspa/tts_cache/voice")
        self.declare_parameter("voice_cache_max_mb", 300)
        # これより長い文は保存しない (その場で合成はする)
        self.declare_parameter("voice_cache_max_chars", 200)
        self.declare_parameter("voice_cache_timeout_s", 60.0)
        self.declare_parameter("voice_cache_warmup_file", "")
        self.declare_parameter("voice_cache_warmup_delay_s", 90.0)
        self.declare_parameter("voice_cache_max_load", 0.0)  # 1分 loadavg がこれ以上ならウォームアップ待ち (0=無効)
        self.declare_parameter("voice_cache_nice", 10)
        # 高品質の声のゲイン。AivisSpeech は素の 0dB で Open JTalk +8dB とほぼ同じ音量。
        self.declare_parameter("voice_cache_volume_db", 0.0)
        # True なら文ごとに Open JTalk 版の発話 RMS へ合わせる (旧挙動)。
        self.declare_parameter("voice_cache_match_loudness", False)
        # エンジン不通を検知したら、この秒数は試さずに飛ばす (毎回タイムアウトまで待たせない)
        self.declare_parameter("voice_cache_down_skip_s", 10.0)
        # 先に出した音がこの秒数ぶん残っている間は次の文を出さない (fv_audio_output の待ち行列は 8 枠)
        self.declare_parameter("playback_backlog_s", 8.0)

        self.get_logger().info(f"Starting FV TTS node (engine={self.engine})")

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
        self.spoken_pub = self.create_publisher(String, self.spoken_topic, 10) if self.spoken_topic else None
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
        # 出す側の共通ロック (再生中フラグ・出した音の終わる時刻・停止の世代。say スレッドとサービスが重なる)
        self._speaking_lock = threading.RLock()
        # 出した音の終わる時刻 (文ごとに流す時の詰まり防止)
        self._published_ends: List[float] = []
        self._closing = False
        self.playback_backlog_s = float(self.get_parameter("playback_backlog_s").value)

        self.cache: Dict[str, CachedAudio] = {}

        self.voice_store: Optional[vc.VoiceCacheStore] = None
        self.voice_worker: Optional[vc.RenderWorker] = None
        self._warmup_timer: Optional[threading.Timer] = None
        self._keepalive_timer: Optional[threading.Timer] = None
        self._warmup_path: Optional[Path] = None
        self._warmup_due = False
        # 選んだエンジン (aivis | pyopenjtalk)。起動時は engine パラメータ、viewer の設定
        # (tts_engine=aivis|openai → aivis, pyopenjtalk) で再起動なしに切り替える。AivisSpeech の
        # 準備 (voice_store / _vc_pool) ができていなくても選択は保つ: その間の発話は失敗として飛ばす
        # (Open JTalk では喋らない)。
        self.engine_selected = self.engine
        self._vc_pool: Optional[concurrent.futures.ThreadPoolExecutor] = None
        self._vc_inflight: set = set()
        # いま合成中の文 (key → Future)。同じ文の要求は同じ合成を待つ (二重に投入しない)
        self._vc_futures: Dict[str, concurrent.futures.Future] = {}
        self._vc_lock = threading.Lock()
        # 声の設定 (話者・速さ・query) の読み書き: 写しを取る間に設定が変わらないように
        self._cfg_lock = threading.Lock()
        self._vc_skip_until = 0.0
        self._vc_skip_reason = ""
        self._aivis_last_use = 0.0
        self.vc_speed = AIVIS_DEFAULT_SPEED
        try:
            self._setup_voice_cache()
        except Exception as exc:  # pylint: disable=broad-except
            # AivisSpeech の準備に失敗: aivis が選ばれている間の発話は失敗として飛ばす (代わりの声は出さない)
            self.get_logger().error(f"AivisSpeech の準備に失敗: {exc}")
            self.voice_store = None
            self.voice_worker = None

        # Open JTalk は選ばれた時だけ使う。無い機体でも起動はする (選ばれた時に失敗としてログに出る)。
        if self.engine == "pyopenjtalk" and load_pyopenjtalk() is None:
            self.get_logger().error(f"pyopenjtalk が使えません: {_IMPORT_ERROR}")

        # /aspa/tts/say は 1 本の作業スレッドで順に喋る (合成待ちで ROS のコールバックを止めない)
        self._say_queue: "queue.Queue" = queue.Queue()
        self._say_thread = threading.Thread(target=self._say_loop, name="fv_tts_say", daemon=True)
        self._say_thread.start()

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
        self.vc_speed = self._clamp_speed(self.get_parameter("aivis_speed").value, AIVIS_DEFAULT_SPEED)
        self.vc_query["speedScale"] = self.vc_speed
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
        self.vc_timeout = float(self.get_parameter("aivis_timeout_s").value)
        self.vc_volume_db = float(self.get_parameter("voice_cache_volume_db").value)
        self.vc_match_loudness = bool(self.get_parameter("voice_cache_match_loudness").value)
        self.vc_down_skip = float(self.get_parameter("voice_cache_down_skip_s").value)
        self.vc_keepalive_s = float(self.get_parameter("aivis_keepalive_s").value)
        self.voice_worker = vc.RenderWorker(
            self.voice_store, self._render_voice,
            meta={"engine": kind},
            log=lambda m: self.get_logger().info(m),
            busy=busy, nice=int(self.get_parameter("voice_cache_nice").value))
        self.voice_worker.start()
        self._vc_pool = concurrent.futures.ThreadPoolExecutor(
            max_workers=2, thread_name_prefix="voice_live")
        self.get_logger().info(
            f"AivisSpeech {'ON' if self.vc_active else '準備のみ (viewer の tts_engine=aivis で ON)'}: {url} "
            f"timeout={self.vc_timeout:.0f}s speaker={self.vc_speaker} speed={self.vc_speed:.2f} "
            f"dir={self.voice_store.dir} "
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
        # 起動直後の温め (最初の 1 文目から速く) と、暇が続いた時の温め
        if self.vc_active:
            self._keepalive_now("起動時")
        self._arm_keepalive()

    @property
    def vc_active(self) -> bool:
        """AivisSpeech で喋る: 選ばれていて、準備もできている。"""
        return (self.engine_selected == "aivis" and self.voice_store is not None
                and self._vc_pool is not None)

    def _aivis_snapshot(self) -> Dict:
        """いまの声の設定 (話者・速さ・query) の写し。キー作成・合成・保存まで同じ値を使う (設定の更新と排他)。"""
        with self._cfg_lock:
            return {"speaker": int(self.vc_speaker), "speed": float(self.vc_speed), "query": dict(self.vc_query)}

    @staticmethod
    def _clamp_speed(value, default: float) -> float:
        try:
            speed = float(value)
        except (TypeError, ValueError):
            return float(default)
        if not np.isfinite(speed):
            return float(default)
        return round(min(AIVIS_SPEED_RANGE[1], max(AIVIS_SPEED_RANGE[0], speed)), 2)

    def _voice_key(self, text: str, volume_db: float, speaker: Optional[int] = None,
                   query: Optional[Dict] = None) -> str:
        spk = self.vc_speaker if speaker is None else int(speaker)
        return vc.make_key(self.vc_kind, spk, self.vc_query if query is None else query, volume_db, text)

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
        job = self._aivis_snapshot()
        speaker = job["speaker"]
        n = 0
        for text in texts:
            for sentence in split_sentences(text):
                if self.voice_worker and len(sentence) <= self.vc_max_chars:
                    n += self.voice_worker.submit(
                        self._voice_key(sentence, self.default_volume_db, speaker, job["query"]), sentence, None,
                        priority=False, job=dict(job))
        self.get_logger().info(
            f"voice cache warmup: {len(texts)} 文中 {n} 文を裏で作成予定 "
            f"(speaker={speaker}, {path.name})")

    # ── 温め (keepalive) ───────────────────────────────
    def _arm_keepalive(self) -> None:
        period = float(getattr(self, "vc_keepalive_s", 0.0) or 0.0)
        if period <= 0:
            return
        with self._speaking_lock:          # destroy_node の cancel と同じロック (終了後に登録し直さない)
            if self._closing:
                return
            if self._keepalive_timer is not None:
                self._keepalive_timer.cancel()
            self._keepalive_timer = threading.Timer(period, self._keepalive_tick)
            self._keepalive_timer.daemon = True
            self._keepalive_timer.start()

    def _keepalive_tick(self) -> None:
        try:
            period = float(getattr(self, "vc_keepalive_s", 0.0) or 0.0)
            idle = time.monotonic() - self._aivis_last_use
            if self.vc_active and idle >= period:
                self._keepalive_now("暇が続いた")
        finally:
            self._arm_keepalive()

    def _keepalive_now(self, why: str) -> None:
        """短い文を 1 つ裏で合成して捨てる (声の読込を保つ)。失敗しても何もしない。"""
        pool = self._vc_pool
        if pool is None or self._closing:
            return
        job = self._aivis_snapshot()

        def run() -> None:
            t0 = time.monotonic()
            try:
                self._render_voice("", KEEPALIVE_TEXT, None, job)
            except Exception as exc:  # pylint: disable=broad-except
                self.get_logger().warn(f"AivisSpeech の温め ({why}) に失敗: {exc}")
                return
            self._aivis_last_use = time.monotonic()
            self.get_logger().info(
                f"AivisSpeech の温め ({why}): {time.monotonic() - t0:.2f}s (speaker={job['speaker']} speed={job['speed']})")

        try:
            pool.submit(run)
        except RuntimeError:  # shutdown 済み
            pass

    # ── 合成 ───────────────────────────────────────────
    def _render_voice(self, _key: str, text: str, ref_rms: Optional[float],
                      job: Optional[Dict] = None):
        """高品質エンジンで合成→出力レートへ→音量 (固定ゲイン or Open JTalk 版に合わせる)。"""
        speaker = int((job or {}).get("speaker", self.vc_speaker))
        query = (job or {}).get("query")
        if not isinstance(query, dict):
            query = self.vc_query
        try:
            pcm, sr, channels, bit_depth = self.vc_backend.synthesize(text, speaker, query)
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

    def _vc_put(self, key: str, text: str, job: Dict, pcm: bytes, sr: int, render_s: float) -> None:
        try:
            self.voice_store.put(key, pcm, sr, {
                "engine": self.vc_kind, "speaker": int(job["speaker"]), "speed": float(job["speed"]),
                "text": vc.normalize_text(text), "render_s": round(render_s, 2)})
        except OSError as exc:
            self.get_logger().warn(f"voice cache: 保存失敗 {exc}")

    def _vc_discard(self, key: Optional[str]) -> None:
        if key is None:
            return
        with self._vc_lock:
            self._vc_inflight.discard(key)
            self._vc_futures.pop(key, None)

    def _vc_late_done(self, key: str, text: str, job: Dict, t0: float, fut) -> None:
        """合成が終わった (待っていた側が受け取る・タイムアウトで飛ばした後でも): 次回のために保存する。

        保存が終わるまで処理中の印 (_vc_futures) は残す: その間に同じ文が来てもキャッシュか Future のどちらかで見つかる。
        """
        try:
            if self._closing or fut.cancelled():
                return
            exc = fut.exception()
            if exc is None:
                pcm, sr = fut.result()
                self._vc_put(key, text, job, pcm, sr, time.monotonic() - t0)
                self._aivis_last_use = time.monotonic()
                self.get_logger().info(f"voice cache: 遅れて保存 '{text[:30]}'")
                return
            if isinstance(exc, vc.EngineDown):
                self._vc_skip_until = time.monotonic() + self.vc_down_skip
                self._vc_skip_reason = str(exc)
        finally:
            self._vc_discard(key)

    def aivis_sentence(self, text: str, volume_db: float) -> CachedAudio:
        """1 文を AivisSpeech で: 1) 声キャッシュ 2) その場で合成 (timeout まで待つ)。失敗は SpeakFailed。

        同じ文が同時に来たら (2 枚のページ・遅れて保存の途中) 同じ合成を待つ (二重に投入しない)。
        """
        if self.voice_store is None or self._vc_pool is None:
            raise SpeakFailed("aivis", "AivisSpeech の準備ができていません (aivis_url / 声キャッシュ)")
        job = self._aivis_snapshot()
        speaker = job["speaker"]
        cacheable = len(vc.normalize_text(text)) <= self.vc_max_chars
        key = self._voice_key(text, volume_db, speaker, job["query"]) if cacheable else None
        if key is not None:
            try:
                hit = self.voice_store.get(key)
            except Exception as exc:  # pylint: disable=broad-except
                self.get_logger().warn(f"voice cache read failed: {exc}")
                hit = None
            if hit is not None:
                self.get_logger().info(f"voice: キャッシュの声 ({self.vc_kind}:{speaker})")
                return self._to_cached_audio(*hit)
        now = time.monotonic()
        if now < self._vc_skip_until:
            raise SpeakFailed("aivis", f"{self._vc_skip_until - now:.0f} 秒前まで不通 ({self._vc_skip_reason})",
                              engine_down=True)
        timeout = float(getattr(self, "vc_timeout", 15.0))
        # 同じ文の合成は 1 つにまとめる (保存しない長文も)。保存は cacheable の時だけ
        dedupe_key = key if key is not None else self._voice_key(text, volume_db, speaker, job["query"])
        shared = False
        with self._vc_lock:
            fut = self._vc_futures.get(dedupe_key)
            if fut is None and key is not None:
                # 直前に別の要求が保存を終えたかもしれない (印を外すのは保存の後なので、ここでもう一度見れば空白が無い)
                try:
                    hit = self.voice_store.get(key)
                except Exception:  # pylint: disable=broad-except
                    hit = None
                if hit is not None:
                    self.get_logger().info(f"voice: キャッシュの声 ({self.vc_kind}:{speaker})")
                    return self._to_cached_audio(*hit)
            if fut is None:
                self._vc_inflight.add(dedupe_key)
                fut = self._vc_pool.submit(self._render_voice, dedupe_key, text, None, job)
                self._vc_futures[dedupe_key] = fut
                own = True
            else:
                shared = True
                own = False
        if own:
            # ロックの外で登録する (完了済みならその場で呼ばれ、_vc_discard が同じロックを取る)
            if key is not None:
                fut.add_done_callback(lambda f, k=dedupe_key, t=text, j=job, t0=now: self._vc_late_done(k, t, j, t0, f))
            else:
                fut.add_done_callback(lambda f, k=dedupe_key: self._vc_discard(k))
        try:
            pcm, sr = self._wait_future(fut, timeout)
        except concurrent.futures.TimeoutError:
            # 合成はそのまま続け、できたら保存 (次回はキャッシュから)。しばらくは待たずに飛ばす
            self._vc_skip_until = time.monotonic() + min(5.0, self.vc_down_skip)
            self._vc_skip_reason = f"{timeout:.0f} 秒で合成が終わらない"
            raise SpeakFailed("aivis", self._vc_skip_reason, engine_down=True)
        except vc.EngineDown as exc:
            self._vc_skip_until = time.monotonic() + self.vc_down_skip
            self._vc_skip_reason = str(exc)
            raise SpeakFailed("aivis", f"不通 {exc}", engine_down=True) from exc
        except Exception as exc:  # pylint: disable=broad-except
            raise SpeakFailed("aivis", str(exc)) from exc
        dt = time.monotonic() - now
        self._aivis_last_use = time.monotonic()
        if key is None:
            self.get_logger().info(f"voice: その場で高品質合成 {dt:.2f}s (speaker={speaker}, 長文なので保存しない)")
        elif shared:
            self.get_logger().info(f"voice: 合成中だった声を待った {dt:.2f}s (speaker={speaker})")
        else:
            self.get_logger().info(f"voice: その場で高品質合成 {dt:.2f}s (speaker={speaker})")
        return self._to_cached_audio(pcm, sr)

    def _wait_future(self, fut, timeout: float):
        """fut.result(timeout) を短い刻みで (終了処理が来たら待ちをやめる)。"""
        deadline = time.monotonic() + timeout
        while True:
            if self._closing:
                raise SpeakFailed("aivis", "終了中")
            left = deadline - time.monotonic()
            if left <= 0:
                raise concurrent.futures.TimeoutError()
            try:
                return fut.result(timeout=min(0.25, left))
            except concurrent.futures.TimeoutError:
                continue

    def active_engine(self) -> str:
        return self.engine_selected

    def destroy_node(self):
        # 受付をやめ、喋りかけを止め、作業スレッドを終えてから ROS のものを壊す
        queue_ = getattr(self, "_say_queue", None)
        with self._speaking_lock:
            self._closing = True
            self.playback_epoch += 1
            for timer in (self._warmup_timer, self._keepalive_timer, self._speaking_timer):
                if timer is not None:
                    timer.cancel()
            self._warmup_timer = self._keepalive_timer = self._speaking_timer = None
            self._speaking = False
            if queue_ is not None:
                # 残っている依頼は捨てて、番兵で作業スレッドを終える (on_say と同じロックの中なので、後から入らない)
                try:
                    while True:
                        queue_.get_nowait()
                except queue.Empty:
                    pass
                queue_.put(None)
        if queue_ is not None:
            thread = getattr(self, "_say_thread", None)
            if thread is not None and thread.is_alive() and thread is not threading.current_thread():
                # Open JTalk の同期合成 (1〜2 秒) は途中で止められないので、終わるまで待つ (上限 15 秒)。
                # それでも残った時は、出す側 (_publish / _report_spoken / begin_speaking) が _closing を見て
                # 壊された publisher に触らない
                thread.join(timeout=15.0)
                if thread.is_alive():
                    self.get_logger().warn("fv_tts の作業スレッドが 15 秒で終わりませんでした (出力は止めてあります)")
        if self.voice_worker is not None:
            self.voice_worker.stop()
        if self._vc_pool is not None:
            self._vc_pool.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()

    # ── 設定 (viewer) ──────────────────────────────────
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
        """viewer の設定: tts_engine (aivis|openai → AivisSpeech、pyopenjtalk → Open JTalk)、
        aivis_speaker で男/女、aivis_speed で速さ。選択は AivisSpeech の準備ができていなくても保つ。"""
        engine = str(obj.get("tts_engine", "") or "").strip().lower()
        switched = False
        if engine:
            selected = "aivis" if engine in AIVIS_SETTING_VALUES else "pyopenjtalk"
            if selected != self.engine_selected:
                self.engine_selected = selected
                switched = True
                self.get_logger().info(
                    f"読み上げエンジン: {'AivisSpeech' if selected == 'aivis' else 'Open JTalk'} (設定 {engine})")
            if selected == "aivis" and self.voice_store is None:
                self.get_logger().error(
                    "設定は AivisSpeech ですが、このノードは AivisSpeech の準備ができていません "
                    "(aivis_url 未設定か準備に失敗)。アナウンスは読み上げできません (別の声では喋りません)")
        if self.voice_store is None:
            return
        changed = False
        with self._cfg_lock:
            if "aivis_speed" in obj:
                speed = self._clamp_speed(obj.get("aivis_speed"), self.vc_speed)
                if abs(speed - self.vc_speed) > 1e-6:
                    self.get_logger().info(f"速さの切替: {self.vc_speed:.2f} -> {speed:.2f}")
                    self.vc_speed = speed
                    self.vc_query["speedScale"] = speed
                    changed = True
            if "aivis_speaker" in obj:
                try:
                    speaker = int(obj["aivis_speaker"])
                    if speaker < 0:
                        raise ValueError("aivis_speaker must be >= 0")
                except (TypeError, ValueError) as exc:
                    self.get_logger().warn(f"aivis_speaker parse failed: {exc}")
                    speaker = self.vc_speaker
                if speaker != self.vc_speaker:
                    self.get_logger().info(
                        f"声の切替: speaker {self.vc_speaker} -> {speaker} ({obj.get('voice', '')})")
                    self.vc_speaker = speaker
                    changed = True
        if (switched or changed) and self.vc_active:
            self._vc_skip_until = 0.0
            # 設定を全部映してから、新しい声・速さで温める (最初の 1 文目から速く)
            self._keepalive_now("切り替え")
            # 起動直後の待ち時間が過ぎていれば、新しい声・速さの決まり文句を裏で作り直す
            if changed and self._warmup_due and self._warmup_path is not None:
                self._enqueue_warmup(self._warmup_path)

    def on_stop(self, _msg: Empty) -> None:
        """音声停止を受けたら再生世代を進める (喋りかけの残りの文も出さない)。"""
        with self._speaking_lock:
            self.playback_epoch += 1
            self._published_ends = []

    # ── 発話 ───────────────────────────────────────────
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
        play = bool(request.play) and self.publish_playback
        if request.play and not self.publish_playback:
            self.get_logger().warn(f"TTS NOT published to playback: play={request.play}, publish_playback={self.publish_playback}")

        clips = self.speak_text(text, voice_id, volume_db, stamp=request_stamp, epoch=request_epoch,
                                play=play, cache_key=request.cache_key or "")
        if not clips:
            response.success = False
            response.message = f"読み上げできませんでした（{self.active_engine()}）"
            return response
        response.frame = self.build_frame(self._concat(clips), stamp=request_stamp, epoch=request_epoch)
        response.success = True
        response.message = "ok"
        response.played = play
        return response

    def on_say(self, msg: String) -> None:
        """Event Bus: 任意テキストを喋る (/aspa/tts/say)。常に再生する。順番に 1 本のスレッドで。"""
        text = (msg.data or "").strip()
        if not text:
            return
        with self._speaking_lock:            # 終了処理 (キューを捨てて番兵を入れる) と排他
            if self._closing:
                return
            self.get_logger().info(f"say topic: text={text[:50]}")
            self._say_queue.put((text, self.get_clock().now().to_msg(), self.playback_epoch))

    def _say_loop(self) -> None:
        while True:
            item = self._say_queue.get()
            if item is None:
                return
            text, stamp, epoch = item
            try:
                self.speak_text(text, self.default_voice, self.default_volume_db,
                                stamp=stamp, epoch=epoch, play=self.publish_playback)
            except Exception as exc:  # pylint: disable=broad-except
                self.get_logger().error(f"say TTS failed: {exc}")

    def speak_text(self, text: str, voice_id: str, volume_db: float, *, stamp=None,
                   epoch: Optional[int] = None, play: bool = True, cache_key: str = "") -> List[CachedAudio]:
        """選んだエンジンで text を喋る。AivisSpeech は文ごとに合成し、できた文から順に出す。

        失敗した発話は飛ばして「読み上げできませんでした（<engine>: 理由）」をログに残す
        (別の声では喋らない)。出した音の列を返す (何も出せなければ [])。喋った / 飛ばした結果は
        spoken_topic (JSON) にも知らせる (viewer → ブラウザ・agent が「どの声か」を記録する)。
        """
        epoch = self.playback_epoch if epoch is None else int(epoch)
        out: List[CachedAudio] = []
        t_start = time.monotonic()
        engine = self.engine_selected
        error = ""
        if self._closing:
            return out
        if engine != "aivis":
            cache_key = cache_key or self.make_cache_key(text, voice_id, volume_db)
            try:
                cached = self.get_cached(text, voice_id, volume_db, cache_key)
            except Exception as exc:  # pylint: disable=broad-except
                failure = SpeakFailed("pyopenjtalk", str(exc))
                self.get_logger().error(str(failure))
                self._report_spoken(text, engine, out, t_start, error=failure.reason)
                return out
            if self._publish(cached, stamp=stamp, epoch=epoch, play=play):
                out.append(cached)
            self._report_spoken(text, engine, out, t_start)
            return out
        if voice_id:
            self.get_logger().debug(f"aivis: voice_id='{voice_id}' は使わない (viewer の男/女の設定)")
        sentences = split_sentences(text) or [text]
        for index, sentence in enumerate(sentences):
            if epoch != self.playback_epoch or self._closing:
                self.get_logger().info(f"停止されたので残り {len(sentences) - index} 文は出さない")
                break
            try:
                cached = self.aivis_sentence(sentence, volume_db)
            except SpeakFailed as exc:
                left = len(sentences) - index
                self.get_logger().error(
                    f"{exc} 「{sentence[:30]}」" + (f" (残り {left - 1} 文も飛ばす)" if exc.engine_down and left > 1 else ""))
                error = error or exc.reason
                if exc.engine_down:
                    break
                continue
            if not self._publish(cached, stamp=stamp, epoch=epoch, play=play):
                break
            out.append(cached)
        self._report_spoken(text, engine, out, t_start, error=error)
        return out

    def _report_spoken(self, text: str, engine: str, clips: List[CachedAudio], t_start: float,
                       error: str = "") -> None:
        pub = getattr(self, "spoken_pub", None)
        if pub is None:
            return
        seconds = 0.0
        for c in clips:
            bps = c.channels * (c.bit_depth // 8)
            seconds += len(c.audio_bytes) / bps / c.sample_rate if bps and c.sample_rate else 0.0
        payload = json.dumps({
            "text": text, "engine": engine, "ok": bool(clips), "clips": len(clips),
            "seconds": round(seconds, 2), "ms": int((time.monotonic() - t_start) * 1000),
            "error": error}, ensure_ascii=False)
        with self._speaking_lock:          # 終了処理の後は (publisher が壊された後なので) 出さない
            if self._closing:
                return
            try:
                pub.publish(String(data=payload))
            except Exception as exc:  # pylint: disable=broad-except
                self.get_logger().debug(f"spoken publish failed: {exc}")

    def _publish(self, cached: CachedAudio, *, stamp, epoch: int, play: bool) -> bool:
        """出す。先に出した音が playback_backlog_s 秒ぶん残っていれば減るまで待つ (fv_audio_output の
        待ち行列あふれ防止)。空きの確認・予約・出すこと・停止の世代の確認は 1 つのロックの中で。
        停止されていれば出さず False。"""
        limit = float(getattr(self, "playback_backlog_s", 0.0) or 0.0)
        while True:
            with self._speaking_lock:
                if self._closing or epoch != self.playback_epoch:
                    return False
                now = time.monotonic()
                self._published_ends = [t for t in self._published_ends if t > now]
                backlog = (max(self._published_ends) - now) if self._published_ends else 0.0
                if limit <= 0 or (backlog <= limit and len(self._published_ends) < 6):
                    frame = self.build_frame(cached, stamp=stamp, epoch=epoch)
                    self.tts_pub.publish(frame)
                    if play:
                        self.play_pub.publish(frame)
                        self.begin_speaking(cached)
                    return True
            time.sleep(min(0.25, max(0.05, backlog - limit)))

    @staticmethod
    def _concat(clips: List[CachedAudio]) -> CachedAudio:
        if len(clips) == 1:
            return clips[0]
        pcm = b"".join(c.audio_bytes for c in clips)
        samples = vc.pcm16_to_float(pcm)
        rms = float(np.sqrt(np.mean(np.square(samples)))) if samples.size else 0.0
        peak = float(np.max(np.abs(samples))) if samples.size else 0.0
        return CachedAudio(pcm, clips[0].sample_rate, clips[0].channels, clips[0].bit_depth, rms, peak)

    def get_cached(self, text: str, voice_id: str, volume_db: float, cache_key: str) -> CachedAudio:
        """Open JTalk: キャッシュ参照→無ければ合成して格納。合成失敗は例外送出 (代わりの声は無い)。"""
        cached = self.cache.get(cache_key)
        if cached is None and self.cache_dir:
            cached = self.load_cache_from_disk(cache_key)
        if cached is not None:
            self.get_logger().info("TTS cache hit (pyopenjtalk)")
            return cached
        self.get_logger().info("TTS cache miss, synthesizing (pyopenjtalk)...")
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
            if self._closing:
                return
            if not self._speaking:
                self._speaking = True
                self.speaking_pub.publish(Bool(data=True))
            # tail 0.15s: 再生完了とフラグ降下のずれを吸収
            now = time.monotonic()
            start = max(now, max(self._published_ends) if self._published_ends else now)
            end = max(now + duration, start + duration) + 0.15
            self._published_ends = [t for t in self._published_ends if t > now] + [end]
            self._speaking_end = max(self._speaking_end, end)
            if self._speaking_timer is not None:
                self._speaking_timer.cancel()
            delay = max(0.05, self._speaking_end - time.monotonic())
            self._speaking_timer = threading.Timer(delay, self._end_speaking)
            self._speaking_timer.daemon = True
            self._speaking_timer.start()

    def _end_speaking(self) -> None:
        with self._speaking_lock:
            if self._closing:
                return
            # 後続発話でタイマが更新された場合はまだ喋っているので降ろさない
            if time.monotonic() + 0.02 < self._speaking_end:
                return
            self._speaking = False
            self._speaking_timer = None
            self.speaking_pub.publish(Bool(data=False))

    def synthesize(self, text: str, voice_id: str, volume_db: float) -> CachedAudio:
        """Open JTalk で合成。CachedAudio(16bit mono)を返す (AivisSpeech は aivis_sentence 側)。"""
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
