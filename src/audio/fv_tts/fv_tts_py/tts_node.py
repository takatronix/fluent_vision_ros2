#!/usr/bin/env python3
import hashlib
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

from fv_tts_py.voicevox_backend import VoicevoxBackend, VoicevoxError

# Set pyopenjtalk dictionary directory to user's home directory
os.environ.setdefault("OPEN_JTALK_DICT_DIR", str(Path.home() / ".pyopenjtalk"))

# Disable tqdm progress bar for pyopenjtalk downloads
os.environ["TQDM_DISABLE"] = "1"

try:
    # Suppress download progress output during pyopenjtalk import
    _original_stderr = sys.stderr
    sys.stderr = open(os.devnull, 'w')
    import pyopenjtalk
    sys.stderr.close()
    sys.stderr = _original_stderr
except ImportError as exc:  # pragma: no cover - import guard
    sys.stderr = _original_stderr
    pyopenjtalk = None
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None


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
        self.declare_parameter("default_volume_db", 0.0)
        # エンジン差替え: "voicevox"(既定) | "pyopenjtalk"
        self.declare_parameter("engine", "voicevox")
        self.declare_parameter("voicevox_url", "http://127.0.0.1:50021")
        self.declare_parameter("voicevox_speaker", 30)  # No.7 (アナウンス)
        # 出力パイプライン (fv_audio_output) のサンプルレート。voicevox は 24kHz を
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
        self.engine = self.get_parameter("engine").get_parameter_value().string_value.strip().lower()
        self.voicevox_url = self.get_parameter("voicevox_url").get_parameter_value().string_value
        self.voicevox_speaker = self.get_parameter("voicevox_speaker").get_parameter_value().integer_value
        self.output_sample_rate = self.get_parameter("output_sample_rate").get_parameter_value().integer_value
        self.say_topic = self.get_parameter("say_topic").get_parameter_value().string_value
        self.speaking_topic = self.get_parameter("speaking_topic").get_parameter_value().string_value
        self.settings_topic = self.get_parameter("settings_topic").value

        self.get_logger().info(f"Starting FV TTS node (engine={self.engine})")

        # エンジンごとに必要リソースを確認 (fallback せず、駄目なら止める)
        self.voicevox: Optional[VoicevoxBackend] = None
        if self.engine == "pyopenjtalk":
            if pyopenjtalk is None:
                raise RuntimeError(
                    f"engine=pyopenjtalk だが pyopenjtalk が使えません: {_IMPORT_ERROR}"
                ) from _IMPORT_ERROR
        elif self.engine == "voicevox":
            self.voicevox = VoicevoxBackend(self.voicevox_url)
            try:
                ver = self.voicevox.version()
                self.get_logger().info(
                    f"VOICEVOX ENGINE {ver} @ {self.voicevox_url}, speaker={self.voicevox_speaker}")
            except VoicevoxError as exc:
                # 起動時点で不通なら分かりやすく警告 (実合成時に RuntimeError で止まる)
                self.get_logger().warn(
                    f"VOICEVOX ENGINE 未応答 ({self.voicevox_url}): {exc}. "
                    f"`curl {self.voicevox_url}/version` で起動確認してください")
        else:
            raise RuntimeError(
                f"未知の engine='{self.engine}' (voicevox | pyopenjtalk のいずれか)")

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

    def on_settings(self, msg: String) -> None:
        try:
            obj = json.loads(msg.data or "{}")
            speaker = int(obj["voicevox_speaker"])
            if speaker < 0:
                raise ValueError("speaker must be >= 0")
        except KeyError:
            return
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f"TTS settings parse failed: {exc}")
            return
        previous = self.voicevox_speaker
        self.voicevox_speaker = speaker
        if previous != speaker:
            self.get_logger().info(
                f"VOICEVOX speaker changed: {previous} -> {speaker}")

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
            cached = self.get_cached(text, voice_id, volume_db, cache_key)
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
            cached = self.get_cached(text, voice_id, volume_db, cache_key)
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
        """エンジンで分岐して合成。CachedAudio(16bit mono)を返す。"""
        if self.engine == "voicevox":
            return self._synthesize_voicevox(text, voice_id, volume_db)
        return self._synthesize_pyopenjtalk(text, voice_id, volume_db)

    def _synthesize_voicevox(self, text: str, voice_id: str, volume_db: float) -> CachedAudio:
        if self.voicevox is None:
            raise RuntimeError("voicevox backend が初期化されていません")
        # voice_id が数値文字列なら speaker override、それ以外は既定 speaker
        speaker = self.voicevox_speaker
        if voice_id:
            try:
                speaker = int(voice_id)
            except ValueError:
                self.get_logger().warn(
                    f"voicevox: voice_id='{voice_id}' は数値でないため既定 speaker={speaker} を使用")
        try:
            pcm_bytes, native_sr, channels, bit_depth = self.voicevox.synthesize(text, speaker)
        except VoicevoxError as exc:
            raise RuntimeError(f"VOICEVOX 合成失敗: {exc}") from exc
        if channels != 1 or bit_depth != 16:
            raise RuntimeError(
                f"VOICEVOX 想定外フォーマット channels={channels} bit_depth={bit_depth} (mono/16bit 想定)")

        waveform = np.frombuffer(pcm_bytes, dtype="<i2").astype(np.float32) / 32768.0
        if volume_db != 0.0:
            waveform = waveform * float(10.0 ** (volume_db / 20.0))
        # 出力パイプライン (fv_audio_output) は固定レート。合致しなければリサンプルする
        # (でないと format mismatch で黙って drop される)。
        out_sr = int(self.output_sample_rate) if self.output_sample_rate > 0 else native_sr
        if native_sr != out_sr and waveform.size:
            n_out = int(round(waveform.size * out_sr / native_sr))
            src_idx = np.linspace(0.0, waveform.size - 1.0, n_out)
            waveform = np.interp(src_idx, np.arange(waveform.size), waveform).astype(np.float32)
            self.get_logger().debug(
                f"voicevox resample {native_sr}->{out_sr}Hz ({waveform.size} samples)")
        else:
            out_sr = native_sr
        waveform = np.clip(waveform, -1.0, 1.0)
        pcm = (waveform * 32767.0).astype(np.int16)
        audio_bytes = pcm.tobytes()
        float_samples = pcm.astype(np.float32) / 32768.0
        rms = float(np.sqrt(np.mean(np.square(float_samples)))) if float_samples.size else 0.0
        peak = float(np.max(np.abs(float_samples))) if float_samples.size else 0.0
        return CachedAudio(audio_bytes, int(out_sr), 1, 16, rms, peak)

    def _synthesize_pyopenjtalk(self, text: str, voice_id: str, volume_db: float) -> CachedAudio:
        if pyopenjtalk is None:
            raise RuntimeError(f"pyopenjtalk is not available: {_IMPORT_ERROR}")
        options = {}
        if voice_id:
            options["voice"] = voice_id
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

    def cache_voice_id(self, voice_id: str) -> str:
        """Resolve the actual voice so changing the default cannot reuse old audio."""
        if self.engine != "voicevox":
            return voice_id
        if voice_id:
            try:
                return str(int(voice_id))
            except ValueError:
                pass
        return str(self.voicevox_speaker)

    def make_cache_key(self, text: str, voice_id: str, volume_db: float) -> str:
        digest = hashlib.sha1()  # nosec B303
        digest.update(self.engine.encode("utf-8"))
        digest.update(text.encode("utf-8"))
        digest.update(self.cache_voice_id(voice_id).encode("utf-8"))
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
        rclpy.shutdown()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
