#!/usr/bin/env python3
"""fv_soundboard — 状況イベント → サウンド + 任意TTS + 記録マーカー (+ variant / sonification)。

1本のイベントバス (`event_topic`, std_msgs/String) を購読し、登録表 (events.yaml) を
引いて:
  1. 対応する心地よいサウンドを鳴らす (出力シンクへ dispatch)
  2. 必要なら TTS 台詞を `tts_say_topic` へ流す (別ノード fv_tts が発話)
  3. normalize したイベントを `event_out_topic` に再送 (episode_recorder が意味マーカーに使う)

イベントの与え方 (`event_topic` の String data) — Event Bus v1.1:
  - "detect"                      … 登録名。既定動作 (音 + speak設定に従う)
  - '{"event":"detect","say":"アスパラ見つけた","speak":true}'  … JSONで個別上書き
  - '{"event":"detect","variant":"grade_A"}'  … 種別: sounds/<sound>_<variant>.wav が
        あればそれを再生。無ければ base を鳴らし種別は TTS で補う (variants マップ参照)。
  - '{"event":"detect","class":"spear","attrs":{"length_mm":320,"curvature":0.15,"confidence":0.92}}'
        … 連続パラメータ → リアルタイム合成 (sonifier)。class/attrs で音色・ピッチ等が変わる。
  - "say:こんにちは"              … 登録外テキストを即喋らせる (音なし)

出力シンク (`output_backends`, 文字列リスト。複数指定で同時多シンク):
  - "aplay" (既定): ローカル ALSA 直再生 (Thor 単体 / USB スピーカー直結)。
  - "audio_frame": PCM を fv_audio/msg/AudioFrame にして `audio_frame_topic`
        (既定 /audio/output/frame) へ publish。実機ロボットでは Pi 側 fv_audio_output が再生。
        fv_audio_output は format 一致 (既定 48kHz mono 16bit) が必要なので output_sample_rate
        へ揃えてから publish する。
  - 例: ["aplay"] = Thorのみ / ["audio_frame"] = Piのみ / ["aplay","audio_frame"] = 両方同時。
  各シンクは独自スレッド+キューで並行再生する (シンク追加が容易な構造 = 進化成長)。

interrupt=true のイベントは各シンクで再生中を止めて割込む (停止/警告)。
  - aplay: 実行中 aplay を SIGTERM で即停止 (確実な割込)。
  - audio_frame: `audio_stop_topic` へ Empty を publish して fv_audio_output のキューを flush。
        ネットワーク越し + 別トピックのため best-effort (README「AudioFrame経路のinterruptの限界」)。
"""
import json
import os
import queue
import subprocess
import tempfile
import threading
import time
import wave
from array import array

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, String

from fv_soundboard.sonifier import Sonifier


class Cue:
    """再生する1音。wav ファイル path か 生 PCM のどちらかを持つ (シンクが各自 materialize)。"""

    __slots__ = ("path", "pcm", "sample_rate", "channels", "bit_depth",
                 "gain", "interrupt", "duration")

    def __init__(self, path=None, pcm=None, sample_rate=None, channels=1, bit_depth=16,
                 gain=1.0, interrupt=False, duration=0.0):
        self.path = path
        self.pcm = pcm
        self.sample_rate = sample_rate
        self.channels = channels
        self.bit_depth = bit_depth
        self.gain = gain
        self.interrupt = interrupt
        self.duration = duration


class AplaySink:
    """ローカル ALSA (aplay) 再生シンク。独自スレッドで直列再生。"""

    name = "aplay"

    def __init__(self, node, device):
        self.node = node
        self.device = device
        self._q: "queue.Queue[Cue]" = queue.Queue()
        self._cur = None
        self._lock = threading.Lock()
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def pending(self):
        return self._q.qsize()

    def enqueue(self, cue):
        self._q.put(cue)

    def interrupt(self):
        try:
            while True:
                self._q.get_nowait()
        except queue.Empty:
            pass
        with self._lock:
            if self._cur and self._cur.poll() is None:
                self._cur.terminate()

    def _run(self):
        while True:
            cue = self._q.get()
            try:
                self._play(cue)
            except Exception as exc:  # noqa: BLE001
                self.node.get_logger().error(f"[aplay] 再生失敗: {exc}")

    def _play(self, cue: Cue):
        tmp = None
        if cue.path is not None and abs(cue.gain - 1.0) <= 1e-3:
            path = cue.path
        elif cue.path is not None:
            path = tmp = self.node._scaled_copy(cue.path, cue.gain)
        else:
            path = tmp = self.node._pcm_to_temp_wav(
                cue.pcm, cue.sample_rate, cue.channels, cue.bit_depth, cue.gain)
        try:
            proc = subprocess.Popen(
                ["aplay", "-q", "-D", self.device, path],
                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            with self._lock:
                self._cur = proc
            _, err = proc.communicate()
            if proc.returncode not in (0, None) and proc.returncode != -15:  # -15=SIGTERM(interrupt)
                self.node.get_logger().warn(
                    f"[aplay] rc={proc.returncode} dev={self.device}: "
                    f"{(err or b'').decode(errors='ignore').strip()[:120]}")
        finally:
            with self._lock:
                self._cur = None
            if tmp:
                self.node._unlink(tmp)


class AudioFrameSink:
    """fv_audio/msg/AudioFrame publish シンク (Pi側 fv_audio_output が再生)。"""

    name = "audio_frame"

    def __init__(self, node, audio_frame_cls, frame_topic, stop_topic, out_sr):
        self.node = node
        self._AudioFrame = audio_frame_cls
        self.out_sr = int(out_sr)
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        qos.history = HistoryPolicy.KEEP_LAST
        self.pub = node.create_publisher(audio_frame_cls, frame_topic, qos)
        self.stop_pub = node.create_publisher(Empty, stop_topic, 10)
        self._q: "queue.Queue[Cue]" = queue.Queue()
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def pending(self):
        return self._q.qsize()

    def enqueue(self, cue):
        self._q.put(cue)

    def interrupt(self):
        try:
            while True:
                self._q.get_nowait()
        except queue.Empty:
            pass
        # 実機側 fv_audio_output のキューを flush (best-effort)
        self.stop_pub.publish(Empty())

    def _run(self):
        while True:
            cue = self._q.get()
            try:
                self._play(cue)
            except Exception as exc:  # noqa: BLE001
                self.node.get_logger().error(f"[audio_frame] 再生失敗: {exc}")

    def _play(self, cue: Cue):
        if cue.pcm is not None:
            pcm_bytes, sr, ch, bd = cue.pcm, cue.sample_rate, cue.channels, cue.bit_depth
        else:
            pcm_bytes, sr, ch, bd = self.node._read_wav(cue.path)
        if bd != 16:
            raise RuntimeError(f"bit_depth={bd} 非対応 (16bit必須)")
        if ch != 1:
            raise RuntimeError(f"channels={ch} 非対応 (mono必須)")
        samples = np.frombuffer(pcm_bytes, dtype="<i2").astype(np.float32)
        # fv_audio_output は固定レート。合致しなければリサンプル (でないと黙って drop される)
        if sr != self.out_sr and samples.size:
            n_out = int(round(samples.size * self.out_sr / sr))
            idx = np.linspace(0.0, samples.size - 1.0, n_out)
            samples = np.interp(idx, np.arange(samples.size), samples).astype(np.float32)
            self.node.get_logger().debug(f"[audio_frame] resample {sr}->{self.out_sr}Hz")
            sr = self.out_sr
        if abs(cue.gain - 1.0) > 1e-3:
            samples = samples * cue.gain
        pcm = np.clip(samples, -32768.0, 32767.0).astype("<i2")
        audio_bytes = pcm.tobytes()

        frame = self._AudioFrame()
        frame.header.stamp = self.node.get_clock().now().to_msg()
        frame.encoding = "PCM16LE"
        frame.sample_rate = int(sr)
        frame.channels = 1
        frame.bit_depth = 16
        frame.rms = 0.0
        frame.peak = 0.0
        frame.vad = False
        frame.data = array('B', audio_bytes)
        frame.epoch = 0  # 0 = fv_audio_output の epoch フィルタ免除 (stop世代を管理しないため)
        self.pub.publish(frame)


class SoundboardNode(Node):
    def __init__(self):
        super().__init__("fv_soundboard")

        share = get_package_share_directory("fv_soundboard")
        self.declare_parameter("registry_file", os.path.join(share, "config", "events.yaml"))
        self.declare_parameter("sounds_dir", os.path.join(share, "sounds"))
        self.declare_parameter("audio_device", "default")   # USBスピーカーは hw:X,0
        self.declare_parameter("master_volume", 0.8)
        self.declare_parameter("event_topic", "/fv/event")
        self.declare_parameter("event_out_topic", "/fv/event/active")  # recorder等が購読
        self.declare_parameter("preview_topic", "/fv/sound/play")
        self.declare_parameter("settings_topic", "/fv/sound/settings")
        self.declare_parameter("tts_say_topic", "/aspa/tts/say")
        self.declare_parameter("speaking_topic", "/aspa/audio/speaking")
        self.declare_parameter("enable_tts", True)
        # 出力シンク (複数指定で同時多シンク): "aplay" | "audio_frame"
        self.declare_parameter("output_backends", ["aplay"])
        self.declare_parameter("audio_frame_topic", "/audio/output/frame")
        self.declare_parameter("audio_stop_topic", "/audio/output/stop")
        self.declare_parameter("output_sample_rate", 48000)  # fv_audio_output と一致させる
        # sonification (連続パラメータ→合成)
        self.declare_parameter("enable_sonification", True)
        self.declare_parameter(
            "sonification_file", os.path.join(share, "config", "sonification.yaml"))

        gp = lambda k: self.get_parameter(k).value
        self.sounds_dir = gp("sounds_dir")
        self.device = gp("audio_device")
        self.master_volume = float(gp("master_volume"))
        self.enable_tts = bool(gp("enable_tts"))
        self.output_sample_rate = int(gp("output_sample_rate"))
        self.robot_enabled = True
        self.disabled_events = set()

        self.registry = self._load_registry(gp("registry_file"))

        # sonifier
        self.sonifier = None
        if bool(gp("enable_sonification")):
            self.sonifier = self._load_sonifier(gp("sonification_file"))

        # 共通出力
        self.say_pub = self.create_publisher(String, gp("tts_say_topic"), 10)
        self.event_out_pub = self.create_publisher(String, gp("event_out_topic"), 10)
        self.speaking_pub = self.create_publisher(Bool, gp("speaking_topic"), 10)

        # 出力シンク構築
        self.sinks = self._build_sinks(
            list(gp("output_backends")), gp("audio_frame_topic"), gp("audio_stop_topic"))

        self.get_logger().info(
            f"fv_soundboard: {len(self.registry)} events, "
            f"sinks={[s.name for s in self.sinks]}, "
            f"sonify={'on' if self.sonifier else 'off'}, sounds={self.sounds_dir}")

        # 入力
        self.create_subscription(String, gp("event_topic"), self._on_event, 20)
        # プレビューは無効化設定を迂回し、recorder markerへも転送しない。
        self.create_subscription(String, gp("preview_topic"), self._on_preview, 20)
        # Scene Viewerが永続設定をlatched publishする。soundboardを再起動しても
        # 最新値を即受信できる。
        settings_qos = QoSProfile(depth=1)
        settings_qos.reliability = ReliabilityPolicy.RELIABLE
        settings_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        settings_qos.history = HistoryPolicy.KEEP_LAST
        self.create_subscription(
            String, gp("settings_topic"), self._on_settings, settings_qos)

        # /aspa/audio/speaking の再生尺ベース管理 (シンク非依存の best-effort 推定)
        self._speaking = False
        self._speaking_end = 0.0
        self._speaking_timer = None
        self._speaking_lock = threading.Lock()

    # ── setup ───────────────────────────────────────────
    def _build_sinks(self, backends, frame_topic, stop_topic):
        backends = [str(b).strip().lower() for b in (backends or []) if str(b).strip()]
        if not backends:
            raise RuntimeError("output_backends が空です (aplay | audio_frame)")
        sinks = []
        audio_frame_cls = None
        for b in backends:
            if b == "aplay":
                sinks.append(AplaySink(self, self.device))
            elif b == "audio_frame":
                if audio_frame_cls is None:
                    audio_frame_cls = self._import_audio_frame()
                sinks.append(AudioFrameSink(
                    self, audio_frame_cls, frame_topic, stop_topic, self.output_sample_rate))
            else:
                raise RuntimeError(f"未知の output_backend '{b}' (aplay | audio_frame)")
        return sinks

    def _import_audio_frame(self):
        try:
            from fv_audio.msg import AudioFrame
            return AudioFrame
        except ImportError as exc:  # noqa: BLE001
            raise RuntimeError(
                "output_backends=audio_frame には fv_audio が必要です。colcon build 済みか、"
                "ローカル再生なら output_backends:=['aplay'] を指定してください: " + str(exc)) from exc

    def _load_registry(self, path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            return data.get("events", {})
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"registry読込失敗 {path}: {exc}")
            return {}

    def _load_sonifier(self, path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                cfg = yaml.safe_load(f) or {}
            son = Sonifier(cfg, logger=self.get_logger())
            self.get_logger().info(
                f"sonifier: {len(son.class_families)} classes, sr={son.sample_rate}Hz ({path})")
            return son
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"sonification読込失敗 {path}: {exc}")
            return None

    def _sound_path(self, stem):
        if not stem:
            return None
        p = os.path.join(self.sounds_dir, stem + ".wav")
        return p if os.path.exists(p) else None

    # ── event handling ──────────────────────────────────
    def _on_settings(self, msg: String):
        try:
            obj = json.loads(msg.data or "{}")
            if not isinstance(obj, dict):
                raise ValueError("object required")
            enabled = bool(obj.get("robot_enabled", True))
            disabled = {
                str(name) for name in obj.get("disabled_events", [])
                if str(name) in self.registry
            }
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f"settings parse失敗: {exc}")
            return
        self.robot_enabled = enabled
        self.disabled_events = disabled
        self.get_logger().info(
            f"audio settings: robot={'on' if enabled else 'off'}, "
            f"disabled={len(disabled)}")

    def _on_preview(self, msg: String):
        self._handle_event(msg, preview=True)

    def _on_event(self, msg: String):
        self._handle_event(msg, preview=False)

    def _handle_event(self, msg: String, preview=False):
        raw = (msg.data or "").strip()
        if not raw:
            return

        # "say:任意テキスト" → 登録外の即時発話
        if raw.startswith("say:"):
            if preview or self.robot_enabled:
                self._speak(raw[4:].strip())
            return

        # JSON個別指定 or 登録名
        say_override = None
        speak_override = None
        variant = None
        cls = None
        attrs = None
        if raw.startswith("{"):
            try:
                obj = json.loads(raw)
                name = obj.get("event", "")
                say_override = obj.get("say")
                speak_override = obj.get("speak")
                variant = obj.get("variant")
                cls = obj.get("class")
                attrs = obj.get("attrs")
            except Exception:  # noqa: BLE001
                self.get_logger().warn(f"JSON parse失敗: {raw[:80]}")
                return
        else:
            name = raw

        spec = self.registry.get(name) if name else None

        # ── recorder へ normalize したマーカーを再送 ──
        # 登録イベント or 構造化データ (variant/class/attrs) がある時のみ forward。
        # 未登録の bare 名は記録を汚さないよう再送しない (下で warn)。
        # 構造化キーがあれば JSON で forward (エピソードに種別を残す)。
        if (not preview
                and (spec is not None or variant is not None
                     or cls is not None or attrs is not None)):
            marker = (spec.get("record") if spec else None) or name or (cls or "")
            self._forward_marker(marker, variant, cls, attrs)

        # 記録マーカーは音の設定に関係なく残す。プレビューだけはOFF設定を
        # 迂回して、設定中に各音を必ず確認できる。
        if not preview and (
                not self.robot_enabled or name in self.disabled_events):
            return

        # ── sonification (連続パラメータ → 合成) ──
        # class/attrs があり sonifier 有効なら合成音を鳴らす。合成音が状況を伝えるので
        # この時は基本の離散 base wav は抑制する (二重再生回避)。marker/TTS は継続。
        sonified = False
        if self.sonifier is not None and (cls or attrs):
            sonified = self._sonify(cls, attrs)

        # ── 離散サウンド (base / variant wav) ──
        if not sonified and spec is not None:
            self._play_discrete(spec, name, variant, say_override, speak_override)
        elif not sonified and spec is None and not (cls or attrs):
            self.get_logger().warn(f"未登録イベント: {name}")

        # sonify 時でも明示 say/speak があれば喋る
        if sonified and spec is not None:
            say = say_override if say_override is not None else spec.get("say", "")
            speak = speak_override if speak_override is not None else bool(spec.get("speak", False))
            if speak and say:
                self._speak(say)

    def _forward_marker(self, marker, variant, cls, attrs):
        if not marker and not (variant or cls or attrs):
            return
        if variant is not None or cls is not None or attrs is not None:
            fwd = {"event": marker}
            if variant is not None:
                fwd["variant"] = variant
            if cls is not None:
                fwd["class"] = cls
            if attrs is not None:
                fwd["attrs"] = attrs
            self.event_out_pub.publish(String(data=json.dumps(fwd, ensure_ascii=False)))
        else:
            self.event_out_pub.publish(String(data=marker))

    def _play_discrete(self, spec, name, variant, say_override, speak_override):
        stem = spec.get("sound", name)
        interrupt = bool(spec.get("interrupt", False))
        variants_map = spec.get("variants", {}) or {}
        vinfo = variants_map.get(variant, {}) if variant else {}
        variant_say = vinfo.get("say") if isinstance(vinfo, dict) else None

        # サウンド解決順: variantマップのsound → <stem>_<variant> → base <stem>
        path = None
        resolved_variant_audio = False
        if variant:
            if isinstance(vinfo, dict) and vinfo.get("sound"):
                path = self._sound_path(vinfo["sound"])
            if path is None:
                path = self._sound_path(f"{stem}_{variant}")
            if path is not None:
                resolved_variant_audio = True
        if path is None:
            path = self._sound_path(stem)

        if path:
            self._emit(path=path, interrupt=interrupt)
        elif stem:
            self.get_logger().warn(f"音源なし: {stem}.wav (event={name})")

        # TTS
        say = say_override if say_override is not None else spec.get("say", "")
        speak = speak_override if speak_override is not None else bool(spec.get("speak", False))
        # variant音源が無ければ base を鳴らし、種別は TTS で補う (明示ログ)
        if variant and not resolved_variant_audio and variant_say:
            self.get_logger().info(
                f"variant音源なし({stem}_{variant}.wav): baseを再生し種別'{variant}'はTTSで補う")
            if say_override is None:
                say = variant_say
            speak = True
        if speak and say:
            self._speak(say)

    def _sonify(self, cls, attrs):
        """class/attrs から合成音を生成して各シンクへ。発火したら True。"""
        cls = cls or ""
        if not self.sonifier.allow(cls):
            return False  # レート制限 (allow内でdebugログ)
        # キュー洪水防止: 上限超過は黙殺せず debug ログ
        pending = max((s.pending() for s in self.sinks), default=0)
        if pending >= self.sonifier.max_queue:
            self.get_logger().debug(
                f"sonify: queue上限({self.sonifier.max_queue})超過につき '{cls}' を drop")
            return True  # 発火扱い (base wav は鳴らさない)
        try:
            pcm, sr, interrupt = self.sonifier.synth(cls, attrs)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"sonify合成失敗 (class={cls}): {exc}")
            return False
        self._emit(pcm=pcm, sample_rate=sr, channels=1, bit_depth=16, interrupt=interrupt)
        return True

    def _speak(self, text):
        if not text:
            return
        if not self.enable_tts:
            self.get_logger().info(f"(TTS無効) say: {text}")
            return
        self.say_pub.publish(String(data=text))

    # ── emit / dispatch ─────────────────────────────────
    def _emit(self, *, path=None, pcm=None, sample_rate=None, channels=1, bit_depth=16,
              interrupt=False, gain=None):
        """音源 (wavパス or 生PCM) を全シンクへ dispatch (同時多シンク並行再生)。"""
        gain = self.master_volume if gain is None else gain
        # 再生尺 (speaking フラグ推定用)
        if pcm is not None:
            bps = channels * (bit_depth // 8)
            duration = len(pcm) / bps / sample_rate if (bps and sample_rate) else 0.0
        else:
            try:
                with wave.open(path, "rb") as w:
                    duration = w.getnframes() / w.getframerate()
            except Exception:  # noqa: BLE001
                duration = 0.0
        cue = Cue(path=path, pcm=pcm, sample_rate=sample_rate, channels=channels,
                  bit_depth=bit_depth, gain=gain, interrupt=interrupt, duration=duration)
        for sink in self.sinks:
            if interrupt:
                sink.interrupt()
            sink.enqueue(cue)
        if self.sinks:
            self._note_speaking(duration)

    def _note_speaking(self, duration):
        """/aspa/audio/speaking を再生尺ベースで上げ下げ (シンク非依存の best-effort)。"""
        with self._speaking_lock:
            if not self._speaking:
                self._speaking = True
                self.speaking_pub.publish(Bool(data=True))
            end = time.monotonic() + duration + 0.15
            self._speaking_end = max(self._speaking_end, end)
            if self._speaking_timer is not None:
                self._speaking_timer.cancel()
            delay = max(0.05, self._speaking_end - time.monotonic())
            self._speaking_timer = threading.Timer(delay, self._end_speaking)
            self._speaking_timer.daemon = True
            self._speaking_timer.start()

    def _end_speaking(self):
        with self._speaking_lock:
            if time.monotonic() + 0.02 < self._speaking_end:
                return
            self._speaking = False
            self._speaking_timer = None
            self.speaking_pub.publish(Bool(data=False))

    # ── audio helpers (シンクから共有) ──────────────────
    def _read_wav(self, path):
        with wave.open(path, "rb") as w:
            channels = w.getnchannels()
            bit_depth = w.getsampwidth() * 8
            sample_rate = w.getframerate()
            pcm = w.readframes(w.getnframes())
        return pcm, sample_rate, channels, bit_depth

    def _pcm_to_temp_wav(self, pcm, sample_rate, channels, bit_depth, gain=1.0):
        samples = np.frombuffer(pcm, dtype="<i2")
        if abs(gain - 1.0) > 1e-3:
            samples = np.clip(samples.astype(np.float32) * gain, -32768, 32767).astype("<i2")
        fd, out = tempfile.mkstemp(suffix=".wav", prefix="son_")
        os.close(fd)
        with wave.open(out, "wb") as w:
            w.setnchannels(channels)
            w.setsampwidth(bit_depth // 8)
            w.setframerate(sample_rate)
            w.writeframes(samples.tobytes())
        return out

    def _scaled_copy(self, src, gain):
        with wave.open(src, "rb") as w:
            params = w.getparams()
            pcm = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2")
        pcm = np.clip(pcm.astype(np.float32) * gain, -32768, 32767).astype("<i2")
        fd, out = tempfile.mkstemp(suffix=".wav", prefix="cue_")
        os.close(fd)
        with wave.open(out, "wb") as w:
            w.setparams(params)
            w.writeframes(pcm.tobytes())
        return out

    @staticmethod
    def _unlink(path):
        try:
            os.unlink(path)
        except OSError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = SoundboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
