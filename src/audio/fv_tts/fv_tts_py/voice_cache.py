#!/usr/bin/env python3
"""声キャッシュ: 「その場は軽量エンジン、裏で高品質エンジン、次回から高品質」。

fv_tts の engine=pyopenjtalk に重ねて使う。初回は Open JTalk で即答し、同じ文を
VOICEVOX 互換エンジン (AivisSpeech 等) で裏で合成して WAV に保存する。次に同じ文が
来たら保存済み WAV を流す。高品質エンジンが落ちていても遅くても Open JTalk で
喋り続ける (無音にしない)。

ROS に依存しない純 Python (numpy のみ) なので単体テストできる。

- VoiceCacheStore : WAV + index.json。原子的書き込み、容量上限の LRU 追い出し
- RenderWorker    : 1本だけの低優先スレッド。重複排除キュー、ライブ要求優先、
                    エンジン不通時は指数バックオフ、文ごとの失敗回数上限
- 音量合わせ      : Open JTalk 版と同じ「発話部分の RMS」に合わせる (声の切替で音量が跳ねない)
"""
from __future__ import annotations

import collections
import hashlib
import io
import json
import os
import re
import tempfile
import threading
import time
import unicodedata
import wave
from pathlib import Path
from typing import Callable, Dict, Optional, Tuple

import numpy as np

INDEX_NAME = "index.json"
_KEY_RE = re.compile(r"^[0-9a-f]{40}$")


# ── key / text ─────────────────────────────────────────
def normalize_text(text: str) -> str:
    """同じ文が別キーにならないよう NFKC + 空白畳み込み。"""
    text = unicodedata.normalize("NFKC", text or "")
    return re.sub(r"\s+", " ", text).strip()


def make_key(kind: str, speaker: int, params: Dict, volume_db: float, text: str) -> str:
    """(エンジン種別, 話者ID, 音が変わる設定, 正規化テキスト) のハッシュ。URL は含めない。"""
    blob = json.dumps({
        "kind": kind,
        "speaker": int(speaker),
        "params": params or {},
        "volume_db": round(float(volume_db), 2),
        "text": normalize_text(text),
    }, ensure_ascii=False, sort_keys=True)
    return hashlib.sha1(blob.encode("utf-8")).hexdigest()  # nosec B303 (ID 用途)


# ── audio helpers ──────────────────────────────────────
def pcm16_to_float(pcm: bytes) -> np.ndarray:
    return np.frombuffer(pcm, dtype="<i2").astype(np.float32) / 32768.0


def float_to_pcm16(wave_f: np.ndarray) -> bytes:
    return (np.clip(wave_f, -1.0, 1.0) * 32767.0).astype("<i2").tobytes()


def active_rms(samples: np.ndarray, sample_rate: int, gate_db: float = 30.0) -> float:
    """発話区間の RMS。20ms フレームのうち最大から gate_db 以内のものだけで計算。

    無音の長さ (エンジンごとに前後ポーズが違う) に引っ張られないようにする。
    """
    if samples.size == 0:
        return 0.0
    frame = max(1, int(sample_rate * 0.02))
    n = samples.size // frame
    if n == 0:
        return float(np.sqrt(np.mean(np.square(samples))))
    frames = samples[: n * frame].reshape(n, frame)
    energy = np.mean(np.square(frames), axis=1)
    peak_e = float(np.max(energy))
    if peak_e <= 0.0:
        return 0.0
    gate = peak_e * (10.0 ** (-gate_db / 10.0))
    return float(np.sqrt(np.mean(energy[energy >= gate])))


def resample(samples: np.ndarray, src_sr: int, dst_sr: int) -> np.ndarray:
    if src_sr == dst_sr or samples.size == 0:
        return samples.astype(np.float32)
    try:
        from math import gcd
        from scipy.signal import resample_poly  # 帯域制限付き (無ければ線形補間)
        g = gcd(int(src_sr), int(dst_sr))
        return resample_poly(samples, dst_sr // g, src_sr // g).astype(np.float32)
    except ImportError:
        n_out = int(round(samples.size * dst_sr / src_sr))
        idx = np.linspace(0.0, samples.size - 1.0, n_out)
        return np.interp(idx, np.arange(samples.size), samples).astype(np.float32)


def match_loudness(samples: np.ndarray, sample_rate: int, target_rms: float,
                   peak_limit: float = 0.98) -> Tuple[np.ndarray, float]:
    """発話 RMS を target_rms に合わせる。ピークは peak_limit を超えたら軽く圧縮。

    戻り値: (波形, 掛けたゲイン dB)
    """
    cur = active_rms(samples, sample_rate)
    if cur <= 0.0 or target_rms <= 0.0:
        return samples, 0.0
    gain = target_rms / cur
    out = samples * gain
    peak = float(np.max(np.abs(out))) if out.size else 0.0
    if peak > peak_limit:
        # 単純クリップで歪ませず、閾値超えだけ tanh で丸めるソフトリミッタ
        knee = peak_limit * 0.8
        over = np.abs(out) > knee
        head = peak_limit - knee
        out[over] = np.sign(out[over]) * (knee + head * np.tanh((np.abs(out[over]) - knee) / head))
    return out.astype(np.float32), float(20.0 * np.log10(gain))


def wav_bytes(pcm: bytes, sample_rate: int) -> bytes:
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(int(sample_rate))
        w.writeframes(pcm)
    return buf.getvalue()


def read_wav(path: Path) -> Tuple[bytes, int]:
    with wave.open(str(path), "rb") as w:
        if w.getnchannels() != 1 or w.getsampwidth() != 2:
            raise ValueError(f"{path}: mono/16bit ではない")
        return w.readframes(w.getnframes()), int(w.getframerate())


def _atomic_write(path: Path, data: bytes) -> None:
    fd, tmp = tempfile.mkstemp(prefix=".tmp-", dir=str(path.parent))
    try:
        with os.fdopen(fd, "wb") as f:
            f.write(data)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


# ── store ──────────────────────────────────────────────
class VoiceCacheStore:
    """<dir>/<key>.wav と <dir>/index.json (人が読めるよう text 入り)。"""

    def __init__(self, cache_dir, max_bytes: int, clock: Callable[[], float] = time.time) -> None:
        self.dir = Path(cache_dir).expanduser()
        self.dir.mkdir(parents=True, exist_ok=True)
        self.max_bytes = int(max_bytes)
        self.clock = clock
        self._lock = threading.Lock()
        self.index: Dict[str, Dict] = {}
        self._load_index()

    def _load_index(self) -> None:
        try:
            data = json.loads((self.dir / INDEX_NAME).read_text(encoding="utf-8"))
            entries = data.get("entries", {}) if isinstance(data, dict) else {}
        except (OSError, ValueError):
            entries = {}
        for key, ent in entries.items():
            if _KEY_RE.match(key) and isinstance(ent, dict) and (self.dir / f"{key}.wav").exists():
                self.index[key] = ent
        # index に無い WAV と書きかけ tmp は掃除 (自分の命名規則のものだけ)
        for p in self.dir.iterdir():
            if (p.suffix == ".wav" and _KEY_RE.match(p.stem) and p.stem not in self.index) \
                    or p.name.startswith(".tmp-"):
                try:
                    p.unlink()
                except OSError:
                    pass

    def _save_index(self) -> None:
        body = json.dumps({"version": 1, "entries": self.index},
                          ensure_ascii=False, indent=1, sort_keys=True)
        _atomic_write(self.dir / INDEX_NAME, body.encode("utf-8"))

    def __contains__(self, key: str) -> bool:
        with self._lock:
            return key in self.index

    def total_bytes(self) -> int:
        with self._lock:
            return sum(int(e.get("bytes", 0)) for e in self.index.values())

    def get(self, key: str) -> Optional[Tuple[bytes, int]]:
        """(pcm16 bytes, sample_rate) を返す。壊れていれば None (呼び出し側は Open JTalk へ)。"""
        with self._lock:
            ent = self.index.get(key)
            if ent is None:
                return None
            path = self.dir / f"{key}.wav"
            try:
                pcm, sr = read_wav(path)
            except (OSError, ValueError, wave.Error, EOFError):
                self.index.pop(key, None)
                self._safe_save()
                return None
            ent["last_used"] = self.clock()
            ent["hits"] = int(ent.get("hits", 0)) + 1
            self._safe_save()
            return pcm, sr

    def put(self, key: str, pcm: bytes, sample_rate: int, meta: Dict) -> None:
        data = wav_bytes(pcm, sample_rate)
        with self._lock:
            _atomic_write(self.dir / f"{key}.wav", data)
            now = self.clock()
            ent = dict(meta)
            ent.update({"bytes": len(data), "created": now, "last_used": now, "hits": 0,
                        "sample_rate": int(sample_rate),
                        "duration_s": round(len(pcm) / 2 / float(sample_rate), 2)})
            self.index[key] = ent
            self._evict_locked(keep=key)
            self._save_index()

    def _evict_locked(self, keep: str) -> None:
        total = sum(int(e.get("bytes", 0)) for e in self.index.values())
        if total <= self.max_bytes:
            return
        for key in sorted(self.index, key=lambda k: self.index[k].get("last_used", 0.0)):
            if total <= self.max_bytes:
                break
            if key == keep:
                continue
            total -= int(self.index[key].get("bytes", 0))
            self.index.pop(key)
            try:
                (self.dir / f"{key}.wav").unlink()
            except OSError:
                pass

    def _safe_save(self) -> None:
        try:
            self._save_index()
        except OSError:
            pass


# ── worker ─────────────────────────────────────────────
class EngineDown(RuntimeError):
    """エンジン自体が不通 (= 文のせいではない)。バックオフして同じ文を後で再試行する。"""


class RenderWorker:
    """裏で 1 件ずつ合成して store に入れるスレッド。

    render(key, text, ref, job) -> (pcm16 bytes, sample_rate) を呼ぶ。job は submit 時に
    渡した dict (話者 ID など、文ごとに違う値)。保存時の index にも書き込む。EngineDown を投げたら
    指数バックオフ (backoff_min→backoff_max)、それ以外の例外は文ごとに失敗回数を数え
    max_attempts で諦める。ライブ要求 (priority=True) はウォームアップより先に処理。
    """

    def __init__(self, store: VoiceCacheStore,
                 render: Callable[[str, str, Optional[float], Dict], Tuple[bytes, int]],
                 meta: Optional[Dict] = None,
                 log: Optional[Callable[[str], None]] = None,
                 max_queue: int = 200, max_attempts: int = 3,
                 backoff_min: float = 5.0, backoff_max: float = 300.0,
                 gap_s: float = 1.0,
                 busy: Optional[Callable[[], bool]] = None,
                 nice: int = 10,
                 sleep: Callable[[float], None] = time.sleep) -> None:
        self.store = store
        self.render = render
        self.meta = meta or {}
        self.log = log or (lambda _m: None)
        self.max_queue = int(max_queue)
        self.max_attempts = int(max_attempts)
        self.backoff_min = float(backoff_min)
        self.backoff_max = float(backoff_max)
        self.gap_s = float(gap_s)
        self.busy = busy  # True の間ウォームアップを待たせる (負荷ガード)
        self.nice = int(nice)
        self._sleep = sleep
        self._live: "collections.deque" = collections.deque()
        self._warm: "collections.deque" = collections.deque()
        self._pending: Dict[str, Tuple[str, Optional[float], Dict]] = {}
        self._failures: Dict[str, int] = {}
        self._cv = threading.Condition()
        self._stop = False
        self._thread: Optional[threading.Thread] = None
        self.backoff = 0.0
        self.rendered = 0

    # 呼び出し側 (ROS コールバック) からは即戻る
    def submit(self, key: str, text: str, ref_rms: Optional[float] = None,
               priority: bool = True, job: Optional[Dict] = None) -> bool:
        with self._cv:
            if key in self.store or self._failures.get(key, 0) >= self.max_attempts:
                return False
            if key in self._pending:
                # ウォームアップ待ちの文がライブで来たら前に出す
                if priority and key in self._warm:
                    self._warm.remove(key)
                    self._live.append(key)
                return False
            if len(self._pending) >= self.max_queue:
                return False
            self._pending[key] = (text, ref_rms, dict(job or {}))
            (self._live if priority else self._warm).append(key)
            self._cv.notify()
            return True

    def queued(self) -> int:
        with self._cv:
            return len(self._pending)

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="voice_cache", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        with self._cv:
            self._stop = True
            self._cv.notify_all()
        if self._thread is not None:
            self._thread.join(timeout)

    def _next(self) -> Optional[Tuple[str, bool]]:
        with self._cv:
            while not self._stop and not self._live and not self._warm:
                self._cv.wait(1.0)
            if self._stop:
                return None
            if self._live:
                return self._live.popleft(), True
            return self._warm.popleft(), False

    def _requeue_front(self, key: str, live: bool) -> None:
        with self._cv:
            (self._live if live else self._warm).appendleft(key)

    def _run(self) -> None:
        if self.nice:
            try:  # Linux はスレッド単位の nice (このスレッドだけ低優先)
                os.setpriority(os.PRIO_PROCESS, threading.get_native_id(), self.nice)
            except (OSError, AttributeError):
                pass
        while True:
            item = self._next()
            if item is None:
                return
            self.step(*item)

    def step(self, key: str, live: bool) -> None:
        """1 件処理 (テストからも直接呼ぶ)。"""
        if not live and self.busy is not None and self.busy():
            self._requeue_front(key, live)
            self._sleep(10.0)
            return
        with self._cv:
            text, ref, job = self._pending[key]
        t0 = time.monotonic()
        try:
            pcm, sr = self.render(key, text, ref, job)
        except EngineDown as exc:
            self.backoff = min(self.backoff_max, max(self.backoff_min, self.backoff * 2))
            self.log(f"voice cache: エンジン不通 ({exc}); {self.backoff:.0f}s 後に再試行")
            self._requeue_front(key, live)
            self._sleep(self.backoff)
            return
        except Exception as exc:  # noqa: BLE001 (文ごとの失敗。Open JTalk は鳴っている)
            n = self._failures.get(key, 0) + 1
            self._failures[key] = n
            self.log(f"voice cache: 合成失敗 {n}/{self.max_attempts} '{text[:30]}': {exc}")
            with self._cv:
                if n >= self.max_attempts:
                    self._pending.pop(key, None)
                else:
                    (self._live if live else self._warm).append(key)
            self._sleep(self.gap_s)
            return
        self.backoff = 0.0
        meta = dict(self.meta)
        meta.update(job)
        meta["text"] = normalize_text(text)
        meta["render_s"] = round(time.monotonic() - t0, 2)
        try:
            self.store.put(key, pcm, sr, meta)
            self.rendered += 1
            self.log(f"voice cache: 保存 '{text[:30]}' ({meta['render_s']}s)")
        except OSError as exc:
            self.log(f"voice cache: 保存失敗 {exc}")
        with self._cv:
            self._pending.pop(key, None)
        self._sleep(self.gap_s)


def load_warmup(path) -> list:
    """YAML (texts: [...] または素のリスト) / 1行1文のテキストを読む。"""
    p = Path(path).expanduser()
    raw = p.read_text(encoding="utf-8")
    if p.suffix in (".yaml", ".yml"):
        import yaml  # python3-yaml
        data = yaml.safe_load(raw) or []
        if isinstance(data, dict):
            data = data.get("texts", [])
        items = [str(t) for t in data if t]
    else:
        items = [ln for ln in raw.splitlines() if ln.strip() and not ln.lstrip().startswith("#")]
    seen, out = set(), []
    for t in items:
        n = normalize_text(t)
        if n and n not in seen:
            seen.add(n)
            out.append(n)
    return out
