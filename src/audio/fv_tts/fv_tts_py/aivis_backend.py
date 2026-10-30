#!/usr/bin/env python3
"""AivisSpeech Engine を叩く TTS バックエンド (fv_tts の engine="aivis" 用)。

ロボット内で動く AivisSpeech Engine (既定 http://127.0.0.1:10101) の 2 段 API を呼ぶ:

  1. POST /audio_query?text=<text>&speaker=<id>   → 合成用クエリ(JSON)
  2. POST /synthesis?speaker=<id>  (body=上記JSON) → mono 16bit WAV (44.1kHz)

返ってきた WAV からヘッダを剥がして PCM(int16) と実サンプルレートを取り出す
(出力レートへのリサンプルは呼び出し側)。スタイル ID は 135718976 (女) /
1310138977 (男) のように大きいが int のまま渡せばよい。
接続不可・HTTP エラー・想定外フォーマットは握りつぶさず例外送出する
(Open JTalk への切替は呼び出し側の声キャッシュ層が行う)。

別の機械の強い GPU で動くエンジンを先に使う時は AivisRouter (aivis_remote_urls): 短い期限で先に試し、
届かない・遅い時はこの機械のエンジンへ自動で戻す。戻った後はしばらく (hold_s) 裏の確かめだけで試し直す
(声を待たせるのは最初の 1 回の短い時間切れだけ)。同じ声にするため、両方に同じ話者 (同じ版のモデル) が要る。

依存は標準ライブラリのみ (urllib / wave)。エンジン本体は外部プロセス
(systemd --user の aivisspeech.service) なので rosdep 依存には含めず、
起動確認は `curl http://127.0.0.1:10101/version`。
"""
import http.client
import io
import json
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import wave

DEFAULT_URL = "http://127.0.0.1:10101"


class AivisError(RuntimeError):
    """AivisSpeech Engine との通信・合成に失敗したときの明示例外。"""


class AivisUnreachable(AivisError):
    """接続できない / タイムアウト (HTTP 応答が無い)。エンジン側の問題。"""


def _wrap(msg: str, exc: Exception) -> AivisError:
    if isinstance(exc, urllib.error.HTTPError):
        return AivisError(f"{msg}: {exc}")
    return AivisUnreachable(f"{msg}: {exc}")


class AivisBackend:
    """AivisSpeech Engine の HTTP クライアント。

    synthesize(text, speaker) -> (pcm_int16_bytes, sample_rate, channels, bit_depth)
    """

    def __init__(self, url: str = DEFAULT_URL, timeout: float = 15.0) -> None:
        self.url = url.rstrip("/")
        self.timeout = float(timeout)

    # ── public ──────────────────────────────────────────
    def synthesize(self, text: str, speaker: int, query_overrides=None):
        """text を speaker で合成し、生 PCM(int16 LE) とフォーマットを返す。

        query_overrides: audio_query の結果へ上書きする項目 (例 {"speedScale": 1.1})。
        """
        if not text:
            raise AivisError("text is empty")
        query = self._audio_query(text, int(speaker))
        if query_overrides:
            try:
                obj = json.loads(query)
            except ValueError as exc:
                raise AivisError(f"audio_query の JSON が壊れている: {exc}") from exc
            obj.update(query_overrides)
            query = json.dumps(obj).encode("utf-8")
        wav_bytes = self._synthesis(query, int(speaker))
        return self._decode_wav(wav_bytes)

    def version(self) -> str:
        """エンジンのバージョン文字列 (疎通確認用)。"""
        try:
            with urllib.request.urlopen(f"{self.url}/version", timeout=self.timeout) as resp:
                return resp.read().decode("utf-8", errors="ignore").strip()
        except Exception as exc:  # noqa: BLE001
            raise _wrap(f"version取得失敗 ({self.url})", exc) from exc

    # ── internal ────────────────────────────────────────
    def _audio_query(self, text: str, speaker: int) -> bytes:
        params = urllib.parse.urlencode({"text": text, "speaker": speaker})
        # audio_query はクエリ文字列で渡す POST (body 無し)
        req = urllib.request.Request(f"{self.url}/audio_query?{params}", data=b"", method="POST")
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                return resp.read()
        except Exception as exc:  # noqa: BLE001
            raise _wrap(f"audio_query失敗 ({self.url}, speaker={speaker})", exc) from exc

    def _synthesis(self, query_json: bytes, speaker: int) -> bytes:
        params = urllib.parse.urlencode({"speaker": speaker})
        req = urllib.request.Request(
            f"{self.url}/synthesis?{params}",
            data=query_json,
            headers={"Content-Type": "application/json"},
            method="POST")
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                return resp.read()
        except Exception as exc:  # noqa: BLE001
            raise _wrap(f"synthesis失敗 ({self.url}, speaker={speaker})", exc) from exc

    @staticmethod
    def _decode_wav(wav_bytes: bytes):
        try:
            with wave.open(io.BytesIO(wav_bytes), "rb") as w:
                channels = w.getnchannels()
                bit_depth = w.getsampwidth() * 8
                sample_rate = w.getframerate()
                pcm = w.readframes(w.getnframes())
        except wave.Error as exc:
            raise AivisError(f"WAV decode失敗: {exc}") from exc
        if bit_depth != 16:
            raise AivisError(f"想定外のbit_depth={bit_depth} (16bit想定)")
        if not pcm:
            raise AivisError("合成結果が空 (PCM 0 bytes)")
        return pcm, int(sample_rate), int(channels), int(bit_depth)


class _DeadlineBackend(AivisBackend):
    """1 文ぶんの期限つきの AivisBackend (別の機械のエンジン用)。接続は connect_s、全体は budget_s まで。転送 (3xx) は追わない。"""

    def __init__(self, url: str, budget_s: float, connect_s: float) -> None:
        super().__init__(url, timeout=budget_s)
        self._deadline = time.monotonic() + float(budget_s)
        self._connect_s = float(connect_s)

    def _post(self, path: str, data: bytes, headers: dict) -> bytes:
        parts = urllib.parse.urlsplit(self.url)
        cls = http.client.HTTPSConnection if parts.scheme == "https" else http.client.HTTPConnection
        conn = cls(parts.hostname, parts.port, timeout=min(self._connect_s, self._remaining()))
        try:
            conn.connect()
            sock = conn.sock               # conn.sock は答えの終わりで None になる
            sock.settimeout(self._remaining())
            conn.request("POST", parts.path + path, body=data, headers=headers)
            resp = conn.getresponse()
            if resp.status != 200:
                raise AivisError(f"{path.split('?')[0]} HTTP {resp.status} ({self.url})")
            chunks = []
            while not resp.isclosed():     # socket の時間切れは 1 回の読みごとなので、少しずつ届く時は全体の期限をここで見る
                sock.settimeout(self._remaining())
                chunk = resp.read(65536)
                if not chunk:
                    break
                chunks.append(chunk)
            return b"".join(chunks)
        except AivisError:
            raise
        except Exception as exc:  # noqa: BLE001
            raise AivisUnreachable(f"{self.url}: {exc}") from exc
        finally:
            conn.close()

    def _remaining(self) -> float:
        left = self._deadline - time.monotonic()
        if left <= 0:
            raise AivisUnreachable(f"{self.url}: 期限切れ")
        return left

    def _audio_query(self, text: str, speaker: int) -> bytes:
        params = urllib.parse.urlencode({"text": text, "speaker": speaker})
        return self._post(f"/audio_query?{params}", b"", {})

    def _synthesis(self, query_json: bytes, speaker: int) -> bytes:
        params = urllib.parse.urlencode({"speaker": speaker})
        return self._post(f"/synthesis?{params}", query_json, {"Content-Type": "application/json"})


class AivisRouter:
    """別の機械のエンジン (remote_urls) を先に、だめならこの機械のエンジン (local) で合成する。AivisBackend と同じ口。

    - 期限: 接続 connect_s、1 文全体 budget_s + budget_per_char_s × 字数。超えたら local (声は必ず local で出す)。
    - 遅い: 合成はできたが slow_s + slow_per_char_s × 字数 より遅い (= local の方が速い) 時も、次から local。
    - 戻った後: hold_s (続けて失敗すると倍に、hold_max_s まで) は試さない。過ぎたら声は local のまま裏で /version を確かめ
      (probe_s まで)、返れば次の文から戻す。起動直後も確かめてから (届かない場所で 1 文目を待たせない)。
    - enabled=False (機体の設定で「機体だけ」): local だけ。
    """

    def __init__(self, local: AivisBackend, remote_urls, *, budget_s: float = 1.0, budget_per_char_s: float = 0.01,
                 connect_s: float = 0.25, slow_s: float = 0.6, slow_per_char_s: float = 0.005,
                 hold_s: float = 30.0, hold_max_s: float = 600.0, probe_s: float = 0.3,
                 clock=time.monotonic, log=None, spawn=None) -> None:
        self.local = local
        self.urls = [str(u).strip().rstrip("/") for u in remote_urls if str(u).strip()]
        self.enabled = True
        self._budget = (float(budget_s), float(budget_per_char_s))
        self._slow = (float(slow_s), float(slow_per_char_s))
        self._connect_s = float(connect_s)
        self._hold = (float(hold_s), float(hold_max_s))
        self._hold_now = float(hold_s)
        self._probe_s = float(probe_s)
        self._clock = clock
        self._log = log or (lambda _m: None)
        self._spawn = spawn or (lambda fn: threading.Thread(target=fn, name="aivis_probe", daemon=True).start())
        self._lock = threading.Lock()
        self._url = ""                     # 今使う入口。"" = 確かめてから (起動直後・失敗の後)
        self._retry_at = float("-inf")     # これより後なら確かめに行く
        self._probing = False
        self.last_source = "local"

    @property
    def url(self) -> str:
        return self.local.url

    def set_enabled(self, on: bool) -> None:
        with self._lock:
            if bool(on) == self.enabled:
                return
            self.enabled = bool(on)
            self._url, self._retry_at, self._hold_now = "", float("-inf"), self._hold[0]
        self._log(f"AivisSpeech: 声を作る場所 = {'先に ' + ', '.join(self.urls) if on else 'この機械だけ'}")

    def _pick(self) -> str:
        """今試す入口。無ければ ""。hold が過ぎていれば裏で確かめを始める (声は待たせない)。"""
        start = False
        with self._lock:
            if not self.enabled or not self.urls:
                return ""
            if self._url:
                return self._url
            if self._clock() >= self._retry_at and not self._probing:
                self._probing = start = True
        if start:
            self._spawn(self._probe)
        return ""

    def _probe(self) -> None:
        found = ""
        for url in self.urls:
            try:
                with urllib.request.urlopen(f"{url}/version", timeout=self._probe_s) as resp:
                    resp.read()
                found = url
                break
            except Exception:  # noqa: BLE001 — 返らなければ次の入口
                continue
        with self._lock:
            self._probing = False
            if not self.enabled:
                return
            if found:
                self._url = found
            else:
                self._retry_at = self._clock() + self._hold_now
                self._hold_now = min(self._hold_now * 2.0, self._hold[1])
        if found:
            self._log(f"AivisSpeech: {found} が返った。次の文からこちらで作る")

    def _give_up(self, url: str, why: str) -> None:
        """この入口はだめだった・遅かった: しばらく local。同じ入口を待っていた他の合成の失敗では 2 回数えない。"""
        with self._lock:
            if self._url != url:
                return
            self._url = ""
            self._retry_at = self._clock() + self._hold_now
            hold = self._hold_now
            self._hold_now = min(self._hold_now * 2.0, self._hold[1])
        self._log(f"AivisSpeech: {url} をやめて {hold:.0f} 秒この機械で作る ({why})")

    def _try_remote(self, url: str, text: str, speaker: int, query_overrides):
        chars = len(text)
        budget = self._budget[0] + self._budget[1] * chars
        started = time.monotonic()
        try:
            result = _DeadlineBackend(url, budget, self._connect_s).synthesize(text, speaker, query_overrides)
        except Exception as exc:  # noqa: BLE001 — どの失敗も local へ (話者が無い・壊れた答えも)
            self._give_up(url, str(exc))
            return None
        took = time.monotonic() - started
        if took > self._slow[0] + self._slow[1] * chars:
            self._give_up(url, f"遅い {took:.2f} 秒")     # この文は使う (作り直すともっと遅い)
        else:
            with self._lock:
                self._hold_now = self._hold[0]
        return result

    def synthesize(self, text: str, speaker: int, query_overrides=None):
        if not text:
            raise AivisError("text is empty")
        url = self._pick()
        if url:
            result = self._try_remote(url, text, speaker, query_overrides)
            if result is not None:
                self.last_source = url
                return result
        self.last_source = "local"
        return self.local.synthesize(text, speaker, query_overrides)

    def warm(self, text: str, speaker: int, query_overrides=None) -> None:
        """暇な時の温め: local の声の読込を保ち (戻った時の 1 文目を遅くしない)、別の機械が今も届くかも見る。local の失敗は呼び手へ。"""
        url = self._pick()
        if url:
            self._try_remote(url, text, speaker, query_overrides)
        self.local.synthesize(text, speaker, query_overrides)

    def version(self) -> str:
        return self.local.version()
