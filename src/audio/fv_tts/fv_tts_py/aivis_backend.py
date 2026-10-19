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

依存は標準ライブラリのみ (urllib / wave)。エンジン本体は外部プロセス
(systemd --user の aivisspeech.service) なので rosdep 依存には含めず、
起動確認は `curl http://127.0.0.1:10101/version`。
"""
import io
import json
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
