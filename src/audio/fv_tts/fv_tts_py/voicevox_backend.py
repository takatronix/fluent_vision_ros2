#!/usr/bin/env python3
"""VOICEVOX ENGINE を叩く TTS バックエンド。

fv_tts のエンジン差替え機構 (engine="voicevox") 用。ローカルで動く
VOICEVOX ENGINE (既定 http://127.0.0.1:50021) の 2 段 API を呼ぶ:

  1. POST /audio_query?text=<text>&speaker=<id>   → 合成用クエリ(JSON)
  2. POST /synthesis?speaker=<id>  (body=上記JSON) → 24kHz mono 16bit WAV

返ってきた WAV からヘッダを剥がして PCM(int16) と実サンプルレートを取り出す。
接続不可・HTTP エラー・想定外フォーマットは握りつぶさず例外送出する (fallback 禁止)。

依存は標準ライブラリのみ (urllib / wave)。エンジン本体は外部プロセスなので
rosdep 依存には含めず、起動確認は `curl http://127.0.0.1:50021/version`。
"""
import io
import urllib.parse
import urllib.request
import wave


class VoicevoxError(RuntimeError):
    """VOICEVOX ENGINE との通信・合成に失敗したときの明示例外。"""


class VoicevoxBackend:
    """VOICEVOX ENGINE の HTTP クライアント。

    synthesize(text, speaker) -> (pcm_int16_bytes, sample_rate, channels, bit_depth)
    """

    def __init__(self, url: str = "http://127.0.0.1:50021", timeout: float = 15.0) -> None:
        self.url = url.rstrip("/")
        self.timeout = float(timeout)

    # ── public ──────────────────────────────────────────
    def synthesize(self, text: str, speaker: int):
        """text を speaker で合成し、生 PCM(int16 LE) とフォーマットを返す。"""
        if not text:
            raise VoicevoxError("text is empty")
        query = self._audio_query(text, int(speaker))
        wav_bytes = self._synthesis(query, int(speaker))
        return self._decode_wav(wav_bytes)

    def version(self) -> str:
        """ENGINE のバージョン文字列 (疎通確認用)。"""
        try:
            with urllib.request.urlopen(f"{self.url}/version", timeout=self.timeout) as resp:
                return resp.read().decode("utf-8", errors="ignore").strip()
        except Exception as exc:  # noqa: BLE001
            raise VoicevoxError(f"version取得失敗 ({self.url}): {exc}") from exc

    # ── internal ────────────────────────────────────────
    def _audio_query(self, text: str, speaker: int) -> bytes:
        params = urllib.parse.urlencode({"text": text, "speaker": speaker})
        # audio_query はクエリ文字列で渡す POST (body 無し)
        req = urllib.request.Request(f"{self.url}/audio_query?{params}", data=b"", method="POST")
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                return resp.read()
        except Exception as exc:  # noqa: BLE001
            raise VoicevoxError(
                f"audio_query失敗 ({self.url}, speaker={speaker}): {exc}") from exc

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
            raise VoicevoxError(
                f"synthesis失敗 ({self.url}, speaker={speaker}): {exc}") from exc

    @staticmethod
    def _decode_wav(wav_bytes: bytes):
        try:
            with wave.open(io.BytesIO(wav_bytes), "rb") as w:
                channels = w.getnchannels()
                bit_depth = w.getsampwidth() * 8
                sample_rate = w.getframerate()
                pcm = w.readframes(w.getnframes())
        except wave.Error as exc:
            raise VoicevoxError(f"WAV decode失敗: {exc}") from exc
        if bit_depth != 16:
            raise VoicevoxError(f"想定外のbit_depth={bit_depth} (16bit想定)")
        if not pcm:
            raise VoicevoxError("合成結果が空 (PCM 0 bytes)")
        return pcm, int(sample_rate), int(channels), int(bit_depth)
