"""fv_tts: 発話ごとの id と、分けて読んだクリップ (文) ごとの終わり方の知らせ (/aspa/tts/playback)。

/aspa/audio/speaking の OFF は publish からの再生尺のタイマーで、本当の鳴り終わりではない (fv_soundboard も同じ topic に
OFF を出す。AivisSpeech は文ごとに流すので、次の文の合成が遅れると文の間でも OFF になる)。聞く側 (aspa_agent の
echo_guard) が「この文は最後まで鳴って、もう鳴っていない」を確かめられるよう、出力 (fv_audio_output) の
``audio/output/playback_done`` (AudioFrame.header.frame_id がそのまま request_id で返る) でクリップごとの終わりを知らせる。

知らせ (JSON。どれも ``event`` ``id`` ``seq`` ``boot`` ``text`` ``t`` を持つ):
  queued  /aspa/tts/say を受けた (まだ合成していない)
  start   喋り始める。``of`` = 分けた文の数、``engine``
  clip    1 クリップの終わり方。``index`` ``of`` ``clip_text`` ``status`` ``seconds`` (``reason``)
            done     出力が最後まで鳴らした (playback_done を受け、出力の遅れ ``latency_s`` の間に止められなかった)
            cut      止められた (audio/output/stop・終了)。鳴る前に止められた文も
            failed   合成できなかった (鳴っていない)
            unknown  鳴り終わりが分からない (playback_done が来ない・出力が 1 つでない・鳴らさない設定)
  end     発話全体の終わり。``status`` (done = 全部の文が done / cut / failed / partial / unknown)、``of`` ``clips``
          (出したクリップの数) ``done`` (done の数) ``sound`` (ほかの音と重なった) ``statuses`` ``seconds`` ``error``
``seq`` はノードの起動 (``boot``) ごとに 1 から増える: 聞く側は飛び・入れ替わり・再起動を見分けられる。

ROS に依らない部品 (呼び手が 1 つのロックの中で呼ぶ。時計は差し替えられる)。
"""
from __future__ import annotations

import itertools
import time
import uuid
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

DONE = "done"
CUT = "cut"
FAILED = "failed"
UNKNOWN = "unknown"
FRAME_PREFIX = "fv_tts:"


@dataclass
class _Clip:
    index: int
    text: str
    frame_id: str = ""
    epoch: int = 0
    seconds: float = 0.0
    expected_end: float = 0.0       # 出した時に見込んだ鳴り終わり (単調時計)。これ + grace で来なければ unknown
    sure: bool = True               # 出力がちょうど 1 つ (最初の playback_done = 鳴り終わり)
    done_at: Optional[float] = None  # playback_done を受けた時刻
    status: str = ""
    reason: str = ""


@dataclass
class _Utt:
    id: str
    text: str
    engine: str = ""
    planned: int = 0
    clips: List[_Clip] = field(default_factory=list)
    started: bool = False
    finished: bool = False          # 出す側が出し終えた (この後クリップは増えない)
    stopped: bool = False           # 途中で止められた (残りの文は出していない)
    sound: bool = False             # 始まってから終わるまでに、ほかの音 (効果音・別のノード) が鳴った
    error: str = ""


class PlaybackTracker:
    """発話ごとの id とクリップの終わり方を数えて知らせる。

    emit(payload): 知らせを 1 つ出す (呼び手が JSON にして publish)。呼び手のロックの中で呼ばれる。
    latency_s: playback_done の後、まだ出力の先に残っているかもしれない音の長さ (fv_audio_output の
        drain_after_frame=false なら ALSA のバッファ ≈ 0.34 s)。この間に止められたら cut。
    grace_s: 見込みの鳴り終わりからこれだけ過ぎても playback_done が来なければ unknown。
    """

    def __init__(self, emit: Callable[[dict], None], *, clock: Callable[[], float] = time.monotonic,
                 wall: Callable[[], float] = time.time, latency_s: float = 0.4, grace_s: float = 5.0,
                 boot: Optional[str] = None) -> None:
        self._emit_fn = emit
        self._clock = clock
        self._wall = wall
        self.latency_s = max(0.0, float(latency_s))
        self.grace_s = max(0.5, float(grace_s))
        self.boot = boot or uuid.uuid4().hex[:8]
        self._n = itertools.count(1)
        self._seq = 0
        self._utts: Dict[str, _Utt] = {}
        self._frames: Dict[str, Tuple[_Utt, _Clip]] = {}
        self._sound_on = False

    # ------------------------------------------------------------ ids
    def new_id(self) -> str:
        return f"{self.boot}-{next(self._n)}"

    @staticmethod
    def frame_id(uid: str, index: int) -> str:
        """AudioFrame.header.frame_id (fv_audio_output が PlaybackDone.request_id で返す)。"""
        return f"{FRAME_PREFIX}{uid}:{index}"

    def in_flight(self) -> int:
        return len(self._utts)

    # ---------------------------------------------------------- notices
    def _emit(self, event: str, utt: _Utt, **fields) -> None:
        self._seq += 1
        payload = {"event": event, "id": utt.id, "seq": self._seq, "boot": self.boot, "text": utt.text,
                   "t": round(float(self._wall()), 3)}
        payload.update(fields)
        try:
            self._emit_fn(payload)
        except Exception:  # pylint: disable=broad-except  # 知らせは出せなくても喋るのは止めない
            pass

    def _clip_notice(self, utt: _Utt, clip: _Clip) -> None:
        extra = {"reason": clip.reason} if clip.reason else {}
        self._emit("clip", utt, index=clip.index, of=utt.planned, clip_text=clip.text, status=clip.status,
                   seconds=round(clip.seconds, 2), **extra)

    # ----------------------------------------------------- speaking side
    def queued(self, uid: str, text: str) -> None:
        if uid in self._utts:
            return
        utt = _Utt(id=uid, text=text)
        self._utts[uid] = utt
        self._emit("queued", utt)

    def start(self, uid: str, text: str, engine: str, planned: int) -> None:
        utt = self._utts.get(uid)
        if utt is None:
            self.queued(uid, text)
            utt = self._utts[uid]
        utt.engine, utt.planned, utt.started = str(engine or ""), int(planned), True
        if self._sound_on:
            utt.sound = True                # 効果音が鳴っている間に始まった
        self._emit("start", utt, engine=utt.engine, of=utt.planned)

    def clip_failed(self, uid: str, index: int, text: str, reason: str) -> None:
        utt = self._utts.get(uid)
        if utt is None:
            return
        clip = _Clip(index=index, text=text, status=FAILED, reason=str(reason or "")[:200])
        utt.clips.append(clip)
        self._clip_notice(utt, clip)

    def clip_published(self, uid: str, index: int, text: str, *, epoch: int, seconds: float,
                       expected_end: Optional[float], sure: bool, played: bool = True) -> str:
        """出した (play=True なら出力の待ち行列へ)。frame_id を返す (AudioFrame.header.frame_id に入れる)。"""
        utt = self._utts.get(uid)
        fid = self.frame_id(uid, index)
        if utt is None:
            return fid
        clip = _Clip(index=index, text=text, frame_id=fid, epoch=int(epoch), seconds=float(seconds or 0.0),
                     expected_end=float(expected_end if expected_end is not None else self._clock() + seconds),
                     sure=bool(sure))
        utt.clips.append(clip)
        if not played:
            clip.status, clip.reason = UNKNOWN, "not played"
            self._clip_notice(utt, clip)
        else:
            self._frames[fid] = (utt, clip)
        return fid

    def finish(self, uid: str, *, stopped: bool = False, error: str = "") -> None:
        """出す側が出し終えた (stopped: 止められて残りの文を出さなかった)。"""
        utt = self._utts.get(uid)
        if utt is None:
            return
        utt.finished = True
        utt.stopped = utt.stopped or bool(stopped)
        utt.error = utt.error or str(error or "")
        self._check(utt)

    # ------------------------------------------------------ output side
    def on_done(self, request_id: str, epoch: int) -> None:
        """audio/output/playback_done (fv_audio_output が 1 フレームを鳴らし終えた)。"""
        rid = str(request_id or "")
        hit = self._frames.get(rid)
        if hit is not None:
            _utt, clip = hit
            if not clip.status and clip.done_at is None and int(epoch) == clip.epoch:
                clip.done_at = self._clock()
            return
        if rid.startswith(FRAME_PREFIX):
            return                          # 自分の音 (終えた後の 2 つ目の知らせ・前の起動の音)
        # ほかの音 (効果音・ファイル・別のノード) が鳴り終えた: 喋り始めている発話はほかの音と重なった
        for utt in self._utts.values():
            if utt.started:
                utt.sound = True

    def on_sound(self, on: bool) -> None:
        """fv_soundboard の speaking (効果音だけの印)。"""
        self._sound_on = bool(on)
        if on:
            for utt in self._utts.values():
                if utt.started:
                    utt.sound = True

    def on_stop(self) -> None:
        """audio/output/stop: 鳴っている・待っているクリップは cut。出している途中の発話も cut。"""
        for utt in list(self._utts.values()):
            if utt.started:
                utt.stopped = True
            for clip in utt.clips:
                if not clip.status:
                    clip.status, clip.reason = CUT, "stop"
                    self._frames.pop(clip.frame_id, None)
                    self._clip_notice(utt, clip)
        for utt in list(self._utts.values()):
            self._check(utt)

    def tick(self) -> None:
        """出力の遅れが過ぎた done を確定し、来ない done を unknown にする (呼び手が数十 ms ごとに)。"""
        now = self._clock()
        touched: List[_Utt] = []
        for fid, (utt, clip) in list(self._frames.items()):
            if clip.status:
                self._frames.pop(fid, None)
                continue
            if clip.done_at is not None and now - clip.done_at >= self.latency_s:
                clip.status = DONE if clip.sure else UNKNOWN
                if not clip.sure:
                    clip.reason = "outputs != 1"
            elif clip.done_at is None and now >= clip.expected_end + self.grace_s:
                clip.status, clip.reason = UNKNOWN, "no playback_done"
            else:
                continue
            self._frames.pop(fid, None)
            self._clip_notice(utt, clip)
            touched.append(utt)
        for utt in touched:
            self._check(utt)

    # ------------------------------------------------------------- end
    @staticmethod
    def end_status(utt: _Utt) -> str:
        statuses = [c.status for c in utt.clips]
        if utt.stopped or CUT in statuses:
            return CUT
        played = [s for s in statuses if s != FAILED]
        if not played:
            return FAILED
        if FAILED in statuses:
            return "partial"
        if UNKNOWN in statuses or len(statuses) != utt.planned or not utt.started:
            return UNKNOWN
        return DONE

    def _check(self, utt: _Utt) -> None:
        if not utt.finished or any(not c.status for c in utt.clips):
            return
        if self._utts.pop(utt.id, None) is None:
            return
        status = self.end_status(utt)
        self._emit("end", utt, status=status, of=utt.planned, clips=sum(1 for c in utt.clips if c.status != FAILED),
                   done=sum(1 for c in utt.clips if c.status == DONE), sound=bool(utt.sound), engine=utt.engine,
                   statuses=[c.status for c in utt.clips], seconds=round(sum(c.seconds for c in utt.clips), 2),
                   error=utt.error)


__all__ = ["CUT", "DONE", "FAILED", "FRAME_PREFIX", "PlaybackTracker", "UNKNOWN"]
