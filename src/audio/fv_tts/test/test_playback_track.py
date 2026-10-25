"""playback_track: 発話ごとの id と、クリップごとの終わり方 (完走・中断・失敗・不明) の知らせ。

聞く側 (aspa_agent の echo_guard) は「自然に完走した 1 クリップの文」だけを特別に扱う。ここでは知らせが
本当の鳴り終わり (playback_done + 出力の遅れ) より前に「done」と言わないこと、止められた・分からない時は
done と言わないことを固定する。
"""
import pytest

pt = pytest.importorskip("fv_tts_py.playback_track")


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


@pytest.fixture
def tr():
    clock = Clock()
    out = []
    t = pt.PlaybackTracker(out.append, clock=clock, wall=lambda: 1.0, latency_s=0.4, grace_s=5.0, boot="b00t")
    t.out, t.clock = out, clock
    return t


def events(tr, kind=None):
    return [e for e in tr.out if kind is None or e["event"] == kind]


def say_one(tr, text="実際に収穫するか確認してください", seconds=2.0, sure=True, epoch=1):
    uid = tr.new_id()
    tr.queued(uid, text)
    tr.start(uid, text, "aivis", 1)
    fid = tr.clip_published(uid, 0, text, epoch=epoch, seconds=seconds, expected_end=tr.clock.t + seconds, sure=sure)
    tr.finish(uid)
    return uid, fid


def test_time_alone_never_makes_done(tr):
    """playback_done が来なければ、見込みの尺 + grace を過ぎても done ではなく unknown。"""
    uid, fid = say_one(tr)
    assert fid == f"fv_tts:{uid}:0" and uid.startswith("b00t-")
    assert [e["event"] for e in tr.out] == ["queued", "start"]
    tr.clock.t += 6.9                                    # 尺 2 s + grace 5 s の手前
    tr.tick()
    assert events(tr, "end") == []
    tr.clock.t += 0.2
    tr.tick()
    clip, = events(tr, "clip")
    assert clip["status"] == "unknown" and clip["reason"] == "no playback_done"
    assert events(tr, "end")[0]["status"] == "unknown"


def test_natural_completion(tr):
    uid, fid = say_one(tr)
    tr.clock.t += 2.0
    tr.on_done(fid, 1)
    tr.tick()
    assert events(tr, "end") == []                       # 出力の遅れ (0.4 s) の間はまだ
    tr.clock.t += 0.39
    tr.tick()
    assert events(tr, "end") == []
    tr.clock.t += 0.02
    tr.tick()
    clip, = events(tr, "clip")
    assert clip["status"] == "done" and clip["index"] == 0 and clip["of"] == 1
    end, = events(tr, "end")
    assert end["status"] == "done" and end["of"] == 1 and end["clips"] == 1 and end["done"] == 1
    assert end["sound"] is False and end["id"] == uid and end["statuses"] == ["done"]
    seqs = [e["seq"] for e in tr.out]
    assert seqs == list(range(1, len(seqs) + 1)) and {e["boot"] for e in tr.out} == {"b00t"}


def test_stop_while_playing_is_cut(tr):
    uid, fid = say_one(tr)
    tr.clock.t += 1.0
    tr.on_stop()
    clip, = events(tr, "clip")
    assert clip["status"] == "cut"
    end, = events(tr, "end")
    assert end["status"] == "cut" and end["done"] == 0
    tr.on_done(fid, 1)                                   # 遅れて来た done は無視
    tr.clock.t += 1.0
    tr.tick()
    assert len(events(tr, "end")) == 1


def test_stop_within_the_output_latency_after_done_is_cut(tr):
    uid, fid = say_one(tr)
    tr.clock.t += 2.0
    tr.on_done(fid, 1)
    tr.clock.t += 0.1
    tr.on_stop()                                         # 出力の先に残っていた音ごと止められた
    tr.clock.t += 1.0
    tr.tick()
    assert events(tr, "end")[0]["status"] == "cut"


def test_done_from_another_epoch_is_not_completion(tr):
    uid, fid = say_one(tr, epoch=3)
    tr.clock.t += 2.0
    tr.on_done(fid, 2)
    tr.clock.t += 1.0
    tr.tick()
    assert events(tr, "end") == []
    tr.clock.t += 10.0
    tr.tick()
    assert events(tr, "end")[0]["status"] == "unknown"


def test_more_than_one_output_is_unknown(tr):
    uid, fid = say_one(tr, sure=False)
    tr.clock.t += 2.0
    tr.on_done(fid, 1)
    tr.clock.t += 1.0
    tr.tick()
    clip, = events(tr, "clip")
    assert clip["status"] == "unknown" and clip["reason"] == "outputs != 1"
    assert events(tr, "end")[0]["status"] == "unknown"


def test_two_sentences_end_after_both_and_report_two(tr):
    text = "一つ目の文です。二つ目の文です。"
    uid = tr.new_id()
    tr.queued(uid, text)
    tr.start(uid, text, "aivis", 2)
    f0 = tr.clip_published(uid, 0, "一つ目の文です。", epoch=1, seconds=1.0, expected_end=101.0, sure=True)
    tr.clock.t += 1.0
    tr.on_done(f0, 1)
    tr.clock.t += 0.5
    tr.tick()                                            # 1 文目は鳴り終えた (文の間: 2 文目はまだ合成中)
    assert [e["status"] for e in events(tr, "clip")] == ["done"] and events(tr, "end") == []
    f1 = tr.clip_published(uid, 1, "二つ目の文です。", epoch=1, seconds=1.0, expected_end=102.5, sure=True)
    tr.finish(uid)
    assert events(tr, "end") == []
    tr.clock.t += 1.0
    tr.on_done(f1, 1)
    tr.clock.t += 0.5
    tr.tick()
    end, = events(tr, "end")
    assert end["status"] == "done" and end["of"] == 2 and end["clips"] == 2   # 聞く側は of=2 を末尾比べに使わない


def test_failed_and_skipped_sentences(tr):
    text = "一つ目の文です。二つ目の文です。"
    uid = tr.new_id()
    tr.queued(uid, text)
    tr.start(uid, text, "aivis", 2)
    tr.clip_failed(uid, 0, "一つ目の文です。", "15 秒で合成が終わらない")
    f1 = tr.clip_published(uid, 1, "二つ目の文です。", epoch=1, seconds=1.0, expected_end=101.0, sure=True)
    tr.finish(uid, error="15 秒で合成が終わらない")
    tr.clock.t += 1.0
    tr.on_done(f1, 1)
    tr.clock.t += 0.5
    tr.tick()
    assert events(tr, "end")[0]["status"] == "partial"
    uid2 = tr.new_id()
    tr.queued(uid2, "だめです。")
    tr.start(uid2, "だめです。", "aivis", 1)
    tr.clip_failed(uid2, 0, "だめです。", "不通")
    tr.finish(uid2, error="不通")
    assert events(tr, "end")[-1]["status"] == "failed" and events(tr, "end")[-1]["clips"] == 0


def test_stopped_before_publishing_anything(tr):
    uid = tr.new_id()
    tr.queued(uid, "古い依頼です。")
    tr.start(uid, "古い依頼です。", "aivis", 1)
    tr.finish(uid, stopped=True)
    assert events(tr, "end")[0]["status"] == "cut"


def test_other_audio_during_the_utterance_is_flagged(tr):
    uid, fid = say_one(tr)
    tr.on_done("", 0)                                     # 効果音 (soundboard の frame は frame_id が空) が鳴り終えた
    tr.clock.t += 2.0
    tr.on_done(fid, 1)
    tr.clock.t += 0.5
    tr.tick()
    end, = events(tr, "end")
    assert end["status"] == "done" and end["sound"] is True


def test_soundboard_flag_during_or_before_the_utterance_is_flagged(tr):
    tr.on_sound(True)                                     # 効果音が鳴っている間に始まった
    uid, fid = say_one(tr)
    tr.on_sound(False)
    tr.clock.t += 2.0
    tr.on_done(fid, 1)
    tr.clock.t += 0.5
    tr.tick()
    assert events(tr, "end")[0]["sound"] is True
    uid2, fid2 = say_one(tr, text="次の文です")
    tr.on_sound(True)                                     # 鳴っている最中に効果音
    tr.on_sound(False)
    tr.clock.t += 2.0
    tr.on_done(fid2, 1)
    tr.clock.t += 0.5
    tr.tick()
    assert events(tr, "end")[1]["sound"] is True


def test_own_done_after_finalizing_is_not_other_audio(tr):
    uid, fid = say_one(tr)
    uid2 = tr.new_id()
    tr.queued(uid2, "次の文です")
    tr.start(uid2, "次の文です", "aivis", 1)
    tr.on_done("fv_tts:zzzz-9:0", 1)                     # 前の起動・終えた後の 2 つ目の done: 自分の音
    tr.clock.t += 2.0
    tr.on_done(fid, 1)
    tr.clock.t += 0.5
    tr.tick()
    assert events(tr, "end")[0]["sound"] is False


def test_not_played_is_unknown(tr):
    uid = tr.new_id()
    tr.queued(uid, "鳴らさない文です")
    tr.start(uid, "鳴らさない文です", "aivis", 1)
    tr.clip_published(uid, 0, "鳴らさない文です", epoch=1, seconds=1.0, expected_end=None, sure=False, played=False)
    tr.finish(uid)
    assert events(tr, "end")[0]["status"] == "unknown"


def test_emit_failure_does_not_break_speaking(tr):
    def boom(_payload):
        raise RuntimeError("publisher gone")
    t = pt.PlaybackTracker(boom, clock=tr.clock, boot="x")
    uid = t.new_id()
    t.queued(uid, "文")
    t.start(uid, "文", "aivis", 1)
    t.finish(uid, stopped=True)
    assert t.in_flight() == 0
