#!/usr/bin/env python3
"""system-health イベント群の離散 cue (wav) を生成する。

既存 25 cue と同じ 48kHz mono 16bit・ベル系(加算合成)の技法で新規生成する。
判別性を重視: sensor/comms の lost=下行(欠落感) / recovered=上行(安心感) を対で作る。

再生成:
    python3 tools/gen_health_cues.py          # sounds/ に書き出し
"""
import os
import wave

import numpy as np

SR = 48000
HERE = os.path.dirname(os.path.abspath(__file__))
SOUNDS = os.path.normpath(os.path.join(HERE, "..", "sounds"))


def _bell(freq, dur, decay=6.0, partials=(1.0, 2.0, 3.01, 4.2), gains=(1.0, 0.55, 0.35, 0.2),
          detune=0.0):
    """加算合成ベル。倍音 + 指数減衰。"""
    n = int(SR * dur)
    t = np.arange(n) / SR
    w = np.zeros(n)
    for m, g in zip(partials, gains):
        w += g * np.sin(2 * np.pi * freq * m * (1.0 + detune) * t)
    env = np.exp(-decay * t) * np.clip(t / 0.004, 0, 1)  # 4ms attack
    return w * env


def _glide(f0, f1, dur, decay=4.0):
    """f0→f1 へ滑らかにグライドする単音 (上行/下行の方向感を出す)。"""
    n = int(SR * dur)
    t = np.arange(n) / SR
    freq = np.linspace(f0, f1, n)
    phase = 2 * np.pi * np.cumsum(freq) / SR
    w = np.sin(phase) + 0.4 * np.sin(2 * phase)
    env = np.exp(-decay * t) * np.clip(t / 0.004, 0, 1)
    return w * env


def _seq(segments, gap=0.0):
    """(wave, start_time) の列を1本のトラックへ合成。"""
    total = max(int((st + len(w) / SR) * SR) for w, st in segments)
    track = np.zeros(total + int(gap * SR) + 1)
    for w, st in segments:
        i = int(st * SR)
        track[i:i + len(w)] += w
    return track


def _norm(w, peak=0.85):
    m = np.max(np.abs(w)) if w.size else 1.0
    if m > 1e-9:
        w = w / m * peak
    return w


def _save(name, w):
    w = _norm(w)
    pcm = (np.clip(w, -1, 1) * 32767).astype("<i2")
    path = os.path.join(SOUNDS, name + ".wav")
    with wave.open(path, "wb") as f:
        f.setnchannels(1)
        f.setsampwidth(2)
        f.setframerate(SR)
        f.writeframes(pcm.tobytes())
    print(f"wrote {path} ({len(pcm) / SR:.2f}s)")


def build():
    cues = {}

    # battery_critical: 深刻・低く反復するパルス (緊急)
    seg = []
    for k in range(3):
        seg.append((_bell(196.0, 0.5, decay=7.0), k * 0.28))
        seg.append((_bell(155.6, 0.5, decay=7.0), k * 0.28 + 0.09))
    cues["battery_critical"] = _seq(seg)

    # sensor_lost: 下行 (欠落感)。3音を下げていき最後は空虚に
    cues["sensor_lost"] = _seq([
        (_bell(880.0, 0.35, decay=8.0), 0.0),
        (_bell(587.3, 0.4, decay=8.0), 0.16),
        (_glide(440.0, 196.0, 0.6, decay=4.5), 0.34),
    ])

    # sensor_recovered: 上行 (安心感)。柔らかく上がって開く
    cues["sensor_recovered"] = _seq([
        (_bell(392.0, 0.4, decay=6.0), 0.0),
        (_bell(587.3, 0.4, decay=6.0), 0.18),
        (_bell(783.99, 0.7, decay=4.0), 0.36),
    ])

    # node_down: 低いエラー的な二連 (鈍い)
    cues["node_down"] = _seq([
        (_bell(311.1, 0.45, decay=7.0, partials=(1.0, 1.5, 2.0), gains=(1.0, 0.6, 0.4)), 0.0),
        (_bell(233.1, 0.6, decay=6.0, partials=(1.0, 1.5, 2.0), gains=(1.0, 0.6, 0.4)), 0.22),
    ])

    # system_error: 一般異常・強め二音下行 (interrupt級)
    cues["system_error"] = _seq([
        (_bell(659.3, 0.3, decay=9.0, detune=0.008), 0.0),
        (_bell(415.3, 0.5, decay=7.0, detune=0.008), 0.16),
    ])

    # thermal_warning: 温度・揺れる上昇シズル
    n = int(SR * 0.9)
    t = np.arange(n) / SR
    freq = 520.0 + 90.0 * np.sin(2 * np.pi * 7.0 * t) + 120.0 * t  # ビブラート+緩上昇
    phase = 2 * np.pi * np.cumsum(freq) / SR
    sizzle = (np.sin(phase) + 0.3 * np.sin(2 * phase)) * np.exp(-2.2 * t) * np.clip(t / 0.005, 0, 1)
    cues["thermal_warning"] = sizzle

    # comms_lost: 通信断・中空な二音下行 (sensor_lost と別キャラ)
    cues["comms_lost"] = _seq([
        (_bell(698.5, 0.3, decay=7.0, partials=(1.0, 3.0), gains=(1.0, 0.4)), 0.0),
        (_glide(523.3, 261.6, 0.5, decay=4.0), 0.2),
    ])

    # comms_recovered: 通信復帰・二音上行
    cues["comms_recovered"] = _seq([
        (_bell(523.3, 0.3, decay=6.0, partials=(1.0, 3.0), gains=(1.0, 0.4)), 0.0),
        (_glide(523.3, 880.0, 0.45, decay=4.0), 0.2),
    ])

    return cues


def main():
    os.makedirs(SOUNDS, exist_ok=True)
    for name, w in build().items():
        _save(name, w)


if __name__ == "__main__":
    main()
