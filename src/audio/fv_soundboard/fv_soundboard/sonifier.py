#!/usr/bin/env python3
"""パラメトリック・ソニフィケーション (連続パラメータ → リアルタイム合成)。

fv_soundboard の「離散cue (25個の wav)」に対する追加層。YOLO 検出のように
class + 連続属性 (length/curvature/confidence …) が高頻度で来るとき、その状況を
"シンセみたいな" 音で聴き分けられるように numpy で即時合成する。

音文法は config/sonification.yaml で宣言 (差替え可):
  - class → 音族 (harvest=ベル系 / health=低めざらつき / safety=異質アテンション)
  - 音族 → 音色 (倍音・減衰・AM・ノイズ・interrupt)
  - class → 下位変化 (spear=長三度和音, off_grade=短三度(濁り), young=高いピン,
    uncertain=デチューン揺れ)
  - 連続属性 → 音: length_mm→ピッチ, curvature→ビブラート深さ, confidence→音量/明るさ

「進化成長」原則: クラス追加は sonification.yaml に 1 行。未知クラスは黙殺せず
明示ログを出し default_family へ落とす (fallback 禁止 = 静かに間違えない)。
"""
import time

import numpy as np


class Sonifier:
    def __init__(self, config, logger=None):
        self._log = logger
        cfg = config or {}
        d = cfg.get("defaults", {}) or {}
        self.sample_rate = int(cfg.get("sample_rate", 44100))
        self.duration = float(d.get("duration", 0.35))
        self.min_interval = float(d.get("min_interval", 0.5))
        self.max_queue = int(d.get("max_queue", 8))
        self.base_freq = float(d.get("base_freq", 440.0))
        self.class_families = cfg.get("class_families", {}) or {}
        self.default_family = cfg.get("default_family", "neutral")
        self.families = cfg.get("families", {}) or {}
        self.class_variants = cfg.get("class_variants", {}) or {}
        self.params = cfg.get("params", {}) or {}
        self._last_emit = {}         # class -> monotonic time (per-class レート制限)
        self._warned_unknown = set()

    # ── logging ─────────────────────────────────────────
    def _info(self, msg):
        if self._log is not None:
            self._log.info(msg)
        else:
            print(msg)

    def _debug(self, msg):
        if self._log is not None:
            self._log.debug(msg)

    # ── class → family ──────────────────────────────────
    def family_for(self, cls):
        fam = self.class_families.get(cls)
        if fam is None:
            if cls not in self._warned_unknown:
                self._warned_unknown.add(cls)
                self._info(
                    f"sonify: 未知クラス '{cls}' → default_family "
                    f"'{self.default_family}' (sonification.yaml に追記可)")
            fam = self.default_family
        return fam

    # ── rate limit ──────────────────────────────────────
    def allow(self, cls):
        """min_interval (既定0.5s/class) 内の連発を抑制。発火可なら True。"""
        now = time.monotonic()
        last = self._last_emit.get(cls, 0.0)
        if now - last < self.min_interval:
            self._debug(f"sonify: rate-limit '{cls}' (間隔 {now - last:.2f}s < {self.min_interval}s)")
            return False
        self._last_emit[cls] = now
        return True

    # ── synthesis ───────────────────────────────────────
    def synth(self, cls, attrs):
        """(pcm_int16_bytes, sample_rate, interrupt) を返す。"""
        attrs = attrs or {}
        fam_name = self.family_for(cls)
        fam = self.families.get(fam_name, {}) or {}
        interrupt = bool(fam.get("interrupt", False))

        # 連続パラメータ → 音
        conf = self._attr(attrs, "confidence")
        gain = self._map(conf, self.params.get("confidence", {}), "gain", default=0.9)
        brightness = self._map(conf, self.params.get("confidence", {}), "brightness", default=1.0)

        length = self._attr(attrs, "length_mm")
        oct_shift = self._map(length, self.params.get("length_mm", {}), "out_octaves", default=0.0)
        freq = self.base_freq * (2.0 ** oct_shift)

        curv = self._attr(attrs, "curvature")
        cparam = self.params.get("curvature", {}) or {}
        vib_depth = self._map(curv, cparam, "vibrato_depth_semitones", default=0.0)
        vib_hz = float(cparam.get("vibrato_hz", 6.0))

        wave = self._render(fam, cls, freq, gain, brightness, vib_hz, vib_depth)
        pcm = (np.clip(wave, -1.0, 1.0) * 32767.0).astype(np.int16)
        return pcm.tobytes(), self.sample_rate, interrupt

    # ── internal ────────────────────────────────────────
    def _render(self, fam, cls, freq, gain, brightness, vib_hz, vib_depth):
        n = max(1, int(self.sample_rate * self.duration))
        t = np.arange(n, dtype=np.float64) / self.sample_rate

        # ビブラート: 半音深さ → 周波数比を時間変調 (0=純音)
        vib_semis = vib_depth * np.sin(2.0 * np.pi * vib_hz * t)
        freq_ratio = 2.0 ** (vib_semis / 12.0)
        phase = 2.0 * np.pi * freq * np.cumsum(freq_ratio) / self.sample_rate

        partials = fam.get("partials", [1.0]) or [1.0]
        pgains = fam.get("partial_gains", [1.0] * len(partials)) or [1.0] * len(partials)
        intervals = self._chord_intervals(self.class_variants.get(cls, {}) or {})

        wave = np.zeros(n)
        for semis, amp, detune in intervals:
            r = 2.0 ** (semis / 12.0) * (1.0 + detune)
            for k, mult in enumerate(partials):
                pg = pgains[k] if k < len(pgains) else 0.0
                # brightness で高次倍音を持ち上げ/抑え (confidence 由来)
                b = pg * (float(brightness) ** k)
                wave += amp * b * np.sin(phase * float(mult) * r)

        # 音族固有: AM (異質さ) / ノイズ (ざらつき)
        am_hz = fam.get("am_hz")
        if am_hz:
            wave *= (0.6 + 0.4 * np.sin(2.0 * np.pi * float(am_hz) * t))
        noise = fam.get("noise")
        if noise:
            wave += float(noise) * np.random.uniform(-1.0, 1.0, n)

        # エンベロープ: 5ms アタック + 指数減衰
        decay = float(fam.get("decay", 4.0))
        env = np.exp(-decay * t)
        attack = np.clip(t / 0.005, 0.0, 1.0)
        wave *= env * attack

        peak = float(np.max(np.abs(wave))) if wave.size else 0.0
        if peak > 1e-9:
            wave = wave / peak
        return wave * float(gain)

    @staticmethod
    def _chord_intervals(variant):
        """class_variants の chord 指定 → [(半音offset, 振幅, デチューン), ...]。"""
        chord = variant.get("chord") if isinstance(variant, dict) else None
        if chord == "major_third":      # A品=長三度和音 (明るい協和)
            return [(0, 1.0, 0.0), (4, 0.7, 0.0), (7, 0.6, 0.0)]
        if chord == "minor_third":      # off_grade=短三度+濁り
            return [(0, 1.0, 0.0), (3, 0.8, 0.0), (6, 0.4, 0.0)]
        if chord == "high_pin":         # young=小さく高いピン (1oct上・単音)
            return [(12, 0.5, 0.0)]
        if chord == "detune_wobble":    # uncertain=デチューン揺れ
            return [(0, 1.0, 0.0), (0, 0.9, 0.012)]
        return [(0, 1.0, 0.0)]          # 既定=root 単音

    @staticmethod
    def _attr(attrs, key):
        v = attrs.get(key)
        if v is None:
            return None
        try:
            return float(v)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _map(value, pcfg, key, default):
        """value(in_range) を out=[a,b] へ線形マップ。value/out 無しは default。"""
        out = (pcfg or {}).get(key)
        if value is None or out is None:
            return default
        in_range = (pcfg or {}).get("in_range", [0.0, 1.0])
        lo, hi = float(in_range[0]), float(in_range[1])
        frac = 0.0 if hi == lo else (float(value) - lo) / (hi - lo)
        frac = min(1.0, max(0.0, frac))
        return float(out[0]) + frac * (float(out[1]) - float(out[0]))
