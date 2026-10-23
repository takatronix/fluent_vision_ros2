"""demand_gate — 入力購読そのものを需要駆動にする小さな状態機械。

## なぜ要るか

このノードは既に「出力に購読者がいなければ推論しない」ゲートを持っている
(`_demanded`)。しかし**購読は止めていなかった**。結果として、誰も refined
depth を読んでいない間も入力を受け取り続ける:

    2026-08-23 実測 (aspa1):
      出力 /d405_depth_refined/* の購読者 : 0
      入力の購読 : 生color 573 Mbps + 生depth 387 Mbps = 960 Mbps
      CPU        : 62.3% (8日間連続)

DDS は購読者ごとに別コピーを送るので、購読しているだけで線に 960 Mbps 乗る。
受信とデシリアライズだけで CPU を1コア近く使い、その全部を捨てていた。
ゲートが GPU は守ったが、帯域と CPU は守っていなかった。

## 規律

  * 需要 = 出力に購読者がいる、**または** capture 要求が来ている。
    capture は購読が無い状態から1枚だけ処理する口なので、需要に含めないと
    永遠にタイムアウトする。
  * 立ち上げ/落としは冪等。同じ状態を二度作らない、二度壊さない。
  * 落とす方向で失敗しても状態は「落ちた」にする — 壊れた購読を掴んだまま
    「生きている」と思い込むと、二度と作り直せなくなる。

ROS に依存しない。購読の生成と破棄を関数として受け取り、状態遷移だけを持つ。
実機で ROS ノードを構築するテストは 2026-08-23 に 52 GB を食って
スタックを落とした前科があるので、ここは単体で試験できる形にしてある。
"""

from __future__ import annotations

from typing import Callable, Optional


class InputSubscriptionGate:
    """入力購読の生死を需要に追従させる。

    `start` は購読を作る関数、`stop` は捨てる関数。どちらも例外を投げてよい
    (`reconcile` が状態を壊さずに扱う)。
    """

    def __init__(self, start: Callable[[], None], stop: Callable[[], None],
                 log: Optional[Callable[[str], None]] = None) -> None:
        self._start = start
        self._stop = stop
        self._log = log or (lambda _m: None)
        self._active = False
        #: 観測用。何回上げ下げしたか (ばたつきの検出に使う)
        self.starts = 0
        self.stops = 0

    @property
    def active(self) -> bool:
        return self._active

    def reconcile(self, demanded: bool) -> bool:
        """需要に合わせて購読を上げ下げする。状態が変わったら True。"""
        if demanded and not self._active:
            try:
                self._start()
            except Exception as exc:            # noqa: BLE001
                # 作れなかった: 状態は「落ちたまま」。次の周期で再試行する。
                self._log(f"入力購読を作れない: {exc}")
                return False
            self._active = True
            self.starts += 1
            self._log("入力購読を開始 (出力に購読者が現れた)")
            return True
        if not demanded and self._active:
            # 先に状態を落としてから壊す。逆にすると、破棄で例外が出たときに
            # 「生きていることになっている壊れた購読」が残って復帰できない。
            self._active = False
            self.stops += 1
            try:
                self._stop()
            except Exception as exc:            # noqa: BLE001
                self._log(f"入力購読の破棄に失敗 (状態は停止として扱う): {exc}")
            self._log("入力購読を停止 (出力に購読者がいない — 帯域とCPUを返す)")
            return True
        return False

    def force_stop(self) -> None:
        """シャットダウン用。需要に関わらず落とす。"""
        self.reconcile(False)
