"""入力購読を需要駆動にする状態機械の契約。

このノードは「出力に購読者がいなければ推論しない」ゲートを元から持って
いたが、**購読は止めていなかった**。DDS は購読者ごとに別コピーを送るので、
購読しているだけで線に乗る (2026-08-23 aspa1 実測: 生color 573 + 生depth
387 = 960 Mbps を受信して捨て、CPU 62% を8日間)。

守るのは:
  1. 需要が無い間は購読を持たない (帯域と CPU を返す)。
  2. 上げ下げが冪等 (同じ状態を二度作らない/壊さない)。
  3. capture 要求は需要に数える (でないと購読ゼロから永遠にタイムアウト)。
  4. 破棄に失敗しても「生きている」と思い込まない (復帰不能を作らない)。

ROS には触らない。実機で ROS ノードを構築するテストは 2026-08-23 に
52 GB を食ってスタックを落とした前科がある。
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from fv_lingbot_depth_py.demand_gate import InputSubscriptionGate  # noqa: E402


@pytest.fixture
def gate():
    ev = {'start': 0, 'stop': 0, 'logs': []}
    g = InputSubscriptionGate(
        start=lambda: ev.__setitem__('start', ev['start'] + 1),
        stop=lambda: ev.__setitem__('stop', ev['stop'] + 1),
        log=ev['logs'].append)
    g.ev = ev
    return g


def test_no_subscription_until_demanded(gate):
    """需要が無い間は購読を持たない。これが 960 Mbps を返す本体。"""
    assert gate.active is False
    assert gate.reconcile(False) is False
    assert gate.ev['start'] == 0


def test_demand_brings_the_subscription_up(gate):
    assert gate.reconcile(True) is True
    assert gate.active is True
    assert gate.ev['start'] == 1


def test_bringing_up_is_idempotent(gate):
    gate.reconcile(True)
    assert gate.reconcile(True) is False      # 状態は変わらない
    assert gate.ev['start'] == 1              # 二重に作らない


def test_losing_demand_tears_the_subscription_down(gate):
    gate.reconcile(True)
    assert gate.reconcile(False) is True
    assert gate.active is False
    assert gate.ev['stop'] == 1


def test_tearing_down_is_idempotent(gate):
    gate.reconcile(True)
    gate.reconcile(False)
    assert gate.reconcile(False) is False
    assert gate.ev['stop'] == 1               # 二重に壊さない


def test_demand_can_come_back(gate):
    """レイヤを入れ直したら復帰すること (常駐モデルの意味が消えない)。"""
    gate.reconcile(True)
    gate.reconcile(False)
    gate.reconcile(True)
    assert gate.active is True
    assert (gate.starts, gate.stops) == (2, 1)


# --- 失敗時の扱い -------------------------------------------------------

def test_start_failure_leaves_it_down_for_retry():
    """作れなかったら「落ちたまま」。次の周期で再試行できること。"""
    calls = {'n': 0}

    def flaky_start():
        calls['n'] += 1
        if calls['n'] == 1:
            raise RuntimeError('topic not up yet')

    logs = []
    g = InputSubscriptionGate(start=flaky_start, stop=lambda: None,
                              log=logs.append)
    assert g.reconcile(True) is False
    assert g.active is False
    assert any('作れない' in m for m in logs)
    # 次の周期で成功する
    assert g.reconcile(True) is True
    assert g.active is True


def test_stop_failure_still_counts_as_stopped():
    """破棄で例外が出ても「生きている」と思い込まない。

    思い込むと、壊れた購読を掴んだまま二度と作り直せなくなる。
    """
    logs = []
    g = InputSubscriptionGate(
        start=lambda: None,
        stop=lambda: (_ for _ in ()).throw(RuntimeError('already destroyed')),
        log=logs.append)
    g.reconcile(True)
    assert g.reconcile(False) is True
    assert g.active is False
    assert any('破棄に失敗' in m for m in logs)
    # 復帰できる
    assert g.reconcile(True) is True


# --- シャットダウン -----------------------------------------------------

def test_force_stop_tears_down_regardless(gate):
    gate.reconcile(True)
    gate.force_stop()
    assert gate.active is False
    assert gate.ev['stop'] == 1


def test_force_stop_when_already_down_is_harmless(gate):
    gate.force_stop()
    assert gate.ev['stop'] == 0


# --- 上げ下げの回数を観測できること -------------------------------------

def test_flapping_is_countable(gate):
    """ばたつきを検出できるように回数を持つ。"""
    for _ in range(3):
        gate.reconcile(True)
        gate.reconcile(False)
    assert (gate.starts, gate.stops) == (3, 3)


# --- ノード側の結線 -----------------------------------------------------

def _node_src():
    path = os.path.join(os.path.dirname(__file__), '..',
                        'fv_lingbot_depth_py', 'lingbot_depth_node.py')
    with open(path, encoding='utf-8') as f:
        return f.read()


def test_node_does_not_subscribe_eagerly():
    """__init__ で購読を張らないこと (張ると需要ゼロでも 960 Mbps 流れる)。"""
    src = _node_src()
    init = src[src.index('    def __init__'):src.index('    # ---- 入力購読の需要駆動')]
    assert 'Subscriber(self, Image' not in init
    assert 'ApproximateTimeSynchronizer' not in init
    assert 'InputSubscriptionGate(' in init


def test_capture_counts_as_demand():
    """capture は購読ゼロから1枚だけ処理する口。需要に含めないと死ぬ。"""
    src = _node_src()
    body = src[src.index('def _input_demanded'):]
    body = body[:body.index('def _reconcile_input')]
    assert '_capture_pending' in body
    assert '_demanded()' in body


def test_capture_reconciles_immediately():
    """タイマ (既定1秒) を待つと capture_timeout_sec を無駄に食う。"""
    src = _node_src()
    body = src[src.index('def _on_capture'):]
    body = body[:body.index('def _on_synced_static_intr')]
    assert '_reconcile_input()' in body
    assert body.index('_reconcile_input()') < body.index('_capture_event.wait')


def test_teardown_drops_the_synchronizer_first():
    """購読だけ消して同期器を残すと、次の購読が古いキューの上に乗る。"""
    src = _node_src()
    body = src[src.index('def _stop_input_subs'):]
    body = body[:body.index('def _input_demanded')]
    assert body.index('self.sync = None') < body.index('destroy_subscription')
