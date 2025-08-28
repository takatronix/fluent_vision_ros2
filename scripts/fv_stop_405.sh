#!/usr/bin/env bash
# ================================================
# Fluent Vision D405 停止スクリプト
# ================================================
set -euo pipefail

# 強制: このマシンはドメインID=1 で運用
export ROS_DOMAIN_ID=1

WS_ROOT="/ros2_ws"

# ROS 2 環境
if [ -f "$WS_ROOT/install/setup.bash" ]; then
  # nounset下での未定義変数エラー回避
  : "${COLCON_TRACE:=}"
  set +u
  # shellcheck disable=SC1090
  source "$WS_ROOT/install/setup.bash"
  set -u
fi

info() { echo -e "[stop_fv405] $*"; }

# 対象プロセスのパターン（できるだけ限定的に）
PATTERN='(fv_realsense_node|fv_object_detector_node|fv_object_mask_generator_node|fv_aspara_analyzer_node|stem_detector_node)'

# 穏やかな停止
info "📦 Sending TERM to D405 processes..."
pkill -f "$PATTERN" >/dev/null 2>&1 || true
sleep 1

# 残存確認（最大5秒待機）
for i in {1..5}; do
  REMAIN=$(pgrep -a -f "$PATTERN" || true)
  if [ -z "$REMAIN" ]; then
    break
  fi
  info "待機中... まだ残っているプロセス:\n$REMAIN"
  sleep 1
  # 2秒目で一度 SIGINT を試みる（python系に効く可能性）
  if [ "$i" -eq 2 ]; then
    pkill -2 -f "$PATTERN" >/dev/null 2>&1 || true
  fi
 done

# まだ残っていれば強制終了
REMAIN=$(pgrep -a -f "$PATTERN" || true)
if [ -n "$REMAIN" ]; then
  info "🛑 Forcing KILL to remaining processes..."
  pkill -9 -f "$PATTERN" >/dev/null 2>&1 || true
  sleep 1
fi

# ノードが消えているか確認（ベストエフォート）
info "🔎 Checking ROS nodes..."
ROS_NODES=$(ros2 node list 2>/dev/null || true)
if echo "$ROS_NODES" | grep -qE '(fv_realsense_d405|fv_object_detector_d405|fv_object_mask_generator_d405|fv_aspara_analyzer_d405|fv_stem_detector_d405)'; then
  info "注意: ROSグラフにノードがまだ見えます（別ユーザ/別マシンの可能性）。"
else
  info "✅ D405-related nodes are no longer visible."
fi

info "完了"
