#!/usr/bin/env bash
# ================================================
# Fluent Vision 一括起動スクリプト
#  - Foxglove Bridge 起動
#  - D405, D415 スタック起動
# ================================================
set -euo pipefail

# 強制: このマシンはドメインID=1 で運用
export ROS_DOMAIN_ID=1

WS_ROOT="/ros2_ws"
CONFIG_DIR="$WS_ROOT/config"

# ROS 2 環境
if [ -f "$WS_ROOT/install/setup.bash" ]; then
  : "${COLCON_TRACE:=}"
  set +u
  # shellcheck disable=SC1090
  source "$WS_ROOT/install/setup.bash"
  set -u
fi

info() { echo -e "[fv_start_all] $*"; }

# Foxglove Bridge 設定（必要に応じて調整）
FG_PORT=${FG_PORT:-8765}
FG_NODE_NAME=${FG_NODE_NAME:-foxglove_bridge}

start_foxglove() {
  if pgrep -a -f "ros2 run foxglove_bridge foxglove_bridge" >/dev/null 2>&1; then
    info "Foxglove Bridge は既に起動しています"
    return 0
  fi
  info "🦊 Starting Foxglove Bridge on port ${FG_PORT} ..."
  ros2 run foxglove_bridge foxglove_bridge \
    --ros-args -r __node:=${FG_NODE_NAME} \
    --ros-args -p port:=${FG_PORT} &
}

start_d405() {
  if [ -x "$WS_ROOT/fv_start_405.sh" ]; then
    "$WS_ROOT/fv_start_405.sh"
  else
    info "⚠️  fv_start_405.sh が見つかりません"
  fi
}

start_d415() {
  if [ -x "$WS_ROOT/fv_start_415.sh" ]; then
    "$WS_ROOT/fv_start_415.sh"
  else
    info "⚠️  fv_start_415.sh が見つかりません"
  fi
}

info "🚀 Starting Foxglove Bridge + D405 + D415..."
start_foxglove
start_d405
start_d415
info "✅ All started"
