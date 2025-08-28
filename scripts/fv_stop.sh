#!/usr/bin/env bash
# ================================================
# Fluent Vision 一括停止スクリプト
#  - D415, D405, Foxglove Bridge の順に停止
# ================================================
set -euo pipefail

# 強制: このマシンはドメインID=1 で運用
export ROS_DOMAIN_ID=1

WS_ROOT="/ros2_ws"

# ROS 2 環境
if [ -f "$WS_ROOT/install/setup.bash" ]; then
  : "${COLCON_TRACE:=}"
  set +u
  # shellcheck disable=SC1090
  source "$WS_ROOT/install/setup.bash"
  set -u
fi

info() { echo -e "[fv_stop_all] $*"; }

stop_foxglove() {
  info "🦊 Stopping Foxglove Bridge..."
  # 広めのパターンで検出
  local PATTERN='(ros2 run foxglove_bridge foxglove_bridge|__node:=foxglove_bridge|(^|/)foxglove_bridge( |$))'

  # TERM を送信
  pkill -f "$PATTERN" >/dev/null 2>&1 || true
  sleep 1

  # 最大5秒待機しつつ、2秒目で INT、最後に KILL
  for i in {1..5}; do
    local REMAIN
    REMAIN=$(pgrep -a -f "$PATTERN" || true)
    if [ -z "$REMAIN" ]; then
      break
    fi
    info "Foxglove still running, wait...\n$REMAIN"
    sleep 1
    if [ "$i" -eq 2 ]; then
      pkill -2 -f "$PATTERN" >/dev/null 2>&1 || true
    fi
  done

  # まだ残っていれば KILL
  if pgrep -a -f "$PATTERN" >/dev/null 2>&1; then
    info "🛑 Forcing KILL to Foxglove Bridge..."
    pkill -9 -f "$PATTERN" >/dev/null 2>&1 || true
    sleep 1
  fi
}

stop_d405() {
  if [ -x "$WS_ROOT/fv_stop_405.sh" ]; then
    "$WS_ROOT/fv_stop_405.sh"
  else
    info "⚠️  fv_stop_405.sh が見つかりません"
  fi
}

stop_d415() {
  if [ -x "$WS_ROOT/fv_stop_415.sh" ]; then
    "$WS_ROOT/fv_stop_415.sh"
  else
    info "⚠️  fv_stop_415.sh が見つかりません"
  fi
}

info "🛑 Stopping D415, D405, Foxglove Bridge..."
stop_d415
stop_d405
stop_foxglove
info "✅ All stopped"
