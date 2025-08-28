#!/usr/bin/env bash
# ================================================
# Fluent Vision 設定リロードスクリプト（D405 / D415 対応）
# - RealSense(fv_realsense_*) と Analyzer(fv_aspara_analyzer_*) の
#   /reload_config を呼び出します
# - 使い方:
#     ./fv_reload_config.sh            # 両方（405,415）を順にリロード
#     ./fv_reload_config.sh 405        # D405 のみ
#     ./fv_reload_config.sh 415        # D415 のみ
# ================================================
set -euo pipefail

# 既定のROS_DOMAIN_ID（未設定なら1）
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"

WS_ROOT="/ros2_ws"

# ROS 2 環境
if [ -f "$WS_ROOT/install/setup.bash" ]; then
  : "${COLCON_TRACE:=}"
  set +u
  # shellcheck disable=SC1090
  source "$WS_ROOT/install/setup.bash"
  set -u
fi

info() { echo -e "[config_reload] $*"; }
err()  { echo -e "[config_reload][ERROR] $*" >&2; }

command -v ros2 >/dev/null 2>&1 || { err "ros2 が見つかりません。setup.bash を読み込んでください"; exit 1; }

info "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

has_service() {
  local srv="$1"
  # Avoid BrokenPipeError noise when piping to grep under 'set -o pipefail'
  set +o pipefail
  ros2 service list 2>/dev/null | grep -q -- "$srv"
  local found=$?
  set -o pipefail
  return $found
}

# Load parameters from YAML if the file exists, then report
param_load_if_exists() {
  local node_name="$1"
  local yaml_path="$2"
  if [ -f "$yaml_path" ]; then
    info "Loading params: $yaml_path -> $node_name"
    # Suppress verbose output but keep errors
    if ! ros2 param load "$node_name" "$yaml_path" >/dev/null; then
      err "param load 失敗: $yaml_path -> $node_name"
      return 1
    fi
  else
    info "YAMLが見つかりません（スキップ）: $yaml_path"
  fi
}

call_reload() {
  local srv="$1"
  if has_service "$srv"; then
    info "Calling: ros2 service call $srv std_srvs/srv/Trigger {}"
    ros2 service call "$srv" std_srvs/srv/Trigger "{}"
  else
    err "サービスが見つかりません: $srv （ノードが起動しているか確認してください）"
    return 1
  fi
}

reload_one() {
  local cam="$1"   # 405 or 415
  local REALSENSE_NODE="fv_realsense_d${cam}"
  local ANALYZER_NODE="fv_aspara_analyzer_d${cam}"
  local REALSENSE_SRV="/${REALSENSE_NODE}/reload_config"
  local ANALYZER_SRV="/${ANALYZER_NODE}/reload_config"

  local rc=0
  # 1) Load YAMLs into nodes (if present)
  param_load_if_exists "/${REALSENSE_NODE}" "/ros2_ws/config/fv_realsense_d${cam}.yaml" || true
  param_load_if_exists "/${ANALYZER_NODE}" "/ros2_ws/config/fv_aspara_analyzer_d${cam}.yaml" || true

  # 2) Trigger reload services
  call_reload "$REALSENSE_SRV" || rc=1
  call_reload "$ANALYZER_SRV"  || rc=1
  return $rc
}

TARGETS=()
case "${1:-both}" in
  405) TARGETS=(405) ;;
  415) TARGETS=(415) ;;
  both|all) TARGETS=(405 415) ;;
  *)
    err "不正な引数: ${1:-}  使用例: ./fv_reload_config.sh [405|415|both]"
    exit 2
    ;;
esac

RC=0
for cam in "${TARGETS[@]}"; do
  info "🔄 Reloading configs for D${cam}..."
  if reload_one "$cam"; then
    info "✅ D${cam} reload_config 完了"
  else
    err "D${cam} reload_config で一部失敗"
    RC=1
  fi
  echo
done

if [ $RC -eq 0 ]; then
  info "🎉 すべての reload_config 呼び出しが成功しました"
else
  err  "一部の reload_config 呼び出しに失敗しました"
fi

exit $RC
