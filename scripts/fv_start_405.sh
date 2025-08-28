#!/usr/bin/env bash
# ================================================
# Fluent Vision D405 起動スクリプト（/ros2_ws/config を優先）
# ================================================
set -euo pipefail

# 強制: このマシンはドメインID=1 で運用
export ROS_DOMAIN_ID=1

WS_ROOT="/ros2_ws"
CONFIG_DIR="$WS_ROOT/config"
PKG_SCRIPT_DIR="$WS_ROOT/src/fluent_vision_ros2/scripts"

# ROS 2 環境
if [ -f "$WS_ROOT/install/setup.bash" ]; then
  # nounset下での未定義変数エラー回避（setup.bashがCOLCON_TRACEを参照する場合がある）
  : "${COLCON_TRACE:=}"
  set +u
  # shellcheck disable=SC1090
  source "$WS_ROOT/install/setup.bash"
  set -u
fi

# 利用するパラメータファイル（存在しなければ scripts 側をフォールバック）
REALSENSE_YAML="${CONFIG_DIR}/fv_realsense_d405.yaml"
[ -f "$REALSENSE_YAML" ] || REALSENSE_YAML="${PKG_SCRIPT_DIR}/fv_realsense_d405.yaml"

OBJ_DET_YAML="${CONFIG_DIR}/fv_object_detector_d405.yaml"
[ -f "$OBJ_DET_YAML" ] || OBJ_DET_YAML="${PKG_SCRIPT_DIR}/fv_object_detector_d405.yaml"

MASK_GEN_YAML="${CONFIG_DIR}/fv_object_mask_generator_d405.yaml"
[ -f "$MASK_GEN_YAML" ] || MASK_GEN_YAML="${PKG_SCRIPT_DIR}/fv_object_mask_generator_d405.yaml"

ANALYZER_YAML="${CONFIG_DIR}/fv_aspara_analyzer_d405.yaml"
[ -f "$ANALYZER_YAML" ] || ANALYZER_YAML="${PKG_SCRIPT_DIR}/fv_aspara_analyzer_d405.yaml"

# Stem Detector 設定
STEM_YAML="${CONFIG_DIR}/fv_stem_detector_d405.yaml"
[ -f "$STEM_YAML" ] || STEM_YAML="${PKG_SCRIPT_DIR}/fv_stem_detector_d405.yaml"

info() { echo -e "[start_fv405] $*"; }

info "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

#######################################################################################
# カメラ系
#######################################################################################
info "📷 Starting RealSense D405 node... (params: $REALSENSE_YAML)"
ros2 run fv_realsense fv_realsense_node \
  --ros-args \
  --params-file "$REALSENSE_YAML" \
  -r __node:=fv_realsense_d405 &

## depth_image_proc は不要（fv_realsense が /fv/d405/registered_points を提供）

# RealSenseのcamera_infoが出るまで待機（最大10秒）
wait_sec=10
info "⏳ Waiting up to ${wait_sec}s for /fv/d405/color/camera_info ..."
(
  set +e
  for i in $(seq 1 ${wait_sec}); do
    if ros2 topic list | grep -q "^/fv/d405/color/camera_info$"; then
      echo "[start_fv405] ✅ camera_info detected"
      exit 0
    fi
    sleep 1
  done
  echo "[start_fv405] ⚠️ camera_info not detected within ${wait_sec}s (continuing)"
) >/dev/null 2>&1

#######################################################################################
# 分析系
#######################################################################################
info "🎯 Starting Object Detector D405 node... (params: $OBJ_DET_YAML)"
ros2 run fv_object_detector fv_object_detector_node \
  --ros-args --params-file "$OBJ_DET_YAML" \
  -r __node:=fv_object_detector_d405 &

#info "🟢 Starting Object Mask Generator D405 node... (params: $MASK_GEN_YAML)"
#ros2 run fv_object_mask_generator fv_object_mask_generator_node \
#  --ros-args --params-file "$MASK_GEN_YAML" \
#  -r __node:=fv_object_mask_generator_d405 &


info "🌿 Starting Stem Detector D405 node... (params: $STEM_YAML)"
ros2 run fv_stem_detector stem_detector_node \
  --ros-args --params-file "$STEM_YAML" \
  -r __node:=fv_stem_detector_d405 &

info "🌱 Starting Aspara Analyzer D405 node... (params: $ANALYZER_YAML)"
ros2 run fv_aspara_analyzer fv_aspara_analyzer_node \
  --ros-args \
  --params-file "$ANALYZER_YAML" \
  -r __node:=fv_aspara_analyzer_d405 &



info "✅ Fluent Vision D405 nodes started!"
info "📊 Use 'ros2 node list' to check running nodes"
