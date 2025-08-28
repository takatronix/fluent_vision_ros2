
#!/usr/bin/env bash
# ================================================
# Fluent Vision D415 起動スクリプト（/ros2_ws/config を優先）
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
REALSENSE_YAML="${CONFIG_DIR}/fv_realsense_d415.yaml"
[ -f "$REALSENSE_YAML" ] || REALSENSE_YAML="${PKG_SCRIPT_DIR}/fv_realsense_d415.yaml"

OBJ_DET_YAML="${CONFIG_DIR}/fv_object_detector_d415.yaml"
[ -f "$OBJ_DET_YAML" ] || OBJ_DET_YAML="${PKG_SCRIPT_DIR}/fv_object_detector_d415.yaml"

MASK_GEN_YAML="${CONFIG_DIR}/fv_object_mask_generator_d415.yaml"
[ -f "$MASK_GEN_YAML" ] || MASK_GEN_YAML="${PKG_SCRIPT_DIR}/fv_object_mask_generator_d415.yaml"

ANALYZER_YAML="${CONFIG_DIR}/fv_aspara_analyzer_d415.yaml"
[ -f "$ANALYZER_YAML" ] || ANALYZER_YAML="${PKG_SCRIPT_DIR}/fv_aspara_analyzer_d415.yaml"

# Stem Detector 設定
STEM_YAML="${CONFIG_DIR}/fv_stem_detector_d415.yaml"
[ -f "$STEM_YAML" ] || STEM_YAML="${PKG_SCRIPT_DIR}/fv_stem_detector_d415.yaml"

info() { echo -e "[start_fv415] $*"; }

info "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

#######################################################################################
# カメラ系
#######################################################################################
info "📷 Starting RealSense D415 node... (params: $REALSENSE_YAML)"
ros2 run fv_realsense fv_realsense_node \
  --ros-args \
  --params-file "$REALSENSE_YAML" \
  -r __node:=fv_realsense_d415 &

## depth_image_proc は不要（fv_realsense が /fv/d415/registered_points を提供）

#######################################################################################
# 分析系
#######################################################################################
info "🎯 Starting Object Detector D415 node... (params: $OBJ_DET_YAML)"
ros2 run fv_object_detector fv_object_detector_node \
  --ros-args --params-file "$OBJ_DET_YAML" \
  -r __node:=fv_object_detector_d415 &

#info "🟢 Starting Object Mask Generator D415 node... (params: $MASK_GEN_YAML)"
#ros2 run fv_object_mask_generator fv_object_mask_generator_node \
#  --ros-args --params-file "$MASK_GEN_YAML" \
#  -r __node:=fv_object_mask_generator_d415 &

info "🌱 Starting Aspara Analyzer D415 node... (params: $ANALYZER_YAML)"
ros2 run fv_aspara_analyzer fv_aspara_analyzer_node \
  --ros-args \
  --params-file "$ANALYZER_YAML" \
  -r __node:=fv_aspara_analyzer_d415 &

info "🌿 Starting Stem Detector D415 node... (params: $STEM_YAML)"
ros2 run fv_stem_detector stem_detector_node \
  --ros-args --params-file "$STEM_YAML" \
  -r __node:=fv_stem_detector_d415 &

info "✅ Fluent Vision D415 nodes started!"
info "📊 Use 'ros2 node list' to check running nodes"
