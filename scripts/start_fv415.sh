#!/bin/bash
# ================================================
# Fluent Vision D415 システム起動スクリプト
# ================================================
# 注: organizedなregistered_pointsはfv_realsenseが直接Publishするため
# depth_image_procは不要になりました。
# ================================================

# -----------------------------------------------------------------
# [0] 前準備フェーズ
# -----------------------------------------------------------------

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)"


# 環境の読み込み（スクリプトの位置からWS_ROOTを解決）
if [ -f "$WS_ROOT/install/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "$WS_ROOT/install/setup.bash"
fi


#######################################################################################
# ここからvision系ノードの起動
#######################################################################################

# ================================================================= 
# [1] カメラノード Color/Depthの最初のソース （超重要）
# ================================================================= 

# -----------------------------------------------------------------
# [1] RealSense D415 ノード起動 
# -----------------------------------------------------------------
echo "📷 Starting RealSense D415 node..."
ros2 run fv_realsense fv_realsense_node \
    --ros-args \
    --params-file "$WS_ROOT/scripts/fv_realsense_d415.yaml" \
    -r __node:=fv_realsense_d415 &


## depth_image_proc 起動は不要（fv_realsenseが /fv/d415/registered_points を提供）

#######################################################################################
# ここから分析系ノードの起動
#######################################################################################

# -----------------------------------------------------------------
# [3] 物体検出 D415 ノード起動
# -----------------------------------------------------------------
echo "🎯 Starting Object Detector D415 node..."
ros2 run fv_object_detector fv_object_detector_node \
    --ros-args --params-file "$WS_ROOT/scripts/fv_object_detector_d415.yaml" \
    -r __node:=fv_object_detector_d415 &

# -----------------------------------------------------------------
# [3b] セグメンテーション（マスク生成）D415 ノード起動
# -----------------------------------------------------------------
echo "🟢 Starting Object Mask Generator D415 node..."
ros2 run fv_object_mask_generator fv_object_mask_generator_node \
    --ros-args --params-file "$WS_ROOT/scripts/fv_object_mask_generator_d415.yaml" \
    -r __node:=fv_object_mask_generator_d415 &

# -----------------------------------------------------------------
# [4] アスパラ分析ノード（重要）
# -----------------------------------------------------------------
echo "🌱 Starting Aspara Analyzer D415 node..."
ros2 run fv_aspara_analyzer fv_aspara_analyzer_node \
    --ros-args \
    --params-file "$WS_ROOT/scripts/fv_aspara_analyzer_d415.yaml" \
    -r __node:=fv_aspara_analyzer_d415 &

# -----------------------------------------------------------------
# 完了
# -----------------------------------------------------------------
echo "✅ Fluent Vision D415 nodes started!"
echo "📊 Use 'ros2 node list' to check running nodes"
echo "🛑 Use './stop_fv.sh' to stop all nodes"