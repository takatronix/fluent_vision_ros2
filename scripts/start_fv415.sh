#!/bin/bash
# ================================================
# Fluent Vision D415 システム起動スクリプト
# ================================================
# depth_image_procがない場合は、
# sudo apt-get install -y ros-$ROS_DISTRO-depth-image-proc
# を実行してください。
# ================================================

# -----------------------------------------------------------------
# [0] 前準備フェーズ
# -----------------------------------------------------------------

# 共有メモリ転送を有効化（高速画像転送）
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=../fastdds_shared_memory.xml
# QoSはコード/YAMLの設定を使う（XMLの上書きを無効化）
export RMW_FASTRTPS_USE_QOS_FROM_XML=0

# 注: スクリプト内での source は行いません（起動を速くするため）。
# 必要なら事前に環境を用意してください（例: `source install/setup.bash`）。
source ../install/setup.bash

############################################
# vision系ノードの生き残りを削除
############################################
echo "🔧 Stopping any existing FV D415 processes..."
pkill -9 -f "fv_realsense_d415" || true
pkill -9 -f "depth_image_proc_d415" || true
pkill -9 -f "fv_object_detector_d415" || true
pkill -9 -f "fv_aspara_analyzer_d415" || true
sleep 1

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
    --params-file "fv_realsense_d415.yaml" \
    -r __node:=fv_realsense_d415 &


# -----------------------------------------------------------------
# [2] depth_image_proc の起動
# 部分的ポイントクラウドの生成（点群処理が重たいためアスパラ領域だけの点群を作成)
# -----------------------------------------------------------------
echo "☁️ Starting depth_image_proc for D415..."
ros2 run depth_image_proc point_cloud_xyzrgb_node --ros-args \
  -p use_sensor_data_qos:=true \
  -p qos_overrides./rgb/image_rect_color.reliability:=best_effort \
  -p qos_overrides./rgb/camera_info.reliability:=best_effort \
  -p qos_overrides./depth_registered/image_rect.reliability:=best_effort \
  -r rgb/image_rect_color:=/fv/d415/color/image_raw \
  -r rgb/camera_info:=/fv/d415/color/camera_info \
  -r depth_registered/image_rect:=/fv/d415/depth/image_rect_raw \
  -r points:=/fv/d415/registered_points \
  -r __node:=depth_image_proc_d415 &

#######################################################################################
# ここから分析系ノードの起動
#######################################################################################

# -----------------------------------------------------------------
# [3] 物体検出 D415 ノード起動
# -----------------------------------------------------------------
echo "🎯 Starting Object Detector D415 node..."
ros2 run fv_object_detector fv_object_detector_node \
    --ros-args --params-file "fv_object_detector_d415.yaml" \
    -r __node:=fv_object_detector_d415 &

# -----------------------------------------------------------------
# [4] アスパラ分析ノード（重要）
# -----------------------------------------------------------------
echo "🌱 Starting Aspara Analyzer D415 node..."
ros2 run fv_aspara_analyzer fv_aspara_analyzer_node \
    --ros-args \
    --params-file "fv_aspara_analyzer_d415.yaml" \
    -r __node:=fv_aspara_analyzer_d415 &

# -----------------------------------------------------------------
# 完了
# -----------------------------------------------------------------
echo "✅ Fluent Vision D415 nodes started!"
echo "📊 Use 'ros2 node list' to check running nodes"
echo "🛑 Use './stop_fv.sh' to stop all nodes"