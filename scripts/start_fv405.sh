#!/bin/bash
# ================================================
# Fluent Vision D405 システム起動スクリプト
# ================================================
# depth_image_procがない場合は、
# sudo apt-get install -y ros-$ROS_DISTRO-depth-image-proc
# を実行してください。
# ================================================

# -----------------------------------------------------------------
# [0] 前準備フェーズ
# -----------------------------------------------------------------

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)"

# 共有メモリ転送を有効化（高速画像転送）
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$WS_ROOT/fastdds_shared_memory.xml"
# QoSはコード/YAMLの設定を使う（XMLの上書きを無効化）
export RMW_FASTRTPS_USE_QOS_FROM_XML=0

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
# [1] RealSense D405 ノード起動
# -----------------------------------------------------------------
echo "📷 Starting RealSense D405 node..."
ros2 run fv_realsense fv_realsense_node \
    --ros-args \
    --params-file "$WS_ROOT/scripts/fv_realsense_d405.yaml" \
    -r __node:=fv_realsense_d405 &


# -----------------------------------------------------------------
# [2] depth_image_proc の起動
# 部分的ポイントクラウドの生成（点群処理が重たいためアスパラ領域だけの点群を作成)
# -----------------------------------------------------------------
echo "☁️ Starting depth_image_proc for D405..."
ros2 run depth_image_proc point_cloud_xyzrgb_node --ros-args \
  -p use_sensor_data_qos:=true \
  -p qos_overrides./rgb/image_rect_color.reliability:=best_effort \
  -p qos_overrides./rgb/camera_info.reliability:=best_effort \
  -p qos_overrides./depth_registered/image_rect.reliability:=best_effort \
  -r rgb/image_rect_color:=/fv/d405/color/image_raw \
  -r rgb/camera_info:=/fv/d405/color/camera_info \
  -r depth_registered/image_rect:=/fv/d405/depth/image_rect_raw \
  -r points:=/fv/d405/registered_points \
  -r __node:=depth_image_proc_d405 &

#######################################################################################
# ここから分析系ノードの起動
#######################################################################################

# -----------------------------------------------------------------
# [3] 物体検出 D405 ノード起動
# -----------------------------------------------------------------
echo "🎯 Starting Object Detector D405 node..."
ros2 run fv_object_detector fv_object_detector_node \
    --ros-args --params-file "$WS_ROOT/scripts/fv_object_detector_d405.yaml" \
    -r __node:=fv_object_detector_d405 &

# -----------------------------------------------------------------
# [4] アスパラ分析ノード（重要）
# -----------------------------------------------------------------
echo "🌱 Starting Aspara Analyzer D405 node..."
ros2 run fv_aspara_analyzer fv_aspara_analyzer_node \
    --ros-args \
    --params-file "$WS_ROOT/scripts/fv_aspara_analyzer_d405.yaml" \
    -r __node:=fv_aspara_analyzer_d405 &

# -----------------------------------------------------------------
# 完了
# -----------------------------------------------------------------
echo "✅ Fluent Vision D405 nodes started!"
echo "📊 Use 'ros2 node list' to check running nodes"
echo "🛑 Use './stop_fv.sh' to stop all nodes"