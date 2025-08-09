#!/bin/bash
# ================================================
# Fluent Vision システム起動スクリプト（重要ポイント）
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
export RMW_FASTRTPS_USE_QOS_FROM_XML=1


# 注: スクリプト内での source は行いません（起動を速くするため）。
# 必要なら事前に環境を用意してください（例: `source install/setup.bash`）。
 source ../install/setup.bash


############################################
# vision系ノードの生き残りを削除
############################################
echo "🔧 Stopping any existing FV processes..."
"./stop_fv.sh"
sleep 1
pkill -9 -f "depth_image_proc" || true
pkill -9 -f "ros2 run depth_image_proc" || true
pkill -9 -f "foxglove_bridge" || true
sleep 1



#######################################################################################
# ここからvision系ノードの起動
#######################################################################################

# ================================================================= 
# [1] カメラノード Color/Depthの最初のソース （超重要）
# ================================================================= 

# -----------------------------------------------------------------
# [1.a] RealSense D415 ノード起動 
# -----------------------------------------------------------------
ros2 run fv_realsense fv_realsense_node \
    --ros-args \
    --params-file "fv_realsense_d415.yaml" \
    -r __node:=fv_realsense_d415 &

# ---------------------------------------
# D415が完全に起動してカメラを確保するまで待つ
# 同時に起動して電力問題で不安定になるのを防ぐ
# ---------------------------------------
echo "⏳ D415 起動待ち..."
for i in {1..15}; do
    if ros2 topic list | grep -q "/fv/d415/color/image_raw"; then
        echo "✅ D415 is ready!"
        echo "⏳ Additional 3 seconds wait to ensure device is fully released..."
        sleep 3  # 追加の3秒待機
        break
    fi
    echo "   D415 起動待ち... ($i/15)"
    sleep 1
done

# -----------------------------------------------------------------
# [1.b] RealSense D405 ノード起動
# -----------------------------------------------------------------
ros2 run fv_realsense fv_realsense_node \
    --ros-args \
    --params-file "fv_realsense_d405.yaml" \
    -r __node:=fv_realsense_d405 &

# -----------------------------------------------------------------
# depth_image_proc の起動
# 部分的ポイントクラウドの生成（点群処理が重たいためアスパラ領域だけの点群を作成)

# [データフロー]
# realsense(color)->objectet_detector(rect)->aspara_analyzer(rect)->depth_image_proc(cloud)->aspara_analyzer

# 以下のトピックが選択したアスパラの点群データを目標
# /fv/d405/registered_points
# /fv/d415/registered_points
# -----------------------------------------------------------------
# [2.a] D405 の depth_image_proc の起動
# -----------------------------------------------------------------
ros2 run depth_image_proc point_cloud_xyzrgb_node --ros-args \
  -r rgb/image_rect_color:=/fv/d405/color/image_raw \
  -r rgb/camera_info:=/fv/d405/color/camera_info \
  -r depth_registered/image_rect:=/fv/d405/depth/image_rect_raw \
  -r points:=/fv/d405/registered_points \
  -r __node:=depth_image_proc_d405 &
# -----------------------------------------------------------------
# [2.b] D415 の depth_image_proc の起動
# -----------------------------------------------------------------
ros2 run depth_image_proc point_cloud_xyzrgb_node --ros-args \
  -r rgb/image_rect_color:=/fv/d415/color/image_raw \
  -r rgb/camera_info:=/fv/d415/color/camera_info \
  -r depth_registered/image_rect:=/fv/d415/depth/image_rect_raw \
  -r points:=/fv/d415/registered_points \
  -r __node:=depth_image_proc_d415 &
  

#######################################################################################
# ここから分析系ノードの起動
#######################################################################################


# 物体検出 D415 ノード起動
echo "🔍 Starting Object Detector D415 node..."
echo "   Model: YOLOv10 (/models/v2_nano_best_fp16_dynamic.xml)"
ros2 run fv_object_detector fv_object_detector_node \
    --ros-args --params-file "fv_object_detector_d415.yaml" \
    -r __node:=fv_object_detector_d415 &

# 物体検出 D405 ノード起動
echo "🔍 Starting Object Detector D405 node..."
echo "   Model: YOLOv10 (/models/v2_nano_best_fp16_dynamic.xml)"
ros2 run fv_object_detector fv_object_detector_node \
    --ros-args --params-file "fv_object_detector_d405.yaml" \
    -r __node:=fv_object_detector_d405 &


# UNet セグメンテーション D415 ノード起動  (とりあえずまだ使わない)
#echo "🎭 Starting UNet Segmentation D415 node..."
#echo "   Model: UNet (/models/unet_asparagus_ch16_256_v1.0_ep20.xml)"
#ros2 launch fv_object_mask_generator fv_object_mask_generator_launch.py \
#    node_name:=fv_object_mask_generator_d415 \
#    config_file:="fv_object_mask_generator_d415.yaml" \
#    input_image_topic:="/fv/d415/color/image_raw" \
#    output_segmentation_mask_topic:="/fv/d415/segmentation_mask/image" \
#    output_colored_mask_topic:="/fv/d415/segmentation_mask/colored" &

# UNet セグメンテーション D405 ノード起動   (とりあえずまだ使わない)
#echo "🎭 Starting UNet Segmentation D405 node..."
#echo "   Model: UNet (/models/unet_asparagus_ch16_256_v1.0_ep20.xml)"
#ros2 launch fv_object_mask_generator fv_object_mask_generator_launch.py \
#    node_name:=fv_object_mask_generator_d405 \
#    config_file:="fv_object_mask_generator_d405.yaml" \
#    input_image_topic:="/fv/d405/color/image_raw" \
#    output_segmentation_mask_topic:="/fv/d405/segmentation_mask/image" \
#    output_colored_mask_topic:="/fv/d405/segmentation_mask/colored" &

# -----------------------------------------------------------------
# アスパラ分析ノード（重要）
# -----------------------------------------------------------------

# アスパラ分析 D415 ノード起動 （重要）
echo "D415 アスパラ分析ノード起動"
ros2 run fv_aspara_analyzer fv_aspara_analyzer_node \
    --ros-args \
    --params-file "fv_aspara_analyzer_d415.yaml" \
    -r __node:=fv_aspara_analyzer_d415 &

# アスパラ分析 D405 ノード起動 （重要）
echo "D405 アスパラ分析ノード起動"
ros2 run fv_aspara_analyzer fv_aspara_analyzer_node \
    --ros-args \
    --params-file "fv_aspara_analyzer_d405.yaml" \
    -r __node:=fv_aspara_analyzer_d405 &


# -----------------------------------------------------------------
# ここからテストやユーティリティノード（不要なら削除)
# -----------------------------------------------------------------

# Foxglove Bridge起動
echo "🦊 Starting Foxglove Bridge..."
ros2 run foxglove_bridge foxglove_bridge &

# レコーダー ノード起動（一時無効化 - パフォーマンス改善のため）
#echo "📹 Starting Recorder node..."
#ros2 launch fv_recorder fv_recorder_launch.py \
#    node_name:=fv_recorder \
#    config_file:="fv_recorder.yaml" &

# トピックリレー起動（/fv/* -> /vision_ai/* 転送） *とりあえずまだ使わない
#echo "🔄 Starting Topic Relay (/fv/* -> /vision_ai/*)..."
#echo "📁 Config file: .relay_vision_ai.yaml"
#ros2 run fv_topic_relay fv_topic_relay_node \
#    --ros-args \
#     --params-file "relay_vision_ai.yaml" &


# -----------------------------------------------------------------
# 完了
# -----------------------------------------------------------------
echo "✅ All Fluent Vision nodes started!"
echo "📊 Use 'ros2 node list' to check running nodes"
echo "🛑 Use './stop_fv.sh' to stop all nodes" 
