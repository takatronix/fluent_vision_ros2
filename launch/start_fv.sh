#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS2 setup
if [ -f "/home/aspara/seedbox-r1/fluent_vision_ros2/install/setup.bash" ]; then
    source /home/aspara/seedbox-r1/fluent_vision_ros2/install/setup.bash
    echo "✅ Main ROS2 setup sourced"
else
    echo "⚠️  Warning: Main ROS2 setup not found. Make sure to source it manually."
fi

# Also source fv_realsense specific setup if available
if [ -f "/home/aspara/seedbox-r1/fluent_vision_ros2/src/sensors/fv_realsense/install/setup.bash" ]; then
    source /home/aspara/seedbox-r1/fluent_vision_ros2/src/sensors/fv_realsense/install/setup.bash
    echo "✅ FV RealSense setup sourced"
fi

echo "🚀 Starting Fluent Vision ROS2 nodes..."
echo "📁 Script directory: $SCRIPT_DIR"

# Use stop_fv.sh to clean up existing processes
echo "🔧 Stopping any existing FV processes..."
"$SCRIPT_DIR/stop_fv.sh"
sleep 1

# RealSense D415 ノード起動 (runコマンドで直接起動)
echo "📷 Starting RealSense D415 node..."
echo "📁 Config file: $SCRIPT_DIR/fv_realsense_d415.yaml"
ros2 run fv_realsense fv_realsense_node \
    --ros-args \
    --params-file "$SCRIPT_DIR/fv_realsense_d415.yaml" \
    -r __node:=fv_realsense_d415 &

# D415が完全に起動してカメラを確保するまで待つ
echo "⏳ Waiting for D415 to initialize..."
for i in {1..15}; do
    if ros2 topic list | grep -q "/fv/d415/color/image_raw"; then
        echo "✅ D415 is ready!"
        echo "⏳ Additional 3 seconds wait to ensure device is fully released..."
        sleep 3  # 追加の3秒待機
        break
    fi
    echo "   Waiting... ($i/15)"
    sleep 1
done

# RealSense D405 ノード起動 (runコマンドで直接起動)
echo "📷 Starting RealSense D405 node..."
echo "📁 Config file: $SCRIPT_DIR/fv_realsense_d405.yaml"
ros2 run fv_realsense fv_realsense_node \
    --ros-args \
    --params-file "$SCRIPT_DIR/fv_realsense_d405.yaml" \
    -r __node:=fv_realsense_d405 &

# 物体検出 D415 ノード起動
echo "🔍 Starting Object Detector D415 node..."
echo "   Model: YOLOv10 (/models/v2_nano_best_fp16_dynamic.xml)"
ros2 launch fv_object_detector fv_object_detector_launch.py \
    node_name:=fv_object_detector_d415 \
    config_file:="$SCRIPT_DIR/fv_object_detector_d415.yaml" \
    input_topic:="/fv/d415/color/image_raw" \
    output_image_topic:="/fv/d415/object_detection/annotated_image" \
    output_detections_topic:="/fv/d415/object_detection/detections" &

# 物体検出 D405 ノード起動
echo "🔍 Starting Object Detector D405 node..."
echo "   Model: YOLOv10 (/models/v2_nano_best_fp16_dynamic.xml)"
ros2 launch fv_object_detector fv_object_detector_launch.py \
    node_name:=fv_object_detector_d405 \
    config_file:="$SCRIPT_DIR/fv_object_detector_d405.yaml" \
    input_topic:="/fv/d405/color/image_raw" \
    output_image_topic:="/fv/d405/object_detection/annotated_image" \
    output_detections_topic:="/fv/d405/object_detection/detections" &

# UNet セグメンテーション D415 ノード起動
echo "🎭 Starting UNet Segmentation D415 node..."
echo "   Model: UNet (/models/unet_asparagus_ch16_256_v1.0_ep20.xml)"
ros2 launch fv_object_mask_generator fv_object_mask_generator_launch.py \
    node_name:=fv_object_mask_generator_d415 \
    config_file:="$SCRIPT_DIR/fv_object_mask_generator_d415.yaml" \
    input_image_topic:="/fv/d415/color/image_raw" \
    output_segmentation_mask_topic:="/fv/d415/segmentation_mask/image" \
    output_colored_mask_topic:="/fv/d415/segmentation_mask/colored" &

# UNet セグメンテーション D405 ノード起動
echo "🎭 Starting UNet Segmentation D405 node..."
echo "   Model: UNet (/models/unet_asparagus_ch16_256_v1.0_ep20.xml)"
ros2 launch fv_object_mask_generator fv_object_mask_generator_launch.py \
    node_name:=fv_object_mask_generator_d405 \
    config_file:="$SCRIPT_DIR/fv_object_mask_generator_d405.yaml" \
    input_image_topic:="/fv/d405/color/image_raw" \
    output_segmentation_mask_topic:="/fv/d405/segmentation_mask/image" \
    output_colored_mask_topic:="/fv/d405/segmentation_mask/colored" &

# レコーダー ノード起動
echo "📹 Starting Recorder node..."
ros2 launch fv_recorder fv_recorder_launch.py \
    node_name:=fv_recorder \
    config_file:="$SCRIPT_DIR/fv_recorder.yaml" &

echo "✅ All Fluent Vision nodes started!"
echo "📊 Use 'ros2 node list' to check running nodes"
echo "🛑 Use './stop_fv.sh' to stop all nodes" 