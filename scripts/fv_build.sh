#!/bin/bash
# Fluent Vision ROS2パッケージ専用ビルドスクリプト
# 使用方法: ./build_fv.sh
# 注意: fluent_libを最優先でビルドし、その後fv_*パッケージ群をビルド

set -euo pipefail

cd /ros2_ws

# build.shと同じ環境設定を使用
# Python 3.10用のPYTHONPATHを設定
export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages

echo "[build_fv] Building fluent_vision packages..."

# 1) fluent_lib を最優先でビルド
echo "[build_fv] Step 1: Building fluent_lib (core library)"
/usr/bin/python3.10 -m colcon build --packages-select fluent_lib

# 2) fv_* パッケージ群を一括ビルド
echo "[build_fv] Step 2: Building all fv_* packages"
/usr/bin/python3.10 -m colcon build \
  --packages-select \
    fv_realsense \
    fv_camera \
    fv_object_detector \
    fv_object_mask_generator \
    fv_face_recognizer \
    fv_aspara_analyzer \
    fv_image_distributor \
    fv_mjpeg_server \
    fv_rtmp_server \
    fv_websocket_server \
    fv_recorder \
    fv_topic_relay \
    fv_stem_detector \
    aspara_vision_ai

echo "[build_fv] All fluent_vision packages built successfully!"
echo "[build_fv] You can now run: source install/setup.bash"
