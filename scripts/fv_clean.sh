#!/bin/bash
# Fluent Vision ROS2パッケージ専用クリーンビルドスクリプト
# 使用方法: ./clean_fv.sh
# 注意: fv_*パッケージのビルド成果物のみを削除してから再ビルド

set -euo pipefail

cd /ros2_ws

echo "[clean_fv] Cleaning fv_* packages..."

# fv_*パッケージのビルド成果物のみを削除
echo "[clean_fv] Removing build artifacts..."
rm -rf build/fv_*
rm -rf install/fv_*

echo "[clean_fv] Clean complete. Starting fresh build..."

# build.shと同じ環境設定を使用してクリーンビルド実行
export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages
./fv_build.sh

echo "[clean_fv] Clean build completed successfully!"
