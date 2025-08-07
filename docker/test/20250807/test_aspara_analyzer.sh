#!/bin/bash
# =============================================================================
# アスパラガスアナライザー実行テストスクリプト
# =============================================================================

echo "===== アスパラガスアナライザー実行テスト ====="
echo "日時: $(date)"
echo ""

# ROS2環境のセットアップ
source /opt/ros/humble/setup.bash
source /home/aspara/seedbox-r1/fluent_vision_ros2/install/setup.bash

# 1. ノードリスト確認
echo "===== 1. 実行可能ノード確認 ====="
ros2 pkg executables fv_aspara_analyzer
echo ""

# 2. パラメータファイル確認
echo "===== 2. 設定ファイル確認 ====="
if [ -f "/home/aspara/seedbox-r1/fluent_vision_ros2/launch/fv_aspara_analyzer_d415.yaml" ]; then
    echo "✓ D415設定ファイル: 存在"
else
    echo "✗ D415設定ファイル: 不在"
fi
echo ""

# 3. トピックリスト確認（現在配信中のトピック）
echo "===== 3. 現在のトピックリスト ====="
ros2 topic list | grep -E "(aspara|d415|d405)" || echo "関連トピックなし"
echo ""

# 4. 依存トピックの確認
echo "===== 4. 必要なトピックの状態 ====="
REQUIRED_TOPICS=(
    "/fv/d415/object_detection/detections"
    "/fv/d415/asparagus/points"
    "/fv/d415/color/camera_info"
    "/fv/d415/color/image_raw"
    "/fv/d415/segmentation_mask/image"
)

for topic in "${REQUIRED_TOPICS[@]}"; do
    if ros2 topic list | grep -q "$topic"; then
        echo "✓ $topic: 配信中"
    else
        echo "✗ $topic: 未配信"
    fi
done
echo ""

# 5. ノード起動テスト（ドライラン）
echo "===== 5. ノード起動テスト（5秒間） ====="
echo "起動コマンド:"
echo "ros2 run fv_aspara_analyzer fv_aspara_analyzer_node \\"
echo "    --ros-args \\"
echo "    --params-file /home/aspara/seedbox-r1/fluent_vision_ros2/launch/fv_aspara_analyzer_d415.yaml"
echo ""

# タイムアウト付き起動テスト
timeout 5 ros2 run fv_aspara_analyzer fv_aspara_analyzer_node \
    --ros-args \
    --params-file /home/aspara/seedbox-r1/fluent_vision_ros2/launch/fv_aspara_analyzer_d415.yaml \
    2>&1 | head -20

echo ""
echo "===== 6. 代替ノード確認 ====="
echo "simple_pointcloud_generator:"
timeout 2 ros2 run fv_aspara_analyzer simple_pointcloud_generator 2>&1 | head -10

echo ""
echo "===== テスト完了 ====="
echo "問題がなければ、必要なトピックを配信してから本実行してください"