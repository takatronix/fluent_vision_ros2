#!/bin/bash

# =============================================================================
# Fluent Vision - D415カメラ用アスパラガス解析システム起動スクリプト
# =============================================================================

echo "===== D415カメラ用アスパラガス解析システム起動 ====="
echo ""

# カラー出力設定
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# 環境設定
source /home/aspara/seedbox-r1/fluent_vision_ros2/install/setup.bash

# プロセス管理用変数
PIDS=()

# 終了処理
cleanup() {
    echo -e "\n${YELLOW}システム停止中...${NC}"
    for pid in "${PIDS[@]}"; do
        if kill -0 $pid 2>/dev/null; then
            kill $pid
        fi
    done
    echo -e "${GREEN}システムを停止しました${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM

# 0. Foxglove Bridge起動
echo -e "${GREEN}[0/5] Foxglove Bridge起動中...${NC}"
ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
PIDS+=($!)
sleep 1

# 1. RealSense D415起動
echo -e "${GREEN}[1/5] RealSense D415起動中...${NC}"
ros2 launch fv_realsense fv_realsense_launch.py \
    config_file:=/home/aspara/seedbox-r1/fluent_vision_ros2/launch/fv_realsense_d415_optimized.yaml \
    camera_namespace:=fv/d415 &
PIDS+=($!)
sleep 3

# 2. YOLO物体検出起動
echo -e "${GREEN}[2/5] YOLO物体検出起動中...${NC}"
ros2 run fv_object_detector fv_object_detector_node \
    --ros-args \
    --params-file /home/aspara/seedbox-r1/fluent_vision_ros2/launch/fv_object_detector_d415.yaml &
PIDS+=($!)
sleep 2

# 3. 点群生成ノード起動
echo -e "${GREEN}[3/5] 点群生成ノード起動中...${NC}"
ros2 run fv_aspara_analyzer simple_pointcloud_generator \
    --ros-args \
    -r __ns:=/fv/d415 &
PIDS+=($!)
sleep 1

# 4. アスパラガス解析ノード起動
echo -e "${GREEN}[4/5] アスパラガス解析ノード起動中...${NC}"
ros2 run fv_aspara_analyzer fv_aspara_analyzer_node \
    --ros-args \
    --params-file /home/aspara/seedbox-r1/fluent_vision_ros2/launch/fv_aspara_analyzer_d415.yaml &
PIDS+=($!)

echo ""
echo -e "${GREEN}===== システム起動完了 =====${NC}"
echo ""
echo "トピック一覧:"
echo "  入力:"
echo "    - /fv/d415/color/image_raw       (カラー画像)"
echo "    - /fv/d415/depth/image_rect_raw  (深度画像)"
echo "    - /fv/d415/object_detection/detections (検出結果)"
echo ""
echo "  出力:"
echo "    - /asparagus/points              (点群)"
echo "    - /fv/d415/aspara_analysis/result (解析結果)"
echo ""
echo "停止: Ctrl+C"
echo ""

# プロセス監視
while true; do
    sleep 5
    for i in "${!PIDS[@]}"; do
        if ! kill -0 ${PIDS[$i]} 2>/dev/null; then
            echo -e "${RED}警告: プロセス ${PIDS[$i]} が停止しました${NC}"
        fi
    done
done