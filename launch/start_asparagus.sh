#!/bin/bash

# =============================================================================
# Fluent Vision - アスパラガス解析システム統合起動スクリプト
# =============================================================================

# カメラタイプをチェック
if [ $# -eq 0 ]; then
    echo "使用方法: $0 [d415|d405]"
    echo "例: $0 d415"
    exit 1
fi

CAMERA_TYPE=$1

# カメラタイプの検証
if [ "$CAMERA_TYPE" != "d415" ] && [ "$CAMERA_TYPE" != "d405" ]; then
    echo "エラー: カメラタイプは d415 または d405 を指定してください"
    exit 1
fi

echo "===== ${CAMERA_TYPE}カメラ用アスパラガス解析システム起動 ====="
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

# 設定ファイルパス
CONFIG_DIR="/home/aspara/seedbox-r1/fluent_vision_ros2/launch"

# 1. RealSense起動
echo -e "${GREEN}[1/4] RealSense ${CAMERA_TYPE}起動中...${NC}"
if [ "$CAMERA_TYPE" = "d415" ]; then
    CONFIG_FILE="${CONFIG_DIR}/fv_realsense_d415_optimized.yaml"
else
    CONFIG_FILE="${CONFIG_DIR}/fv_realsense_d405.yaml"
fi

ros2 launch fv_realsense fv_realsense_launch.py \
    config_file:=${CONFIG_FILE} \
    camera_namespace:=fv/${CAMERA_TYPE} &
PIDS+=($!)
sleep 3

# 2. YOLO物体検出起動
echo -e "${GREEN}[2/4] YOLO物体検出起動中...${NC}"
ros2 run fv_object_detector fv_object_detector_node \
    --ros-args \
    --params-file ${CONFIG_DIR}/fv_object_detector_${CAMERA_TYPE}.yaml &
PIDS+=($!)
sleep 2

# 3. 点群生成ノード起動（トピック名を動的に設定）
echo -e "${GREEN}[3/4] 点群生成ノード起動中...${NC}"
ros2 run fv_aspara_analyzer simple_pointcloud_generator \
    --ros-args \
    -r __ns:=/fv/${CAMERA_TYPE} \
    -r /fv/d415/depth/image_rect_raw:=/fv/${CAMERA_TYPE}/depth/image_rect_raw \
    -r /fv/d415/color/image_raw:=/fv/${CAMERA_TYPE}/color/image_raw \
    -r /fv/d415/object_detection/detections:=/fv/${CAMERA_TYPE}/object_detection/detections \
    -r /asparagus/points:=/fv/${CAMERA_TYPE}/asparagus/points &
PIDS+=($!)
sleep 1

# 4. アスパラガス解析ノード起動
echo -e "${GREEN}[4/4] アスパラガス解析ノード起動中...${NC}"
ros2 run fv_aspara_analyzer fv_aspara_analyzer_node \
    --ros-args \
    --params-file ${CONFIG_DIR}/fv_aspara_analyzer_${CAMERA_TYPE}.yaml &
PIDS+=($!)

echo ""
echo -e "${GREEN}===== システム起動完了 =====${NC}"
echo ""
echo "トピック一覧:"
echo "  入力:"
echo "    - /fv/${CAMERA_TYPE}/color/image_raw       (カラー画像)"
echo "    - /fv/${CAMERA_TYPE}/depth/image_rect_raw  (深度画像)"
echo "    - /fv/${CAMERA_TYPE}/object_detection/detections (検出結果)"
echo ""
echo "  出力:"
echo "    - /fv/${CAMERA_TYPE}/asparagus/points      (点群)"
echo "    - /fv/${CAMERA_TYPE}/aspara_analysis/result (解析結果)"
echo ""
echo "確認コマンド:"
echo "  ros2 topic list | grep ${CAMERA_TYPE}"
echo "  ros2 topic hz /fv/${CAMERA_TYPE}/asparagus/points"
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