#!/bin/bash
# RTABMap ステータス確認スクリプト
# 用途：RTABMapの動作状況、センサー状態、パフォーマンスの確認

set -e  # エラー時終了

echo "=== RTABMap ステータス確認 ==="
echo "日時: $(date)"
echo ""

# 色付き出力用の定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# RTABMapノード設定
RTABMAP_NODE="/rtabmap"

# 1. RTABMapプロセス確認
echo -e "${BLUE}=== 1. プロセス確認 ===${NC}"
if pgrep -x "rtabmap" > /dev/null; then
    echo -e "✓ RTABMap: ${GREEN}実行中${NC}"
    RTABMAP_PID=$(pgrep -x "rtabmap")
    echo "  プロセスID: $RTABMAP_PID"
    echo "  CPU/Memory使用率:"
    ps -p $RTABMAP_PID -o pid,ppid,user,%cpu,%mem,time,command --no-headers || echo "  プロセス情報取得失敗"
else
    echo -e "✗ RTABMap: ${RED}停止中${NC}"
fi

if pgrep -f "rgbd_odometry" > /dev/null; then
    echo -e "✓ RGBD Odometry: ${GREEN}実行中${NC}"
else
    echo -e "✗ RGBD Odometry: ${RED}停止中${NC}"
fi

if pgrep -f "rtabmap_viz" > /dev/null; then
    echo -e "✓ RTABMapViz: ${GREEN}実行中${NC}"
else
    echo -e "- RTABMapViz: ${YELLOW}停止中${NC}"
fi

echo ""

# 2. ROS2ノード確認
echo -e "${BLUE}=== 2. ROS2ノード確認 ===${NC}"
if command -v ros2 &> /dev/null; then
    echo "ROS2ノード一覧:"
    ros2 node list | grep -E "(rtabmap|rgbd)" | while read -r node; do
        echo -e "  ✓ ${GREEN}$node${NC}"
    done
    
    # ノードが見つからない場合
    if ! ros2 node list | grep -q rtabmap; then
        echo -e "  ✗ ${RED}RTABMapノードが見つかりません${NC}"
    fi
else
    echo -e "✗ ${RED}ROS2コマンドが利用できません${NC}"
fi

echo ""

# 3. センサートピック確認
echo -e "${BLUE}=== 3. センサートピック確認 ===${NC}"

check_topic_with_rate() {
    local topic=$1
    local description=$2
    local timeout=5
    
    echo -n "  $description ($topic): "
    
    if timeout $timeout ros2 topic list | grep -q "^$topic$"; then
        # トピック存在確認
        echo -n -e "${GREEN}存在${NC} - "
        
        # 発行レート確認
        local rate_output=$(timeout 3 ros2 topic hz "$topic" 2>/dev/null | tail -1)
        if [[ $rate_output == *"average rate"* ]]; then
            echo -e "${GREEN}$rate_output${NC}"
        else
            echo -e "${YELLOW}レート測定失敗${NC}"
        fi
    else
        echo -e "${RED}見つかりません${NC}"
    fi
}

# センサートピックチェック
check_topic_with_rate "/fv/d415/color/image_raw" "D415カラー画像"
check_topic_with_rate "/fv/d415/depth/image_rect_raw" "D415深度画像"
check_topic_with_rate "/fv/d415/color/camera_info" "D415カメラ情報"
check_topic_with_rate "/fv/d405/color/image_raw" "D405カラー画像"
check_topic_with_rate "/livox/lidar" "Livox LiDAR"
check_topic_with_rate "/livox/imu" "Livox IMU"
check_topic_with_rate "/odom" "オドメトリ"

echo ""

# 4. RTABMapトピック確認
echo -e "${BLUE}=== 4. RTABMapトピック確認 ===${NC}"
check_topic_with_rate "/rtabmap/grid_map" "占有格子地図"
check_topic_with_rate "/rtabmap/cloud_map" "点群地図"
check_topic_with_rate "/rtabmap/info" "RTABMap情報"
check_topic_with_rate "/rtabmap/mapData" "マップデータ"

echo ""

# 5. TF確認
echo -e "${BLUE}=== 5. TF（座標変換）確認 ===${NC}"
if command -v ros2 &> /dev/null; then
    echo "TF フレーム一覧:"
    timeout 5 ros2 run tf2_tools view_frames 2>/dev/null || echo "  TF取得に失敗"
    
    # 重要なTFチェック
    check_tf() {
        local source_frame=$1
        local target_frame=$2
        local description=$3
        
        echo -n "  $description ($source_frame -> $target_frame): "
        if timeout 3 ros2 run tf2_ros tf2_echo "$source_frame" "$target_frame" &>/dev/null; then
            echo -e "${GREEN}OK${NC}"
        else
            echo -e "${RED}NG${NC}"
        fi
    }
    
    check_tf "map" "base_link" "マップ→ベース"
    check_tf "base_link" "d415_link" "ベース→D415"
    check_tf "base_link" "livox_frame" "ベース→LiDAR"
fi

echo ""

# 6. RTABMapパラメータ確認
echo -e "${BLUE}=== 6. RTABMapパラメータ確認 ===${NC}"
if ros2 node list | grep -q rtabmap; then
    echo "重要なパラメータ:"
    
    get_param() {
        local param_name=$1
        local description=$2
        local value=$(ros2 param get $RTABMAP_NODE "$param_name" 2>/dev/null | awk '{print $NF}')
        printf "  %-25s: %s\n" "$description" "$value"
    }
    
    get_param "Mem/IncrementalMemory" "モード（true=マッピング）"
    get_param "database_path" "データベースパス"
    get_param "frame_id" "ベースフレーム"
    get_param "Rtabmap/DetectionRate" "検出レート"
    get_param "Optimizer/Strategy" "最適化戦略"
    get_param "Grid/CellSize" "グリッドセルサイズ"
else
    echo -e "  ${RED}RTABMapノードが実行されていません${NC}"
fi

echo ""

# 7. データベース確認
echo -e "${BLUE}=== 7. データベース確認 ===${NC}"
DATABASE_DIR="/home/aspara/seedbox-r1/fluent_vision_ros2/rtabmap/databases"

if [ -d "$DATABASE_DIR" ]; then
    echo "データベースファイル一覧:"
    find "$DATABASE_DIR" -name "*.db" -exec ls -lh {} \; 2>/dev/null | while read -r line; do
        echo "  $line"
    done
    
    # データベースファイル数
    db_count=$(find "$DATABASE_DIR" -name "*.db" | wc -l)
    echo "データベースファイル数: $db_count"
else
    echo -e "  ${YELLOW}データベースディレクトリが存在しません: $DATABASE_DIR${NC}"
fi

echo ""

# 8. システムリソース確認
echo -e "${BLUE}=== 8. システムリソース確認 ===${NC}"
echo "CPU使用率:"
top -bn1 | grep "Cpu(s)" | awk '{print "  "$1" "$2" "$3}'

echo "メモリ使用率:"
free -h | awk 'NR==2{printf "  使用中: %s/%s (%.2f%%)\n", $3,$2,$3*100/$2}'

echo "ディスク使用量:"
df -h /home/aspara/seedbox-r1/fluent_vision_ros2 | awk 'NR==2{printf "  %s使用中 / %s (使用率%s)\n", $3,$2,$5}'

echo ""

# 9. サービス確認
echo -e "${BLUE}=== 9. RTABMapサービス確認 ===${NC}"
if ros2 node list | grep -q rtabmap; then
    echo "利用可能なサービス:"
    ros2 service list | grep rtabmap | while read -r service; do
        echo -e "  ✓ ${GREEN}$service${NC}"
    done
else
    echo -e "  ${RED}RTABMapノードが実行されていません${NC}"
fi

echo ""

# 10. 推奨アクション
echo -e "${BLUE}=== 10. 推奨アクション ===${NC}"
if ! pgrep -x "rtabmap" > /dev/null; then
    echo -e "${YELLOW}RTABMapが実行されていません。以下のコマンドで開始してください:${NC}"
    echo "  マッピング: ./start_mapping.sh"
    echo "  ローカライゼーション: ./start_localization.sh"
elif ! ros2 topic list | grep -q "/fv/d415/color/image_raw"; then
    echo -e "${YELLOW}カメラトピックが見つかりません。カメラノードを確認してください。${NC}"
elif ! ros2 topic list | grep -q "/livox/lidar"; then
    echo -e "${YELLOW}LiDARトピックが見つかりません。LiDARノードを確認してください。${NC}"
else
    echo -e "${GREEN}システムは正常に動作しているようです。${NC}"
    echo "  モード切り替え: ./switch_mode.sh"
    echo "  可視化: RTABMapViz または RViz を使用"
fi

echo ""
echo "=== ステータス確認完了 ==="
echo "日時: $(date)"