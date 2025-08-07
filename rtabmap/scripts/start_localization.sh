#!/bin/bash
# RTABMap ローカライゼーション開始スクリプト
# 用途：既存マップを使用した自己位置推定

set -e  # エラー時終了

echo "=== RTABMap ローカライゼーション開始 ==="
echo "日時: $(date)"

# 設定
DATABASE_PATH="/home/aspara/seedbox-r1/fluent_vision_ros2/rtabmap/databases/rtabmap.db"
RTABMAPVIZ="true"  # 可視化有効
RVIZ="false"       # RViz無効（軽量化）

# 引数処理
while [[ $# -gt 0 ]]; do
    case $1 in
        --database)
            DATABASE_PATH="$2"
            shift 2
            ;;
        --no-viz)
            RTABMAPVIZ="false"
            shift
            ;;
        --rviz)
            RVIZ="true"
            shift
            ;;
        --help|-h)
            echo "使用方法: $0 [オプション]"
            echo "オプション:"
            echo "  --database PATH   既存データベースファイルパス"
            echo "  --no-viz          可視化無効"
            echo "  --rviz            RViz有効"
            echo "  --help, -h        このヘルプを表示"
            exit 0
            ;;
        *)
            echo "不明なオプション: $1"
            echo "ヘルプ: $0 --help"
            exit 1
            ;;
    esac
done

echo "データベースパス: $DATABASE_PATH"
echo "可視化: $RTABMAPVIZ"
echo "RViz: $RVIZ"

# データベース存在確認
if [ ! -f "$DATABASE_PATH" ]; then
    echo "エラー: データベースファイルが見つかりません: $DATABASE_PATH"
    echo ""
    echo "利用可能なデータベース:"
    find "/home/aspara/seedbox-r1/fluent_vision_ros2/rtabmap/databases" -name "*.db" 2>/dev/null || echo "データベースが見つかりません"
    exit 1
fi

echo "データベース確認: $DATABASE_PATH ($(du -h "$DATABASE_PATH" | cut -f1))"

# 既存のRTABMapプロセス確認
if pgrep -x "rtabmap" > /dev/null; then
    echo "警告: RTABMapプロセスが既に実行中です"
    read -p "停止しますか？ (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "既存プロセスを停止中..."
        pkill -x "rtabmap" || true
        sleep 2
    else
        echo "中断しました"
        exit 1
    fi
fi

# ROS2環境確認
if ! command -v ros2 &> /dev/null; then
    echo "エラー: ROS2が見つかりません"
    echo "ROS2環境をセットアップしてください"
    exit 1
fi

# トピック確認
echo "センサートピック確認中..."
TIMEOUT=10

check_topic() {
    local topic=$1
    local description=$2
    if timeout $TIMEOUT ros2 topic list | grep -q "$topic"; then
        echo "✓ $description: $topic"
        return 0
    else
        echo "✗ $description: $topic (見つかりません)"
        return 1
    fi
}

# 必要なトピックをチェック
TOPICS_OK=true

check_topic "/fv/d415/color/image_raw" "D415カラー画像" || TOPICS_OK=false
check_topic "/fv/d415/depth/image_rect_raw" "D415深度画像" || TOPICS_OK=false
check_topic "/fv/d415/color/camera_info" "D415カメラ情報" || TOPICS_OK=false
check_topic "/livox/lidar" "Livox LiDAR" || TOPICS_OK=false

if [ "$TOPICS_OK" = false ]; then
    echo "警告: 一部のセンサートピックが見つかりません"
    read -p "続行しますか？ (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "中断しました"
        exit 1
    fi
fi

# データベース読み取り専用確認
if [ ! -r "$DATABASE_PATH" ]; then
    echo "エラー: データベースファイルを読み取れません: $DATABASE_PATH"
    exit 1
fi

# ローカライゼーション開始
echo ""
echo "=== ローカライゼーション開始 ==="
echo "既存マップ使用: $(basename "$DATABASE_PATH")"
echo "Ctrl+C で停止します"
echo ""

# Launch ファイル実行
cd /home/aspara/seedbox-r1/fluent_vision_ros2

ros2 launch rtabmap/launch/rtabmap_rgbd.launch.py \
    config:="localization" \
    localization:=true \
    database_path:="$DATABASE_PATH" \
    rtabmapviz:="$RTABMAPVIZ" \
    rviz:="$RVIZ"

echo ""
echo "=== ローカライゼーション終了 ==="
echo "日時: $(date)"