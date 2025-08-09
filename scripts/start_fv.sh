#!/bin/bash
# ================================================
# Fluent Vision システム起動スクリプト（両カメラ同時起動）
# ================================================
# 個別のカメラ起動スクリプトを呼び出して
# D415とD405の両方を起動します
# ================================================

echo "🚀 Starting Fluent Vision System (Both Cameras)"
echo "================================================"

# スクリプトのディレクトリに移動
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 共有環境変数の設定
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=../fastdds_shared_memory.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=0

# 既存プロセスの停止
echo "🔧 Stopping any existing FV processes..."
"./stop_fv.sh"
sleep 1
pkill -9 -f "depth_image_proc" || true
pkill -9 -f "ros2 run depth_image_proc" || true
pkill -9 -f "foxglove_bridge" || true
sleep 1

# D415カメラシステムを起動
echo ""
echo "📷 [1/3] Starting D415 Camera System..."
echo "----------------------------------------"
./start_fv415.sh &
D415_PID=$!

# D415が起動するまで待機
echo "⏳ Waiting for D415 to initialize..."
for i in {1..20}; do
    if ros2 topic list 2>/dev/null | grep -q "/fv/d415/color/image_raw"; then
        echo "✅ D415 system is ready!"
        break
    fi
    echo "   Waiting for D415... ($i/20)"
    sleep 1
done

# カメラ間の競合を避けるため少し待機
echo "⏳ Waiting 3 seconds before starting D405..."
sleep 3

# D405カメラシステムを起動
echo ""
echo "📷 [2/3] Starting D405 Camera System..."
echo "----------------------------------------"
./start_fv405.sh &
D405_PID=$!

# D405が起動するまで待機
echo "⏳ Waiting for D405 to initialize..."
for i in {1..20}; do
    if ros2 topic list 2>/dev/null | grep -q "/fv/d405/color/image_raw"; then
        echo "✅ D405 system is ready!"
        break
    fi
    echo "   Waiting for D405... ($i/20)"
    sleep 1
done

# Foxglove Bridge起動（共通ユーティリティ）
echo ""
echo "🦊 [3/3] Starting Foxglove Bridge..."
echo "----------------------------------------"
ros2 run foxglove_bridge foxglove_bridge &

# 完了メッセージ
echo ""
echo "================================================"
echo "✅ All Fluent Vision nodes started!"
echo "================================================"
echo ""
echo "📊 Use 'ros2 node list' to check running nodes"
echo "🛑 Use './stop_fv.sh' to stop all nodes"
echo ""
echo "💡 Individual camera control:"
echo "   - ./start_fv405.sh : Start D405 only"
echo "   - ./start_fv415.sh : Start D415 only"
echo "   - ./stop_fv.sh     : Stop all nodes"
echo ""
echo "Process IDs:"
echo "   - D415 system: $D415_PID"
echo "   - D405 system: $D405_PID"