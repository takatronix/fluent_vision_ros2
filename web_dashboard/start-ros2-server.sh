#!/bin/bash

# ROS2 FluentVision Server Starter
# スマホやタブレットからアクセス可能にする

echo "🚀 FluentVision ROS2 Server Starting..."

# IPアドレスを取得
IP=$(hostname -I | awk '{print $1}')
echo "📡 Server IP: $IP"

# ROS2環境のソース
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "Starting services..."

# 1. ROSBridge WebSocketサーバーを起動（すべてのインターフェースで待ち受け）
echo "✅ Starting ROSBridge on ws://$IP:9090"
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 address:=0.0.0.0 &
ROSBRIDGE_PID=$!

# 2. Web Video Server（MJPEG配信）も起動する場合
if command -v ros2 &> /dev/null && ros2 pkg list | grep -q web_video_server; then
    echo "✅ Starting Web Video Server on http://$IP:8080"
    ros2 run web_video_server web_video_server --ros-args -p port:=8080 &
    VIDEO_PID=$!
fi

# 3. FluentVision Webサーバーを起動
echo "✅ Starting FluentVision Web UI on http://$IP:8000"
cd "$(dirname "$0")"
python3 -m http.server 8000 --bind 0.0.0.0 &
WEB_PID=$!

echo ""
echo "========================================="
echo "🎉 FluentVision Server Ready!"
echo "========================================="
echo ""
echo "📱 スマホ/タブレットからアクセス:"
echo "   http://$IP:8000"
echo ""
echo "🖥️ PCからアクセス:"
echo "   http://localhost:8000"
echo ""
echo "📡 ROS2 WebSocket:"
echo "   ws://$IP:9090"
echo ""
echo "📹 MJPEG Streams:"
echo "   http://$IP:8080"
echo ""
echo "========================================="
echo "Press Ctrl+C to stop all services"
echo ""

# Ctrl+Cで全プロセスを終了
trap "echo 'Stopping services...'; kill $ROSBRIDGE_PID $VIDEO_PID $WEB_PID 2>/dev/null; exit" INT

# プロセスを待機
wait