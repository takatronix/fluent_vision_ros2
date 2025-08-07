#!/bin/bash

# FluentVision ROS2 Backend Setup
# 最小限の必要パッケージをインストール

echo "🚀 FluentVision Backend Setup"
echo "================================"

# ROS2のバージョン確認
if ! command -v ros2 &> /dev/null; then
    echo "❌ ROS2がインストールされていません"
    echo "先にROS2 (Humble推奨) をインストールしてください"
    exit 1
fi

echo "✅ ROS2が検出されました"

# 必要な2つのパッケージをインストール

echo ""
echo "📦 必要なパッケージをインストールします..."
echo ""

# 1. rosbridge_suite (WebSocket通信用) - 必須
echo "1️⃣ rosbridge_suite (WebSocket通信)"
sudo apt update
sudo apt install -y ros-humble-rosbridge-suite

# 2. web_video_server (MJPEG配信用) - オプション但し推奨
echo "2️⃣ web_video_server (MJPEG配信)"
sudo apt install -y ros-humble-web-video-server

# 3. image_transport_plugins (画像圧縮用) - オプション但し推奨
echo "3️⃣ image_transport_plugins (画像圧縮)"
sudo apt install -y ros-humble-image-transport-plugins

echo ""
echo "✅ インストール完了！"
echo ""
echo "================================"
echo "📋 最小構成での起動方法:"
echo "================================"
echo ""
echo "# ターミナル1: rosbridge起動"
echo "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
echo ""
echo "# ターミナル2: あなたのROS2ノード起動"
echo "ros2 run your_package your_node"
echo ""
echo "# ブラウザでアクセス"
echo "http://$(hostname -I | awk '{print $1}'):8000"
echo ""
echo "================================"

# 簡易起動スクリプトも作成
cat > start-fluentvision-backend.sh << 'EOF'
#!/bin/bash
# FluentVision Backend Quick Start

source /opt/ros/humble/setup.bash

echo "Starting ROSBridge WebSocket Server..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 &

echo ""
echo "✅ Backend Ready!"
echo "WebSocket: ws://localhost:9090"
echo ""
echo "Available topics:"
ros2 topic list

wait
EOF

chmod +x start-fluentvision-backend.sh

echo "✅ セットアップ完了!"
echo "📝 start-fluentvision-backend.sh で簡単起動できます"