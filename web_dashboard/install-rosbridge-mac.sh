#!/bin/bash

# MacOS + Conda環境でのrosbridge_suiteインストール

echo "🍎 MacOS ROS2 rosbridge_suite Setup"
echo "===================================="
echo ""

# conda環境をアクティベート
echo "📦 Conda環境 'ros2' をアクティベート..."
source ~/miniconda3/etc/profile.d/conda.sh
conda activate ros2

# 方法1: conda-forgeから試す
echo "方法1: conda-forgeからインストールを試みます..."
conda install -c conda-forge ros-humble-rosbridge-suite -y 2>/dev/null

if [ $? -ne 0 ]; then
    echo "❌ conda-forgeにパッケージが見つかりません"
    echo ""
    echo "方法2: pipでインストールを試みます..."
    
    # 必要な依存パッケージ
    pip install tornado pymongo twisted autobahn

    # rosbridge_suiteをソースからインストール
    echo "方法3: ソースからビルドします..."
    
    cd ~/ros2_ws/src 2>/dev/null || mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
    
    # rosbridge_suiteをクローン
    if [ ! -d "rosbridge_suite" ]; then
        git clone https://github.com/RobotWebTools/rosbridge_suite.git -b ros2
    fi
    
    cd ~/ros2_ws
    
    echo "🔨 ビルド中..."
    colcon build --packages-select rosbridge_server rosbridge_library rosbridge_msgs
    
    echo ""
    echo "✅ インストール完了!"
    echo ""
    echo "===================================="
    echo "使用方法:"
    echo "===================================="
    echo ""
    echo "# 1. 環境をセットアップ"
    echo "conda activate ros2"
    echo "source ~/ros2_ws/install/setup.bash"
    echo ""
    echo "# 2. rosbridgeを起動"
    echo "ros2 run rosbridge_server rosbridge_websocket"
    echo ""
fi

echo ""
echo "===================================="
echo "代替案: シンプルなWebSocketサーバー"
echo "===================================="
echo ""
echo "rosbridgeが使えない場合は、Pythonで簡易サーバーを作成できます。"
echo "simple-ros2-websocket.py を確認してください。"