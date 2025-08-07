# ROS2とFluentVisionの接続方法

## 🚀 ROS2側のセットアップ

### 1. rosbridge_suiteのインストール

```bash
# ROS2環境で実行
sudo apt update
sudo apt install ros-humble-rosbridge-suite

# または source installから
cd ~/ros2_ws/src
git clone https://github.com/RobotWebTools/rosbridge_suite.git
cd ~/ros2_ws
colcon build --packages-select rosbridge_suite
source install/setup.bash
```

### 2. rosbridgeサーバーの起動

```bash
# WebSocketサーバーを起動（デフォルトポート: 9090）
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# カスタムポートで起動
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=8080
```

## 📡 利用可能なROS2データ

### 映像ストリーム（Image）
```javascript
// FluentVisionで設定
ストリームタイプ: ROS Topic
ROSBridge URL: ws://localhost:9090
Topic: /camera/image_raw/compressed
```

### ポイントクラウド（PointCloud2）
```javascript
// FluentVisionで設定
ストリームタイプ: PointCloud
ROSBridge URL: ws://localhost:9090  
Topic: /camera/depth/points
```

### センサーデータ（各種）
- `/scan` - LiDARスキャンデータ
- `/imu/data` - IMUデータ
- `/odom` - オドメトリ
- `/tf` - 座標変換

## 🔧 実際の接続例

### fluent_vision_ros2との連携

```bash
# 1. ROS2ノードを起動
cd ~/Projects/fluent_vision_ros2/launch
./start_fv.sh

# 2. rosbridgeを起動
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# 3. 利用可能なトピックを確認
ros2 topic list | grep fv
# /fv/d415/color/image_raw
# /fv/d415/depth/image_rect_raw
# /fv/d415/depth/points
# /fv/d405/color/image_raw
# /fv/d405/depth/image_rect_raw
```

### FluentVisionアプリでの設定

1. **「+ ストリーム追加」をクリック**

2. **RealSense D415カメラ映像**
   - ストリーム名: `D415 Color`
   - タイプ: `ROS Topic`
   - ROSBridge: `ws://localhost:9090`
   - Topic: `/fv/d415/color/image_raw/compressed`

3. **深度ポイントクラウド**
   - ストリーム名: `D415 PointCloud`
   - タイプ: `PointCloud`
   - ROSBridge: `ws://localhost:9090`
   - Topic: `/fv/d415/depth/points`

## 🛠️ トラブルシューティング

### CORS（Cross-Origin）エラーの場合

rosbridgeの起動時にCORSを無効化：
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  unregister_timeout:=100000 \
  websocket_external_port:=null
```

### 画像が表示されない場合

圧縮形式を使用：
```bash
# image_transport_pluginsをインストール
sudo apt install ros-humble-image-transport-plugins

# 圧縮トピックを確認
ros2 topic list | grep compressed
```

### パフォーマンス最適化

```javascript
// トピックのスロットリング（app.jsに追加）
topic.queue_length = 1;  // 最新フレームのみ保持
topic.throttle_rate = 100;  // 100msごとに更新
```

## 📊 データフロー

```
ROS2ノード
    ↓
[カメラ/センサー]
    ↓
ROS2トピック (/fv/d415/color/image_raw)
    ↓
rosbridge_server (WebSocket: 9090)
    ↓
FluentVision Webアプリ
    ↓
[ブラウザで表示]
```

## 🎯 活用例

1. **ロボット監視ダッシュボード**
   - 複数カメラの同時表示
   - LiDARデータの可視化
   - ステータス監視

2. **SLAM可視化**
   - ポイントクラウド表示
   - 地図生成の監視
   - 経路表示

3. **センサーフュージョン**
   - 複数センサーの統合表示
   - リアルタイムデータ分析

## 🔗 関連リンク

- [rosbridge_suite Documentation](http://wiki.ros.org/rosbridge_suite)
- [roslibjs (JavaScript Library)](http://wiki.ros.org/roslibjs)
- [web_video_server (代替案)](http://wiki.ros.org/web_video_server)