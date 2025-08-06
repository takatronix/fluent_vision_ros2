# FV WebSocket Server

FluentVision用のWebSocketサーバーです。ROS2画像トピックをWebSocket経由でリアルタイム配信します。

## 概要

fv_websocket_serverは、ROS2の画像トピックを受信し、WebSocketプロトコルを使用してWebブラウザやモバイルアプリにリアルタイム配信するサーバーノードです。

## 機能

- **WebSocket配信**: リアルタイムWebSocketストリーミング
- **バイナリデータ**: 効率的な画像データ配信
- **マルチクライアント**: 複数のクライアントに同時配信
- **自動再接続**: クライアント側の自動再接続対応
- **設定可能**: YAMLファイルによる柔軟な設定
- **Boost.Asio使用**: 高性能な非同期I/O

## 入力/出力

### 入力
- **画像トピック**: `sensor_msgs/msg/Image`
  - デフォルト: `/camera/color/image_raw`

### 出力
- **WebSocket配信**: WebSocket経由で画像データ配信
- **JSONメタデータ**: 画像情報のJSON配信

## 設定

### パラメータ

| パラメータ | デフォルト値 | 説明 |
|-----------|-------------|------|
| `input_image_topic` | `/camera/color/image_raw` | 入力画像トピック |
| `server.port` | `8080` | WebSocketサーバーポート |
| `server.host` | `0.0.0.0` | サーバーホスト |
| `stream.quality` | `85` | JPEG圧縮品質 |
| `stream.fps` | `30.0` | ストリームフレームレート |
| `stream.max_width` | `1280` | 最大画像幅 |
| `stream.max_height` | `720` | 最大画像高さ |
| `websocket.enable_compression` | `true` | WebSocket圧縮有効 |

## 使用方法

### 1. ビルド

```bash
cd fluent_vision_ros2
colcon build --packages-select fv_websocket_server
```

### 2. 起動

```bash
# 基本的な起動
ros2 launch fv_websocket_server websocket_server.launch.py

# カスタム設定で起動
ros2 launch fv_websocket_server websocket_server.launch.py \
  input_topic:=/my_camera/image_raw \
  server_port:=9090
```

### 3. クライアント接続

JavaScriptクライアント例：
```javascript
const ws = new WebSocket('ws://[IPアドレス]:8080/stream');
ws.onmessage = function(event) {
    // 画像データの処理
    const imageData = event.data;
    // 画像表示処理
};
```

## 依存関係

- ROS2 Humble
- OpenCV 4.x
- WebSocket++ (websocketpp)
- Boost.Asio
- sensor_msgs
- cv_bridge
- image_transport
- std_msgs

## 開発状況

⚠️ **開発中**: このノードは現在開発中です。実装は完了していません。

## ライセンス

MIT License 