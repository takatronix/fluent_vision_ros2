# FV Image Distributor

FluentVision用の画像配信サーバーです。ROS2画像トピックを複数のクライアントに配信します。

## 概要

fv_image_distributorは、ROS2の画像トピックを受信し、複数のクライアントに同時配信するサーバーノードです。Webブラウザやモバイルアプリからのアクセスに対応しています。

## 機能

- **マルチクライアント対応**: 複数のクライアントに同時配信
- **HTTP/HTTPS対応**: 標準的なWebプロトコルで配信
- **画像圧縮**: JPEG圧縮による帯域幅最適化
- **リアルタイム配信**: 低遅延での画像配信
- **設定可能**: YAMLファイルによる柔軟な設定
- **Boost.Asio使用**: 高性能な非同期I/O

## 入力/出力

### 入力
- **画像トピック**: `sensor_msgs/msg/Image`
  - デフォルト: `/camera/color/image_raw`

### 出力
- **HTTP配信**: Webブラウザでアクセス可能
- **WebSocket配信**: リアルタイム配信（オプション）

## 設定

### パラメータ

| パラメータ | デフォルト値 | 説明 |
|-----------|-------------|------|
| `input_image_topic` | `/camera/color/image_raw` | 入力画像トピック |
| `server.port` | `8080` | HTTPサーバーポート |
| `server.host` | `0.0.0.0` | サーバーホスト |
| `image.quality` | `85` | JPEG圧縮品質 |
| `image.max_width` | `1280` | 最大画像幅 |
| `image.max_height` | `720` | 最大画像高さ |
| `streaming.fps` | `30.0` | 配信フレームレート |

## 使用方法

### 1. ビルド

```bash
cd fluent_vision_ros2
colcon build --packages-select fv_image_distributor
```

### 2. 起動

```bash
# 基本的な起動
ros2 launch fv_image_distributor image_distributor.launch.py

# カスタム設定で起動
ros2 launch fv_image_distributor image_distributor.launch.py \
  input_topic:=/my_camera/image_raw \
  server_port:=9090
```

### 3. アクセス

Webブラウザで以下のURLにアクセス：
```
http://[IPアドレス]:8080/stream
```

## 依存関係

- ROS2 Humble
- OpenCV 4.x
- Boost.Asio
- sensor_msgs
- cv_bridge
- image_transport
- std_msgs

## 開発状況

⚠️ **開発中**: このノードは現在開発中です。実装は完了していません。

## ライセンス

MIT License 