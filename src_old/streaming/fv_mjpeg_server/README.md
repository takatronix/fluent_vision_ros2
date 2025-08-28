# FV MJPEG Server

FluentVision用のMJPEGストリーミングサーバーです。ROS2画像トピックをMJPEGストリームとして配信します。

## 概要

fv_mjpeg_serverは、ROS2の画像トピックを受信し、MJPEG（Motion JPEG）ストリームとしてWebブラウザに配信するサーバーノードです。標準的なWebブラウザでリアルタイム画像を確認できます。

## 機能

- **MJPEG配信**: 標準的なMJPEGストリーミング
- **Webブラウザ対応**: プラグイン不要でアクセス可能
- **低遅延**: リアルタイムに近い配信
- **設定可能**: YAMLファイルによる柔軟な設定
- **マルチクライアント**: 複数のブラウザから同時アクセス
- **画像圧縮**: JPEG圧縮による帯域幅最適化

## 入力/出力

### 入力
- **画像トピック**: `sensor_msgs/msg/Image`
  - デフォルト: `/camera/color/image_raw`

### 出力
- **MJPEGストリーム**: HTTP経由でMJPEG配信
- **Webページ**: ストリーム表示用HTML

## 設定

### パラメータ

| パラメータ | デフォルト値 | 説明 |
|-----------|-------------|------|
| `input_image_topic` | `/camera/color/image_raw` | 入力画像トピック |
| `server.port` | `8080` | HTTPサーバーポート |
| `server.host` | `0.0.0.0` | サーバーホスト |
| `stream.quality` | `85` | JPEG圧縮品質 |
| `stream.fps` | `30.0` | ストリームフレームレート |
| `stream.max_width` | `1280` | 最大画像幅 |
| `stream.max_height` | `720` | 最大画像高さ |

## 使用方法

### 1. ビルド

```bash
cd fluent_vision_ros2
colcon build --packages-select fv_mjpeg_server
```

### 2. 起動

```bash
# 基本的な起動
ros2 launch fv_mjpeg_server mjpeg_server.launch.py

# カスタム設定で起動
ros2 launch fv_mjpeg_server mjpeg_server.launch.py \
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
- ament_index_cpp
- sensor_msgs
- cv_bridge
- image_transport
- std_msgs

## 開発状況

⚠️ **開発中**: このノードは現在開発中です。実装は完了していません。

## ライセンス

MIT License 