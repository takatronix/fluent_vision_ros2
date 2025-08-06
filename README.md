# FluentVision ROS2

ROS2ベースのリアルタイムビジョンシステム

## パッケージ構成

### AI/ML (`src/ai/`)
- **fv_face_recognizer**: リアルタイム顔検出・認識（推論時間11-14ms、30FPS）
- **fv_object_detector**: 物体検出システム

### センサー (`src/sensors/`)
- **fv_camera**: 汎用カメラインターフェース  
- **fv_realsense**: Intel RealSense深度カメラドライバ

### ストリーミング (`src/streaming/`)
- **fv_rtmp_server**: RTMPストリーミングサーバー
- **fv_mjpeg_server**: MJPEGストリーム配信
- **fv_websocket_server**: WebSocket配信
- **fv_image_distributor**: HTTP画像配信
- **fv_recorder**: 動画録画

## インストール

```bash
cd ~/FluentVision/fluent_vision_ros2
colcon build --symlink-install
source install/setup.bash
```

## 使用方法

### 顔認識ノード
```bash
./run_face_recognizer.sh
```

## 作者

Takashi Otsuka (@takatronix)