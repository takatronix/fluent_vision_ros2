# FV Camera

軽量で高性能なカメラ用ROS2ノードです。USBカメラ、ラズパイカメラ、その他の標準カメラをサポートします。

## 特徴

- 🚀 **軽量設計**: 深度機能を排除し、標準カメラ機能に特化
- 📷 **多カメラ対応**: デバイスインデックスによるカメラ選択
- 🔧 **設定可能**: YAMLファイルによる柔軟な設定
- 📡 **ネットワーク最適化**: 圧縮画像トピック対応
- 🎯 **サービス提供**: カメラ情報取得・設定変更サービス
- 🔄 **TF対応**: カメラフレームのTF配信
- 🎨 **OpenCV対応**: 標準的なUSBカメラドライバー使用

## 必要な依存関係

```bash
# ROS2 Humble
sudo apt install ros-humble-rclcpp ros-humble-sensor-msgs ros-humble-cv-bridge
sudo apt install ros-humble-tf2-ros ros-humble-image-transport ros-humble-compressed-image-transport
sudo apt install ros-humble-geometry-msgs ros-humble-std-srvs

# OpenCV
sudo apt install libopencv-dev
```

## ビルド

```bash
# fluent_visionディレクトリで
cd /home/ros2/fluent_vision

# ROS2ワークスペースにコピー
cp -r fv_camera /home/ros2/ros2_ws/src/

# ビルド
cd /home/ros2/ros2_ws
colcon build --packages-select fv_camera
source install/setup.bash
```

## 使用方法

### 1. カメラ一覧表示

```bash
ros2 run fv_camera list_cameras_node
```

### 2. 基本的な起動

```bash
ros2 launch fv_camera fv_camera_launch.py
```

### 3. カスタム設定で起動

```bash
ros2 launch fv_camera fv_camera_launch.py \
    node_name:=my_camera \
    config_file:=/path/to/custom_config.yaml
```



## 設定

`config/default_config.yaml`で以下の設定が可能です：

- **カメラ選択**: auto/index
- **解像度・フレームレート**: 640x480/30fps (デフォルト)
- **カメラプロパティ**: 明度、コントラスト、彩度、色相、ゲイン、露出
- **ストリーム設定**: カラー画像、圧縮画像
- **圧縮設定**: JPEG品質、圧縮トピック有効/無効
- **TF設定**: フレーム名、変換パラメータ
- **サービス設定**: get_camera_info/set_camera_settings

## トピック

### 配信トピック

- `image_raw` - カラー画像
- `image_raw/compressed` - 圧縮カラー画像
- `camera_info` - カメラ情報

### サービス

- `get_camera_info` - カメラ情報取得
- `set_camera_settings` - カメラ設定変更

## 例

### カメラ情報取得サービス

```bash
ros2 service call /fv_camera_node/get_camera_info \
    fv_camera/srv/GetCameraInfo \
    "{camera_name: ''}"
```

### カメラ設定変更サービス

```bash
ros2 service call /fv_camera_node/set_camera_settings \
    fv_camera/srv/SetCameraSettings \
    "{width: 1280, height: 720, fps: 30, brightness: 50, contrast: 60}"
```

## 複数カメラ対応

複数のUSBカメラを使用する場合：

1. カメラ一覧を確認
2. 各カメラに異なるノード名を設定
3. デバイスインデックスで特定のカメラを選択

```bash
# カメラ1 (インデックス0)
ros2 launch fv_camera fv_camera_launch.py \
    node_name:=camera_front \
    config_file:=config/camera1.yaml

# カメラ2 (インデックス1)
ros2 launch fv_camera fv_camera_launch.py \
    node_name:=camera_side \
    config_file:=config/camera2.yaml
```

## トラブルシューティング

### カメラが見つからない

```bash
# カメラ一覧確認
ros2 run fv_camera list_cameras_node

# USB権限確認
lsusb

# udevルール確認
sudo cat /etc/udev/rules.d/99-video.rules
```

### カメラが動作しない

```bash
# カメラテスト
ros2 run fv_camera list_cameras_node

# 他のアプリケーションでカメラを使用していないか確認
lsof /dev/video*
```

### ビルドエラー

```bash
# 依存関係確認
rosdep install --from-paths src --ignore-src -r -y

# クリーンビルド
rm -rf build/ install/
colcon build --packages-select fv_camera
```

## 設定例

### 高解像度設定

```yaml
fv_usb_camera:
  ros__parameters:
    camera:
      width: 1920
      height: 1080
      fps: 30
    camera_info:
      enable_compressed_topics: true
      compressed_quality: 90
```

### 軽量設定（Raspberry Pi等）

```yaml
fv_usb_camera:
  ros__parameters:
    camera:
      width: 320
      height: 240
      fps: 10
    camera_info:
      enable_compressed_topics: false
    services:
      get_camera_info_enabled: false
      set_camera_settings_enabled: false
    tf:
      enabled: false
```

## ライセンス

MIT License 