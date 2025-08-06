# FV RealSense

軽量で高性能なIntel RealSense深度カメラ用ROS2ノードです。

## 特徴

- 🚀 **軽量設計**: AI機能を排除し、深度カメラ機能に特化
- 📷 **多カメラ対応**: シリアル番号、デバイス名、インデックスによるカメラ選択
- 🔧 **設定可能**: YAMLファイルによる柔軟な設定
- 📡 **ネットワーク最適化**: 圧縮画像トピック対応
- 🎯 **サービス提供**: 2D座標から3D座標への変換サービス
- 🔄 **TF対応**: カメラフレームのTF配信
- 🎨 **可視化対応**: 深度カラーマップ生成

## 必要な依存関係

```bash
# ROS2 Humble
sudo apt install ros-humble-rclcpp ros-humble-sensor-msgs ros-humble-cv-bridge
sudo apt install ros-humble-tf2-ros ros-humble-pcl-conversions ros-humble-pcl-ros
sudo apt install ros-humble-image-transport ros-humble-compressed-image-transport
sudo apt install ros-humble-geometry-msgs ros-humble-std-srvs

# Intel RealSense SDK
sudo apt install librealsense2-dev librealsense2-dkms

# OpenCV
sudo apt install libopencv-dev
```

## ビルド

```bash
# fluent_visionディレクトリで
cd /home/ros2/fluent_vision

# ROS2ワークスペースにコピー
cp -r fv_realsense /home/ros2/ros2_ws/src/

# ビルド
cd /home/ros2/ros2_ws
colcon build --packages-select fv_realsense
source install/setup.bash
```

## 使用方法

### 1. カメラ一覧表示

```bash
python3 fv_realsense/scripts/list_cameras.py
```

### 2. 基本的な起動

```bash
ros2 launch fv_realsense fv_realsense_launch.py
```

### 3. カスタム設定で起動

```bash
ros2 launch fv_realsense fv_realsense_launch.py \
    node_name:=my_camera \
    config_file:=/path/to/custom_config.yaml
```

## 設定

`config/default_config.yaml`で以下の設定が可能です：

- **カメラ選択**: auto/serial/name/index
- **解像度・フレームレート**: 640x480/30fps (デフォルト)
- **ストリーム設定**: color/depth/infrared/pointcloud
- **圧縮設定**: JPEG品質、圧縮トピック有効/無効
- **TF設定**: フレーム名、変換パラメータ
- **サービス設定**: get_distance/get_camera_info

## トピック

### 配信トピック

- `color/image_raw` - カラー画像
- `color/image_raw/compressed` - 圧縮カラー画像
- `depth/image_rect_raw` - 深度画像
- `depth/colormap` - 深度カラーマップ
- `depth/color/points` - ポイントクラウド
- `color/camera_info` - カメラ情報
- `depth/camera_info` - 深度カメラ情報

### サービス

- `get_distance` - 2D座標から3D座標への変換
- `get_camera_info` - カメラ情報取得

## 例

### 距離測定サービス

```bash
# ピクセル座標(320, 240)の3D座標を取得
ros2 service call /fv_realsense_node/get_distance \
    fv_realsense/srv/GetDistance \
    "{x: 320, y: 240, frame_id: 'color_optical_frame'}"
```

### カメラ情報取得

```bash
ros2 service call /fv_realsense_node/get_camera_info \
    fv_realsense/srv/GetCameraInfo \
    "{camera_name: ''}"
```

## 複数カメラ対応

複数のRealSenseカメラを使用する場合：

1. カメラ一覧を確認
2. 各カメラに異なるノード名を設定
3. シリアル番号またはデバイス名で特定のカメラを選択

```bash
# カメラ1 (シリアル番号指定)
ros2 launch fv_realsense fv_realsense_launch.py \
    node_name:=camera_front \
    config_file:=config/camera1.yaml

# カメラ2 (デバイス名指定)
ros2 launch fv_realsense fv_realsense_launch.py \
    node_name:=camera_side \
    config_file:=config/camera2.yaml
```

## トラブルシューティング

### カメラが見つからない

```bash
# カメラ一覧確認
python3 fv_realsense/scripts/list_cameras.py

# USB権限確認
lsusb | grep RealSense

# udevルール確認
sudo cat /etc/udev/rules.d/99-realsense-libusb.rules
```

### ビルドエラー

```bash
# 依存関係確認
rosdep install --from-paths src --ignore-src -r -y

# クリーンビルド
rm -rf build/ install/
colcon build --packages-select fv_realsense
```

## ライセンス

MIT License 