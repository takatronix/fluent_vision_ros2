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
- 🖱️ **キャリブレーション支援**: クリックベースの表示モード機能

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
cd /home/aspara/seedbox-r1/fluent_vision_ros2

# ビルド
colcon build --packages-select fv_realsense
source install/setup.bash
```

## 使用方法

### 1. カメラ一覧表示

```bash
python3 src/sensors/fv_realsense/scripts/list_cameras.py
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

### 4. D405のUSB転送バッファを拡張する

D405のcolorとdepthを同時に30 FPSで使用すると、標準`uvcvideo`のUSB転送要求数ではフレームが欠損する場合があります。

`fv-uvcvideo`は、実行中のカーネルと一致するソースおよびビルドツリーを検証し、`UVC_URBS=32`の任意モジュールを導入します。

Jetson Linux R38.4ではNVIDIA公式ソースを使用します。

通常のUbuntuでは、実行中カーネルと同一バージョンの`linux-headers`および`linux-source`をAPTで自動解決します。

```bash
sudo fv-uvcvideo install --reload
fv-uvcvideo status
```

D405の欠番改善を実機で確認した環境はJetson Linux R38.4です。

通常のUbuntu 24.04では、arm64とx86_64の標準`6.8.0-138-generic`カーネル用モジュールについて、ビルド、互換性検証および導入を確認しています。

これら以外の環境では確認プロンプトを表示し、`y`を選んだ場合だけ厳密なソース、ビルドおよびモジュール互換性検証を実行します。

非対話環境から未検証の構成を試す場合は、`--allow-unsupported`を明示します。

元のモジュールへ戻す場合は、次を実行します。

```bash
sudo fv-uvcvideo uninstall --reload
```

署名強制が有効な環境では、登録済みのUbuntu Machine Owner Keyを使用します。

鍵を明示する場合は、`--signing-key`と`--signing-cert`を同時に指定します。

設計と検証条件は[`docs/design/fv_realsense_d405_streaming.ja.md`](../../../docs/design/fv_realsense_d405_streaming.ja.md)に記載しています。

### 5. DDS socket bufferを設定する

大容量画像をDDSで送受信するホストでは、socket buffer上限を設定します。

```bash
source <workspace>/install/setup.bash
sudo "$(command -v setup_dds_receive_buffer.sh)"
sudo "$(command -v setup_dds_send_buffer.sh)"
```

CycloneDDSを使うprocessは16 MiBのsend bufferを要求します。

```xml
<Internal>
  <SocketSendBufferSize min="16 MiB" max="16 MiB"/>
</Internal>
```

設定後にDDS processを再起動します。

## 設定

`config/default_config.yaml`で以下の設定が可能です：

- **カメラ選択**: auto/serial/name/index
- **解像度・フレームレート**: 320x240/10fps (Raspberry Pi対応)
- **ストリーム設定**: color/depth/infrared/pointcloud
- **圧縮設定**: JPEG品質、圧縮トピック有効/無効
- **TF設定**: フレーム名、変換パラメータ
- **サービス設定**: get_distance/get_camera_info/set_mode

## トピック

### 配信トピック

- `color/image_raw` - カラー画像
- `color/image_raw/compressed` - 圧縮カラー画像
- `depth/image_rect_raw` - 深度画像
- `depth/colormap` - 深度カラーマップ
- `depth/color/points` - ポイントクラウド
- `color/camera_info` - カメラ情報
- `depth/camera_info` - 深度カメラ情報

### 購読トピック

- `click_event` - クリックイベント（geometry_msgs/Point）

### サービス

- `get_distance` - 2D座標から3D座標への変換
- `get_camera_info` - カメラ情報取得
- `set_mode` - 表示モード設定（0: 表示なし、1: カーソルのみ、2: カーソル+座標+距離）

## 表示モード（キャリブレーション用）

fv_realsenseは3つの表示モードをサポートしています：

### モード0: 表示なし
- クリックしても何も表示されない
- マーカーや座標情報を表示しない

### モード1: カーソルのみ
- クリック時に緑色のカーソルを表示（10秒間）
- 座標や距離情報は表示しない

### モード2: カーソル + 座標 + 距離
- クリック時に緑色のカーソルを表示（10秒間）
- XY座標（ピクセル座標）を表示
- XYZ座標（3D距離、メートル単位）を表示

### モード切り替え例

```bash
# モード0（表示なし）に設定
ros2 service call /fv_realsense/set_mode fv_realsense/srv/SetMode "{mode: 0}"

# モード1（カーソルのみ）に設定
ros2 service call /fv_realsense/set_mode fv_realsense/srv/SetMode "{mode: 1}"

# モード2（カーソル + 座標 + 距離）に設定
ros2 service call /fv_realsense/set_mode fv_realsense/srv/SetMode "{mode: 2}"
```

### 使用方法

1. ノードを起動
2. モードを設定（上記コマンド）
3. `click_event`トピックにクリック座標を送信
4. 画像にマーカーが表示される（10秒間）

## 例

### 距離測定サービス

```bash
# ピクセル座標(320, 240)の3D座標を取得
ros2 service call /fv_realsense/get_distance \
    fv_realsense/srv/GetDistance \
    "{x: 320, y: 240, frame_id: 'color_optical_frame'}"
```

### カメラ情報取得

```bash
ros2 service call /fv_realsense/get_camera_info \
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
python3 src/sensors/fv_realsense/scripts/list_cameras.py

# USB権限確認
lsusb | grep RealSense

# udevルール確認
sudo cat /etc/udev/rules.d/99-realsense-libusb.rules
```

### D405のフレームが欠損する

```bash
fv-uvcvideo status
```

`Fluent Vision override: installed`と`UVC URBs: 32`が表示されることを確認します。

`install`が「uvcvideoはすでに読み込まれている」と警告した場合は、カメラ利用プロセスを停止して`--reload`付きで再実行するか、OSを再起動します。

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
