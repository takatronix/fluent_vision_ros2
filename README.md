# FluentVision ROS2

ROS2ベースのリアルタイムビジョンシステム

## パッケージ構成

### AI/ML (`src/ai/`)
- **fv_object_detector**: YOLOv10を使用した物体検出システム
- **fv_object_mask_generator**: UNet使用のセマンティックセグメンテーション

### センサー (`src/sensors/`)
- **fv_realsense**: Intel RealSense深度カメラドライバ（D415, D405対応）

### ストリーミング (`src/streaming/`)
- **fv_recorder**: 録画・再生システム
- **fv_player**: バッグファイル再生

## システム構成

### カメラ
- **D415**: `/fv/d415/color/image_raw`, `/fv/d415/depth/image_rect_raw`
- **D405**: `/fv/d405/color/image_raw`, `/fv/d405/depth/image_rect_raw`

### AI処理
- **物体検出**: YOLOv10による推論結果を画像上にバウンディングボックス描画
- **セグメンテーション**: UNetによるピクセル単位分類、推論時間・デバイス情報表示

## インストール

```bash
cd /home/aspara/seedbox-r1/fluent_vision_ros2
colcon build --symlink-install
source install/setup.bash
```

## 使用方法

### 全システム起動
```bash
cd launch
./start_fv.sh
```

### システム停止
```bash
cd launch
./stop_fv.sh
```

### トピック確認
```bash
ros2 topic list | grep fv
```

## 作者

Takashi Otsuka (@takatronix)