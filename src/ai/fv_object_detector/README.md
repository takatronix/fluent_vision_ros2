# FV Object Detector

FluentVision用の物体検知マイクロサービスです。シンプルで再利用しやすい設計で、物体検知機能を独立したサービスとして提供します。

## 機能

- **物体検知**: YOLOv10を使用したリアルタイム物体検知
- **オブジェクトトラッキング**: フレーム間でのオブジェクトID追跡
- **アノテーション画像**: 検出結果を描画した画像の出力
- **検出情報**: 標準的なROS2メッセージ形式での検出結果出力
- **設定可能**: YAML設定ファイルによる柔軟な設定

## 入力/出力

### 入力
- **画像トピック**: `sensor_msgs/msg/Image`
  - デフォルト: `/camera/color/image_raw`

### 出力
- **アノテーション画像**: `sensor_msgs/msg/Image`
  - デフォルト: `/object_detection/annotated_image`
- **検出情報**: `vision_msgs/msg/Detection2DArray`
  - デフォルト: `/object_detection/detections`

## 設定

### パラメータ

| パラメータ | デフォルト値 | 説明 |
|-----------|-------------|------|
| `input_image_topic` | `/camera/color/image_raw` | 入力画像トピック |
| `output_image_topic` | `/object_detection/annotated_image` | 出力画像トピック |
| `output_detections_topic` | `/object_detection/detections` | 検出結果トピック |
| `processing_frequency` | `10.0` | 処理頻度（Hz） |
| `enable_tracking` | `true` | オブジェクトトラッキング有効化 |
| `enable_visualization` | `true` | 可視化有効化 |

### モデル設定

モデル設定はJSONファイルで管理されます：

```json
{
  "model": {
    "name": "YOLOv10",
    "type": "yolov10",
    "path": "/home/takatronix/FluentVision/models/yolov10n.pt",
    "device": "CPU",
    "input_width": 640,
    "input_height": 640,
    "confidence_threshold": 0.5,
    "nms_threshold": 0.45,
    "min_area": 100.0
  },
  "classes": [
    "person", "bicycle", "car", ...
  ]
}
```

## 使用方法

### 1. ビルド

```bash
cd fluent_vision_ros2
colcon build --packages-select fv_object_detector
```

### 2. 起動

```bash
# 基本的な起動
ros2 launch fv_object_detector object_detector.launch.py

# カスタムトピックで起動
ros2 launch fv_object_detector object_detector.launch.py \
  input_topic:=/my_camera/image_raw \
  output_image_topic:=/my_detection/image \
  output_detections_topic:=/my_detection/objects
```

### 3. トピック確認

```bash
# トピック一覧確認
ros2 topic list | grep object_detection

# 検出結果確認
ros2 topic echo /object_detection/detections

# 画像確認（Foxglove Studio等で）
ros2 topic echo /object_detection/annotated_image
```

## アーキテクチャ

```
fv_object_detector/
├── include/fv_object_detector/
│   ├── detection_data.hpp      # 検出データ構造体
│   ├── object_tracker.hpp      # オブジェクトトラッカー
│   ├── ai_model.hpp           # AIモデル基底クラス
│   └── yolov10_model.hpp      # YOLOv10モデル
├── src/
│   ├── fv_object_detector_node.cpp  # メインノード
│   ├── object_tracker.cpp           # トラッカー実装
│   ├── ai_model.cpp                 # AIモデル実装
│   └── yolov10_model.cpp            # YOLOv10実装
├── config/
│   ├── default_config.yaml          # デフォルト設定
│   └── yolov10_config.json          # モデル設定
└── launch/
    └── object_detector.launch.py    # 起動ファイル
```

## 特徴

- **マイクロサービス設計**: 独立したサービスとして動作
- **再利用可能**: 他のシステムに簡単に統合可能
- **設定可能**: YAML/JSON設定ファイルによる柔軟な設定
- **標準準拠**: ROS2標準メッセージ形式を使用
- **高性能**: OpenVINOによる高速推論
- **トラッキング**: 連続性のあるオブジェクトID付与

## 依存関係

- ROS2 Humble
- OpenCV
- OpenVINO
- nlohmann/json
- yaml-cpp

## ライセンス

MIT License 