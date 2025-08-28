# FV Face Recognizer

FluentVision用のリアルタイム顔検出・認識システムです。

## 概要

fv_face_recognizerは、リアルタイムで顔を検出し、登録された顔データベースと照合して個人を識別するROS2ノードです。

## 機能

- **リアルタイム顔検出**: OpenCV DNNを使用した高速顔検出
- **顔認識**: 登録済み顔データベースとの照合
- **顔登録**: 新しい顔の登録・管理機能
- **可視化**: 検出結果の画像上への描画
- **設定可能**: YAMLファイルによる柔軟な設定

## 入力/出力

### 入力
- **画像トピック**: `sensor_msgs/msg/Image`
  - デフォルト: `/camera/color/image_raw`

### 出力
- **検出結果画像**: `sensor_msgs/msg/Image`
  - デフォルト: `/face_recognition/annotated_image`
- **検出情報**: `std_msgs/msg/String`
  - デフォルト: `/face_recognition/detections`
- **認識結果**: `std_msgs/msg/String`
  - デフォルト: `/face_recognition/recognitions`

## 設定

### パラメータ

| パラメータ | デフォルト値 | 説明 |
|-----------|-------------|------|
| `input_image_topic` | `/camera/color/image_raw` | 入力画像トピック |
| `output_image_topic` | `/face_recognition/annotated_image` | 出力画像トピック |
| `output_detections_topic` | `/face_recognition/detections` | 検出結果トピック |
| `output_recognitions_topic` | `/face_recognition/recognitions` | 認識結果トピック |
| `processing_frequency` | `10.0` | 処理頻度（Hz） |
| `confidence_threshold` | `0.5` | 検出信頼度しきい値 |
| `face_database_path` | `./face_database` | 顔データベースパス |

## 使用方法

### 1. ビルド

```bash
cd fluent_vision_ros2
colcon build --packages-select fv_face_recognizer
```

### 2. 起動

```bash
# 基本的な起動
ros2 launch fv_face_recognizer face_recognizer.launch.py

# カスタム設定で起動
ros2 launch fv_face_recognizer face_recognizer.launch.py \
  input_topic:=/my_camera/image_raw \
  confidence_threshold:=0.7
```

### 3. 顔登録

```bash
# 顔登録サービス
ros2 service call /face_recognizer/register_face \
  fv_face_recognizer/srv/RegisterFace \
  "{name: 'John Doe', image_topic: '/camera/color/image_raw'}"
```

## 依存関係

- ROS2 Humble
- OpenCV 4.x
- OpenCV DNN
- sensor_msgs
- cv_bridge
- std_msgs
- geometry_msgs
- visualization_msgs

## 開発状況

⚠️ **開発中**: このノードは現在開発中です。実装は完了していません。

## ライセンス

MIT License 