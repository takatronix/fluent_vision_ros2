# FV Aspara Analyzer

アスパラガスの3D点群データを分析し、形状品質を判定するROS2ノード

## 概要

fv_aspara_analyzerは、YOLOによる2D検出結果とRealSenseの点群データを組み合わせて、アスパラガスの品質分析を行います。

## 主要機能

### 1. 点群フィルタリング
- **2D→3D投影**: YOLO検出のbounding boxに対応する点群のみを抽出
- **距離フィルタリング**: 指定距離範囲内の点群のみを処理
- **カメラ歪み補正**: RealSenseカメラの歪み補正を適用

### 2. ノイズリダクション
- **Voxel Grid Filtering**: 5mmサイズでダウンサンプリング
- **Statistical Outlier Removal**: 統計的外れ値除去
- **パラメータ調整可能**: 近傍点数と標準偏差しきい値

### 3. アスパラガス品質分析
- **根本位置推定**: 点群の最下部から根本位置を特定
- **真っ直ぐ度計算**: PCAによる主成分分析で曲がり度を測定
- **長さ測定**: 点群の範囲から実際の長さを算出
- **収穫可能判定**: 23-50cmの長さ範囲と真っ直ぐ度で判定

## 入力データ

### 必須トピック
- **検出結果**: `/fv/d415/object_detection/detections` (vision_msgs/Detection2DArray)
- **点群データ**: `/fv/d415/points2` (sensor_msgs/PointCloud2)  
- **カメラ情報**: `/fv/d415/camera_info` (sensor_msgs/CameraInfo)

### オプション
- **セグメンテーションマスク**: `/fv/d415/segmentation_mask/image` (sensor_msgs/Image)

## 出力データ

### 発行トピック
- **フィルタリング済み点群**: `aspara_filtered_pointcloud` (sensor_msgs/PointCloud2)

### TFフレーム
- **根本位置**: `aspara_[id]_root` (geometry_msgs/TransformStamped)

### ログ出力例
```
[INFO] Aspara ID:0, Confidence:0.85, Length:0.267m, Straightness:0.82, Harvestable:YES
```

## パラメータ設定

### 基本設定
```yaml
min_confidence: 0.5           # 最低信頼度
pointcloud_distance_min: 0.1  # 最小処理距離
pointcloud_distance_max: 2.0  # 最大処理距離
```

### ノイズリダクション設定
```yaml
noise_reduction_neighbors: 50  # 統計フィルタの近傍点数
noise_reduction_std_dev: 1.0   # 標準偏差倍数
voxel_leaf_size: 0.005         # ボクセルサイズ(5mm)
```

### 品質判定設定
```yaml
harvest_min_length: 0.23      # 最小収穫長さ(23cm)
harvest_max_length: 0.50      # 最大収穫長さ(50cm)  
straightness_threshold: 0.7   # 真っ直ぐ度しきい値
```

## 使用方法

### ビルド
```bash
cd /ros2_ws
colcon build --packages-select fv_aspara_analyzer
source install/setup.bash
```

### 起動（D415カメラ）
```bash
ros2 launch fv_aspara_analyzer fv_aspara_analyzer_d415_launch.py
```

### 起動（D405カメラ）
```bash
ros2 launch fv_aspara_analyzer fv_aspara_analyzer_d405_launch.py
```

### デバッグモード
```bash
ros2 launch fv_aspara_analyzer fv_aspara_analyzer_d415_launch.py log_level:=debug
```

## トピック確認

### 処理結果確認
```bash
# フィルタリングされた点群をRVizで確認
ros2 run rviz2 rviz2

# TFフレーム確認
ros2 run tf2_tools view_frames

# ログ確認
ros2 topic echo /rosout
```

## 技術仕様

### 処理フロー
1. **YOLO検出結果受信** → 最高信頼度のアスパラを選択
2. **点群フィルタリング** → 2D bounding boxに対応する3D領域を抽出
3. **ノイズ除去** → Voxel + Statistical filtering
4. **品質分析** → 根本推定 + 真っ直ぐ度 + 長さ測定
5. **結果出力** → 点群とTF発行、ログ出力

### パフォーマンス
- **処理時間**: ~100-200ms/フレーム (点群サイズ依存)
- **メモリ使用量**: ~10-20MB (点群とノイズ除去処理)

## 依存関係

- ROS2 Humble
- PCL (Point Cloud Library)
- OpenCV
- vision_msgs
- tf2

## トラブルシューティング

### よくある問題

1. **点群が取得できない**
   - RealSenseノードが起動しているか確認
   - トピック名が正しいか確認

2. **検出結果が来ない**
   - fv_object_detectorが起動しているか確認
   - 信頼度しきい値が適切か確認

3. **ノイズが多い**
   - `noise_reduction_neighbors`を増加
   - `noise_reduction_std_dev`を減少

4. **処理が重い**
   - `voxel_leaf_size`を増加（より粗いダウンサンプリング）
   - 点群の距離範囲を狭める