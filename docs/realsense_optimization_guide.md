# RealSense最適化ガイド - アスパラガス検出用

## 問題：なぜ重いのか？

### 1. **ポイントクラウド生成の負荷**
- 640x480 @ 15fps = **毎秒460万点**の3D計算
- 各点でRGB + XYZ = 6つの値
- 毎秒約110MBのデータ生成

### 2. **不要な処理**
- 全てのフィルタを有効化 → CPU負荷増大
- 圧縮処理 → さらにCPU負荷
- 高解像度・高FPS → 必要以上のデータ

## 最適設定の選び方

### アスパラガスの特性を考慮

1. **静的な対象物**
   - 動かない → 低FPS（6fps）で十分
   - 時間フィルタ不要

2. **サイズが既知**
   - 長さ：20-50cm
   - 太さ：1-2cm
   - 必要解像度：5mm程度

3. **検出範囲が限定的**
   - カメラから0.2-1.5m
   - 視野角の一部のみ使用

## 推奨設定

### 軽量版（CPU使用率 < 30%）
```yaml
camera:
  color_width: 424
  color_height: 240
  color_fps: 6
  depth_width: 424
  depth_height: 240
  depth_fps: 6

streams:
  pointcloud_enabled: false  # 重要！

pointcloud:
  min_distance: 0.2
  max_distance: 1.5
```

### 標準版（CPU使用率 30-50%）
```yaml
camera:
  color_width: 640
  color_height: 480
  color_fps: 6
  depth_width: 640
  depth_height: 480
  depth_fps: 6

streams:
  pointcloud_enabled: false
  
pointcloud:
  spatial_filter_enabled: true
  decimation_enabled: true
  decimation_scale: 2
```

### 高品質版（CPU使用率 50-70%）
```yaml
camera:
  color_width: 640
  color_height: 480
  color_fps: 15
  
streams:
  pointcloud_enabled: true
  
pointcloud:
  全フィルタ有効
```

## ポイントクラウド生成の代替手段

### 方法1：depth_image_procを使用（推奨）
```bash
# RealSenseはdepth/colorのみ配信
# 別ノードでポイントクラウド生成
ros2 run depth_image_proc point_cloud_xyzrgb_node \
  --ros-args \
  -r depth/image_rect:=/fv/d415/depth/image_rect_raw \
  -r depth/camera_info:=/fv/d415/depth/camera_info \
  -r rgb/image_rect_color:=/fv/d415/color/image_raw \
  -r rgb/camera_info:=/fv/d415/color/camera_info \
  -r points:=/fv/d415/points
```

### 方法2：FluentCloudで必要時のみ生成
```cpp
// 深度画像から必要な部分だけポイントクラウド化
auto cloud = FluentCloud::fromDepth(depth_image, camera_info)
    .cropBox(roi_min, roi_max)  // 関心領域のみ
    .downsample(0.005);         // 5mmボクセル
```

### 方法3：オンデマンド生成
```cpp
// 検出があった時だけポイントクラウド生成
if (yolo_detected_asparagus) {
    generatePointCloudForROI(detection_bbox);
}
```

## パフォーマンステスト方法

```bash
# 1. 現在の負荷確認
/tmp/test_realsense_performance.sh

# 2. CPU使用率モニタリング
htop  # realsenseプロセスを確認

# 3. トピック周波数確認
ros2 topic hz /fv/d415/points

# 4. データ量確認
ros2 topic bw /fv/d415/points
```

## トラブルシューティング

### Q: それでも重い
A: 
- USB3.0ポートを使用しているか確認
- 他のUSBデバイスを外す
- カーネルのUSB帯域幅制限を確認

### Q: ポイントクラウドが表示されない
A: 
- `pointcloud_enabled: false`の場合は外部生成が必要
- TFが正しく設定されているか確認
- frame_idが一致しているか確認

### Q: 精度が落ちた
A:
- min_distance/max_distanceを調整
- 解像度を段階的に上げてテスト
- 必要な部分だけ高解像度化

## まとめ

**アスパラガス検出には高解像度・高FPSは不要**

- 424x240 @ 6fps で十分な精度
- ポイントクラウドは外部生成
- 必要な領域だけ処理
- CPU使用率を30%以下に抑える

これにより、複数カメラの同時使用や、他の処理（AI推論等）のリソースを確保できます。