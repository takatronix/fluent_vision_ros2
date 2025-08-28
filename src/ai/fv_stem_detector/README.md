## fv_stem_detector — アスパラ茎検出ノード

指定された複数の矩形領域内から、アスパラ茎の最下点（カットポイント）を検出します。結果はカラー画像へのオーバーレイ（矩形＋赤丸）と、各領域の `region_id, x, y, detected` 配列として公開します。

### 入出力トピック
- カラー画像入力: `camera_topic` (`sensor_msgs/Image` BGR8)
- 深度画像入力: `depth_topic` (`sensor_msgs/Image` 16UC1 等)
- 点群入力（任意）: `pointcloud_topic` (`sensor_msgs/PointCloud2`)
- オーバーレイ画像出力: `output_annotated_image_topic` (`sensor_msgs/Image`)
- 検出配列出力（固定領域）: `output_detection_fixed_topic` (`fv_stem_detector/StemDetectionArray`)
- 検出配列出力（YOLO領域）: `output_detection_yolo_topic` (`fv_stem_detector/StemDetectionArray`)

### 出力トピック詳細
- 画像オーバーレイ: `output_annotated_image_topic`
  - **型**: `sensor_msgs/Image`（`BGR8`）
  - **内容**: 入力カラー画像に、固定領域=赤枠、YOLO領域=青枠、検出カットポイント=赤丸を重畳
  - **ヘッダ**: `header` は入力カラー画像のものを継承、`frame_id` は `publish_frame_id`（既定: `fv_d405_color_optical_frame`）
- 検出（固定領域）: `output_detection_fixed_topic`
  - **型**: `fv_stem_detector/StemDetectionArray`
  - **内容**: パラメータ `check_regions`/`check_regions_flat` で与えた固定矩形ごとの検出結果（2D: `x,y`、3D[m]: `camera_x,camera_y,camera_z`）
- 検出（YOLO領域）: `output_detection_yolo_topic`
  - **型**: `fv_stem_detector/StemDetectionArray`
  - **内容**: サブスクライブした `vision_msgs/Detection2DArray`（`detection_topic`）の各BBoxを動的領域として用いた検出結果（2D: `x,y`、3D[m]: `camera_x,camera_y,camera_z`）。`region_id` はメッセージ到着順の連番

メッセージ仕様（抜粋）
```
msg/StemDetection.msg
int32 region_id
int32 x
int32 y
bool detected
float32 camera_x  # カメラ座標系[m]
float32 camera_y  # カメラ座標系[m]
float32 camera_z  # カメラ座標系[m]

msg/StemDetectionArray.msg
fv_stem_detector/StemDetection[] detections
```

### 主なパラメータ
- `check_regions`（YAML 側で配列指定）例:
  ```yaml
  check_regions:
    - { id: 0, x: 100, y: 100, w: 240, h: 320 }
    - { id: 1, x: 400, y: 120, w: 220, h: 300 }
  ```
- ノード側は `check_regions_flat`（`[id,x,y,w,h,...]`）で受け取り可能
- 描画設定: `overlay.rect_color_bgr`, `overlay.rect_thickness`, `overlay.cut_point_color_bgr`, `overlay.cut_point_radius`, `overlay.cut_point_thickness`
- 追加描画設定: `overlay.fixed_rect_color_bgr`（赤）, `overlay.yolo_rect_color_bgr`（青）
- YOLO検出トピック: `detection_topic`（`vision_msgs/Detection2DArray`）
- QoS: `qos.input_*`, `qos.output_*`
- カメラ内参: `camera_info_topic`（例: `/fv/d405/color/camera_info`）
- 深度スケール[m]: `depth_scale_m`（既定: `0.001`。`depth` が 16UC1 の mm 相当なら 0.001）

### 座標系（3D出力）
- `x_m,y_m,z_m` は `publish_frame_id` のカメラ光学座標系（REP 103）で[m]。一般に Z: 前方+, X: 右+, Y: 下+

### 実行例
```bash
ros2 run fv_stem_detector stem_detector_node \
  --ros-args --params-file /ros2_ws/config/fv_stem_detector_d405.yaml \
  -r __node:=fv_stem_detector_d405
```

### 設計書
- `doc/アスパラ茎検出ノード設計書.md` を参照
 - `doc/アルゴリズム詳細.md` を参照


