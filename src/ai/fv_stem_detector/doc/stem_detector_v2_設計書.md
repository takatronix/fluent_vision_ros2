## stem_detector v2 設計書（茎推定に特化・段階導入）

作成日: 2025-08-17 / 対象: `fv_stem_detector`

### 目的・範囲
- 本ノードは「茎推定」に特化し、2D検出（YOLO矩形）を入力として、ROI点群の前処理→軸（根本/頂点/長さ）推定を行う。
- v2では以下を段階導入: 1)アプローチ点取得 2)Z帯抽出 3)地面カット 4)ノイズ/点群調整 5)根本/頂点/長さ推定。将来拡張として 6)ボーン/曲がり 7)太さ。

---

## 入出力（想定）

- 購読
  - `camera_topic`: `sensor_msgs/Image`（BGR8）
  - `depth_topic`: `sensor_msgs/Image`（16UC1）
  - `camera_info_topic`: `sensor_msgs/CameraInfo`
  - `detection_topic`: `vision_msgs/Detection2DArray`（YOLO矩形）
  - （任意）`registered_points_topic`: `sensor_msgs/PointCloud2`（カラー整列済み・organized）

- 発行
  - `output_annotated_image_topic`: BBoxや推定ベクトルのオーバーレイ付画像（任意）
  - `output_roi_pointcloud_topic`: Z帯適用後のROI点群（デバッグ/評価用）
  - `output_stem_vectors_topic`: 茎ベクトル配列（root/tip/length/dir/confidence）（新規）

---

## 依存/再利用

- 既存処理の移植元
  - アプローチ点（中央値）・Z帯抽出: `fv_aspara_analyzer` v2 最小実装を準用
  - ノイズ除去（Voxel/SOR）: legacy `aspara_pointcloud_processor` と `fluent_lib/fluent_cloud/filters.hpp`
  
  

- ライブラリ
  - PCL: VoxelGrid, StatisticalOutlierRemoval,（将来）RANSAC直線/平面
  - OpenCV: 2D描画/補助

---

## 処理ステップ（段階導入）

### 1) アプローチポジション取得（移植）
- 入力: YOLOの矩形（Detection2D）
- 手順: BBox中心近傍の小窓から深度を収集し、中央値で `approach_z` を算出
- 主パラメータ
  - `approach.depth.window_px`（既定: 5）
  - `approach.depth.min_valid`（既定: 5）
  - `approach.depth.strategy`（`median` 固定）
  - `depth_unit_m_16u`（D405=0.0001, D415=0.001）

### 2) Z座標での帯域抽出（±5cm、移植）
- ROI=YOLO矩形を深度解像度に合わせてスライス→画素ごと3D復元（fx,fy,cx,cy）
- フィルタ: z ∈ [approach_z − minus_m, approach_z + plus_m]
- 主パラメータ
  - `depth_band.enable`（既定: true）
  - `depth_band.minus_m`（既定: 0.05）
  - `depth_band.plus_m`（既定: 0.05）

### 3) 地面カット（地表面/ベッド除去）
- 既定: PassThrough でカメラ光学座標の `y` 軸をクリップ（REP103で Y は下方向+）。
  - 理由: 2)のZ帯（前後±）で奥行き外乱は抑制済み。残る地面・土の混入は画面下側（大きいY）に集中するため、`y` でのしきいが合理的。
- 任意: RANSAC 平面（将来ON）
  - ROI下帯から地面平面をRANSAC推定し、距離しきい内を除去。PassThroughで不足する場面（カメラチルト大）に対応。
- 主パラメータ（例）
  - `ground_cut.enable`（bool, 既定: true）
  - `ground_cut.method`（`pass_through` 既定 | `plane`）
  - `ground_cut.axis`=`y`（`pass_through`用）
  - `ground_cut.min`/`max`（`pass_through`用）
  - `ground_cut.auto_max_p90_offset_m`（既定: 0.005 = 5mm。ROI内点群の`y`の90パーセンタイル + このオフセットを `max` として自動適用。ログに推定値を出力し、後で固定値に調整）
  - `ground_cut.plane.distance_thresh_m`（例: 0.01〜0.02）
  - `ground_cut.plane.sample_band_ratio`（下部帯域比、例: 0.2）

### 4) ノイズリダクション/点群調整
- VoxelGrid（ダウンサンプリング）→ StatisticalOutlierRemoval（SOR）の順で安定化
- 主パラメータ（D405/D415の推奨初期値）
  - D405: `voxel.leaf_m`=0.003〜0.004, `sor.mean_k`=40〜60, `sor.stddev_mul`=1.0〜1.5
  - D415: `voxel.leaf_m`=0.004〜0.006, `sor.mean_k`=40〜60, `sor.stddev_mul`=1.0〜1.5

### 5) 根本・頂点・長さ推定（v2コア）
- 軸推定: PCAで主成分ベクトル `v_axis` を取得（最大固有値の固有ベクトル）
- 投影: 各点を `v_axis` に射影し、スカラー最小/最大を `s_min/s_max` とする
- 根本/頂点: `p_root = c + s_min * v_axis`, `p_tip = c + s_max * v_axis`（`c`は点群重心）
- 長さ: `length_m = ||p_tip - p_root||`
- 信頼度例: 点数、PCA固有値比、SOR除去率から合成

--- （オプション） ---

### 6) ボーン（骨格）/曲がり推定（将来）
- 軸に沿ったスライスで逐次重心を辿り骨格ポリラインを生成→曲率（移動窓）で曲がり量を定量化
- 代替: RANSAC直線との差分や二次曲線フィットの残差で「曲がり」を数値化
- 主パラメータ例
  - `skeleton.slice_step_m`（例: 0.01）
  - `skeleton.curvature.window_count`（例: 5〜9）

### 7) 太さ推定（将来）
- 軸からの半径分布（点の軸距離）をロバスト統計（中央値/IQR）で半径推定→直径=2r
- 先端/根本近傍は外乱が多いため中央帯のみ採用
- 主パラメータ例
  - `thickness.central_ratio`（例: 0.3〜0.6）
  - `thickness.percentile`（例: 50=中央値）

---

## メッセージ仕様（新規案）

```
# msg/StemVector.msg（新規案）
std_msgs/Header header
int32 id
geometry_msgs/Point root_camera
geometry_msgs/Point tip_camera
geometry_msgs/Vector3 dir_camera
float32 length_m
float32 confidence

# msg/StemVectorArray.msg（新規案）
std_msgs/Header header
StemVector[] stems
```

備考: v2初期は `id` は未付与でも可（追跡導入時に付与）。

---

## パラメータ（最小セット + 推奨初期値）

共通
- `depth_unit_m_16u`: D405=0.0001, D415=0.001
- `approach.depth.window_px`=5, `approach.depth.min_valid`=5, `approach.depth.strategy`="median"
- `depth_band.enable`=true, `depth_band.minus_m`=0.05, `depth_band.plus_m`=0.05
- `ground_cut.enable`=true, `ground_cut.method`="pass_through", `ground_cut.axis`="y"
- `voxel.leaf_m`, `sor.mean_k`, `sor.stddev_mul`（カメラ別値を推奨）

長さ・信頼度（デフォルト）
- `min_length_m`=0.10, `max_length_m`=0.40
- `cut_eligible_min_length_m`=0.23（カット対象の下限。出力自体は `min_length_m` で制御し、UI/ロボット側で0.23以上を選別する想定）
- `min_confidence`=0.60（PCA固有値比ベース。定義: `lambda1 / (lambda1+lambda2+lambda3)`）
- `yolo.min_confidence`=0.40（YOLOの検出信頼度フィルタ。PCA信頼度とは独立に評価）

カメラ別推奨
- D405（近接・高分解能）
  - `voxel.leaf_m`: 0.003〜0.004
  - `sor.mean_k`: 50
  - `sor.stddev_mul`: 1.2
- D415（中距離）
  - `voxel.leaf_m`: 0.004〜0.006
  - `sor.mean_k`: 50
  - `sor.stddev_mul`: 1.2

ゲート閾値（ID対応付け用・初期値）
- `max_root_distance_m`: D405=0.04, D415=0.06
- `max_angle_diff_deg`: 20
- `max_length_diff_m`: 0.05

---

## QoS/パフォーマンス
- 初期は `best_effort` / 小サイズキューで十分。ネットワーク越しや録画要件で `reliable` に切替。
- 計算はROI内のみ。Z帯抽出後の点数は数千規模想定→Voxel/SOR後は数百〜千。

---

## 可視化/デバッグ
- 画像: BBox, アプローチZ、推定ベクトル（root→tip）を重畳
- 点群: `output_roi_pointcloud_topic` をRVizで確認（色=RGB）
- ログ: 点数、PCA固有値比、長さ[m]、信頼度、処理時間ms

### 出力アノテーション仕様（`output_annotated_image_topic`）
- テキスト表示（BBox左上付近、複数行）
  1. `アスパラ#<ID> <3D信頼度%>`（小数なし）
  2. `クラスID: 0`（将来 1=茎 に拡張可）
  3. `2D信頼度: <YOLO%>`（小数なし）
  4. `3D信頼度: <PCA%>`（`lambda1/(lambda1+lambda2+lambda3)` を%化、小数なし）
  5. `アプローチ距離: <xx.x> cm`（一桁小数）
  6. `長さ推定(直線): <xx.x> cm`（一桁小数）

- 図形描画
  - 根本: 赤丸（BGR: [0,0,255]、半径6px、太さ2）
  - 穂先: 黄色丸（BGR: [0,255,255]、半径6px、太さ2）
  - 根本↔穂先の直線: 水色（BGR: [255,255,0]、太さ1px、LINE_AA）

- 既定の描画パラメータ（必要最小）
  - `overlay.text_color_bgr`: [255,255,255]
  - `overlay.text_scale`: 0.6
  - `overlay.text_thickness`: 2
  - `overlay.point_radius`: 6
  - `overlay.point_thickness`: 2
  - `overlay.axis_line_thickness`: 1
  - `overlay.root_color_bgr`: [0,0,255]
  - `overlay.tip_color_bgr`: [0,255,255]
  - `overlay.axis_line_color_bgr`: [255,255,0]

---

## 実装計画（段階）
1. v2-Stage1: 1)アプローチ 2)Z帯 3)地面PassThrough 4)Voxel/SOR 5)長さ + 3D IDトラッカ（必須）
2. v2-Stage2: 3)地面RANSAC/閾値自動化、ID安定化（世界座標/誤対応抑制）
3. v2-Stage3: 6)曲がり、7)太さ

---

## 決定事項（v2.1）
- 地面カット軸: 既定 `y`。RANSAC は後段で導入（初期は PassThrough のみ）。
- 出力フレーム: 既定 `color_optical_frame`。`output_frame_id` で変更可とする。
- 点群ソース: 初期は Depth+CameraInfo のROI復元を使用。`use_registered_points` フラグで organized点群に切替可能にする（既定OFF）。
- ID付与: v2初期から実装（トラッカ必須）。

---

## 未確定事項・相談
- 地面カット軸: 現場のカメラ姿勢で `y`/`z` どちら基準が適切か（座標系定義の確認）。
- `registered_points` の利用方針: ROIが小さいため現状はDepth→復元で十分だが、organized点群を使う切替を入れるか。
- 3Dフレーム: 出力frame_idは `color_optical_frame` で良いか（D405/D415共通運用）。
- ID管理: v2初期は未付与で、後続のトラッカ導入時に `id` 付与でよいか。

---

## 参考（再利用箇所）
- Voxel/SOR（fluent_lib）: `src/fluent_vision_ros2/src/common/fluent_lib/include/fluent_lib/fluent_cloud/filters.hpp`
- legacy パイプライン: `src/.../fv_aspara_analyzer/legacy/src/aspara_pointcloud_processor.cpp`

以上。


