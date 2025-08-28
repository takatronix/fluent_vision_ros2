## stem_detector v3 現在の問題（イベント駆動・YOLO起点）

作成: 2025-08-17 / 対象: `fv_stem_detector`

### 目的
- 重い処理が「カラー/深度到着ごと」に走ってしまう現状を廃止し、YOLO検出イベントをトリガに最新のカラー/深度で処理するイベント駆動型へ刷新する。
- 2Dと3Dの信頼度表記を厳密に分離（UI混同の解消）。
- フレーム同期/古さチェック/プロファイリングを標準装備し、負荷と遅延を可視化。

---

## 現状の問題点（抜粋）
- `onImageDepth()`（カラー/深度到着）で常に検出処理が走り、YOLOが無いフレームでも重い処理が実行される。
- 非同期運用で深度の古さチェックが弱く、古いDepthでの処理や再試行が起きうる。
- オーバーレイの「2D信頼度（YOLO）」と「3D信頼度（PCA）」が混同して表示される。

---

## 基本方針（イベント駆動）
- サブスクライブは継続: Color, Depth, CameraInfo, YOLO(Detection2DArray)。
- 最新の Color/Depth をリングバッファ（または単一スロット）に保持。
- 処理のトリガは YOLO のコールバック `onDetections()` のみ。
  - 直近の Color/Depth から「時間差が閾値以下」のペアを取得して処理。
  - ペアが見つからなければスキップ（待機オーバーレイのみ）。
- `onImageDepth()` は原則「パブリッシュ有無の確認と keepalive 表示」のみ（重い処理をしない）。

---

## 入出力
- 購読
  - `camera_topic`: `sensor_msgs/Image` BGR8
  - `depth_topic`: `sensor_msgs/Image` 16UC1（D405: 0.1mm, D415: 1mm）
  - `camera_info_topic`: `sensor_msgs/CameraInfo`（カラー整列前提）
  - `detection_topic`: `vision_msgs/Detection2DArray`（YOLO）
- 発行
  - `output_annotated_image_topic`: 可視化画像（矩形・アプローチ点・root/tip・ラベル）
  - `output_detection_fixed_topic`: 固定領域の `StemDetectionArray`（必要時）
  - `output_detection_yolo_topic`: YOLO領域の `StemDetectionArray`

---

## フレームバッファ/同期仕様
- Color/Depth それぞれ直近 N=3 程度を保持（リング）。
- YOLO到着時に以下でペア選定:
  1. `|t_color - t_depth| <= sync_slop_ms`
  2. `now - max(t_color, t_depth) <= buffer.max_age_ms`
  3. 見つからなければ最も近い組み合わせを試し、閾値超ならスキップ
- 既定: `sync_slop_ms=30`, `buffer.max_age_ms=150`（D405/D415運用に合わせ調整）

---

## 処理フロー（YOLOイベント）
1) YOLOバウンディングボックス群を受信
2) フレームペア取得（上記バッファ仕様）
3) 各BBoxに対し ROI を切出し
   - 方式A: `2d_depth`（既存最小）
     - 2Dの形状抽出→最下点候補→Depthバリデーション
     - 出力: 2D cut point（px）と3D座標（カメラ座標, m）
   - 方式B: `pointcloud_linefit`（v2のPCA直線）
     - ROI→アプローチZ（中央値）→Z帯抽出→Groundカット→Voxel→SOR→PCA主軸→根本/頂点
     - 出力: root/tip/dir/length_m と 3D cut point（root）
4) オーバーレイ描画
   - ID, YOLO 2D信頼度（％：小数なし）、3D信頼度（PCA固有値比, ％）
   - アプローチ距離、長さ（cm）等
5) メッセージ発行（`StemDetectionArray` 等）
6) プロファイリングログ出力（後述）

---

## 表示仕様（2D/3D信頼度の分離）
- 行0: `アスパラ#<ID> <YOLO%>` ← YOLOの2D信頼度（Detection2Dのscore）
- 行1: `2D信頼度: <YOLO%>`
- 行2: `3D信頼度: <PCA%>`（`lambda1/sum(lambda)`×100、pointcloud_linefit時のみ）
- 行3: `アプローチ距離: <xx.x> cm`
- 行4: `長さ推定(直線): <xx.x> cm`（pointcloud_linefit時）
- 行5: `処理: <xx.x> ms`（各ROIの処理時間）

---

## パラメータ（新設/重要）
```yaml
# 例: /ros2_ws/config/fv_stem_detector_d405.yaml
event:
  mode: "yolo_trigger"        # yolo_trigger | legacy_sync
buffer:
  max_age_ms: 150              # 最新フレームの許容古さ
  sync_slop_ms: 30             # color-depth 時間差の許容
profile:
  enable: true
  every_n: 10                  # Nフレーム毎にログ
overlay:
  show_region_labels: true
  label_text_color_bgr: [255,255,255]
  approach_point_color_bgr: [255,0,255]
```

既存キーは継承（`detection.method`, `depth_scale_m`, `depth_band.*`, `voxel.*`, `sor.*`, `pca.min_confidence`, `length_gate.*`, `qos.*` など）。

---

## プロファイリングログ（RCLCPP_INFO）
- フレーム単位
  - `[frame] yolo=N regs=K pair_age=XXms total=YY.ms`
- ROI単位（pointcloud_linefit）
  - `[roi] make=?.?ms pass=?.?ms filt=?.?ms axis=?.?ms total=?.?ms pts(raw/pass/sor)=A/B/C conf=PP% len=LL.cm`
- ROI単位（2d_depth）
  - `[roi2d] morph=?.?ms cc=?.?ms pick=?.?ms depth_fix=?.?ms total=?.?ms`

---

## スレッド/負荷設計
- YOLOコールバックは軽量化し、実処理はワーカースレッドへディスパッチ（`rclcpp::executors::MultiThreadedExecutor` を想定）。
- バックプレッシャー: 同時実行中は新規ヨーロイベントを間引く（`skip_if_busy` オプション）。

---

## 実装タスク（マイルストーン）
1) イベント駆動骨格
   - バッファ（color/depth）とペア選定、`event.mode` 実装
   - `onImageDepth()` から重い処理を撤去
2) 2D+Depth 経路移植（現行最小）+ プロファイルログ
3) 3D（pointcloud_linefit）移植 + プロファイルログ
4) 表示の2D/3D信頼度分離 + 文言修正
5) YAMLパラメータ整理（D405/D415 初期値同梱）

---

## 互換性/移行
- 既存のトピック名は維持。
- 旧 `use_sync` は `event.mode=legacy_sync` として残すが既定は `yolo_trigger` に変更。
- 2D/3D信頼度のUIは分離表示に変更（旧混同表示は廃止）。

以上。


