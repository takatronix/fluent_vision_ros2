## 3D ID管理 設計 v2（茎ベクトルベース）

作成日: 2025-08-17 / 対象: `fv_stem_detector`

### 目的
- YOLOの2D矩形に依存したIDではなく、3Dの茎ベクトル（root/tip/dir/length）を用いて、長さ範囲でのゲート＋安定なID付与を実現する。

---

## 入出力（前提）
- 入力
  - `vision_msgs/Detection2DArray`（YOLO矩形）
  - `sensor_msgs/Image`（BGR8）, `sensor_msgs/Image`（16UC1 depth）, `sensor_msgs/CameraInfo`
  - （任意）`sensor_msgs/PointCloud2`（organized registered points）
- 出力
  - `StemVectorArray`（新規案）: id, root, tip, dir, length_m, confidence
  - `sensor_msgs/PointCloud2`（任意）: ROI Z帯後の点群（デバッグ）

---

## パイプライン概要
1) 候補生成（既存移植）
   - YOLO矩形→アプローチZ（中央値）→Z帯抽出（±5cm）→Voxel/SOR→PCA主軸
   - `p_root`, `p_tip`, `dir`, `length_m`, `conf` を算出

2) 事前ゲート（長さ・信頼度）
   - `min_length_m <= length_m <= max_length_m`（デフォルト: 0.10〜0.40）
   - `confidence >= min_confidence`（PCA固有値比, 既定: 0.60）
   - 参考: カット対象の長さ下限は 0.23m（`cut_eligible_min_length_m`）
   - ゲート外はID付与対象外（捨てる）

3) データアソシエーション（既存Trackとの対応付け）
   - ゲーティング閾値
     - ルート距離: `max_root_distance_m`
     - 角度差: `max_angle_diff_deg`
     - 長さ差: `max_length_diff_m`
   - コスト関数（割当最適化）
     - `cost = w_root * ||Δroot|| + w_angle * θ + w_len * |Δlength|`
     - θは方向ベクトルのなす角（degまたはrad）
   - 割当アルゴリズム
     - N小規模（≲15）: ハンガリアン法
     - N中規模: ゲート内近傍のみで貪欲 or KD-Tree＋局所最適

4) トラック更新（IDの存続）
   - 割当済み: 平滑更新（EMA）
     - `root = (1-α_pos)*root + α_pos*root_new`
     - `dir` は正規化ベクトルを球面線形補間（近似: 正規化EMA）
     - `length` は`α_len`でEMA
   - 未割当: `missed_count++`。`missed_count > max_missed_frames`で破棄
   - 新規: 未割当候補は `birth_suppression_frames` を考慮し重複生成を抑制

5) 出力
   - `StemVectorArray` として id 付き茎ベクトル配列を発行
   - 任意: ROI Z帯点群（可視化）

---

## パラメータ（最小セット + 推奨初期）

- 長さ・信頼度（デフォルト）
  - `min_length_m`=0.10, `max_length_m`=0.40
  - `cut_eligible_min_length_m`=0.23（運用上のカット対象下限）
  - `min_confidence`=0.60（PCA固有値比: `lambda1/(lambda1+lambda2+lambda3)`）
  - `yolo.min_confidence`=0.40（YOLOの検出フィルタ。PCAの信頼度とは独立）

- ゲーティング
  - `max_root_distance_m`: D405=0.03〜0.05, D415=0.05〜0.08
  - `max_angle_diff_deg`: 20
  - `max_length_diff_m`: 0.05

- アソシエーション重み
  - `w_root`=1.0, `w_angle`=0.3, `w_len`=0.2

- トラック寿命/確定
  - `max_missed_frames`=5, `min_hits_to_confirm`=2, `birth_suppression_frames`=2

- 平滑化
  - `ema.alpha_pos`=0.4, `ema.alpha_dir`=0.3, `ema.alpha_len`=0.3

- 座標/フレーム
  - `output_frame_id`: 既定 `color_optical_frame`（カメラフレームで運用）
  - `world_frame`（任意）: 将来TFでの世界追跡に拡張可

カメラ別（推奨）
- D405: `max_root_distance_m` を小さめ（0.04付近）、Voxel=0.003〜0.004、SOR mean_k=50, std=1.2
- D415: `max_root_distance_m` をやや緩め（0.06付近）、Voxel=0.004〜0.006、SOR mean_k=50, std=1.2

点群ソース
- `use_registered_points`=false（既定）。Depth+CameraInfoでROI復元。

---

## 用語補足（ゲート閾値とは）
- データアソシエーションの前に「明らかに不適切な対応」を除外するための閾値。
- 今回は、候補と既存IDの組み合わせが以下を満たすものだけを割当候補とする:
  - ルート距離 ≤ `max_root_distance_m`
  - 角度差 ≤ `max_angle_diff_deg`
  - 長さ差 ≤ `max_length_diff_m`
 これにより、誤対応を抑え、ハンガリアンの計算も軽くなる。

---

## メッセージ（新規案）
```
# msg/StemVector.msg
std_msgs/Header header
int32 id
geometry_msgs/Point root_camera
geometry_msgs/Point tip_camera
geometry_msgs/Vector3 dir_camera
float32 length_m
float32 confidence

# msg/StemVectorArray.msg
std_msgs/Header header
StemVector[] stems
```
備考: 初期はid未使用でも可。トラッカ導入とともに付与。

---

## 可視化/表示
- 画像: `アスパラ#ID {Confidence}%` の形式でラベル表示（小数なし、%のみ）
- 点群: ROI Z帯の`PointCloud2`をRVizで表示（色=RGB）

---

## 実装ノート
- PCAはEigenで実装。RANSAC平面/直線はPCL（将来）。
- KD-Tree（pcl::KdTreeFLANN）で近傍検索を効率化。
- 処理はROI内のみでスループット確保。ハンガリアンはNが少ない前提で採用。

---

## 未確定事項・質問
1. ラベル表示の仕様（UIに合わせる）
2. 将来の `world_frame` 追跡要否
3. RANSAC平面導入のタイミング（初期はPassThroughで運用）

---

## 参照
- 点群前処理: `fluent_lib/fluent_cloud/filters.hpp`
- 既存候補生成: `fv_aspara_analyzer` v2 最小点群抽出

以上。


