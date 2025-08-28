## AsparaDetection 連携インターフェース仕様（v4）

作成: 2025-08-17 / 対象: `fv_stem_detector` → `AparaDetection`（受け側）

### 目的
- `fv_stem_detector` がYOLO起点で推定した茎のカット情報を、低遅延・必要最小情報で `AparaDetection` に渡すためのI/Fを定義する。

---

## トピック一覧（送信: fv_stem_detector）
- YOLO矩形に対するカット情報（推奨・主トピック）
  - 名称: `output_detection_yolo_topic`（例: `/fv/d405/stem_detector/yolo_xy`）
  - 型: `fv_stem_detector/StemDetectionArray`
  - 周期: YOLOイベントごと（間引き可）

- 固定矩形のカット情報（任意・補助）
  - 名称: `output_detection_fixed_topic`（例: `/fv/d405/stem_detector/fixed_xy`）
  - 型: `fv_stem_detector/StemDetectionArray`
  - 周期: YOLOイベントごと（固定領域も毎回評価）

- 3Dベクトル情報（root/tip/dir/length, 信頼度）
  - 名称: `output_stem_vectors_topic`（例: `/fv/d405/stem_detector/stem_vectors`）
  - 型: `fv_stem_detector/StemVectorArray`（新規）
  - 周期: YOLOイベントごと（`detection.method=pointcloud_linefit` 時に発行）

---

## メッセージ拡張（後方互換）
既存 `msg/StemDetection.msg` に World座標と「画面可視」「観測時刻」を追記する（v4.1）。

```text
# 既存
int32 region_id
int32 x
int32 y
bool detected
float32 camera_x
float32 camera_y
float32 camera_z

# v4.1 で追記（後方互換）
float32 world_x   # TFで camera_optical から world_frame へ変換（失敗時 NaN）
float32 world_y
float32 world_z
bool    visible   # 今回のスナップショットで画面に見えていたか（ROI内にありDepth有効）
builtin_interfaces/Time observed_at  # 当該StemDetectionの観測時刻（Colorのheader.stamp準拠）
```

備考:
- `StemDetectionArray` は配列のまま（変更なし）。
- v4.0 実装では World/visible/observed_at は未送信（デフォルト値）。v4.1 で運用導入。

---

## 新規メッセージ: 3Dベクトル（root/tip/dir/length）
茎の3D幾何情報をまとめて提供するための追加I/F。

```text
# msg/StemVector.msg
std_msgs/Header header
int32 id                                # 追跡用ID（未使用時は-1）
geometry_msgs/Point root_camera         # カメラ座標系[m]
geometry_msgs/Point tip_camera
geometry_msgs/Vector3 dir_camera        # 正規化方向ベクトル
float32 length_m                        # 直線長[m]
float32 confidence                      # PCA由来の信頼度[0..1]

# 追加（任意, v4.1+）
geometry_msgs/Point root_world          # world_frame[m]（TF失敗時はNaNを埋める運用）
geometry_msgs/Point tip_world
geometry_msgs/Vector3 dir_world

# msg/StemVectorArray.msg
std_msgs/Header header
fv_stem_detector/StemVector[] stems
```

備考:
- `header.frame_id` は `publish_frame_id`（カメラ光学座標）を既定とし、world系を併記。
- `confidence` は PCA固有値比 `lambda1/(lambda1+lambda2+lambda3)` を想定。

---

## フィールドの意味と運用
- `region_id`: 対象領域のID。固定矩形は設定ID、YOLOは連番（追跡で安定化予定）。
- `x, y`: 画像上のカットポイント（px）。
- `detected`: 有効に確定できた場合 true。
- `camera_*`: カメラ光学座標（REP103, m）。深度が無効なら NaN。
- `world_*`: 指定 `world_frame`（既定: `world`）での座標（m）。TF失敗時 NaN。
- `visible`: 今回フレームで可視か（ROIが画像範囲内にあり、かつcut-pointのDepthが有効な場合にtrue）。
- `observed_at`: 観測時刻。Colorの`header.stamp`を転記（深度ずれは許容方針）。

---

## 追加メタ情報の受け渡し（当面の方針）
- YOLOの2D信頼度（score）はオーバーレイ表示に用い、メッセージには含めない（軽量維持）。
- 3D信頼度（PCA比）は将来 `StemVector` 系メッセージにて提供予定（設計 `3D_ID管理設計_v2.md` 参照）。
 - 可視性と観測時刻は本メッセージで提供（v4.1）。AparaDetection側で滞留時間や可視継続時間の計算に使用可能。

---

## パラメータ例（送信側）
```yaml
/**:
  ros__parameters:
    output_detection_yolo_topic: "/fv/d405/stem_detector/yolo_xy"
    output_detection_fixed_topic: "/fv/d405/stem_detector/fixed_xy"
    output_stem_vectors_topic: "/fv/d405/stem_detector/stem_vectors"
    world_frame: "world"           # TFターゲット（AparaDetectionの想定に合わせる）
    tf.timeout_sec: 0.05            # TF取得のタイムアウト
```

---

## AparaDetection 側での利用想定
- 受信: `StemDetectionArray` を購読し、各 `StemDetection` の `camera_* / world_*` をそのまま使用。
- 表示・判断:
  - 2D UI: `x, y` をガイド。
  - 3D UI/制御: `world_*` を主に参照（未サポート環境は `camera_*` を利用）。
- 欠損ハンドリング:
  - `detected=false` または `world_*` が NaN の要素は、UI表示のみ・制御対象外とする。

---

## バージョンと互換性
- v4.0: 既存 `StemDetectionArray` のまま（`camera_*` のみ有効）。
- v4.1: `world_*` 追記版を配信開始。AparaDetection は `world_*` が存在すれば優先使用、無い場合は `camera_*` にフォールバック。

以上。


