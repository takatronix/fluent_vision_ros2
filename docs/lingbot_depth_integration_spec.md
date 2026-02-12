# FluentVision x LingBot-Depth 統合仕様書

## 1. 目的
- `ros2_ws` の既存深度ノード群を前提に、`LingBot-Depth` の深度補完・補正機能を `fluent_vision_ros2` に取り込む。
- 取り込み可否、必要依存、ROS2 I/O 仕様、実装方針を定義する。

## 2. 既存ノード整理（調査結果）

### 2.1 fluent_vision_ros2 側
- `fv_realsense`:
  - 入力: RealSense センサー
  - 出力: `color/image_raw`, `depth/image_rect_raw`, `depth/camera_info`, `depth/color/points` 等
  - 役割: 生 RGB-D 配信
- `fv_depth_features`:
  - `fv_depth_bilateral_node`: 深度平滑化
  - `fv_depth_normals_node`: 法線・RGBD生成
  - `fv_pointcloud_node`: 深度→点群
- `fv_pointcloud_pipeline`:
  - ROI/マスク連携で点群フィルタ、地面除去、クラスタ抽出

### 2.2 vlabor_ros2 側（参照）
- `vlabor_launch/depth_camera_node.py`: UVC深度カメラ配信
- `vlabor_launch/depth_normals_node.py`: 深度法線/RGBD生成
- 既存運用は「RGB/Depth/CameraInfo を同期し、下流で点群処理」が中心。

## 3. LingBot-Depth 調査結果（一次情報ベース）
- モデル API: `MDMModel.from_pretrained(...)` / `model.infer(image, depth_in, intrinsics, ...)`
- 入力:
  - RGB: `[B,3,H,W]` (0..1)
  - raw depth: `[B,H,W]` (meter)
  - intrinsics: 正規化済み `[B,3,3]` (`fx/W, fy/H, cx/W, cy/H`)
- 出力:
  - `depth`（補正後深度）
  - `points`（カメラ座標系点群）
  - `mask`（有効領域）
- 依存:
  - `torch==2.6.0`, `torchvision`, `xformers==0.0.29.post2`, `opencv-python`, `huggingface_hub` など
  - 初回実行時に Hugging Face から `model.pt` ダウンロード可能

## 4. 取り込み可否
- 結論: **取り込み可能**
- 理由:
  - `fv_realsense` が `RGB + depth + camera_info` を既に配信しており、LingBot-Depth 入力要件を満たす。
  - 出力の `refined depth` と `points` は既存の `fv_depth_features`/`fv_pointcloud_pipeline` と接続可能。
- 前提:
  - GPU 実行環境（CUDA）を推奨。CPU 実行は遅延が大きい可能性が高い。
  - `xformers` と PyTorch バージョン整合が必要。

## 5. 統合ノード仕様（新規）

### 5.1 ノード名
- `fv_lingbot_depth`（パッケージ: `fv_lingbot_depth`）

### 5.2 入力トピック
- `color_topic`（`sensor_msgs/msg/Image`, 既定: `/fv/d405/color/image_raw`）
- `depth_topic`（`sensor_msgs/msg/Image`, 既定: `/fv/d405/depth/image_rect_raw`）
- `camera_info_topic`（`sensor_msgs/msg/CameraInfo`, 既定: `/fv/d405/depth/camera_info`）

### 5.3 出力トピック
- `refined_depth_topic`（`sensor_msgs/msg/Image`, `32FC1`, 既定: `depth_refined/image_rect_raw`）
- `mask_topic`（`sensor_msgs/msg/Image`, `mono8`, 既定: `depth_refined/mask`）
- `pointcloud_topic`（`sensor_msgs/msg/PointCloud2`, XYZ float32, 既定: `depth_refined/points`）

### 5.4 同期方式
- `ApproximateTimeSynchronizer`（RGB/Depth/CameraInfo 3トピック）
- パラメータ:
  - `sync_queue_size`（既定: 10）
  - `sync_slop_sec`（既定: 0.03）

### 5.5 主要パラメータ
- モデル/推論:
  - `model_id`（HF repo id）
  - `local_model_path`（ローカル重み優先）
  - `device`（`auto|cuda|cpu`）
  - `use_fp16`
  - `apply_mask`
  - `resolution_level`
- 深度処理:
  - `depth_scale_16uc1`（既定: 0.001）
  - `min_depth_m`, `max_depth_m`
  - `point_stride`
- 運用:
  - `fallback_passthrough`（モデル未ロード時に入力深度をそのまま出す）
  - `frame_id_override`

### 5.6 処理フロー
1. RGB/Depth/CameraInfo 同期受信
2. 深度を meter に統一（16UC1 は `depth_scale_16uc1` で変換）
3. intrinsics 正規化（`fx/W, fy/H, cx/W, cy/H`）
4. LingBot-Depth 推論
5. `depth`/`mask`/`points` を ROS2 トピックへ publish
6. 点群は stride・距離閾値・有限値でフィルタして配信

## 6. 必要な追加依存（導入時）
- ROS 側:
  - `rclpy`, `message_filters`, `cv_bridge`, `sensor_msgs`, `std_msgs`
- Python 側（pip/venv 推奨）:
  - `torch==2.6.0`, `torchvision`, `xformers==0.0.29.post2`
  - `opencv-python`, `numpy`, `huggingface_hub`
  - `lingbot-depth` 本体（`pip install -e .` もしくは同等の import 可能状態）

## 7. 導入ステップ（推奨）
1. PoC: 新ノード単体で `d405` 1台入力に対して補正深度を確認
2. 接続: 下流の `fv_pointcloud_pipeline` 入力を raw depth から refined depth に切替
3. 評価: 欠損率、法線品質、点群ノイズ、処理FPSを比較
4. 本番: 複数カメラ（D405/D415）展開、モデル選択（pretrain/DC）最適化

## 8. リスクと対策
- リスク: GPUメモリ不足 / 推論遅延
  - 対策: 解像度低減、`resolution_level` 調整、stride増加、片側カメラ優先
- リスク: 依存バージョン衝突（torch/xformers）
  - 対策: 専用venvと固定バージョン運用
- リスク: モデル未取得時の停止
  - 対策: `fallback_passthrough=true` で運転継続

## 9. 実装ステータス
- 本仕様に基づき、`fv_lingbot_depth` ノードを新規作成済み（本コミットで追加）。

## 10. fluent_vision_system への組み込み方針
- `fluent_vision_system` ランチャは `params_file` に加えて `nodes[].parameters`（inline dict）をサポート。
- 既存 `/config/fluent_vision_system.yaml` には、以下どちらでも統合可能:
  1. `params_file` で `fv_lingbot_depth_d405.yaml` を参照
  2. `parameters` を直接定義（推奨: 構成管理しやすい）
- サンプル:
  - `src/system/fluent_vision_system/config/fv_lingbot_depth_d405.yaml`
  - `src/system/fluent_vision_system/config/fluent_vision_system.lingbot_depth.sample.yaml`
