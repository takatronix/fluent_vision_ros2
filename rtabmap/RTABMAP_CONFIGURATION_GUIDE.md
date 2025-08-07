# RTABMap 設定ガイド

## 概要

RTABMap (Real-Time Appearance-Based Mapping) の包括的な設定ガイドです。公式ソースから調査した情報をもとに、必須設定項目と重要なパラメータをまとめています。

## 調査したリソース

- **GitHub**: https://github.com/introlab/rtabmap
- **ROS2パッケージ**: https://github.com/introlab/rtabmap_ros
- **公式Wiki**: http://wiki.ros.org/rtabmap_ros
- **チュートリアル**: 高度なパラメータ調整、ロボット設定など

---

## 1. 必須設定項目

### 1.1 フレーム設定（TF構築）
```yaml
frame_id: "base_link"              # ロボットベースフレーム
map_frame_id: "map"               # マップフレーム  
odom_frame_id: "odom"             # オドメトリフレーム
```

**重要**: TF（座標変換）の構築は必須です。以下の変換が必要：
- `map` → `odom` （RTABMapが発行）
- `odom` → `base_link` （オドメトリノードが発行）
- `base_link` → `camera_link` （静的変換）
- `base_link` → `lidar_frame` （静的変換）

### 1.2 センサー入力設定
```yaml
subscribe_depth: true              # RGB-D使用
subscribe_stereo: false           # ステレオカメラ使用（depthと排他）
subscribe_scan: true              # LiDAR使用
subscribe_rgb: true               # RGB画像使用
```

### 1.3 トピック設定
```yaml
rgb_topic: "/fv/d415/color/image_raw"
depth_topic: "/fv/d415/depth/image_rect_raw"
camera_info_topic: "/fv/d415/color/camera_info"
scan_topic: "/livox/lidar"
imu_topic: "/livox/imu"          # オプション（精度向上）
odom_topic: "/odom"
```

### 1.4 基本モード設定
```yaml
Mem/IncrementalMemory: "true"     # true=マッピング, false=ローカライゼーション
database_path: "rtabmap.db"       # データベースファイルパス
```

---

## 2. 重要設定項目

### 2.1 視覚オドメトリ
```yaml
Odom/Strategy: "0"                # 0=Frame-to-Map, 1=Frame-to-Frame
Kp/DetectorStrategy: "6"          # 特徴点検出器（6=GFTT推奨）
Kp/MaxFeatures: "1000"           # 最大特徴点数
Vis/MinInliers: "12"             # 最小インライア数
```

### 2.2 ループクロージャ
```yaml
Rtabmap/DetectionRate: "1.0"      # 検出レート（Hz）
Rtabmap/LoopThr: "0.11"          # ループ閾値（低いほど厳しい）
Rtabmap/TimeThr: "0"             # 時間閾値（0=無制限）
```

### 2.3 最適化設定
```yaml
Optimizer/Strategy: "1"           # 0=TORO, 1=g2o, 2=GTSAM（g2o推奨）
Optimizer/Iterations: "20"        # 最適化反復回数
Reg/Force3DoF: "false"           # 3DoF制約（地上移動ロボットはtrue）
```

### 2.4 グリッドマップ
```yaml
Grid/FromDepth: "true"            # 深度画像からグリッド生成
Grid/CellSize: "0.05"            # セルサイズ（m）
Grid/RangeMax: "8.0"             # 最大処理範囲（m）
Grid/3D: "false"                 # 2D占有格子地図
```

---

## 3. 用途別設定例

### 3.1 屋内マッピング
- **特徴**: 詳細な特徴点検出、小さなセルサイズ
- **設定**: `configs/indoor_mapping.yaml`
- **特徴点数**: 多め（1000点）
- **セルサイズ**: 細かく（0.05m）
- **範囲**: 近距離（5-8m）

### 3.2 屋外マッピング
- **特徴**: 処理負荷軽減、長距離対応
- **設定**: `configs/outdoor_mapping.yaml`  
- **特徴点数**: 控えめ（500点）
- **セルサイズ**: 粗く（0.1m）
- **範囲**: 長距離（15-20m）

### 3.3 ローカライゼーション
- **特徴**: 既存マップ使用、位置推定専用
- **設定**: `configs/localization.yaml`
- **モード**: `Mem/IncrementalMemory: false`
- **データベース**: 読み取り専用

---

## 4. センサー構成別設定

### 4.1 RGB-Dカメラ構成
- **D415（遠距離）**: メインカメラ、8m範囲
- **D405（近距離）**: サブカメラ、1m範囲
- **設定**: 深度範囲の調整が重要

### 4.2 LiDAR + RGB-D構成
- **Livox LiDAR**: 3D点群、長距離対応
- **IMU**: 姿勢安定化
- **設定**: ICP統合による高精度位置推定

### 4.3 マルチセンサー統合
- **同期設定**: `approx_sync: true`
- **キューサイズ**: `queue_size: 30`
- **タイムアウト**: `wait_for_transform: 0.2`

---

## 5. パフォーマンス調整

### 5.1 処理軽減設定
```yaml
Mem/ImagePreDecimation: "2"       # 画像前処理縮小
Kp/MaxFeatures: "300"            # 特徴点数制限
Rtabmap/DetectionRate: "0.5"     # 検出レート低下
```

### 5.2 メモリ最適化
```yaml
Mem/WorkingMemorySize: "12"      # ワーキングメモリ制限
Mem/STMSize: "30"                # 短期メモリ制限
Mem/ReduceGraph: "true"          # グラフ簡約化
```

### 5.3 リアルタイム設定
```yaml
Optimizer/Iterations: "10"        # 反復数削減
Grid/DepthDecimation: "2"        # 深度間引き
```

---

## 6. ROS2での起動方法

### 6.1 Launch ファイル
```bash
ros2 launch rtabmap/launch/rtabmap_rgbd.launch.py \
    config:=indoor_mapping \
    localization:=false \
    rtabmapviz:=true
```

### 6.2 スクリプト実行
```bash
# マッピング開始
./scripts/start_mapping.sh --indoor

# ローカライゼーション開始  
./scripts/start_localization.sh --database rtabmap.db

# モード切り替え
./scripts/switch_mode.sh toggle

# ステータス確認
./scripts/check_status.sh
```

---

## 7. よく使用される設定パラメータ

### 7.1 品質vs速度のトレードオフ
| パラメータ | 高品質 | 高速処理 | 説明 |
|-----------|--------|----------|------|
| Kp/MaxFeatures | 1000 | 300 | 特徴点数 |
| Optimizer/Iterations | 30 | 10 | 最適化反復 |
| Mem/ImagePreDecimation | 1 | 4 | 画像縮小率 |
| Rtabmap/DetectionRate | 2.0 | 0.5 | 検出レート |

### 7.2 環境別推奨設定
| 環境 | セルサイズ | 最大範囲 | 特徴点数 | 検出レート |
|------|-----------|----------|----------|-----------|
| 屋内精密 | 0.05m | 5m | 1000 | 2.0Hz |
| 屋内標準 | 0.05m | 8m | 800 | 1.0Hz |
| 屋外標準 | 0.1m | 15m | 500 | 1.0Hz |
| 屋外高速 | 0.2m | 20m | 300 | 0.5Hz |

---

## 8. トラブルシューティング

### 8.1 よくある問題
1. **TF不備**: 座標変換が正しく設定されていない
2. **トピック同期**: センサーデータの時刻同期問題  
3. **メモリ不足**: 長時間動作でのメモリリーク
4. **処理遅延**: リアルタイム処理の遅れ

### 8.2 解決方法
1. **TF確認**: `ros2 run tf2_tools view_frames`
2. **同期設定**: `approx_sync: true`, `queue_size: 30`
3. **メモリ制限**: `Mem/WorkingMemorySize`, `Mem/STMSize` 調整
4. **パフォーマンス**: `performance.yaml` 設定使用

---

## 9. アスパラガス農業向けカスタマイズ

### 9.1 特殊設定
```yaml
# 作物検出用関心領域
Kp/RoiRatios: "0.0 0.2 1.0 0.8"  # 中央60%領域に注目

# 地面レベル調整（畑の凹凸対応）
Grid/MaxGroundAngle: "30"         # 傾斜地対応
Grid/MaxGroundHeight: "0.2"       # 地面高さ許容範囲

# 近距離詳細観察（アスパラガス検出）
Kp/MinDepth: "0.3"               # D405近距離対応
Kp/MaxDepth: "8.0"               # D415遠距離活用
```

### 9.2 データ保存設定
```yaml
# アスパラガス位置記録用
database_path: "asparagus_map_$(date).db"

# セッション管理
Rtabmap/CreateIntermediateNodes: "true"  # 詳細軌道記録
```

---

## 10. まとめ

RTABMapは100以上のパラメータを持つ高度なSLAMシステムです。用途に応じた適切な設定により、以下が実現できます：

- **高精度マッピング**: 屋内環境での詳細3Dマップ作成
- **ロバストなローカライゼーション**: 既存マップでの確実な自己位置推定  
- **リアルタイム処理**: パフォーマンス最適化による即応性
- **マルチセンサー統合**: RGB-D + LiDAR + IMU の効果的活用

このガイドの設定ファイルとスクリプトを活用することで、効率的にRTABMapを導入・運用できます。