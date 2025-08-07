# FV Aspara Analyzer 設計書

## 概要

アスパラガスの 3D 点群データを分析し、形状品質を判定する ROS2 ノード

## ノード情報

- **ノード名**: `fv_aspara_analyzer`
- **パッケージ**: `fv_aspara_analyzer`
- **設定ファイル**:
  - `fv_aspara_analyzer_d405.yaml` (RealSense D405 用)
  - `fv_aspara_analyzer_d415.yaml` (RealSense D415 用)

## 入力データ仕様

### カメラデータ (fv_realsense ノードから)

- **カラー画像**: `/fv/d405/color/image_raw` または `/fv/d415/color/image_raw`
- **点群データ**: `/fv/d405/points2` または `/fv/d415/points2`
- **カメラ情報**: `/fv/d405/camera_info` または `/fv/d415/camera_info`
- **カメラ TF**: `camera_link` → `camera_color_optical_frame`

### 検出データ (fv_object_detector ノードから)

- **アスパラ検出結果**:
  - D405: `/fv/d405/object_detection/detections`
  - D415: `/fv/d415/object_detection/detections`
- **信頼度**: 0.0-1.0 の範囲
- **クラス**: "Asparagus", "Spear"

### マスクデータ (fv_object_mask_generator ノードから)

- **アスパラマスク画像**:
  - D405: `/fv/d405/segmentation_mask/image`
  - D415: `/fv/d415/segmentation_mask/image`
- **マスク形式**: バイナリマスク (0=背景, 1=アスパラガス)
- **マスクの信頼性**: 白い部分（値=1）にはほぼ確実にアスパラガスが存在
- **注意点**: マスクが存在しない場合もある（検出失敗時）
- **カラーマスク**:
  - D405: `/fv/d405/segmentation_mask/colored`
  - D415: `/fv/d415/segmentation_mask/colored`

## 設定パラメータ

### 基本設定

```yaml
# 処理対象のアスパラの最低信頼度
min_confidence: 0.5

# 有効点群データの距離範囲
pointcloud_distance_min: 0.1 # 最小距離 (m)
pointcloud_distance_max: 2.0 # 最大距離 (m)

# 根本推定パラメータ
root_estimation_baseline_ratio: 0.1 # 根本推定時のベースライン比率

# アスパラフィルター距離
aspara_filter_distance: 0.05 # 5cm

# 収穫条件
harvest_conditions:
  min_length: 0.23 # 収穫可能な最小長さ（23cm）
  max_length: 0.50 # 収穫可能な最大長さ（50cm）
  min_confidence: 0.7 # 収穫判定の最低信頼度
```

### 主成分分析（PCA）パラメータ

```yaml
# 次元削減後の目標次元数
pca_target_dimensions: 2

# 分散の説明率の閾値（0.0-1.0）
pca_explained_variance_ratio: 0.90 # 90%

# PCA計算の最小点数
pca_min_points: 10
```

### 曲がり度判定パラメータ

```yaml
# 曲がり度の閾値（正規化された値）
curvature_threshold: 0.3

# グレード判定の境界値
grade_good_threshold: 0.3 # この値以下ならグレード1（良）
grade_bad_threshold: 0.6 # この値以上ならグレード2（悪）
```

### ID 管理パラメータ

```yaml
# ハイブリッドID追跡設定（推奨）
id_tracking:
  # 基本設定
  max_frames_without_update: 15 # 5秒（15フレーム）更新なしで削除
  id_stabilization_frames: 3 # 3フレーム以内のID変更を吸収

  # 2D追跡設定
  position_2d_threshold: 0.05 # 2D位置の閾値（5cm）
  size_threshold: 0.1 # サイズ閾値（10%）
  confidence_threshold: 0.3 # 信頼度閾値

  # 3D追跡設定（より正確）
  position_3d_threshold: 0.03 # 3D位置の閾値（3cm）
  velocity_threshold: 0.1 # 速度閾値（10cm/s）

  # 特徴点追跡設定
  use_feature_tracking: true # 特徴点追跡使用
  min_feature_matches: 3 # 最小特徴点マッチング数
  feature_confidence_weight: 0.4 # 特徴点の重み

  # 追跡方法の優先順位
  tracking_methods:
    - "3d_position" # 最優先：3D位置
    - "feature_points" # 次優先：特徴点
    - "2d_position" # 最後：2D位置
```

# メモリ管理

memory_management:
max_aspara_count: 20 # 最大アスパラガス数
cleanup_interval: 1.0 # クリーンアップ間隔（秒）
max_memory_usage: 100 # 最大メモリ使用量（MB）

````

## 主成分分析（PCA）の詳細仕様

### PCA の目的

- **次元削減**: 3D 点群データを 2 次元に圧縮
- **形状特徴抽出**: アスパラガスの本質的な形状を表現
- **計算効率化**: 後続処理の高速化

### 次元数の選択理由

- **2 次元**: アスパラガスの曲がりを表現するのに最適
- **1 次元**: 直線アスパラのみ対応（曲がり表現不可）
- **3 次元**: 計算量増加で効果薄い

### 分散説明率の意味

- **90%**: 元のデータの 90%の情報を保持
- **95%以上**: 次元削減効果が薄い
- **85%以下**: 形状情報の損失が大きい

### PCA座標系のTF生成方法

#### 1. 主成分軸の計算
```cpp
// PCA結果から主成分軸を取得
Eigen::Vector3f principal_axis_1 = pca_result.eigenvectors.col(0);  // 第1主成分
Eigen::Vector3f principal_axis_2 = pca_result.eigenvectors.col(1);  // 第2主成分
Eigen::Vector3f principal_axis_3 = principal_axis_1.cross(principal_axis_2); // 外積でZ軸
```

#### 2. TF行列の構築
```cpp
// 回転行列の構築
Eigen::Matrix3f rotation_matrix;
rotation_matrix.col(0) = principal_axis_1;  // X軸
rotation_matrix.col(1) = principal_axis_2;  // Y軸
rotation_matrix.col(2) = principal_axis_3;  // Z軸

// 平行移動（点群の重心）
Eigen::Vector3f centroid = pca_result.mean;
```

#### 3. TFの正規化
```cpp
// 回転行列の正規化（直交行列の保証）
rotation_matrix = rotation_matrix.normalized();
```

## 曲がり度判定の詳細仕様

### 曲がり度の計算方法

1. **PCA 結果から曲率計算**:

   ```python
   # 2次元点群から曲率を計算
   curvature = calculate_curvature(pca_points)
````

2. **角度変化の累積**:

   ```python
   # 連続する点間の角度変化を累積
   angle_change = sum(abs(angle_diff[i] - angle_diff[i-1]) for i in range(1, n))
   ```

3. **直線からの偏差**:
   ```python
   # 理想的な直線からの平均偏差
   deviation = mean_distance_from_line(pca_points, ideal_line)
   ```

### グレード判定基準

- **グレード 0（未判定）**: 初期状態
- **グレード 1（良）**: 曲がり度 ≤ 0.3
- **グレード 2（悪）**: 曲がり度 > 0.3

## 基本処理フロー

### 1. データ待機

- アスパラガス検出データの更新を待機
- 最低信頼度以上のアスパラを抽出
- 距離が近く、信頼度が高い順にソート

### 2. ID 管理・追跡

- **ハイブリッド追跡システム**: 3D 位置・特徴点・2D 位置を組み合わせた高精度追跡
- **3D 位置優先**: 根本位置の 3D 座標で最も正確な追跡
- **特徴点マッチング**: アスパラガスの表面特徴で補助追跡
- **2D 位置補完**: 3D 情報が不足時のフォールバック
- **ID 安定化**: 短時間の ID 変更を吸収（3 フレーム以内）
- **古いデータ削除**: 一定時間（5 秒）更新されないデータを自動削除
- **選択状態保持**: 追跡中のアスパラガスが選択状態を維持

### 3. アスパラ選択

- 現在選択中の ID がリストにない場合、先頭のアスパラを選択
- 選択状態を管理

### 4. 点群データ取得

- 有効距離範囲内の点群のみ抽出
- 距離フィルタリング適用

### 5. マスク画像取得

- 選択されたアスパラのマスク画像を取得
- 2D 矩形情報を抽出

### 6. 投射エリア切り抜き

- カメラ情報と TF から 3D 投影領域を計算
- アスパラ矩形領域の点群のみ抽出
- **セグメンテーションマスク活用**: マスクの白い部分（アスパラガス領域）の点群を優先的に抽出
- **マスク補完**: マスクが存在しない場合は矩形領域のみで処理

### 7. 根本推定

- 投射エリア下部の点群を分析
- ベースライン比率分上のラインをスキャン
- 点群密集ポイントを根本位置として特定
- TF として保存

### 8. 点群フィルタリング

- 根本からアスパラフィルター距離内の点群のみ保持
- 余計な領域の点群を削除

### 9. 主成分分析

- フィルタリングされた点群に PCA 適用
- 2 次元に次元削減
- 分散説明率をチェック
- **PCA 座標系の TF 生成**: 主成分軸を基準とした座標系を生成
- **TF の方向性**: 第 1 主成分軸を X 軸、第 2 主成分軸を Y 軸、外積で Z 軸を決定

### 10. 曲がり度判定

- PCA 結果から曲がり度を計算
- 閾値と比較してグレード判定

### 10.5. 収穫可能性判定

- アスパラガスの長さを測定（23cm以上で収穫可能）
- 品質判定結果を確認（グレード1以上で収穫準備完了）
- 収穫可能フラグと収穫準備完了フラグを設定

### 11. データ更新・クリーンアップ

- 分析結果をアスパラガス情報に保存
- 古いデータの自動削除（5 秒以上更新なし）
- メモリ使用量の監視

### 12. rtabmap DB登録（予定）

- **判定済みアスパラ**: 品質判定完了後、rtabmap DBに登録（予定）
- **収穫済みアスパラ**: カット完了後、rtabmap DBに登録（予定）
- **位置情報保存**: 3D座標とTF情報をDBに保存（予定）
- **品質情報保存**: グレード、曲がり度、長さなどの品質データを保存（予定）
- **収穫情報保存**: 収穫時刻、収穫位置、収穫者情報を保存（予定）

## 出力データ仕様

### トピック出力

```yaml
# 点群データ（予定）
aspara_[id]_pointcloud: sensor_msgs/PointCloud2

# PCA結果（予定）
aspara_[id]_pca: sensor_msgs/PointCloud2

# TF出力（予定）
aspara_[id]_pca_tf: geometry_msgs/TransformStamped
aspara_[id]_root_tf: geometry_msgs/TransformStamped
```

### サービス

```yaml
# 次のアスパラ選択（予定）
select_next_aspara: std_srvs/Trigger

# 前のアスパラ選択（予定）
select_prev_aspara: std_srvs/Trigger

# アスパラ判定完了（予定）
mark_aspara_judged: std_srvs/Trigger

# アスパラ収穫完了（予定）
mark_aspara_harvested: std_srvs/Trigger

# 収穫可能アスパラ一覧取得（予定）
get_harvestable_aspara: std_srvs/Trigger
```

## アスパラガス情報構造体

```cpp
struct AsparaInfo {
    int id;                           // アスパラID
    int stable_id;                    // 安定化されたID（追跡用）
    bool is_selected;                 // 選択状態
    float confidence;                 // 信頼度
    float length;                     // 長さ (m)
    float curvature;                  // 曲がり度
    int grade;                        // グレード (0:未判定, 1:良, 2:悪)
    bool is_harvestable;              // 収穫可能フラグ（23cm以上）
    bool is_harvest_ready;            // 収穫準備完了フラグ（品質判定済み）

    // 追跡情報
    ros::Time last_update_time;       // 最後の更新時刻
    int frames_without_update;        // 更新なしフレーム数
    geometry_msgs::Point last_position_3d; // 前回の3D位置
    cv::Rect last_bounding_box;       // 前回の2D矩形
    std::vector<cv::KeyPoint> last_features; // 前回の特徴点
    float last_velocity;              // 前回の速度
    int tracking_confidence;          // 追跡信頼度（0-100）

    // 2D情報
    cv::Rect bounding_box_2d;         // 2D矩形情報
    cv::Mat image_2d;                 // 2D画像
    cv::Mat mask_2d;                  // 2Dマスク
    geometry_msgs::TransformStamped tf_2d;  // 2D画像のTF

    // 3D情報
    sensor_msgs::PointCloud2 pointcloud;     // 点群データ
    geometry_msgs::TransformStamped tf_3d;   // 点群のTF
    cv::Mat mask_3d;                         // 3Dマスク
    geometry_msgs::TransformStamped tf_mask_3d; // 3DマスクのTF

    // 分析結果
    sensor_msgs::PointCloud2 pca_result;     // PCA結果
    geometry_msgs::TransformStamped tf_pca;  // PCAのTF
    geometry_msgs::Point root_position_3d;   // 根本の3D座標
    geometry_msgs::Point root_position_2d;   // 根本の2D座標
    geometry_msgs::TransformStamped tf_root; // 根本のTF

    // 検出情報
    geometry_msgs::TransformStamped tf_detection; // 検出したアスパラのTF
    
    // rtabmap DB情報（予定）
    bool is_registered_to_db;     // DB登録済みフラグ（予定）
    std::string db_node_id;       // rtabmap DBのノードID（予定）
    ros::Time db_registration_time; // DB登録時刻（予定）
    bool is_harvested;            // 収穫済みフラグ（予定）
    ros::Time harvest_time;       // 収穫時刻（予定）
};
```

## エラーハンドリング

### 主要エラーケース

1. **点群データ不足**: 最小点数未満の場合
2. **PCA 計算失敗**: 特異値分解エラー
3. **PCA 座標系生成失敗**: 主成分軸の計算エラー
4. **根本推定失敗**: 密集ポイントが見つからない
5. **マスク取得失敗**: マスク画像が存在しない（検出失敗時）
6. **セグメンテーション失敗**: マスクが全く生成されない場合
7. **ID 追跡失敗**: 同一アスパラガスの追跡が困難
8. **メモリ不足**: アスパラガス数が上限を超過
9. **rtabmap DB接続失敗**: DBへの登録が失敗（予定）
10. **DB登録重複**: 同一アスパラガスの重複登録（予定）

### エラー対応

- ログ出力による詳細なエラー情報
- デフォルト値による処理継続
- エラー状態のサービス応答
- **マスク失敗時の対応**: 矩形領域のみで処理継続
- **セグメンテーション失敗時の対応**: 検出結果のみで処理継続
- **PCA 失敗時の対応**: デフォルト座標系で処理継続
- **ID 追跡失敗時の対応**: 新しい ID として処理継続
- **メモリ不足時の対応**: 古いデータを強制削除
- **rtabmap DB失敗時の対応**: ローカルキャッシュに保存、後で再試行（予定）
- **収穫判定失敗時の対応**: デフォルトで収穫不可として処理（予定）

## パフォーマンス要件

### 処理時間

- 1 本のアスパラ分析: 100ms 以下
- 複数アスパラ同時処理: 500ms 以下

### メモリ使用量

- 点群データ: 最大 10MB
- 画像データ: 最大 5MB
- 総メモリ使用量: 50MB 以下

## 設定ファイル例

### fv_aspara_analyzer_d405.yaml

```yaml
fv_aspara_analyzer:
  ros__parameters:
    # カメラ設定
    camera_topic: "/fv/d405/color/image_raw"
    pointcloud_topic: "/fv/d405/points2"
    camera_info_topic: "/fv/d405/camera_info"

    # 検出・マスク設定
    detection_topic: "/fv/d405/object_detection/detections"
    mask_topic: "/fv/d405/segmentation_mask/image"
    colored_mask_topic: "/fv/d405/segmentation_mask/colored"

    # 基本パラメータ
    min_confidence: 0.5
    pointcloud_distance_min: 0.1
    pointcloud_distance_max: 2.0
    root_estimation_baseline_ratio: 0.1
    aspara_filter_distance: 0.05

    # PCAパラメータ
    pca_target_dimensions: 2
    pca_explained_variance_ratio: 0.90
    pca_min_points: 10

    # 曲がり度パラメータ
    curvature_threshold: 0.3
    grade_good_threshold: 0.3
    grade_bad_threshold: 0.6
```

### fv_aspara_analyzer_d415.yaml

```yaml
fv_aspara_analyzer:
  ros__parameters:
    # カメラ設定
    camera_topic: "/fv/d415/color/image_raw"
    pointcloud_topic: "/fv/d415/points2"
    camera_info_topic: "/fv/d415/camera_info"

    # 検出・マスク設定
    detection_topic: "/fv/d415/object_detection/detections"
    mask_topic: "/fv/d415/segmentation_mask/image"
    colored_mask_topic: "/fv/d415/segmentation_mask/colored"

    # 基本パラメータ
    min_confidence: 0.5
    pointcloud_distance_min: 0.1
    pointcloud_distance_max: 2.0
    root_estimation_baseline_ratio: 0.1
    aspara_filter_distance: 0.05

    # PCAパラメータ
    pca_target_dimensions: 2
    pca_explained_variance_ratio: 0.90
    pca_min_points: 10

    # 曲がり度パラメータ
    curvature_threshold: 0.3
    grade_good_threshold: 0.3
    grade_bad_threshold: 0.6
```

## 実装時の注意点

### 1. メモリ管理

- 点群データの適切な解放
- 画像データのメモリリーク防止

### 2. スレッド安全性

- 複数アスパラの同時処理
- 共有データの排他制御

### 3. エラー回復

- 一時的なデータ取得失敗の処理
- システムの安定性確保

### 4. ログ出力

- デバッグ情報の詳細出力
- エラー状態の明確な記録
