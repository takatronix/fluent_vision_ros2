# FV Recorder - FluentVision録画・再生システム

## 概要

FV Recorderは、ROS2トピックの録画・再生を行うシステムです。複数のカメラトピック（depth、color等）を同時に記録し、後でシミュレーションとして再生できます。

## クイックスタート（自動録画）

1) ビルドとセットアップ

```bash
colcon build --packages-select fv_recorder
source install/setup.bash
```

2) 設定ファイル（例）で自動録画を有効化（単一カメラ）

`src/streaming/fv_recorder/config/recorder_config.yaml`

```yaml
fv_recorder:
  ros__parameters:
    recording:
      input_topics:
        - "/test_camera/image_raw"
      output_directory: "/home/takatronix/recordings"
      segment_duration: 300
      retention_days: 7
      date_format: "YYYYMMDD"
      auto_recording: true
      default_format: "mp4"
    preview:
      enabled: true
      output_topic: "/fv_recorder/preview"
```

3) 設定ファイルを指定して起動

```bash
ros2 run fv_recorder fv_recorder_node --ros-args --params-file src/streaming/fv_recorder/config/recorder_config.yaml
```

起動直後に自動で録画が開始され、例として次のようなファイルが作成されます:

```
/home/takatronix/recordings/20250812_102804_segment_0.mp4
```

録画の開始/停止（トピック制御）

```bash
ros2 topic pub /fv_recorder/recording_control std_msgs/msg/Bool "data: true"  --once  # 開始
ros2 topic pub /fv_recorder/recording_control std_msgs/msg/Bool "data: false" --once  # 停止
```

### 動画へ時刻を焼き込む（オプション）

保存する動画に撮影時刻をオーバーレイして記録できます。

`config/recorder_config.yaml`

```yaml
fv_recorder:
  ros__parameters:
    recording:
      video_time_overlay: true
      video_time_overlay_format: "%Y-%m-%d %H:%M:%S"  # 例: 2025-08-12 10:28:04
```

実行時に直接指定する場合:

```bash
ros2 run fv_recorder fv_recorder_node --ros-args \
  -p recording.video_time_overlay:=true \
  -p recording.video_time_overlay_format:="%Y-%m-%d %H:%M:%S" \
  --params-file src/streaming/fv_recorder/config/recorder_config.yaml
```

## 機能

### 録画機能
- 複数トピックの同時録画
- 時間分割保存（設定可能な分数）
- ファイル名形式: `YYYYMMDD_HHMMSS_(topic).mp4`
- 古いファイルの自動削除

### 再生機能
- 録画データのシミュレーション再生
- 複数トピックの同期再生
- 日付指定による録画データ選択
- 再生速度調整

### 外部制御
- サービス呼び出しによる制御
- トピックメッセージによる制御
- 自動録画機能

## 複数トピック指定方法

### 1. 設定ファイルでの指定

`config/recorder_config.yaml`で複数トピックを配列として指定：

```yaml
fv_recorder:
  ros__parameters:
    recording:
      input_topics:
        - "/fv_realsense/color/image_raw"
        - "/fv_realsense/depth/image_raw"
        - "/fv_realsense/depth/colormap"
        - "/fv_realsense/pointcloud"
        - "/camera/left/image_raw"
        - "/camera/right/image_raw"
        - "/lidar/points"
        - "/imu/data"
        - "/gps/fix"
```

### 2. サービス呼び出しでの動的指定

録画開始時に動的にトピックを指定：

```bash
# カスタムトピックリストで録画開始
ros2 service call /fv_recorder/start_recording fv_recorder/srv/StartRecording "{
  recording_directory: '/home/user/recordings',
  date_format: '20241201',
  segment_duration: 300,
  retention_days: 7,
  input_topics: ['/fv_realsense/color/image_raw', '/fv_realsense/depth/image_raw', '/camera/left/image_raw']
}"

# 設定ファイルのデフォルトトピックを使用
ros2 service call /fv_recorder/start_recording fv_recorder/srv/StartRecording "{
  recording_directory: '/home/user/recordings',
  date_format: '20241201',
  segment_duration: 300,
  retention_days: 7,
  input_topics: []
}"
```

### 3. サポートするトピックタイプ

以下のROS2メッセージタイプをサポート：

- **画像トピック**:
  - `sensor_msgs/msg/Image` - カラー画像、深度画像
  - `sensor_msgs/msg/CompressedImage` - 圧縮画像

- **点群トピック**:
  - `sensor_msgs/msg/PointCloud2` - 3D点群データ

- **センサーデータ**:
  - `sensor_msgs/msg/Imu` - IMUデータ
  - `sensor_msgs/msg/NavSatFix` - GPSデータ
  - `sensor_msgs/msg/LaserScan` - レーザースキャンデータ

- **その他**:
  - `std_msgs/msg/String` - 文字列データ
  - `std_msgs/msg/Bool` - 真偽値データ
  - `geometry_msgs/msg/PoseStamped` - 位置姿勢データ

## すべてのROS2トピック対応

fv_recorderは**すべてのROS2トピック**を録画・再生できます！

### ✅ 対応メッセージタイプ

#### 基本メッセージタイプ
- `sensor_msgs/msg/Image` - 画像データ
- `std_msgs/msg/String` - 文字列データ
- `std_msgs/msg/Bool` - 真偽値データ

#### センサーメッセージ
- `sensor_msgs/msg/PointCloud2` - 3D点群データ
- `sensor_msgs/msg/Imu` - IMUデータ（加速度、角速度）
- `sensor_msgs/msg/NavSatFix` - GPSデータ
- `sensor_msgs/msg/LaserScan` - レーザースキャンデータ
- `sensor_msgs/msg/CompressedImage` - 圧縮画像データ

#### ジオメトリメッセージ
- `geometry_msgs/msg/PoseStamped` - 位置姿勢データ
- `geometry_msgs/msg/Twist` - 速度データ
- `geometry_msgs/msg/TransformStamped` - 座標変換データ

#### ナビゲーションメッセージ
- `nav_msgs/msg/Odometry` - オドメトリデータ
- `nav_msgs/msg/Path` - 経路データ
- `nav_msgs/msg/OccupancyGrid` - 占有格子地図

#### 標準メッセージ
- `std_msgs/msg/Float32` - 32ビット浮動小数点
- `std_msgs/msg/Int32` - 32ビット整数
- `std_msgs/msg/Header` - ヘッダー情報

#### その他のメッセージ
- **任意のカスタムメッセージ** - 汎用シリアライゼーション対応

### 🔧 汎用メッセージ処理

fv_recorderは以下の方法で任意のROS2メッセージを処理します：

#### 1. **汎用サブスクリプション**
- `rclcpp::GenericSubscription`を使用
- メッセージタイプを事前に知る必要がない
- 実行時にメッセージタイプを自動検出

#### 2. **メッセージタイプ推測**
トピック名からメッセージタイプを自動推測：
```cpp
// 推測ロジック例
if (topic_name.find("image") != std::string::npos) {
    return "sensor_msgs/msg/Image";
} else if (topic_name.find("pointcloud") != std::string::npos) {
    return "sensor_msgs/msg/PointCloud2";
} else if (topic_name.find("imu") != std::string::npos) {
    return "sensor_msgs/msg/Imu";
}
// ... その他の推測ロジック
```

#### 3. **フォールバック機能**
- 汎用サブスクリプションが失敗した場合
- 特定のメッセージタイプでサブスクリプションを試行
- 複数の方法でトピックへの接続を試行

### 📋 使用例

#### 任意のトピックを録画
```bash
# カスタムメッセージを含む任意のトピックを録画
ros2 service call /fv_recorder/start_recording fv_recorder/srv/StartRecording "{
  input_topics: [
    '/custom_sensor/data',
    '/robot/status',
    '/ai/detection_results',
    '/unknown/format/topic'
  ],
  output_format: 'json'
}"
```

#### メッセージタイプの確認
```bash
# トピックのメッセージタイプを確認
ros2 topic info /custom_sensor/data

# 利用可能なトピック一覧
ros2 topic list

# トピックの詳細情報
ros2 topic echo /custom_sensor/data --once
```

### 🎯 対応可能なトピック例

#### ロボットシステム
```bash
# 移動ロボットの全センサーデータ
ros2 service call /fv_recorder/start_recording fv_recorder/srv/StartRecording "{
  input_topics: [
    '/robot/camera/color/image_raw',
    '/robot/camera/depth/image_raw',
    '/robot/lidar/points',
    '/robot/imu/data',
    '/robot/gps/fix',
    '/robot/odom',
    '/robot/cmd_vel',
    '/robot/status',
    '/robot/battery',
    '/robot/emergency_stop'
  ],
  output_format: 'rosbag'
}"
```

#### AIシステム
```bash
# AI推論システムの全データ
ros2 service call /fv_recorder/start_recording fv_recorder/srv/StartRecording "{
  input_topics: [
    '/camera/image_raw',
    '/ai/detections',
    '/ai/classifications',
    '/ai/pose_estimations',
    '/ai/tracking_results',
    '/ai/confidence_scores',
    '/ai/processing_time',
    '/ai/system_status'
  ],
  output_format: 'json'
}"
```

#### カスタムシステム
```bash
# 任意のカスタムメッセージ
ros2 service call /fv_recorder/start_recording fv_recorder/srv/StartRecording "{
  input_topics: [
    '/my_package/custom_data',
    '/another_package/unknown_format',
    '/third_party/sensor_output',
    '/experimental/results'
  ],
  output_format: 'csv'
}"
```

### ⚠️ 注意事項

#### 1. **メッセージタイプの推測精度**
- トピック名からメッセージタイプを推測
- 推測が間違った場合でも録画は継続
- 再生時に正しいメッセージタイプが必要

#### 2. **パフォーマンス**
- 汎用サブスクリプションは型付きサブスクリプションより遅い
- 大量のトピックを同時録画する場合は注意
- メモリ使用量が増加する可能性

#### 3. **互換性**
- ROS2 Bag形式（`.db3`）が最も互換性が高い
- カスタムメッセージはROS2 Bag形式を推奨
- JSON/CSV形式は基本メッセージタイプに最適化

### 🔍 トラブルシューティング

#### トピックが録画されない場合
```bash
# 1. トピックの存在確認
ros2 topic list | grep your_topic

# 2. メッセージタイプ確認
ros2 topic info /your_topic

# 3. トピックのパブリッシュ確認
ros2 topic echo /your_topic --once

# 4. ログ確認
ros2 run fv_recorder fv_recorder_node
```

#### カスタムメッセージの録画
```bash
# カスタムメッセージパッケージのビルド確認
colcon build --packages-select your_custom_package

# 環境のソース確認
source install/setup.bash

# カスタムメッセージの型確認
ros2 interface list | grep your_custom
```

## 使用方法

### 1. ビルドと起動

```bash
# ビルド
cd FluentVision/fluent_vision_ros2
colcon build --packages-select fv_recorder

# ソース
source install/setup.bash

# 起動（paramsファイル指定での単体起動）
ros2 run fv_recorder fv_recorder_node --ros-args --params-file src/streaming/fv_recorder/config/recorder_config.yaml
```

### 2. 録画制御

#### サービスによる制御

```bash
# 基本録画開始（設定ファイルのデフォルトを使用）
ros2 service call /fv_recorder/start_recording fv_recorder/srv/StartRecording "{}"

# カスタムトピックで録画開始
ros2 service call /fv_recorder/start_recording fv_recorder/srv/StartRecording "{
  recording_directory: '/home/user/recordings',
  date_format: '20241201',
  segment_duration: 300,
  retention_days: 7,
  input_topics: ['/fv_realsense/color/image_raw', '/fv_realsense/depth/image_raw', '/camera/left/image_raw', '/imu/data']
}"

# 録画停止
ros2 service call /fv_recorder/stop_recording fv_recorder/srv/StopRecording "{}"
```

#### トピックによる制御

```bash
# Bool型トピックで録画開始/停止
ros2 topic pub /fv_recorder/recording_control std_msgs/msg/Bool "data: true"   # 録画開始
ros2 topic pub /fv_recorder/recording_control std_msgs/msg/Bool "data: false"  # 録画停止

# String型トピックでコマンド送信
ros2 topic pub /fv_recorder/recording_command std_msgs/msg/String "data: 'start'"      # 録画開始
ros2 topic pub /fv_recorder/recording_command std_msgs/msg/String "data: 'stop'"       # 録画停止
ros2 topic pub /fv_recorder/recording_command std_msgs/msg/String "data: 'toggle'"     # 録画切り替え
```

### 3. 再生制御

```bash
# 再生開始
ros2 service call /fv_recorder/start_playback fv_recorder/srv/StartPlayback "{recording_directory: '/home/user/recordings', date_format: '20241201', playback_speed: 1.0, loop_playback: false}"

# 再生停止
ros2 service call /fv_recorder/stop_playback fv_recorder/srv/StopPlayback "{playback_id: '20241201_143022'}"
```

### 4. 状態確認

```bash
# 録画状態確認
ros2 topic echo /fv_recorder/status

# 再生状態確認
ros2 topic echo /fv_player/status

# 利用可能なトピック確認
ros2 topic list | grep -E "(fv_realsense|camera|lidar|imu)"
```

## 設定

### 設定ファイル: `config/recorder_config.yaml`

```yaml
fv_recorder:
  ros__parameters:
    # 録画設定
    recording:
      input_topics:
        - "/fv_realsense/color/image_raw"
        - "/fv_realsense/depth/image_raw"
        - "/fv_realsense/depth/colormap"
        - "/fv_realsense/pointcloud"
        - "/camera/left/image_raw"
        - "/camera/right/image_raw"
        - "/lidar/points"
        - "/imu/data"
        - "/gps/fix"
      output_directory: "/home/takatronix/recordings"
      segment_duration: 300  # 5分
      retention_days: 7      # 7日間保持
      date_format: "YYYYMMDD"
      auto_recording: false  # 自動録画フラグ

    # 再生設定
    playback:
      output_topics:
        - "/fv_recorder/color/image_raw"
        - "/fv_recorder/depth/image_raw"
        - "/fv_recorder/depth/colormap"
        - "/fv_recorder/pointcloud"
        - "/fv_recorder/camera/left/image_raw"
        - "/fv_recorder/camera/right/image_raw"
        - "/fv_recorder/lidar/points"
        - "/fv_recorder/imu/data"
        - "/fv_recorder/gps/fix"
      playback_speed: 1.0    # 1.0 = 実時間
      loop_playback: false   # ループ再生

    # 外部制御設定
    control:
      enable_topic_control: true    # トピック制御有効
      enable_service_control: true  # サービス制御有効
      control_topics:
        recording_control: "/fv_recorder/recording_control"
        recording_command: "/fv_recorder/recording_command"
```

### Launch引数

```bash
# カスタム設定で起動
ros2 launch fv_recorder fv_recorder.launch.py \
  recording_directory:=/custom/recordings \
  segment_duration:=600 \
  retention_days:=14
```

## トピック一覧

### 入力トピック（録画対象）
- `/fv_realsense/color/image_raw` - カラー画像
- `/fv_realsense/depth/image_raw` - 深度画像
- `/fv_realsense/depth/colormap` - 深度カラーマップ
- `/fv_realsense/pointcloud` - 点群データ
- `/camera/left/image_raw` - 左カメラ画像
- `/camera/right/image_raw` - 右カメラ画像
- `/lidar/points` - LiDAR点群
- `/imu/data` - IMUデータ
- `/gps/fix` - GPSデータ

### 出力トピック（再生時）
- `/fv_recorder/color/image_raw` - 再生カラー画像
- `/fv_recorder/depth/image_raw` - 再生深度画像
- `/fv_recorder/depth/colormap` - 再生深度カラーマップ
- `/fv_recorder/pointcloud` - 再生点群データ
- `/fv_recorder/camera/left/image_raw` - 再生左カメラ画像
- `/fv_recorder/camera/right/image_raw` - 再生右カメラ画像
- `/fv_recorder/lidar/points` - 再生LiDAR点群
- `/fv_recorder/imu/data` - 再生IMUデータ
- `/fv_recorder/gps/fix` - 再生GPSデータ

### 制御トピック
- `/fv_recorder/recording_control` - 録画制御（Bool型）
- `/fv_recorder/recording_command` - 録画コマンド（String型）
- `/fv_recorder/status` - 録画状態
- `/fv_player/status` - 再生状態

## サービス一覧

### 録画サービス
- `/fv_recorder/start_recording` - 録画開始
- `/fv_recorder/stop_recording` - 録画停止

### 再生サービス
- `/fv_recorder/start_playback` - 再生開始
- `/fv_recorder/stop_playback` - 再生停止

## ファイル形式

録画ファイルはROS2 Bag形式（SQLite3）で保存されます：
- ファイル名: `YYYYMMDD_HHMMSS_segment_N.db3`
- 保存場所: 設定で指定したディレクトリ
- 自動削除: 設定日数経過後に自動削除

## 使用例

### マルチカメラシステムでの使用

```bash
# 複数カメラの同時録画
ros2 service call /fv_recorder/start_recording fv_recorder/srv/StartRecording "{
  recording_directory: '/home/user/multicamera_recordings',
  date_format: '20241201',
  segment_duration: 600,
  retention_days: 30,
  input_topics: [
    '/camera_front/color/image_raw',
    '/camera_front/depth/image_raw',
    '/camera_left/color/image_raw',
    '/camera_right/color/image_raw',
    '/camera_rear/color/image_raw',
    '/lidar/points',
    '/imu/data',
    '/gps/fix'
  ]
}"
```

### AI開発での使用

```bash
# AI学習用データの録画
ros2 service call /fv_recorder/start_recording fv_recorder/srv/StartRecording "{
  recording_directory: '/home/user/ai_training_data',
  date_format: '20241201',
  segment_duration: 300,
  retention_days: 90,
  input_topics: [
    '/fv_realsense/color/image_raw',
    '/fv_realsense/depth/image_raw',
    '/fv_realsense/pointcloud',
    '/object_detection/objects',
    '/pose_estimation/poses'
  ]
}"
```

## 注意事項

1. **ディスク容量**: 長時間録画時は十分なディスク容量を確保してください
2. **パフォーマンス**: 高解像度・高フレームレート時はCPU使用率が高くなります
3. **同期**: 複数トピックの同期はROS2のタイムスタンプに依存します
4. **権限**: 録画ディレクトリへの書き込み権限が必要です
5. **トピック存在確認**: 録画開始前に指定したトピックが存在することを確認してください

## トラブルシューティング

### よくある問題

1. **録画が開始されない**
   - 入力トピックが存在するか確認
   - ディレクトリの書き込み権限を確認

2. **再生時にエラーが発生**
   - 録画ファイルが存在するか確認
   - ファイルの破損を確認

3. **メモリ使用量が高い**
   - セグメント時間を短くする
   - 入力トピック数を減らす

4. **特定のトピックが録画されない**
   - トピック名のスペルを確認
   - トピックが実際にパブリッシュされているか確認

### ログ確認

```bash
# 録画ノードのログ
ros2 run fv_recorder fv_recorder_node

# 再生ノードのログ
ros2 run fv_recorder fv_player_node

# トピック一覧確認
ros2 topic list

# トピック情報確認
ros2 topic info /fv_realsense/color/image_raw
``` 

## 多様なフォーマット対応

fv_recorderは以下の出力フォーマットをサポートしています：

### サポートフォーマット

#### 1. **ROS2 Bag形式** (`.db3`)
- ROS2標準の録画形式
- 完全なメタデータとタイムスタンプ保持
- ROS2ツールでの再生・解析が可能

#### 2. **JSON形式** (`.json`)
- 人間が読みやすい構造化データ
- メタデータとメッセージデータを階層化
- 外部ツールでの解析が容易

#### 3. **YAML形式** (`.yaml`)
- 設定ファイルとしても使用可能
- 読みやすく、編集しやすい形式
- メタデータとメッセージを構造化

#### 4. **CSV形式** (`.csv`)
- スプレッドシートでの解析が可能
- 軽量で高速な処理
- 時系列データの分析に適している

#### 5. **動画形式** (`.mp4`, `.avi`)
- 画像トピックを動画として保存
- 標準的な動画プレーヤーで再生可能
- ストリーミング配信にも対応

### フォーマット指定方法

#### 設定ファイルでの指定
```yaml
recording:
  default_format: "json"  # デフォルトフォーマット
```

#### サービス呼び出しでの指定
```bash
# JSON形式で録画
ros2 service call /fv_recorder/start_recording fv_recorder/srv/StartRecording "{
  recording_directory: '/home/user/recordings',
  date_format: '20241201',
  segment_duration: 300,
  retention_days: 7,
  input_topics: ['/fv_realsense/color/image_raw', '/fv_realsense/depth/image_raw'],
  output_format: 'json'
}"

# MP4動画形式で録画
ros2 service call /fv_recorder/start_recording fv_recorder/srv/StartRecording "{
  recording_directory: '/home/user/recordings',
  date_format: '20241201',
  segment_duration: 300,
  retention_days: 7,
  input_topics: ['/fv_realsense/color/image_raw'],
  output_format: 'mp4'
}"

# CSV形式で録画
ros2 service call /fv_recorder/start_recording fv_recorder/srv/StartRecording "{
  recording_directory: '/home/user/recordings',
  date_format: '20241201',
  segment_duration: 300,
  retention_days: 7,
  input_topics: ['/imu/data', '/gps/fix'],
  output_format: 'csv'
}"
```

### フォーマット別使用例

#### JSON形式での録画
```bash
# センサーデータをJSON形式で録画
ros2 service call /fv_recorder/start_recording fv_recorder/srv/StartRecording "{
  input_topics: ['/imu/data', '/gps/fix', '/temperature'],
  output_format: 'json'
}"
```

生成されるJSONファイル：
```json
{
  "metadata": {
    "format": "json",
    "created_at": 1701234567,
    "version": "1.0"
  },
  "messages": [
    {
      "timestamp": 1701234567123456789,
      "topic": "/imu/data",
      "type": "sensor_msgs/msg/Imu",
      "data": {
        "linear_acceleration": {"x": 0.1, "y": 0.2, "z": 9.8},
        "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0}
      }
    }
  ]
}
```

#### CSV形式での録画
```bash
# 数値データをCSV形式で録画
ros2 service call /fv_recorder/start_recording fv_recorder/srv/StartRecording "{
  input_topics: ['/temperature', '/pressure', '/humidity'],
  output_format: 'csv'
}"
```

生成されるCSVファイル：
```csv
timestamp,topic,type,data
1701234567123456789,/temperature,std_msgs/msg/Float32,23.5
1701234567123456790,/pressure,std_msgs/msg/Float32,1013.25
1701234567123456791,/humidity,std_msgs/msg/Float32,65.2
```

#### 動画形式での録画
```bash
# カメラ画像をMP4動画として録画
ros2 service call /fv_recorder/start_recording fv_recorder/srv/StartRecording "{
  input_topics: ['/fv_realsense/color/image_raw'],
  output_format: 'mp4'
}"
```

### フォーマット別特徴

| フォーマット | 拡張子 | 特徴 | 用途 |
|-------------|--------|------|------|
| ROSBag2 | `.db3` | ROS2標準、完全なメタデータ | ROS2開発、デバッグ |
| JSON | `.json` | 読みやすい、構造化 | データ解析、API連携 |
| YAML | `.yaml` | 設定ファイル互換 | 設定管理、ドキュメント |
| CSV | `.csv` | 軽量、高速 | 時系列分析、スプレッドシート |
| MP4 | `.mp4` | 標準動画形式 | プレゼンテーション、配信 |
| AVI | `.avi` | 非圧縮動画 | 高品質保存、編集 |

### フォーマット選択のガイドライン

#### ROS2開発用途
- **推奨**: `rosbag` (`.db3`)
- **理由**: ROS2ツールとの完全な互換性

#### データ解析用途
- **数値データ**: `csv` (軽量、高速)
- **構造化データ**: `json` (読みやすい)
- **設定データ**: `yaml` (編集しやすい)

#### プレゼンテーション用途
- **動画配信**: `mp4` (圧縮効率が良い)
- **高品質保存**: `avi` (非圧縮)

#### AI開発用途
- **学習データ**: `json` (メタデータ保持)
- **動画データ**: `mp4` (標準形式)
- **センサーデータ**: `csv` (軽量) 