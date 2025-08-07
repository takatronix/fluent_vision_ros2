# FluentVision ROS2

ROS2ベースのリアルタイムビジョンシステム - アスパラガス収穫ロボット向けに最適化

## 🚀 概要

FluentVision ROS2は、Intel RealSense深度カメラとAI技術を組み合わせた高精度ビジョンシステムです。アスパラガス収穫ロボットのための物体検出、セグメンテーション、距離測定機能を提供します。

## 📦 パッケージ構成

### 🤖 AI/ML (`src/ai/`)
- **fv_object_detector**: YOLOv10を使用した汎用物体検出システム
- **fv_object_mask_generator**: UNet使用のセマンティックセグメンテーション
- **fv_aspara_detector**: アスパラガス専用検出器（YOLOv10ベース）
- **fv_aspara_analyzer**: アスパラガス品質分析システム
- **fv_face_recognizer**: リアルタイム顔認識システム（開発中）

### 📷 センサー (`src/sensors/`)
- **fv_realsense**: Intel RealSense深度カメラドライバ（D415, D405対応）
  - キャリブレーション支援機能（表示モード0/1/2）
  - クリック座標取得・3D距離測定
- **fv_camera**: 汎用カメラドライバ
- **livox_ros_driver2**: LiDARセンサードライバ

### 🌐 ストリーミング (`src/streaming/`)
- **fv_recorder**: 録画・再生システム
- **fv_mjpeg_server**: MJPEGストリーミングサーバー（開発中）
- **fv_websocket_server**: WebSocketリアルタイムストリーミング（開発中）
- **fv_image_distributor**: 画像配信サーバー（開発中）
- **fv_rtmp_server**: RTMPストリーミングサーバー

### 🛠️ ユーティリティ (`src/utils/`)
- **fv_image_filter**: 画像フィルタリング・処理ユーティリティ
- **fv_topic_relay**: トピック中継・変換ツール（開発中）

### 🗺️ SLAM (`rtabmap/`)
- **RTAB-Map**: リアルタイムSLAMシステム
- マッピング・ナビゲーション機能

## 🎯 主要機能

### アスパラガス収穫支援
- **高精度検出**: YOLOv10ベースのアスパラガス検出
- **品質分析**: サイズ・形状・成熟度の自動判定
- **距離測定**: 3D座標による収穫位置の正確な特定
- **キャリブレーション**: 視覚的支援機能付きカメラ校正

### リアルタイム処理
- **低遅延**: 最適化されたパイプライン処理
- **マルチスレッド**: 並列処理による高効率化
- **GPU加速**: CUDA対応（オプション）

### 柔軟なストリーミング
- **複数フォーマット**: MJPEG, WebSocket, RTMP対応
- **マルチクライアント**: 同時接続対応
- **圧縮最適化**: 帯域幅に応じた自動調整

## 🛠️ インストール

### 前提条件
- ROS2 Humble
- Intel RealSense SDK 2.0
- OpenCV 4.x
- CUDA 11.x（オプション）

### ビルド
```bash
cd /home/aspara/seedbox-r1/fluent_vision_ros2
colcon build --symlink-install
source install/setup.bash
```

## 🚀 使用方法

### 全システム起動
```bash
cd launch
./start_fv.sh
```

### アスパラガス収穫システム起動
```bash
# D415使用
./start_asparagus_d415.sh

# D405使用  
./start_asparagus_d405.sh
```

### AI制御
```bash
# AI開始
./ai_start

# AI停止
./ai_stop

# AI状態確認
./ai_status
```

### システム停止
```bash
./stop_fv.sh
```

## 📊 トピック構成

### カメラストリーム
- **D415**: `/fv/d415/color/image_raw`, `/fv/d415/depth/image_rect_raw`
- **D405**: `/fv/d405/color/image_raw`, `/fv/d405/depth/image_rect_raw`

### AI処理結果
- **物体検出**: `/fv/object_detection/result`
- **セグメンテーション**: `/fv/segmentation/result`
- **アスパラガス検出**: `/fv/aspara_detection/result`
- **品質分析**: `/fv/aspara_analysis/result`

### サービス
- **モード設定**: `/fv_realsense/set_mode`
- **距離取得**: `/fv_realsense/get_distance`
- **カメラ情報**: `/fv_realsense/get_camera_info`

## ⚙️ 設定

### カメラ設定
各カメラ用の設定ファイルが`launch/`ディレクトリに用意されています：
- `fv_realsense_d415.yaml` / `fv_realsense_d405.yaml`
- `fv_aspara_analyzer_d415.yaml` / `fv_aspara_analyzer_d405.yaml`

### パフォーマンス調整
- 解像度: 320x240（Raspberry Pi対応）〜 1280x720
- FPS: 10fps（標準）〜 30fps（高品質）
- AI推論: CPU/GPU選択可能

## 🔧 開発・デバッグ

### ログ確認
```bash
# リアルタイムログ
ros2 topic echo /fv/aspara_detection/result

# ノード状態
ros2 node list
ros2 node info /fv_aspara_detector
```

### カメラ確認
```bash
# シリアル番号確認
python3 check_camera_serials.py

# トピック一覧
ros2 topic list | grep fv
```

## 📝 ライセンス

MIT License

## 👨‍💻 作者

Takashi Otsuka (@takatronix)

## 🤝 貢献

プルリクエストやイシューの報告を歓迎します。

---

**FluentVision ROS2** - 未来の農業を支えるビジョンシステム 🌱