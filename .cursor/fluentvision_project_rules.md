# FluentVision ROS2 プロジェクト専用ルール

## 🎯 プロジェクトの本質的目的

### 美しい映像配信システムの実現
**🌟 いかに便利に、楽に、簡単に、スムースに、高速に映像や情報を美しくさまざまなコンピュータに配信できるか**

### FluentVision ROS2の真の価値
- **究極目標**: 映像・情報配信の革命的プラットフォーム
- **6つのコア価値**: 便利・楽・簡単・スムース・高速・美しい
- **技術スタック**: ROS2 Humble + C++ + Intel RealSense + AI/ML (YOLOv10, UNet),Unity,XREAL
- **主要機能**: リアルタイム映像配信、AI処理、マルチプラットフォーム対応

## 📦 プロジェクト構成

### ディレクトリ構造
```
fluent_vision_ros2/
├── README.md                 # プロジェクト全体概要・クイックスタート
├── docs/                     # プロジェクト全体ドキュメント
│   ├── design/              # システム設計書
│   ├── api/                 # API仕様書  
│   ├── guides/              # 操作・設定ガイド
│   └── troubleshooting/     # トラブルシューティング
├── src/                     # 実装コード
│   ├── ai/                  # AI/ML処理
│   ├── sensors/             # センサードライバ
│   ├── streaming/           # ストリーミングサーバー
│   ├── utils/               # ユーティリティ
│   └── common/              # 共通ライブラリ
├── tests/                   # テストコード
├── examples/                # サンプルコード
└── launch/                  # 起動スクリプト
```

### パッケージレベル構成例
```
src/ai/fv_aspara_analyzer/
├── README.md                # パッケージ概要・使用方法
├── docs/                    # パッケージ固有文書
│   ├── design.md           # アスパラ分析設計書
│   ├── usage.md            # 使用方法・パラメータ
│   └── api.md              # API仕様
├── src/                     # 実装コード
├── include/                 # ヘッダーファイル
└── package.xml              # パッケージ設定
```

## 🌊 FluentVision哲学の実装

### 6つのコア価値の実装指針

#### 1. 🛠️ 便利（Convenient）
```cpp
✅ 実装指針:
- ワンライン起動: ./start_fv
- ワンライン停止: ./stop_fv
- 自動設定: カメラ自動検出・設定
- 統合CLI: 全機能を単一コマンドで制御
- プラグアンドプレイ: 設定不要で即座に動作

例: 超便利なAPI
auto stream = EasyStream::camera("/dev/video0")
    .to_web("http://0.0.0.0:8080")
    .start();  // これだけで配信開始
```

#### 2. 😌 楽（Easy）
```cpp
✅ 実装指針:
- 設定ファイル最小化
- デフォルト値で最適動作
- エラー自動回復
- 自動リトライ機能

例: 設定レスでの簡単起動
FluentStream::autoDetect().broadcast();
```

#### 3. 🎯 簡単（Simple）
```cpp
✅ 実装指針:
- API呼び出し最小化
- 直感的なメソッド名
- チェーン可能なインターフェース
- ドキュメント不要レベルの明確さ

例: 1行で完結する処理
Image::load("input.jpg").ai_detect().stream_to_web().save("result.jpg");
```

#### 4. 🌊 スムース（Smooth）
```cpp
✅ 実装指針:
- フレーム落ちゼロ
- 適応的品質調整
- 帯域幅自動最適化
- レイテンシ最小化

例: スムースな配信制御
auto stream = SmoothStream::from(camera)
    .adaptive_quality()      // 帯域に応じて自動調整
    .zero_frame_drop()       // フレーム落ち防止
    .low_latency_mode()      // 低遅延モード
    .to_multiple_clients();  // マルチクライアント対応
```

#### 5. ⚡ 高速（Fast）
```cpp
✅ 実装指針:
- C++による最適化（Python比100倍速）
- マルチスレッド並列処理
- GPU加速活用
- ゼロコピー操作

例: 超高速処理
FastProcessor::parallel()
    .gpu_accelerated()
    .zero_copy()
    .process_realtime(stream)
    .distribute_to_all();
```

#### 6. 🎨 美しい（Beautiful）
```cpp
✅ 実装指針:
- エレガントなAPI設計
- 読みやすいコード
- 美しい映像品質
- 洗練されたUI

例: 美しいFluentAPI
BeautifulVision::from(camera)
    .enhance_quality()
    .apply_artistic_filter()
    .overlay_elegant_ui()
    .stream_with_style();
```

## 🔧 開発環境・コマンド

### ビルド環境
```bash
# プロジェクトディレクトリ
cd /home/aspara/seedbox-r1/fluent_vision_ros2

# 標準ビルド
colcon build --symlink-install

# 特定パッケージビルド
colcon build --packages-select [package_name]

# ユーザーは`source install/setup.bash`を避けたい（遅いため）
```

### 起動・停止
```bash
# 統合CLI使用
./fv start              # 全ノード起動
./fv stop               # 全ノード停止
./fv status             # 状態確認

# AI操作
./fv ai start [d415|d405|both]
./fv ai stop [d415|d405|both]

# 従来方式
./launch/start_fv.sh    # 全システム起動
./launch/stop_fv.sh     # システム停止
```


## 🏗️ FluentLibライブラリ設計原則

### fluent_cloudライブラリの役割
```cpp
✅ 汎用的な点群処理のみ含める:
- フィルタリング、ノイズ除去
- 座標変換、幾何学的変換
- 汎用的な3D処理アルゴリズム

❌ アプリケーション固有処理は禁止:
- アスパラガス専用解析
- 特定農作物の品質判定
- 収穫可能判定ロジック
```

### ライブラリ更新時の必須作業
```bash
# fluent_libライブラリのリビルド（必須）
colcon build --packages-select fluent_lib
source install/setup.bash

# 使用するノードもリビルド（必須）
colcon build --packages-select fv_aspara_analyzer
```

### 現在実装済みの美しいAPI

#### Image処理クラス
```cpp
// 思考の流れがそのままコードに
Image::load("input.jpg")
    .resize(640, 480)
    .blur(2.0)
    .bright(1.2)
    .circle(100, 100, 50, Colors::GREEN)
    .text("Processed")
    .save("output.jpg");
```

#### Cloud処理クラス  
```cpp
// 直感的な点群操作
Cloud::fromRGBD(rgb, depth)
    .filter(0.1, 5.0)
    .downsample(0.01)
    .removeNoise()
    .colorByHeight()
    .publish("/processed_cloud");
```

#### FluentVision高度処理
```cpp
// ROS2統合での画像処理
FluentVision::from(ros_image)
    .gaussianBlur(2.0)
    .canny(100, 200)
    .detectObjects("yolo")
    .drawText("Detected", Point(10, 30), GREEN)
    .toImageMsg();
```

## 🎛️ 重要な設定・パラメータ

### カメラ設定
- **D415/D405**: 解像度320x240〜1280x720、FPS 10-30
- **シリアル番号管理**: 自動検出・マッピング機能あり

### AI処理設定
- **YOLOv10**: 物体検出（persons, objects）
- **UNet**: セマンティックセグメンテーション
- **アスパラ分析**: 長さ23-50cm、真っ直ぐ度0.7以上が収穫可能

### ストリーミング設定
- **WebSocket**: リアルタイム双方向通信
- **MJPEG**: HTTP画像ストリーミング
- **RTMP**: 外部配信対応

### 性能目標
- **メモリ使用量**: 5-10MB（現在10-20MB）
- **処理時間**: 50-100ms（現在100-200ms）
- **C++ vs Python性能差**: 100倍

## 🚨 よくある問題と対処法

### カメラ接続問題
```bash
# RealSense問題時
pkill -f realsense && # デバイス再接続

# 権限問題
sudo chmod 666 /dev/video*
```

### ビルドエラー
```bash
# 依存関係問題
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# キャッシュクリア
rm -rf build/ install/ log/
colcon build --symlink-install
```

## 🔗 重要なトピック・サービス

### カメラストリーム
```
/fv/d415/color/image_raw        # D415カラー画像
/fv/d405/depth/image_rect_raw   # D405深度画像
```

### AI処理結果
```
/fv/object_detection/result     # 物体検出結果
/fv/aspara_detection/result     # アスパラガス検出結果
/fv/aspara_analysis/result      # 品質分析結果
```

### サービス
```
/fv_realsense/set_mode         # 表示モード切り替え
/fv_realsense/get_distance     # 3D距離取得
/fv_realsense/get_point_cloud  # ポイントクラウド取得
```

## 📊 開発優先順位

### 高優先度
1. **性能最適化** - メモリ使用量削減、処理時間短縮
2. **品質向上** - アスパラガス認識精度向上
3. **安定性向上** - エラーハンドリング強化

### 中優先度
4. **機能追加** - 太さ測定、複数同時分析
5. **パラメータ調整** - 自動調整機能
6. **ログ・デバッグ** - パフォーマンス統計

### 低優先度
7. **インターフェース改善** - REST API、Web UI
8. **拡張機能** - 他野菜対応、ML予測

## 🌟 プロジェクト固有の制約・要求

### Python使用制限
```bash
❌ 禁止: 本番コードでのPython使用
- 画像処理、点群処理をPythonで実装
- リアルタイム処理をPythonで実装
- ROS2ノードのPython実装

✅ Python許可範囲（限定的）:
- テスト・実験スクリプトのみ
- 設定ファイル生成補助
- ビルド補助スクリプト
```

### 配信システム設計の原則

#### マルチプラットフォーム配信
```cpp
// あらゆるデバイスへの配信を簡単に
UniversalBroadcaster broadcaster;
broadcaster.add_target("web", "http://localhost:8080")
           .add_target("mobile", "rtmp://mobile.server")  
           .add_target("unity", "ws://unity.client")
           .add_target("ros2", "/fv/stream/output")
           .start_broadcast(camera_stream);

// 自動フォーマット変換
- Web → MJPEG/WebSocket
- Mobile → RTMP/HLS  
- Unity → WebSocket/Binary
- ROS2 → sensor_msgs::Image
```

---

**🌟 プロジェクトミッション**: 映像配信を革命的に便利・楽・簡単・スムース・高速・美しくすることで、あらゆるコンピュータ間の映像・情報共有を次世代レベルに引き上げる。FluentVisionの"Flow Like Water"哲学で、技術の美しさと実用性を両立させる。
