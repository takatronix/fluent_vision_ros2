# FV RTMP Server for ROS2

FluentVision用のRTMPサーバーノードです。DJI Flyアプリとの互換性を提供し、RTMPストリームをROS2画像トピックに変換します。

## 機能

- RTMPサーバーとして動作（ポート1935）
- DJI Flyアプリとの互換性
- ROS2画像トピックへの変換
- 低遅延モード対応
- 圧縮画像オプション
- 自動再接続機能

## セットアップ

### 1. 依存関係のインストール

```bash
# FFmpegライブラリ
sudo apt update
sudo apt install libavcodec-dev libavformat-dev libavutil-dev libswscale-dev

# OpenCV
sudo apt install libopencv-dev

# yaml-cpp
sudo apt install libyaml-cpp-dev
```

### 2. ROS2ワークスペースに配置

```bash
# ROS2ワークスペースのsrcディレクトリにコピー
cp -r fv_rtmp_server ~/ros2_ws/src/

# パッケージをビルド
cd ~/ros2_ws
colcon build --packages-select fv_rtmp_server
source install/setup.bash
```

## 使用方法

### 基本的な使用方法

```bash
# パッケージビルド後
ros2 run fv_rtmp_server fv_rtmp_server_node
```

### Launchファイルを使用

```bash
# 基本的な起動
ros2 launch fv_rtmp_server fv_rtmp_server.launch.py

# パラメータを指定して起動
ros2 launch fv_rtmp_server fv_rtmp_server.launch.py \
    server_port:=1935 \
    topic_name:=/camera/image_raw \
    target_width:=1280 \
    target_height:=720 \
    compressed:=true
```

### DJI Flyアプリでの接続

1. DJI Flyアプリを起動
2. カメラ設定で「カスタムRTMP」を選択
3. RTMP URLを入力: `rtmp://[IPアドレス]:1935/live/s`
4. ストリーミング開始

## パラメータ

| パラメータ名 | デフォルト値 | 説明 |
|-------------|-------------|------|
| `server.port` | `1935` | RTMPサーバーのポート |
| `server.endpoint` | `/live/s` | RTMPエンドポイント |
| `output.topic_name` | `/fv_rtmp_server/camera/image_raw` | 出力トピック名 |
| `output.frame_id` | `camera_link` | TFフレームID |
| `output.target_width` | `640` | 出力画像幅 |
| `output.target_height` | `480` | 出力画像高さ |
| `output.publish_rate` | `30.0` | パブリッシュレート |
| `output.compressed` | `false` | 圧縮画像としてパブリッシュ |
| `processing.enable_low_latency` | `true` | 低遅延モード有効 |
| `processing.buffer_size` | `1` | バッファサイズ |
| `processing.quality_mode` | `fast` | 品質モード |

## トピック

### パブリッシュ

- `/fv_rtmp_server/camera/image_raw` - 生画像
- `/fv_rtmp_server/camera/image_raw/compressed` - 圧縮画像（image_transport）
- `/fv_rtmp_server/status` - サーバー状態
- `/fv_rtmp_server/o4/camera_info` - O4 Air Unit カメラ情報
- `/fv_rtmp_server/o4_pro/camera_info` - O4 Pro Air Unit カメラ情報

## トラブルシューティング

### 接続エラー
- ポート1935が使用可能か確認
- ファイアウォール設定を確認
- ネットワーク接続を確認

### 画像が表示されない
- トピックが正しくパブリッシュされているか確認
```bash
ros2 topic info /fv_rtmp_server/camera/image_raw
```
- 画像ビューアーでトピックを選択して表示

### DJI Flyアプリで接続できない
- IPアドレスが正しいか確認
- ポート1935が開放されているか確認
- ネットワーク設定を確認

## ライセンス

Apache-2.0 License

## 作者

Takashi Otsuka (takatronix@gmail.com) 