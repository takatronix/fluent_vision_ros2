# FV Audio

FluentVisionの音声入力専用ROS2ノードです。PortAudioを利用して任意のマイク/ライン入力からPCMデータを取得し、ROSトピックやサービスを通じて配信します。

## 機能
- PCMデータの低遅延Publish (`audio/frame`トピック)
- RMS/Peak算出（VADは`fv_audio_vad`へ分離）
- デバイス列挙サービス (`ListDevices`)
- ホットスワップサービス (`SwitchDevice`)
- シンプルな録音サービス (`Record`)
- Diagnostic出力（XRUN/レイテンシ監視）

## ビルド
```bash
sudo apt install portaudio19-dev
colcon build --packages-select fv_audio
source install/setup.bash
```

## 起動
```bash
ros2 launch fv_audio fv_audio_launch.py
```

## インターネットラジオ配信サンプル
Icecast等へ送出したい場合は、`scripts/radio_streamer.py`を使うことで
`fv_audio/msg/AudioFrame`を`ffmpeg`経由でMP3に変換しHTTP PUTできます。

```bash
ros2 run fv_audio radio_streamer.py \
  --ros-args -p output_url:=http://source:hackme@localhost:8000/live
```

※ `ffmpeg`が必要です（`sudo apt install ffmpeg`）。

詳細な設計は [`DESIGN.md`](DESIGN.md) を参照してください。
