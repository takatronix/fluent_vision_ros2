# FV Audio Output

`fv_audio_output`は`fv_audio/msg/AudioFrame`をALSAデバイスに出力するROS2ノードです。`fv_tts`や通知ノードがPublishする`audio/output/frame`を購読し、スピーカーへPCM16LEのまま再生します。

## 依存
- ALSA (`libasound2-dev`)
- `fv_audio`メッセージ

## 起動
```bash
ros2 run fv_audio_output fv_audio_output_node --ros-args --params-file install/fv_audio_output/share/fv_audio_output/config/default.yaml
```

主なパラメータ:
- `audio.device_id`: ALSAデバイス名（例: `default`, `hw:1,0`）
- `audio.volume`: 全AudioFrameへ適用する初期マスター音量 (0.0〜1.0)。
- `audio.volume_topic`: 実行中のマスター音量を受ける `std_msgs/Float32`。
  TRANSIENT_LOCAL QoSで、UI再起動後も最新値を受け取れます。
- `audio.sample_rate`, `audio.channels`, `audio.bit_depth`: フレームと一致している必要があります。
- `audio.start_threshold_frames`: ALSAが再生を開始するフレーム数。0は従来どおり
  バッファの半分、1は短い通知音を即時開始します。
- `audio.drain_after_frame`: trueなら各AudioFrameを実デバイスが再生し終わるまで待ってから
  完了通知を出します。離散cue/TTS向けで、連続ストリームではfalseを使用します。
- `queue.max_frames`: バッファに保持するフレーム数。溢れると古いフレームから破棄します。

`fv_tts`と組み合わせる場合、`audio/output/frame`トピックを共有すればテキスト合成結果が自動で再生されます。

## ファイル再生サービス

`/audio/output/play_file`サービスを使用して、WAVファイルを直接再生できます。

### サービス定義
```bash
ros2 interface show fv_audio_output/srv/PlayFile
```

### 使用例

警告音を再生:
```bash
ros2 service call /audio/output/play_file fv_audio_output/srv/PlayFile \
  "{file_path: '/home/aspa/ros2_ws/sounds/warning.wav', volume_scale: 1.0, interrupt: false}"
```

ビープ音を即座に再生（キューをクリア）:
```bash
ros2 service call /audio/output/play_file fv_audio_output/srv/PlayFile \
  "{file_path: '/home/aspa/ros2_ws/sounds/beep.wav', volume_scale: 0.5, interrupt: true}"
```

### サンプル音声

`/home/aspa/ros2_ws/sounds/`にサンプル音声が用意されています:
- `beep.wav` - 短いビープ音 (800Hz, 0.15秒)
- `notification.wav` - 通知音 (600Hz, 0.2秒)
- `success.wav` - 成功音 (1000Hz, 0.3秒)
- `warning.wav` - 警告音 (400Hz, 0.5秒)
- `error.wav` - エラー音 (2連ビープ, 300Hz)

全て48kHz/モノラル/16bit PCMフォーマットです。
