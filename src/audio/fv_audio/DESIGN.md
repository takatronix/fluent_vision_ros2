# FV Audio ノード設計

`fv_camera`（標準RGBカメラ）や`fv_realsense`（深度カメラ）と同じ思想で、音声入出力をROS2トピックに載せる専用ノードを追加する。ロボットの視覚ストリームに音声情報を重畳し、遠隔オペレーションや録画で同期されたマルチメディアデータを扱えるようにする。実装ではキャプチャとVAD/解析を役割分離し、リアルタイム性と拡張性を確保する。

## 1. 目的と位置づけ
- **ストリーミング統合**: `fv_image_distributor`や`fv_rtmp_server`と同じUI/CLI経路で音声も提供し、WebダッシュボードのUXを揃える。
- **録画同期**: `fv_recorder`が映像と同じタイムスタンプ空間でPCMを保存できるようにする。
- **デバイス多様性**: USBマイク、ライン入力、アナログ→USB変換などの複数デバイスを`fv_camera`のように簡単に切り替えられる設計。
- **リアルタイム指向**: `fv_camera`等が採用しているベストエフォート/KeepLast(1)の低遅延QoSを踏襲し、~50ms以内の往復遅延を目標にする。

## 2. 主要ユースケース
1. **遠隔観察**: 映像と同じWebビューで現場音をリアルタイム監視。
2. **アラート検知**: VADやdBレベルをトリガーとして`fv_topic_relay`→`fv_aspara_analyzer`へ通知。
3. **録画・再生**: `fv_recorder`で音声付き収穫ログを作成、再生時に同期。
4. **ボイスメモ**: アクションを用いて短い音声ノートを生成し、ダッシュボードから共有。

## 3. 依存関係
| 種別 | 候補ライブラリ | 理由 |
| --- | --- | --- |
| 音声入出力 | ALSA API（libasound2） | Linux配備を想定し、直接ALSAを叩いて依存を最小化。 |
| 符号化 | libsamplerate、libopus (オプション) | PCMをWeb送信用にOpusへ変換。 |
| ROS2 | `rclcpp`, `std_srvs`, `diagnostic_msgs` | 既存ノードと同じament依存。 |
| 共通ライブラリ | `fluent_lib` | メトリクス、設定ヘルパー、CLI統合。 |

## 4. 構成概要
```
┌────────────────────────────┐      ┌────────────────────────────┐
│ fv_audio_node (Capture)   │      │ fv_audio_vad_node (VAD)     │
│  ├ DeviceManager(Alsa)    │      │  ├ AudioFrame subscriber    │
│  ├ CaptureWorker          │----->│  ├ RMS/Hysteresis detector  │
│  ├ Level Publisher        │      │  └ audio/vad publisher      │
│  ├ RecorderHook           │      └────────────────────────────┘
│  └ Diagnostics            │
└────────────────────────────┘
```
`fv_camera`と同様にキャプチャノードは「デバイス確保・トピック出力・サービス」へ集中させ、VAD/ウェイクワードなどの解析は別ノードで差し替え可能にする。

## 5. ROSインターフェース

### 5.1 トピック
| 名前 | 型 | QoS | 内容 |
| --- | --- | --- | --- |
| `~/audio/frame` | `fv_audio/msg/AudioFrame` | SensorData QoS | PCM + メタ情報（sample_rate, channels, bit_depth, RMS等）。 |
| `~/audio/levels` | `std_msgs/msg/Float32MultiArray` | SystemDefault | チャンネル毎のRMS/ピーク。UIメーター表示用。 |
| `~/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Default | XRUN、遅延、バッファ状態。 |
| `~/audio/vad` | `std_msgs/msg/Bool` | (別ノード) | `fv_audio_vad`が出す音声検出結果。 |

### 5.2 サービス
| 名前 | 型 | 概要 |
| --- | --- | --- |
| `~/list_devices` | `fluent_audio/srv/ListDevices` | ALSA/PortAudioデバイス一覧と対応サンプルレートを返す。 |
| `~/switch_device` | `fluent_audio/srv/SwitchDevice` | ノード稼働中に別デバイスへ切替。設定YAMLを書き換えずにホットスワップ。 |
| `~/record` | `fluent_audio/srv/Record` | 任意ディレクトリにWAV/FLACを出力。`fv_recorder`が内部的に呼ぶ想定。 |

### 5.3 アクション（任意）
| 名前 | 型 | 用途 |
| --- | --- | --- |
| `~/capture_clip` | `fluent_audio/action/CaptureClip` | 指定長のクリップを取得し、完了時にファイルパスを返す。現場メモ用途。 |

## 6. パラメータ設計
`config/default_audio.yaml`にまとめる。`fv_camera`の`camera.*`階層を踏襲し、`audio.*`と`pipeline.*`の2系統で整理する（VADしきい値等は`fv_audio_vad`側のYAMLで設定）。

| パラメータ | 例 | 説明 |
| --- | --- | --- |
| `audio.device_selector.mode` | `auto / serial / name / index` | `fv_realsense`の`device_selector`に準拠。 |
| `audio.device_selector.identifier` | `"hw:1,0"` | ALSA/PortAudio向け識別子。 |
| `audio.sample_rate` | `48000` | Hz。 |
| `audio.channels` | `1 or 2` | モノラル/ステレオ。 |
| `audio.bit_depth` | `16 / 24 / 32` | PCMビット深度。 |
| `audio.chunk_ms` | `10` | Captureワーカーのバッファ長 (ms)。 |
| `pipeline.encoding` | `pcm16 / pcm32 / float32` | `AudioFrame`の格納形式。 |
| `pipeline.noise_suppression` | `off / light / strong` | WebRTC-NS等のプリセット（将来）。 |
| `pipeline.agc_target_db` | `-12` | 自動ゲインの目標dBFS（将来）。 |
| `pipeline.publish_waveform` | `false` | 波形データを追加出力する場合にtrue。 |
| `websocket_bridge.enabled` | `true` | `fv_websocket_server`へPCM/Opus中継。 |
| `recorder.auto_attach` | `true` | `fv_recorder`起動時に自動で録音連携。 |

## 7. ランチ構成
```
ros2 launch fv_audio fv_audio_launch.py \
    node_name:=fv_audio_headset \
    config_file:=config/headset.yaml
```
- `fv_camera`と同様、`node_name`と`config_file`を引数化。
- 複数デバイスを1ファイルで管理する場合は`LaunchDescription`内で`ComposableNodeContainer`を採用し、名前空間で切り分ける。
- `./fv audio start [profile]` を`fv` CLIに追加し、`launch/fv_audio_start.py`を呼ぶ。`fv ai ...`と同じUX。

## 8. 録画・ストリーミング連携
1. `fv_recorder`は`audio.auto_attach`がtrueの場合に`/fv/audio/<name>/audio/pcm`を自動購読し、映像TSに合わせてWAVチャンクを生成。
2. Webダッシュボード:
   - `fv_audio` → `fv_websocket_server`へOpus 48 kHz monoをPush
   - ブラウザ側でMSE/WebAudioに流し、`fv_image_distributor`のMJPEGと同期表示。
3. RTMP/RTSP:
   - 既存`fv_rtmp_server`にPCMをPipeし、FFmpegでAAC化して映像とMux。

## 9. モニタリング / フォールトハンドリング
- `diagnostics`トピックでXRUN検出、入力なし（サイレンス）を通知。
- `fluent_lib::metrics`を利用し、`audio_xruns_total`や`audio_latency_ms`をエクスポート。
- 異常時の自動再接続: `switch_device`サービスを内部呼び出しして再初期化。`fv_camera`の再接続ロジックと同様に3回リトライ後にERRORレベルログ。

## 10. 実装フェーズ案
1. **Skeleton**: `fv_audio`パッケージ雛形、ALSA列挙ノード、診断出力のみ。
2. **Capture & Publish**: PCM Publish/レベル算出、`list_devices`サービス。
3. **デバイス制御**: `switch_device`、パラメータ動的再読込、再接続処理。
4. **VAD分離**: `fv_audio_vad`ノードを実装し、閾値/ヒステリシスをパラメータ化。
5. **録画/CLI統合**: `fv_recorder`フック、`./fv audio`コマンド追加。
6. **Web連携**: Opusブリッジ、WebUI更新、最終ドキュメント化。

## 11. テスト方針
- **Unit**: Captureワーカーのリングバッファ、`fv_audio_vad`のしきい値/ヒステリシス。
- **Integration**: 仮想ALSAデバイス (`snd-aloop`) を使い、`ros2 bag`でPCMを保存して再生一致を確認。
- **System**: `fv_camera` + `fv_audio`同時起動でCPU負荷、XRUN無しを確認（60分連続運用テスト）。

この設計に基づいて`fv_audio`ノードを実装すれば、既存のカメラ/深度ノードと同じ運用フローで音声を取り扱えるようになる。
