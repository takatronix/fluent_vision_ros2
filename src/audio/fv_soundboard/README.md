# fv_soundboard

状況イベントを受けて **① 心地よいサウンド ② 任意 TTS ③ 記録マーカー** を出す
サウンドボードノード。離散 cue (wav) に加えて、**種別(variant)** と
**連続パラメータのリアルタイム合成(sonification)** に対応する。

## Event Bus 契約 (v1.1)

| topic | 型 | 向き | 用途 |
|-------|----|------|------|
| `/fv/event` | std_msgs/String | 入力 | 状況イベント (登録名 / JSON / `say:<text>`) |
| `/fv/event/active` | std_msgs/String | 出力 | 正規化した記録マーカー (recorder/UI が購読) |
| `/aspa/tts/say` | std_msgs/String | 出力 | 任意テキスト→発話 (fv_tts が消費) |
| `/aspa/audio/speaking` | std_msgs/Bool | 出力 | 再生中フラグ |

## `/fv/event` の与え方

```bash
# 1) 登録名 (既定動作: 音 + registry の speak 設定)
ros2 topic pub -1 /fv/event std_msgs/String "{data: 'success'}"

# 2) JSON で個別上書き
ros2 topic pub -1 /fv/event std_msgs/String '{data: "{\"event\":\"detect\",\"say\":\"見つけた\",\"speak\":true}"}'

# 3) 種別 (variant): sounds/detect_grade_A.wav があればそれを再生、無ければ base + 種別TTS
ros2 topic pub -1 /fv/event std_msgs/String '{data: "{\"event\":\"detect\",\"variant\":\"grade_A\"}"}'

# 4) 連続パラメータ → リアルタイム合成 (sonification)
ros2 topic pub -1 /fv/event std_msgs/String '{data: "{\"event\":\"detect\",\"class\":\"spear\",\"attrs\":{\"length_mm\":320,\"curvature\":0.15,\"confidence\":0.92}}"}'

# 5) 登録外テキストを即発話 (音なし)
ros2 topic pub -1 /fv/event std_msgs/String "{data: 'say:アスパラを収穫します'}"
```

`class`/`attrs` が無ければ従来通りの離散 cue 動作。構造化キー (variant/class/attrs) が
あった場合は `/fv/event/active` へ **JSON** で forward し、エピソードに種別が残る
(bare 名入力なら bare 名のまま)。

## 登録表 (events.yaml)

各イベントは `sound` / `say` / `speak` / `interrupt` / `record` / `category` を持つ。
案件固有の台詞・マッピングは、このファイルをコピーして `registry_file` パラメータで
差し替える (aspa は「アスパラ」用語で上書き)。

### 種別 (variants)

イベントに任意の `variants:` マップ (variant名 → `{sound?, say?}`) を持てる。解決順:

```
sounds/<variants[v].sound>.wav  →  sounds/<sound>_<v>.wav  →  base sounds/<sound>.wav
```

variant 専用音源が無ければ base を鳴らし、`variants[v].say` を TTS で補う (明示ログ)。

## ソニフィケーション (sonification.yaml)

YOLO 検出のように `class` + 連続属性 (length_mm / curvature / confidence …) が
高頻度で来る状況を、numpy で即時合成して "シンセみたいな" 音で聴き分ける追加層。

- 音族: harvest=ベル系 / health=低めざらつき / safety=異質アテンション(interrupt級) / neutral=単音
- spear 族内: A品=長三度和音 / off_grade=短三度(濁り) / young=高いピン / uncertain=デチューン揺れ
- `length_mm`→ピッチ (長いほど低い) / `curvature`→ビブラート深さ / `confidence`→音量・明るさ
- レート制限 `min_interval` (既定 0.5s/class) + キュー上限で音の洪水を防ぐ (溢れは debug ログ)
- **クラス追加は `class_families` に 1 行**。未知クラスは明示ログ + `default_family` に落ちる。

離散 cue と両立し、`class`/`attrs` が来たときだけ合成音を鳴らす (base wav は抑制)。

## 出力シンク (`output_backends`)

物理出力先を **リスト** で指定。複数指定で同時多シンク (各シンクは独自スレッドで並行再生)。

```yaml
# config/soundboard.yaml
output_backends: ["aplay"]                 # Thor ローカル ALSA (現状の既定)
# output_backends: ["audio_frame"]         # Pi 側 fv_audio_output へ AudioFrame publish
# output_backends: ["aplay", "audio_frame"]# Thor と Pi 両方で同時に鳴らす
```

- **aplay**: `aplay -D <audio_device>` で直接再生。Thor 単体 / USB スピーカー直結用。
- **audio_frame**: PCM を `fv_audio/msg/AudioFrame` にして `audio_frame_topic`
  (既定 `/audio/output/frame`) へ publish。実機ロボットでは Pi 側の `fv_audio_output`
  (C++/ALSA) が購読して鳴らす。将来の正式経路。

将来 browser 用 WS sink 等もシンクとして追加できる構造 (進化成長)。

### USB スピーカー

Thor の現状の音声出力は HDMI (card 0)。USB スピーカー接続後は次で確認して指定する:

```bash
aplay -l                       # card 番号を確認 (例 card 2)
# soundboard.yaml / launch 引数:
#   audio_device: "hw:2,0"
```

### AudioFrame 経路の interrupt の限界

`fv_audio_output` は format 一致 (既定 **48kHz mono 16bit**) が必要なので、AudioFrame
シンクは `output_sample_rate` へリサンプルしてから publish する (sonifier は既定 48kHz)。

interrupt (割込) は:
- **aplay**: 実行中の `aplay` を SIGTERM で即停止 — 確実。
- **audio_frame**: `audio_stop_topic` (既定 `/audio/output/stop`) へ Empty を publish して
  `fv_audio_output` のキューを flush する。ネットワーク越し + 別トピックのため
  **best-effort** で、割込フレームと stop の到達順によっては短い取りこぼし/重なりが起きうる。
  嘘の即時割込は実装していない (fallback 禁止)。

## system-health イベント

バッテリ/センサ/通信/温度の異常を音で判別する `category: health` 群を登録済み
(`battery_critical` / `sensor_lost` / `sensor_recovered` / `node_down` / `system_error`
/ `thermal_warning` / `comms_lost` / `comms_recovered`、`battery_low` は `low_battery` の別名)。
sensor/comms は lost=下行(欠落感) / recovered=上行(安心感) の対で判別しやすくしている。
camera/D555 watchdog 等の監視ノードが `/fv/event` に publish する想定。

```bash
ros2 topic pub -1 /fv/event std_msgs/String "{data: 'sensor_lost'}"
ros2 topic pub -1 /fv/event std_msgs/String "{data: 'battery_critical'}"
```

音源の再生成: `python3 tools/gen_health_cues.py`

## 依存
- `rclpy` / `std_msgs` / `ament_index_python` / `python3-yaml` / `python3-numpy`
- `alsa-utils` (aplay シンク使用時)
- `fv_audio` (audio_frame シンク使用時。AudioFrame msg)

## 起動
```bash
ros2 launch fv_soundboard fv_soundboard.launch.py
# 例: USB スピーカー / 両シンク
ros2 launch fv_soundboard fv_soundboard.launch.py audio_device:=hw:2,0
```
