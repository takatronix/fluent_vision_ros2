# FV TTS

`fv_tts`はエンジン差替え式のTTSノードです。`fv_audio/msg/AudioFrame`をPublishしつつ、
`fv_tts/srv/Speak`サービスと`/aspa/tts/say`トピックの2系統で発話を提供します。

## エンジン (差替え式)

| engine | 実装 | 特徴 |
|--------|------|------|
| `voicevox` (既定) | VOICEVOX ENGINE (HTTP) | 高品質・ずんだもん等。**別プロセスの ENGINE 起動が必要** (24kHz出力→自動リサンプル) |
| `pyopenjtalk` | Open JTalk | 追加サーバ不要・オフライン。音量控えめ |

`engine`パラメータで切り替えます。「進化成長」原則に沿ってバックエンドは抽象化されており、
`fv_tts_py/voicevox_backend.py`のように新エンジンを足せます。**fallbackはしません** — 選んだ
エンジンが不通なら明示的な`RuntimeError`で止まります。

### VOICEVOX ENGINE

別プロセスで VOICEVOX ENGINE を起動しておきます (既定 `http://127.0.0.1:50021`)。

```bash
curl http://127.0.0.1:50021/version   # 疎通確認
```

- `voicevox_url`: ENGINE のURL
- `voicevox_speaker`: 話者ID (既定 3 = ずんだもんノーマル)。`Speak.voice_id`に数値文字列を渡すと個別上書き
- ENGINE は 24kHz mono 16bit で返すため、`output_sample_rate` (既定48000) へリサンプルしてから
  `audio/output/frame`へ流します。**この値は`fv_audio_output`の`audio.sample_rate`と一致させること**
  (不一致だと format mismatch で無音になります)。

## 依存
- VOICEVOX ENGINE (engine=voicevox 時、外部プロセス)
- `pyopenjtalk` (engine=pyopenjtalk 時)
- `python3-numpy`

## 起動
```bash
ros2 launch fv_tts fv_tts.launch.py
# または
ros2 run fv_tts fv_tts_node --ros-args -p engine:=voicevox -p voicevox_speaker:=3
```

## 発話の2系統

### 1. Speak サービス (合成結果を返す)
```bash
ros2 service call /fv_tts/speak fv_tts/srv/Speak "{text: 'こんにちは', voice_id: '', play: true, volume_db: 0.0, cache_key: ''}"
```
`play: true`で`audio/output/frame`へPublishし、`fv_audio_output`が購読していれば再生されます。

### 2. `/aspa/tts/say` トピック (Event Bus v1)
```bash
ros2 topic pub -1 /aspa/tts/say std_msgs/String "{data: 'アスパラを見つけました'}"
```
`fv_soundboard`が状況イベントの台詞をここへ流します。受信すると即合成・再生し、
再生中は`/aspa/audio/speaking` (std_msgs/Bool) を`true`にして再生尺の経過後`false`に戻します。

## Event Bus 契約 (v1)
- `/aspa/tts/say` (std_msgs/String): 任意テキスト→発話 (本ノードが消費)
- `/aspa/audio/speaking` (std_msgs/Bool): 再生中フラグ (本ノードが publish)

## キャッシュ
`cache_dir`（既定 `~/.fluent_voice_cache`）にPCMとメタ情報を保存します。`cache_key`で共有可能。

## 音量
`default_volume_db`は engine 依存です。voicevox は元々大きいので `0.0` 前後、pyopenjtalk は
控えめなので `+8.0` 程度を推奨します。
