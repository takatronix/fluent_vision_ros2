# FV TTS

`fv_tts`はエンジン差替え式のTTSノードです。`fv_audio/msg/AudioFrame`をPublishしつつ、
`fv_tts/srv/Speak`サービスと`/aspa/tts/say`トピックの2系統で発話を提供します。

## エンジン (差替え式)

| engine | 実装 | 特徴 |
|--------|------|------|
| `pyopenjtalk` (既定) | Open JTalk | 軽量・追加サーバ不要・オフライン。音量控えめ |
| `voicevox` | VOICEVOX ENGINE (HTTP) | 高品質・話者選択可。**別プロセスの ENGINE 起動が必要** (24kHz出力→自動リサンプル) |

`engine`パラメータで切り替えます。「進化成長」原則に沿ってバックエンドは抽象化されており、
`fv_tts_py/voicevox_backend.py`のように新エンジンを足せます。**fallbackはしません** — 選んだ
エンジンが不通なら明示的な`RuntimeError`で止まります。

### VOICEVOX ENGINE

別プロセスで VOICEVOX ENGINE を起動しておきます (既定 `http://127.0.0.1:50021`)。

```bash
curl http://127.0.0.1:50021/version   # 疎通確認
```

- `voicevox_url`: ENGINE のURL
- `voicevox_speaker`: 話者ID (既定 30 = No.7 / アナウンス)。`Speak.voice_id`に数値文字列を渡すと個別上書き
- ENGINE は 24kHz mono 16bit で返すため、`output_sample_rate` (既定48000) へリサンプルしてから
  `audio/output/frame`へ流します。**この値は`fv_audio_output`の`audio.sample_rate`と一致させること**
  (不一致だと format mismatch で無音になります)。

## 声キャッシュ (その場は Open JTalk、裏で作って次からキャッシュの声)

VOICEVOX 互換エンジン (AivisSpeech 等) は高品質だが、混んだ CPU では 6 秒の文に 10 秒以上
かかりライブでは間に合わない。そこで `engine:=pyopenjtalk` のまま次の動きを足せる:

1. キャッシュに無い文 → 今まで通り Open JTalk で**即**喋る (遅延ゼロ)。同じ文を裏で予約
2. 裏スレッド (1本・nice 10・重複排除・ライブ優先・タイムアウト付き・エンジン不通時は
   指数バックオフ 5→300s・文ごと 3 回失敗で諦め) が高品質の声で合成し WAV に保存
3. 次に同じ文 → 保存した WAV を流す
4. `voice_cache_warmup_file` の文を起動 `voice_cache_warmup_delay_s` 秒後から裏で作っておく
   (`voice_cache_max_load` 以上の loadavg の間は待つ)

エンジンが無い・遅い・壊れていても Open JTalk で喋るので**無音にならない**。
`voice_id` 指定の要求と `voice_cache_max_chars` (120) 超の文はキャッシュを使わない。

| パラメータ / launch 引数 | 環境変数 (launch の既定値) | 既定 |
|---|---|---|
| `voice_cache_engine` | `ASPA_TTS_VOICE_CACHE_ENGINE` | `""` (無効) / `voicevox` |
| `voice_cache_url` | `ASPA_TTS_VOICE_CACHE_URL` | `http://127.0.0.1:10101` (AivisSpeech) |
| `voice_cache_speaker` | `ASPA_TTS_VOICE_CACHE_SPEAKER` | `-1` (要設定。例 135718976) |
| `voice_cache_dir` | `ASPA_TTS_VOICE_CACHE_DIR` | `~/.aspa/tts_cache/voice` |
| `voice_cache_max_mb` | `ASPA_TTS_VOICE_CACHE_MAX_MB` | `300` (超えたら最終使用が古い順に削除) |
| `voice_cache_warmup_file` | `ASPA_TTS_VOICE_CACHE_WARMUP` | `""` (相対名は `share/fv_tts/config/`) |
| `voice_cache_max_load` | `ASPA_TTS_VOICE_CACHE_MAX_LOAD` | `0` (ガード無し) |

ほか ROS パラメータのみ: `voice_cache_query` (audio_query 上書き JSON 例 `{"speedScale":1.1}`)、
`voice_cache_timeout_s` (60)、`voice_cache_warmup_delay_s` (90)、`voice_cache_nice` (10)、
`voice_cache_max_chars` (120)。

起動スクリプトは変えずに、機体の `~/.aspa/env` で有効化できる:

```bash
export ASPA_TTS_VOICE_CACHE_ENGINE=voicevox
export ASPA_TTS_VOICE_CACHE_URL=http://127.0.0.1:10101
export ASPA_TTS_VOICE_CACHE_SPEAKER=135718976      # 楽町音穏_T3モデル ノーマル
export ASPA_TTS_VOICE_CACHE_WARMUP=voice_cache_warmup_aspa.yaml
```

- **キー**: sha1(エンジン種別, 話者ID, `voice_cache_query`, 音量dB, NFKC 正規化テキスト)。URL は含めない
- **保存**: `<key>.wav` (48kHz mono 16bit、`output_sample_rate`) + `index.json` (文・作成/最終使用時刻・
  回数)。tmp→rename の原子的書き込み。壊れた WAV は捨てて Open JTalk へ
- **音量**: 同じ文の Open JTalk 版 (`default_volume_db` 込み) と「発話区間の RMS」を揃え、
  ピークはソフトリミッタで 1.0 未満に。声が切り替わっても音量が跳ねない
- **中身を見る**: `python3 -m json.tool ~/.aspa/tts_cache/voice/index.json`。消すならディレクトリごと削除
- `engine:=voicevox` のときは無効 (もともと全部高品質)
- テスト: `python3 -m pytest src/audio/fv_tts/test -q` (ノード部分は ROS 環境で実行、無ければ skip)

## 依存
- `pyopenjtalk-plus` (`import pyopenjtalk`互換、engine=pyopenjtalk 時)
- VOICEVOX ENGINE (engine=voicevox 時のみ、外部プロセス)
- `python3-numpy`
- `python3-yaml` (声キャッシュのウォームアップ)、`python3-scipy` (任意。あればリサンプル品質が上がる)

## 起動
```bash
ros2 launch fv_tts fv_tts.launch.py
# または
ros2 launch fv_tts fv_tts.launch.py engine:=voicevox default_volume_db:=0.0
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
