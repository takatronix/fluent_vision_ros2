# FV TTS

`fv_tts`はエンジン差替え式のTTSノードです。`fv_audio/msg/AudioFrame`をPublishしつつ、
`fv_tts/srv/Speak`サービスと`/aspa/tts/say`トピックの2系統で発話を提供します。

## エンジン

| engine | 実装 | 特徴 |
|--------|------|------|
| `pyopenjtalk` | Open JTalk | 軽量・追加サーバ不要・オフライン。音量控えめ (+8dB)。**選んだ時だけ使う** |
| `aivis` | AivisSpeech Engine (ロボット内、HTTP) | 高品質・男 (既定) / 女・少し速め (speedScale 1.3)。文ごとに合成して、できた文から順に流す |

**選んだエンジンが失敗しても、別の声で代わりに喋ることはしない** (2026-10-24 オーナー:
「設定で aivis, openjtalk, openai 選べればいいだけ」)。失敗した発話は飛ばして
`読み上げできませんでした（<engine>: 理由）` をログ (ERROR) に残す。viewer の設定 `tts_engine` が
`openai` (会話の返事だけ OpenAI、aspa_agent 側) のとき、このノードのアナウンスは `aivis` で喋る。

`engine` パラメータは起動時のエンジン。クライアントは `fv_tts_py/aivis_backend.py`
(`/audio_query` → `/synthesis`、標準ライブラリのみ)。

- `aivis_url`: AivisSpeech Engine の URL。`""` = 未設定 (engine=aivis なら `http://127.0.0.1:10101`)。
  launch の既定値は環境変数 `ASPA_AIVIS_URL` (旧名 `ASPA_VOICEVOX_URL` も読むが、廃止予定のログを出す)。
  **engine=pyopenjtalk でも URL が設定されていれば**、viewer の設定で再起動なしに AivisSpeech へ切り替えられる
- `aivis_speaker`: スタイル ID。`1310138977` = 男 (既定)、`135718976` = 女。viewer の設定で切り替わる
- `aivis_speed`: 話す速さ (speedScale、既定 1.3 = 少し速め、0.5〜2.0)。viewer の設定 `aivis_speed` で切り替わる
- `aivis_timeout_s`: 1 文の合成をここまで待つ (既定 15 秒)。超えたらその発話は飛ばす (合成はそのまま続けて、
  できたら保存。次回はキャッシュから)
- `aivis_keepalive_s`: 暇がこの秒数続いたら短い文 (「はい。」) を裏で合成して捨て、声の読込を保つ (既定 180 秒、0 = しない)。
  起動直後と、設定で AivisSpeech に切り替えた直後にも 1 回温める
- AivisSpeech は 44.1kHz で返すため、`output_sample_rate` (既定48000) へリサンプルしてから流します。
  **この値は `fv_audio_output` の `audio.sample_rate` と一致させること** (不一致だと無音・早送り)
- 旧設定 `engine: voicevox` (VOICEVOX 対応は削除済み) は止めずに警告を出し、`aivis_url` があれば
  `aivis`、無ければ `pyopenjtalk` として動かします

```bash
curl http://127.0.0.1:10101/version   # AivisSpeech の疎通確認 (systemd --user の aivisspeech.service)
```

## AivisSpeech の喋り方 (文ごとに合成 + 声キャッシュ)

1. 文に分ける (。！？ と改行。短い切れ端は前につなぐ、120 字を超える文は読点で分ける)
2. 文ごとに: キャッシュにあればその WAV、無ければその場で合成して保存 (`aivis_timeout_s` まで待つ)。
   できた文から順に `audio/output/frame` へ出す (先に出した音が `playback_backlog_s` (8 秒) ぶん残っている間は待つ:
   fv_audio_output の待ち行列は 8 枠)
3. 失敗したら: 不通・タイムアウトはその発話の残りも飛ばし、`voice_cache_down_skip_s` (10 秒) は試さずに飛ばす
   (毎回待たせない)。文だけの失敗 (HTTP 4xx/5xx など) はその文を飛ばして次の文へ。**どの場合も Open JTalk では喋らない**
4. 停止 (`audio/output/stop`) が来たら、喋りかけの残りの文は出さない
5. 裏スレッド (1本・nice 10・重複排除・エンジン不通時は指数バックオフ 5→300s・文ごと 3 回失敗で諦め) が
   `voice_cache_warmup_file` の文を起動 `voice_cache_warmup_delay_s` 秒後から作っておく
   (`voice_cache_max_load` 以上の loadavg の間は待つ)。声・速さを変えたら作り直す

`/aspa/tts/say` は 1 本の作業スレッドで順に喋る (合成待ちで ROS のコールバックを止めない)。
`voice_cache_max_chars` (200) を超える文は保存しない (その場で合成はする)。

| パラメータ / launch 引数 | 環境変数 (launch の既定値) | 既定 |
|---|---|---|
| `aivis_url` | `ASPA_AIVIS_URL` | `""` (engine=aivis なら `http://127.0.0.1:10101`) |
| `aivis_speaker` | `ASPA_AIVIS_SPEAKER` | `1310138977` (男) / `135718976` (女) |
| `aivis_speed` | `ASPA_TTS_AIVIS_SPEED` | `1.3` |
| `aivis_timeout_s` | `ASPA_TTS_AIVIS_TIMEOUT` | `15.0` |
| `aivis_keepalive_s` | `ASPA_TTS_AIVIS_KEEPALIVE` | `180.0` (0 = しない) |
| `voice_cache_dir` | `ASPA_TTS_VOICE_CACHE_DIR` | `~/.aspa/tts_cache/voice` |
| `voice_cache_max_mb` | `ASPA_TTS_VOICE_CACHE_MAX_MB` | `300` (超えたら最終使用が古い順に削除) |
| `voice_cache_warmup_file` | `ASPA_TTS_VOICE_CACHE_WARMUP` | `""` (相対名は `share/fv_tts/config/`) |
| `voice_cache_max_load` | `ASPA_TTS_VOICE_CACHE_MAX_LOAD` | `0` (ガード無し) |
| `voice_cache_volume_db` | `ASPA_TTS_VOICE_CACHE_VOLUME_DB` | `0.0` (AivisSpeech 素の音量 ≒ Open JTalk +8dB) |

旧 `voice_cache_live_budget_s` / `ASPA_TTS_VOICE_CACHE_LIVE_BUDGET` (間に合わなければ Open JTalk) は廃止
(launch が注意を出す)。ほか ROS パラメータのみ: `aivis_query` (audio_query 上書き JSON。`speedScale` は
`aivis_speed` が入る)、`voice_cache_timeout_s` (60)、`voice_cache_warmup_delay_s` (90)、`voice_cache_nice` (10)、
`voice_cache_max_chars` (200)、`voice_cache_down_skip_s` (10)、`playback_backlog_s` (8)。

機体の `~/.aspa/env` で有効化できる (起動スクリプトは変えない):

```bash
export ASPA_AIVIS_URL=http://127.0.0.1:10101
export ASPA_TTS_VOICE_CACHE_WARMUP=voice_cache_warmup_aspa.yaml
```

- **viewer の設定 (`/aspa/tts/settings`) で切替 (再起動不要)**: `tts_engine` が `aivis` か `openai` なら AivisSpeech、
  `pyopenjtalk` なら Open JTalk。`aivis_speaker` で男/女、`aivis_speed` で速さを切替。
  切替後は新しい声・速さでウォームアップ文を裏で作り直す。起動直後は engine=aivis なら AivisSpeech、それ以外は
  viewer の設定が届くまで Open JTalk
- **キー**: sha1(`aivis`, スタイルID, `aivis_query` (speedScale 込み), 音量dB, NFKC 正規化テキスト)。URL は含めない
- **保存**: `<key>.wav` (48kHz mono 16bit、`output_sample_rate`) + `index.json` (文・作成/最終使用時刻・
  回数)。tmp→rename の原子的書き込み。壊れた WAV は捨ててその場で作り直す
- **音量**: 既定は固定ゲイン `voice_cache_volume_db` (0 dB)。`voice_cache_match_loudness:=true` なら
  文ごとに Open JTalk 版 (`default_volume_db` 込み) と「発話区間の RMS」を揃える (ソフトリミッタ付き)
- **中身を見る**: `python3 -m json.tool ~/.aspa/tts_cache/voice/index.json`。消すならディレクトリごと削除
- テスト: `python3 -m pytest src/audio/fv_tts/test -q` (ROS が無い機械では test/conftest.py の代用で import する)

## 依存
- `pyopenjtalk-plus` (`import pyopenjtalk`互換。Open JTalk を選んだ時に要る。無くても起動はする)
- AivisSpeech Engine (engine=aivis 時のみ、外部プロセス)
- `python3-numpy`
- `python3-yaml` (ウォームアップ)、`python3-scipy` (任意。あればリサンプル品質が上がる)

## 起動
```bash
ros2 launch fv_tts fv_tts.launch.py
# または
ros2 launch fv_tts fv_tts.launch.py engine:=aivis
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
`default_volume_db` は Open JTalk のゲイン (控えめなので `+8.0`)。AivisSpeech の声は
`voice_cache_volume_db` (既定 `0.0`、ピーク 0 dBFS なのでこれ以上は割れる)。
