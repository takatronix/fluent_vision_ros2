# FV TTS

`fv_tts`はエンジン差替え式のTTSノードです。`fv_audio/msg/AudioFrame`をPublishしつつ、
`fv_tts/srv/Speak`サービスと`/aspa/tts/say`トピックの2系統で発話を提供します。

## エンジン

| engine | 実装 | 特徴 |
|--------|------|------|
| `pyopenjtalk` (既定) | Open JTalk | 軽量・追加サーバ不要・オフライン。音量控えめ (+8dB) |
| `aivis` | AivisSpeech Engine (ロボット内、HTTP) + Open JTalk | 高品質・女/男。エンジンが不通・遅いときは Open JTalk で喋る (無音にしない) |

`engine` パラメータで選びます。クライアントは `fv_tts_py/aivis_backend.py`
(`/audio_query` → `/synthesis`、標準ライブラリのみ)。

- `aivis_url`: AivisSpeech Engine の URL。`""` = 未設定 (engine=aivis なら `http://127.0.0.1:10101`)。
  launch の既定値は環境変数 `ASPA_AIVIS_URL` (旧名 `ASPA_VOICEVOX_URL` も読むが、廃止予定のログを出す)。
  **engine=pyopenjtalk でも URL が設定されていれば**、viewer の設定で再起動なしに AivisSpeech へ切り替えられる
- `aivis_speaker`: スタイル ID。`135718976` = 女 (既定)、`1310138977` = 男。viewer の設定で切り替わる
- AivisSpeech は 44.1kHz で返すため、`output_sample_rate` (既定48000) へリサンプルしてから流します。
  **この値は `fv_audio_output` の `audio.sample_rate` と一致させること** (不一致だと無音)
- 旧設定 `engine: voicevox` (VOICEVOX 対応は削除済み) は止めずに警告を出し、`aivis_url` があれば
  `aivis`、無ければ `pyopenjtalk` として動かします

```bash
curl http://127.0.0.1:10101/version   # AivisSpeech の疎通確認 (systemd --user の aivisspeech.service)
```

## AivisSpeech の喋り方 (声キャッシュ + Open JTalk の保険)

AivisSpeech は GPU なら 1 文 0.3〜0.7 秒だが、未読込の声や高負荷では遅くなる。そこで:

1. キャッシュにある文 → 保存した AivisSpeech の WAV を流す
2. キャッシュに無い文 → エンジンが元気なら `voice_cache_live_budget_s` (2 秒) まで
   その場の合成を待って流し、保存する。間に合わない・不通 → Open JTalk で即喋り、
   AivisSpeech 版は裏で作る (不通を検知したら `voice_cache_down_skip_s` (30 秒) は待たずに Open JTalk)
3. 裏スレッド (1本・nice 10・重複排除・ライブ優先・タイムアウト付き・エンジン不通時は
   指数バックオフ 5→300s・文ごと 3 回失敗で諦め) が AivisSpeech で合成し WAV に保存
4. `voice_cache_warmup_file` の文を起動 `voice_cache_warmup_delay_s` 秒後から裏で作っておく
   (`voice_cache_max_load` 以上の loadavg の間は待つ)

`voice_id` 指定の要求と `voice_cache_max_chars` (120) 超の文は Open JTalk で喋る。

| パラメータ / launch 引数 | 環境変数 (launch の既定値) | 既定 |
|---|---|---|
| `aivis_url` | `ASPA_AIVIS_URL` | `""` (engine=aivis なら `http://127.0.0.1:10101`) |
| `aivis_speaker` | `ASPA_AIVIS_SPEAKER` | `135718976` (女) / `1310138977` (男) |
| `voice_cache_dir` | `ASPA_TTS_VOICE_CACHE_DIR` | `~/.aspa/tts_cache/voice` |
| `voice_cache_max_mb` | `ASPA_TTS_VOICE_CACHE_MAX_MB` | `300` (超えたら最終使用が古い順に削除) |
| `voice_cache_warmup_file` | `ASPA_TTS_VOICE_CACHE_WARMUP` | `""` (相対名は `share/fv_tts/config/`) |
| `voice_cache_max_load` | `ASPA_TTS_VOICE_CACHE_MAX_LOAD` | `0` (ガード無し) |
| `voice_cache_live_budget_s` | `ASPA_TTS_VOICE_CACHE_LIVE_BUDGET` | `2.0` (0 = その場合成しない) |
| `voice_cache_volume_db` | `ASPA_TTS_VOICE_CACHE_VOLUME_DB` | `0.0` (AivisSpeech 素の音量 ≒ Open JTalk +8dB) |

ほか ROS パラメータのみ: `aivis_query` (audio_query 上書き JSON 例 `{"speedScale":1.1}`)、
`voice_cache_timeout_s` (60)、`voice_cache_warmup_delay_s` (90)、`voice_cache_nice` (10)、
`voice_cache_max_chars` (120)。

機体の `~/.aspa/env` で有効化できる (起動スクリプトは変えない):

```bash
export ASPA_AIVIS_URL=http://127.0.0.1:10101
export ASPA_TTS_VOICE_CACHE_WARMUP=voice_cache_warmup_aspa.yaml
```

- **viewer の設定 (`/aspa/tts/settings`) で切替 (再起動不要)**: `tts_engine` が `aivis` なら AivisSpeech ON、
  `pyopenjtalk` なら OFF (Open JTalk のみ)。`aivis_speaker` で女/男を切替。
  切替後は新しい声でウォームアップ文を裏で作り直す。起動直後は engine=aivis なら ON、それ以外は
  viewer の設定が届くまで OFF
- **キー**: sha1(`aivis`, スタイルID, `aivis_query`, 音量dB, NFKC 正規化テキスト)。URL は含めない
- **保存**: `<key>.wav` (48kHz mono 16bit、`output_sample_rate`) + `index.json` (文・作成/最終使用時刻・
  回数)。tmp→rename の原子的書き込み。壊れた WAV は捨てて Open JTalk へ
- **音量**: 既定は固定ゲイン `voice_cache_volume_db` (0 dB)。`voice_cache_match_loudness:=true` なら
  文ごとに Open JTalk 版 (`default_volume_db` 込み) と「発話区間の RMS」を揃える (ソフトリミッタ付き)
- **中身を見る**: `python3 -m json.tool ~/.aspa/tts_cache/voice/index.json`。消すならディレクトリごと削除
- テスト: `python3 -m pytest src/audio/fv_tts/test -q` (ノード部分は ROS 環境で実行、無ければ skip)

## 依存
- `pyopenjtalk-plus` (`import pyopenjtalk`互換。どちらのエンジンでも必要)
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
