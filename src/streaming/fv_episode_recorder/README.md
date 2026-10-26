# fv_episode_recorder

VLA training data recorder — rosbag + camera mp4 + markers + retention + replay.

OSS, ROS2 distro-agnostic (Humble / Jazzy). Used by both vlabor and aspa-navigation.

## Status

**Phase 1 Step 1 (walking skeleton)** — minimal start/stop with meta.json output.
Bag recording, mp4 segments, frames sidecar, preflight, active_lock, markers,
retention, replay come in later steps.

## Quick test

```bash
colcon build --packages-select fv_episode_msgs fv_episode_recorder
source install/setup.bash
ros2 launch fv_episode_recorder fv_episode_recorder_launch.py output_dir:=/tmp/datasets

# in another shell
curl -X POST http://localhost:8083/api/v1/episodes \
  -H 'Content-Type: application/json' \
  -d '{"task_description":"Pick up the white cube","profile":"piper_single_teleop"}'
# → 201 {"episode_id":"01J...","started_at":"...","bag_path":"...","meta_path":"..."}

curl -X POST http://localhost:8083/api/v1/episodes/<id>/stop \
  -H 'Content-Type: application/json' -d '{"outcome":"success"}'

curl http://localhost:8083/api/v1/episodes
```

## Event Bus → markers (Event Bus Contract v1)

The recorder subscribes to `/fv/event/active` (`std_msgs/String`, param
`event_active_topic`) — the situation-event stream fv_soundboard re-publishes.
While an episode is recording, every event becomes a point-in-time marker
(`kind="event"`) on that episode's timeline, so the *same* Event Bus that
drives sound + speech also produces the learning-data annotations.

Asparagus harvest cycle `detect → approach → grasp → harvest/cut → place →
success|failure` lands as ordered markers:

- phase events (`detect/approach/grasp/harvest/cut/place`) → tagged `phase`
- boundaries/outcomes (`auto_start/goal_reached/success/failure/stop`) →
  tagged `boundary`; `success`/`failure` carry a marker `outcome`
  (`success`/`abort`).

Payloads are accepted either bare (`"detect"`) or structured
(`{"event":"detect","variant":"grade_A"}`). Structured keys (variant / grade /
class / confidence / …) are stored as marker `attributes`; distinct detect
variants are rolled up to `meta.detected_variants` at stop. NOTE: fv_soundboard
currently forwards only the bare marker name — the structured payload requires
the producer to publish JSON on `/fv/event/active` (see `event_bridge.py`).

The bridge does **not** start/stop recording; episode lifecycle stays on the
REST/batch path. Auto-segmentation (auto_start → start, place/success → stop)
is a documented follow-up. Disable the bridge with `event_marker_enabled:=false`.

ASPA bring-up: `ros2 launch aspa_launch fv_episode_recorder.launch.py`.

## LeRobot 書き出し (深度・収穫の試み単位)

`fv_episode_recorder.lerobot_materializer` はエピソードディレクトリを公式
LeRobot v3.0 データセット (`lerobot` でそのまま学習できる形) に書き出す。
以下の追加オプションはすべて既定 OFF で、指定しなければ従来と同じ出力
(+ `meta/fv_episodes.jsonl` が増えるだけ)。

```bash
# 従来どおり: 1 ディレクトリ = 1 エピソード。outcome / tag で絞り込み
python3 -m fv_episode_recorder.lerobot_materializer EP_DIR... \
    --out /data/lerobot/aspa_run --repo-id vegetalia/aspa_run \
    --outcome success --tag harvest            # --outcome は any-of、--tag は all-of (繰り返し可)

# 収穫の試み単位: windows.json の 1 窓 = 1 LeRobot エピソード + 深度も同梱
python3 -m fv_episode_recorder.lerobot_materializer \
    --windows windows.json --include-depth \
    --camera-map main=top_camera,arm=arm_camera_1,depth=main,arm_depth=arm \
    --state-topic ... --action-topic ... \
    --out /data/lerobot/aspa_attempts --repo-id vegetalia/aspa_attempts
```

`windows.json` (リスト、または `{"windows": [...]}`):

```json
[{"episode_dir": "/data/episodes/aspa2/2026-10-21/01J...",
  "start_s": 12.0, "end_s": 48.5,
  "task": "アスパラを収穫する",
  "extra": {"attempt_id": "A-0007", "result": "failure", "reason": "slip",
            "mode": "auto", "session_id": "S-01"}}]
```

- `start_s`/`end_s` は meta.json `started_at` からの秒。絶対時刻なら
  `start_unix`/`end_unix` (両方あれば unix 優先)。片側省略 = その端まで。
- 窓の外のフレームは追加されない。録画と関節の重なり区間の外にある窓は
  警告して捨てる (エピソード番号は詰める)。
- 同じソースの窓はまとめて処理する (デコード 1 回)。`episode_index` 順は
  「ソースの初出順 → そのソース内の窓の記載順」。対応は fv_episodes.jsonl の
  `window.index` (= windows.json 内の位置) で引く。
- task の優先度: 窓の `task` > `--task` > meta.json `task_description`。
- ライブラリ呼び出し: `materialize(episode_dirs, out_root, repo_id, fps, camera_map,
  state_topic, action_topic, task_override, ..., windows=None, include_depth=False,
  outcomes=None, tags=None)` (`windows` は `load_windows(path)` /
  `normalize_windows(list)` の戻り値)。

出力に足されるもの:

| パス | 内容 |
|---|---|
| `meta/fv_episodes.jsonl` | `episode_index` ごとに 1 行: `source_episode_id`, `episode_dir`, `episode_path`, `profile`, `robot_id`, `operator`, `parent_session_id`, `task`, `task_description`, `started_at`, `outcome`, `tags`, `pinned`, `detected_variants`, `window` ({index,start_s,end_s,start_unix,end_unix} / null), `extra` (窓の extra そのまま: attempt_id / result / reason …), `n_frames`, `fps`, `t0_stamp_ns`, `t_end_stamp_ns`, `t0_offset_s`, `calibration`, `depth_keys` |
| `calibration` | `{"cameras": {fvカメラ名: {key, kind, camera_info}}, "tf_static": [{parent, child, xyz, quat_xyzw}] / null}`。camera_info は meta.json cameras[] の `camera_info` (プロファイルのカメラに `camera_info_topic` を書くと収録時に最初の CameraInfo を保存)、tf_static は bag の `/tf_static` (rosbags で読む)。どちらも無ければ null |
| `depth/<key>/<source_episode_id>/0000.mp4` | `--include-depth` 時。depth_hevc10 の録画をバイト単位でそのままコピー (再エンコード・再量子化なし、ソース 1 本につき 1 回) |
| `depth/<key>/<source_episode_id>/depth_meta.json`, `frames.parquet` | 復号パラメータとフレームごとのヘッダ時刻 |
| `meta/info.json` の `fv_depth` | `{"version": 1, "keys": {<key>: {...}}}`。key ごとに `path_template`, `meta_template`, `frames_template`, `codec`=hevc, `pix_fmt`=yuv420p10le, `quantisation`=inverse_depth_10bit, `d_min_counts`, `d_max_counts`, `depth_scale_mm_per_count`, `luma_levels`=1023, `invalid_luma`=0, `far_handling`, `width`, `height`, `fps`, `decode`=`fv_episode_recorder.depth10.decode_video`, `loader`, `actual_pix_fmt` (ffprobe 実測), `bit_depth_ok`, `episodes` ({"<episode_index>": {file, meta_file, frames_file, frame_from, frame_to, n_frames, t0_stamp_ns, from_stamp_ns, to_stamp_ns, first_frame_timestamp_s, d_min/d_max/scale, actual_pix_fmt, bit_depth_ok}}) |

深度 key は `observation.depth.<--camera-map で写した名前>`。LeRobot の
features には入れない (8-bit 動画エンコーダを通さないため)。読み出しは:

```python
from fv_episode_recorder.lerobot_materializer import load_depth_frames
for t, depth_mm in load_depth_frames(root, episode_index=3, key="observation.depth.main"):
    ...  # t = その LeRobot エピソードの時刻 [s] (先頭フレームは t=0 時点で有効だった深度フレームなので僅かに負もあり)
# unit="counts" (生カウント uint16、D405 の 0.1 mm 分解能を保つ) / "mm_f32" も可
```

`frame_from`/`frame_to` は復号順のフレーム番号 (両端含む)、時刻は
`(frames.parquet の ros_stamp_ns - t0_stamp_ns) / 1e9`。LeRobot フレーム k
(timestamp k/fps) と同じ時刻の深度は「その時刻以前で最新の深度フレーム」。

### 深度の実効量子化 (inverse-depth 10-bit)

距離 d の量子化ステップは `Δd ≈ d² · (1/d_min − 1/d_max) / 1022` (1022 区間、
丸め誤差は最大 ±Δd/2。センサ自体の分解能 1 カウント未満にはならない)。

| カメラ | 範囲 | 40/70 mm | 100 mm | 300 mm | 500 mm | 1 m | 2 m | 4 m |
|---|---|---|---|---|---|---|---|---|
| D555 (400–4000 counts @1 mm) | 0.4–4.0 m | 0.35 mm (@400 mm) | – | – | 0.55 mm | 2.2 mm | 8.8 mm | 35 mm |
| D405 (700–15000 counts @0.1 mm) | 7 cm–1.5 m | 0.065 mm (@70 mm) | 0.13 mm | 1.2 mm | 3.3 mm | 13 mm | – | (1.5 m: 30 mm) |

これに HEVC の符号化誤差が乗る (depth10 の PoC 実測: 把持帯 7–30 cm で中央値
0.2 mm、全作業範囲で中央値 3–6 mm)。

**注意 (8-bit 化):** `actual_pix_fmt` が `nv12`/`yuv420p` の録画はエンコーダが
黙って 8-bit で書いたもので、10-bit コードが約 255 段に落ちている (上表の
ステップが約 4 倍)。書き出しはそのまま行うが `bit_depth_ok=false` を立てて
警告する。収録側は depth10.probe_encoder_10bit で実際の出力 pix_fmt を確認し、
8-bit しか出ないエンコーダ (例: 一部ビルドの hevc_nvenc) は使わず libx265 に落ちる。

### bag のストレージ (任意)

プロファイルの `episode_recorder` に `bag_storage: mcap` と
`bag_storage_preset: zstd_fast` を書くと `ros2 bag record -s mcap
--storage-preset-profile zstd_fast` になる (要 `rosbag2_storage_mcap`)。
既定は従来どおり `-s sqlite3`。API サーバの bag 読み出し (関節グラフ・深度
プレビュー) は metadata.yaml の `storage_identifier` を見て自動で切り替える。

## Design

See `vlabor_ros2/docs/調整中/20260531_VLAbor_エピソード保存再生機能_設計書.md`.
