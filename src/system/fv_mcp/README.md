# fv_mcp — LLMからFluentVisionを操作するMCPサーバー / MCP server for LLM control

An MCP (Model Context Protocol) server that lets an LLM **rewire FluentVision
at runtime** — both the ROS pipeline graph and the live GPU screen.

LLMが**実行中のFluentVisionを組み替える**ためのMCPサーバーです。ROSパイプライン
グラフと、稼働中のGPU画面（fluent_stage Scene）の両方を対象にします。

## Tools

| Tool | What it does |
|---|---|
| `list_node_types` | 全`node_manifest.yaml`を集約したノードパレット（LLMが配線に使う語彙） |
| `list_pipelines` / `get_pipeline` | 組み込み+ユーザーのパイプライン一覧・取得 |
| `validate_pipeline` | エディタスキーマ（フラット`nodes:`）の検証 |
| `save_pipeline` | 検証合格時のみ `~/.fluent_vision/pipelines/` へ保存 |
| `run_pipeline` / `stop_pipeline` | 稼働中の`fv_pipeline_editor`のWS APIへプロキシ（起動経路は1本だけ） |
| `scene_describe` | `fvsc describe --json` — content/filter/attribute の自己記述。**シーンを書く前に必ず読む** |
| `scene_validate` | `.fvs`の型検査（画面には触れない） |
| `scene_apply` | 検証→**原子的ファイル差し替え**→scene_webがフレーム境界でスワップ→`/status`で結果確認 |
| `scene_status` | ライブ画面のdigest・reload回数・エラー・lint警告 |

**Safety model / 安全モデル**: `scene_apply`は fvsc validate に合格しない限り
ファイルに触れません。合格しても、scene_web側が再度フル検証してから
フレーム境界で差し替えるため、**壊れたシーンが画面に出る経路はゼロ**です
（人間がエディタで編集する時と同一の保証）。

## Setup

Not a ROS node — plain Python. Requires the `mcp` SDK, `pyyaml`, `aiohttp`.

```bash
# Claude Code での登録例（リポジトリルートで）
claude mcp add fluent-vision \
  -e FV_REPO_ROOT=$(pwd) \
  -e FV_SCENE_FILE=/path/to/live.fvs \
  -- python3 -m fv_mcp.server
```

| Env | Default | 意味 |
|---|---|---|
| `FV_REPO_ROOT` | auto-detect | リポジトリルート |
| `FV_EDITOR_URL` | `http://localhost:8095` | fv_pipeline_editor |
| `FV_FVSC` | `<repo>/core/fluent_stage/build/fvsc` | fvsc バイナリ |
| `FV_SCENE_FILE` | (unset) | scene_webが監視中の`.fvs`（`scene_apply`に必須） |
| `FV_SCENE_WEB_URL` | `http://localhost:8791` | scene_web |

## Typical LLM session / 典型的な流れ

```
1. scene_describe            → 使える部品を知る
2. scene_status              → 今の画面のdigestを確認
3. (LLMが .fvs を生成/編集)
4. scene_validate            → 事前検証
5. scene_apply               → フレーム境界で原子的に反映、statusで確認
```

Runtime *shader authoring* (LLM writing new GLSL) is a separate fluent_stage
engine feature — design first, per the project covenant. This server covers
recombination of the existing 30 filters / 13 content types, which is already
expressive enough for most live-screen edits.
