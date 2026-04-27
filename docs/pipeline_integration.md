# FluentVision Pipeline Integration Guide

FluentVision パイプラインエディタと各ノードパッケージの連携仕様。

---

## 1. node_manifest.yaml 仕様

各 fv パッケージは `config/node_manifest.yaml` を配置してノード定義を宣言する。

### 1.1 ファイル構造

```yaml
nodes:
  - key: <一意のテンプレートキー>
    package: <ros2 パッケージ名>
    exec: <実行可能ファイル名>
    category: sensor | ai | streaming | audio | ui | utils
    label: "UI表示名"

    inputs:
      - param: <入力パラメータパス>
        type: image | depth | camera_info | pointcloud | detections | audio | any
        label: "表示名"  # 省略可: param から自動生成

    outputs:
      - param: <出力パラメータパス>
        type: <ポート型>
        label: "表示名"  # 省略可

    default_parameters:
      <パラメータ名>: <デフォルト値>
```

### 1.2 フィールド説明

| フィールド | 必須 | 説明 |
|---|---|---|
| `key` | Yes | テンプレートの一意識別子。通常はパッケージ名と同じ |
| `package` | Yes | `ros2 run <package>` で使う名前 |
| `exec` | Yes | `ros2 run ... <exec>` で使う実行ファイル名 |
| `category` | Yes | カテゴリ。UIの色・バッジ・パレット分類に使用 |
| `label` | Yes | エディタ上の表示名 |
| `inputs` | Yes | 入力ポート一覧（空配列可） |
| `outputs` | Yes | 出力ポート一覧（空配列可） |
| `default_parameters` | No | ノード追加時のデフォルトパラメータ |

### 1.3 ポート型一覧

| 型 | UIカラー | ROS2 メッセージ型 |
|---|---|---|
| `image` | 水色 `#4fc3f7` | `sensor_msgs/Image` |
| `depth` | 紫 `#ab47bc` | `sensor_msgs/Image` (16UC1/32FC1) |
| `camera_info` | グレー `#78909c` | `sensor_msgs/CameraInfo` |
| `pointcloud` | 緑 `#66bb6a` | `sensor_msgs/PointCloud2` |
| `detections` | オレンジ `#ffa726` | `fv_msgs/DetectionArray` |
| `audio` | 赤 `#ef5350` | `fv_msgs/AudioFrame` |
| `any` | 薄灰 `#bdbdbd` | 任意（Topic Relay 等） |

### 1.4 互換性ルール

- 同じ型同士 → 接続可
- `any` → どの型とも接続可
- `image` 入力 ← `depth` 出力 → 接続可（どちらも sensor_msgs/Image）

### 1.5 パラメータパスの記法

- フラット: `input_topic` → ROS パラメータ `input_topic`
- ネスト: `topics.color` → ROS パラメータ `topics.color`
  - エディタ上のラベルは最後のドット以降（`color`）
  - トピック解決時はドットをスラッシュに変換
    （例: `topics.color` → `<namespace>/<node_name>/topics/color`）

---

## 2. Lazy Publishing 実装ガイド

### 2.1 原則

**サブスクライバーがいる出力だけ処理・配信する。**

ON/OFF パラメータ（`streams.depth_enabled` 等）は**廃止**。
代わりに `get_subscription_count()` でサブスクライバーの存在を確認する。

### 2.2 C++ 実装パターン

```cpp
// 毎フレームのコールバック内
if (color_pub_->get_subscription_count() > 0) {
    color_pub_->publish(color_msg);
}

// 重い処理は条件付きで実行
if (depth_colormap_pub_->get_subscription_count() > 0) {
    auto colormap = createDepthColormap(depth_frame);
    depth_colormap_pub_->publish(colormap);
}

// ストリーム全体の ON/OFF
bool need_depth =
    depth_pub_->get_subscription_count() > 0 ||
    depth_colormap_pub_->get_subscription_count() > 0 ||
    pointcloud_pub_->get_subscription_count() > 0;
```

### 2.3 Python 実装パターン

```python
def on_image_callback(self, msg):
    detections = self.model.predict(msg)

    if self.overlay_pub.get_subscription_count() > 0:
        overlay = self.draw_overlay(msg, detections)
        self.overlay_pub.publish(overlay)

    if self.detections_pub.get_subscription_count() > 0:
        self.detections_pub.publish(detections)
```

### 2.4 クールダウン

ハードウェアストリーム（RealSense depth 等）の場合、
サブスクライバー数が 0 になってから **3〜5秒** 待ってから停止する。

```cpp
if (!need_depth) {
    if (depth_idle_since_ == 0) depth_idle_since_ = now();
    if (now() - depth_idle_since_ > 5.0) disableDepthStream();
} else {
    depth_idle_since_ = 0;
    if (!depth_streaming_) enableDepthStream();
}
```

### 2.5 廃止パラメータ

| パッケージ | 廃止パラメータ |
|---|---|
| fv_realsense | `streams.depth_enabled`, `streams.depth_colormap_enabled`, `streams.pointcloud_enabled`, `streams.infrared_enabled`, `camera_info.enable_camera_info`, `camera_info.enable_compressed_topics` |
| fv_instance_seg | `publish_detections`, `publish_overlay` |

---

## 3. パイプライン YAML 仕様

### 3.1 ファイル構造

```yaml
name: "パイプライン名"
description: "説明"
created: "2026-02-14T10:00:00"
modified: "2026-02-14T12:00:00"

system:
  camera_start_delay: 2.0
  default_start_delay: 0.5
  container_start_delay: 1.0

containers:
  - id: <一意のコンテナID>
    enable: true
    template: <container_template_key>
    parameters:
      <key>: <value>

nodes:
  - id: <一意のノードID>
    enable: true
    package: <パッケージ名>
    exec: <実行ファイル名>
    node_name: <ROS2ノード名>
    namespace: /fv
    parameters:
      <key>: <value>
    _template: <テンプレートキー>

layout:
  <node_id>: { x: 100, y: 200, width: 210 }
```

### 3.2 保存先

| ディレクトリ | 用途 |
|---|---|
| `~/.fluent_vision/pipelines/` | デフォルト（ユーザー用） |
| `fluent_vision_ros2/pipelines/` | ビルトイン（読み取り専用） |
| 任意のディレクトリ | UIから切り替え可能 |

### 3.3 Container 拡張

重い推論 runtime を Docker に分離する場合、pipeline YAML に `containers:` を持てる。

- `template`: `docker/**/container_manifest.yaml` の `containers[].key`
- `parameters`: template の `default_parameters` を上書き
- `system.container_start_delay`: container 起動後に次へ進むまでの待ち時間

典型例:

```yaml
containers:
  - id: openpi_runtime_1
    enable: true
    template: openpi_runtime
    parameters:
      container_name: "fv-openpi-runtime"
      env:
        OPENPI_PORT: "8000"
```

---

## 4. WebSocket API

エディタバックエンド（port 8095）が提供する WebSocket エンドポイント。

### 4.1 メッセージ一覧

| 方向 | type | 説明 |
|---|---|---|
| → | `get_templates` | テンプレート一覧を要求 |
| ← | `templates` | テンプレート + ポート互換表 |
| → | `list_pipelines` | パイプラインファイル一覧を要求 |
| ← | `pipeline_list` | ユーザー + ビルトインのリスト |
| → | `load_pipeline` | パイプライン読み込み |
| ← | `pipeline_data` | パイプライン内容 |
| → | `save_pipeline` | パイプライン保存 |
| ← | `save_result` | 保存結果 |
| → | `delete_pipeline` | パイプライン削除 |
| ← | `delete_result` | 削除結果 |
| → | `set_pipeline_dir` | 保存先ディレクトリ変更 |
| → | `launch_node` | ノード単体起動 |
| ← | `launch_result` | 起動結果 |
| → | `stop_node` | ノード単体停止 |
| ← | `stop_result` | 停止結果 |
| → | `launch_pipeline` | パイプライン全体起動 |
| → | `stop_pipeline` | 全ノード停止 |
| ← | `node_status` | ステータス変更通知 |
| → | `get_statuses` | 全ステータスを要求 |
| ← | `statuses` | 全ステータス |

---

## 5. エディタの起動

```bash
# 単独起動
ros2 run fv_pipeline_editor editor_node

# パラメータ指定
ros2 run fv_pipeline_editor editor_node --ros-args \
  -p port:=8095 \
  -p pipeline_dir:=/path/to/pipelines

# vlabor からは iframe で埋め込み
# http://localhost:8095/
```

---

## 6. 命名規則

| 種別 | 規則 | 例 |
|---|---|---|
| パッケージ名 | `fv_<機能名>` | `fv_realsense`, `fv_instance_seg` |
| 実行ファイル名 | `fv_<機能名>_node` | `fv_realsense_node` |
| テンプレートキー | パッケージ名と同じ | `fv_realsense` |
| ノード名（実行時） | `<短縮名>_<番号>` | `realsense_1`, `instance_seg_1` |
| デフォルト namespace | `/fv` | |
| トピックパス | `/<ns>/<node_name>/<param_path>` | `/fv/realsense_1/color/image_raw` |
