# FluentVision PointCloud Web Streaming 設計（v1）

## 1. 目的

FluentVision の設計思想（低遅延・実運用・安全運用）に合わせて、
`PointCloud2` を Web ダッシュボードへ高効率に配信する。

この設計の狙いは次の 3 点。

- 点群のリアルタイム表示を滑らかにする（回転・ズーム・密度変更に耐える）
- 既存 VLABOR `/ws` 制御系を壊さない（責務分離）
- サブスクライバ不在時は処理停止して帯域/CPU を守る

## 2. FluentVision コンセプトとの整合

既存ルールへの準拠。

- `node_manifest.yaml` でノード定義を公開（Pipeline Editor 連携）
- `category: streaming` で配置
- `Lazy Publishing` 原則に従い、購読者 0 なら重処理を止める
- `pointcloud` 型ポートを明示して配線可能にする

参照:
- `docs/pipeline_integration.md`
- `src/streaming/fv_websocket_server/config/node_manifest.yaml`

## 3. 方針（rosbridge との役割分離）

`rosbridge` は開発初期や汎用連携には有効だが、点群高負荷配信には不利。

本番方針:

- 軽量制御/状態: 既存 `/ws`（JSON）
- 画像: 既存 streaming 系（MJPEG/WS）
- 点群: 新規 `fv_pointcloud_ws_server`（バイナリ WS）

## 4. 新規パッケージ提案

- パッケージ名: `fv_pointcloud_ws_server`
- 配置: `src/streaming/fv_pointcloud_ws_server`
- 実行: `fv_pointcloud_ws_server_node`
- 言語: C++（既存 streaming 群と同系統）

### 4.1 ノード責務

- `sensor_msgs/msg/PointCloud2` を購読
- ROI/距離/ダウンサンプルで配信前最適化
- WebSocket で複数クライアントへバイナリ配信
- クライアント毎の要求品質（profile）を受け取り反映
- 接続数/送信遅延/ドロップ率をメトリクス出力

### 4.2 `node_manifest.yaml`（案）

```yaml
nodes:
  - key: fv_pointcloud_ws_server
    package: fv_pointcloud_ws_server
    exec: fv_pointcloud_ws_server_node
    category: streaming
    label: "PointCloud WS Server"

    inputs:
      - param: pointcloud_topic
        type: pointcloud

    outputs: []

    default_parameters:
      pointcloud_topic: ""
      server.port: 9070
      server.host: "0.0.0.0"
      stream.profile: "balanced"
      stream.max_points: 20000
      stream.min_range_m: 0.20
      stream.max_range_m: 2.50
      stream.voxel_m: 0.010
      lazy.cooldown_sec: 5.0
```

## 5. データフロー

```text
ROS2 PointCloud2
  -> fv_pointcloud_ws_server
    -> [need_streaming?]
      no: 処理停止
      yes:
        1) range/ROI filter
        2) voxel downsample
        3) max_points clamp
        4) binary encode
        5) websocket broadcast
  -> Browser PointCloudViewer (WebGL)
```

## 6. WebSocket プロトコル設計

制御チャネル（JSON）とデータチャネル（Binary）を同一 WS 上で扱う。

### 6.1 Control Message（JSON）

Client -> Server:

- `hello`
- `subscribe`
- `unsubscribe`
- `set_profile`
- `set_filters`（`max_points`, `range`, `voxel`）
- `ping`

Server -> Client:

- `hello_ack`
- `subscribed`
- `status`
- `error`
- `pong`

### 6.2 Binary Frame（v1）

ヘッダ固定 + 点群配列。

```c
struct FvPcHeaderV1 {
  char     magic[4];      // "FVPC"
  uint16_t version;       // 1
  uint16_t header_size;   // sizeof(FvPcHeaderV1)
  uint64_t stamp_ns;      // ROS time
  uint32_t seq;
  uint32_t point_count;
  uint8_t  point_format;  // 0: xyz_f32_rgb_u8
  uint8_t  flags;         // bit0: little endian
  uint16_t reserved;
  float    min_range_m;
  float    max_range_m;
};
```

`point_format=0` payload（1点 16 bytes）:

- `float x`
- `float y`
- `float z`
- `uint8 r, g, b, a`

この形式は JS 側で `DataView` 直読でき、デコードが軽い。

## 7. パフォーマンス設計

前提: 1点 = 16 bytes（ヘッダ除く）

- `preview`: 5k points @ 10Hz  -> 約 0.8 MB/s
- `balanced`: 20k points @ 15Hz -> 約 4.8 MB/s
- `quality`: 50k points @ 20Hz -> 約 16.0 MB/s

### 7.1 適応制御

- 送信キュー滞留時は自動的に `max_points` を段階的に下げる
- クライアント復帰後に段階的に戻す
- 古いフレームは破棄（latest-only）

### 7.2 Lazy 実装

次の全条件が false なら購読停止または重処理停止。

- WS クライアント数 > 0
- `subscribe` 済みクライアントが 1 以上

停止はクールダウン（既定 5秒）後に実施。

## 8. 安全設計

- 既定 bind: `127.0.0.1`（ローカルのみ）
- 外部公開時は `allowed_origins` と `auth.token` を必須化
- 受信 JSON サイズ上限を設定
- `set_profile`/`set_filters` は上限下限を厳密チェック
- 無効入力は `error` 返却し無視

## 9. Web Viewer 設計（既存 pointcloud-viewer.js 拡張）

現状の `ROSLIB` モードに加えて `fvws` モードを追加。

- `mode: "rosbridge" | "fvws"`
- `fvws` 時は `ws://host:9070/pc` に接続
- Binary frame を直接 decode して `THREE.BufferGeometry` 更新
- UIで `profile`, `max_points`, `range` を送信

後方互換:

- 既存 `connectToROS()/subscribeToPointCloud()` は維持
- 新規 `connectToFVWS()/subscribeFVWS()` を追加

## 10. 実装フェーズ

### Phase 1（最小価値）

- `fv_pointcloud_ws_server` 雛形
- PointCloud2 -> Binary 配信
- 1クライアント接続確認
- `pointcloud-viewer.js` に `fvws` 受信追加

完了条件:

- D405 点群がブラウザで 10Hz 以上で表示
- ノードクラッシュなし 30分連続動作

### Phase 2（運用品質）

- Lazy + cooldown
- adaptive quality
- 複数クライアント
- メトリクス（fps/latency/drop）

完了条件:

- クライアント 0 で処理ほぼ停止
- クライアント 2 同時でも UI 操作が維持

### Phase 3（統合）

- `node_manifest` 追加
- `pipelines/pointcloud_web_preview.yaml` 追加
- VLABOR からの実験ページ導線追加
- README/運用手順整備

## 11. テスト戦略

- Unit: binary pack/unpack, parameter clamp, filter logic
- Integration: ROS2 bag 再生 + WS 受信の E2E
- Soak: 2時間連続配信（メモリリーク/再接続）
- Cross-arch: amd64 + arm64（AGX Thor）

## 12. OSS 公開品質チェック

- API と wire format を `docs/protocols/fv_pointcloud_ws_v1.md` に固定
- バージョン互換ポリシー明記（v1 -> v2）
- サンプルクライアント（JS最小）同梱
- Bench 結果（CPU/帯域/遅延）を README 公開

---

この設計は「まず動く」より一段上の、
`FluentVision で長期運用できる点群基盤` を目的にしています。
