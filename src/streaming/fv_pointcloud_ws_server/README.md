# fv_pointcloud_ws_server

FluentVision向けの PointCloud2 バイナリ WebSocket 配信ノード。

## 概要

- 入力: `sensor_msgs/msg/PointCloud2`
- 出力: WebSocket Binary frame (`FVPC` header + xyzrgba payload)
- 特徴:
  - クライアント0時に購読停止（Lazy）
  - 距離フィルタ
  - 点数上限
  - サンプル間引き
  - `stream.codec=lz4` で高速圧縮（自動フォールバック）

## 主要パラメータ

- `pointcloud_topic` (string)
- `server.host` (string, default: `0.0.0.0`)
- `server.port` (int, default: `9070`)
- `stream.max_points` (int, default: `20000`)
- `stream.sample_step` (int, default: `2`)
- `stream.min_range_m` (double, default: `0.20`)
- `stream.max_range_m` (double, default: `2.50`)
- `stream.codec` (string, `none|lz4`, default: `none`)
- `stream.compression_min_bytes` (int, default: `65536`)
- `lazy.cooldown_sec` (double, default: `5.0`)

## プロトコル

- `FVPC v1`: 非圧縮 payload（従来）
- `FVPC v2`: `codec` + `payload_size/raw_size` を持つ拡張ヘッダ
  - `codec=1` は LZ4 圧縮 payload
  - 圧縮率が悪い場合は自動で `v1` にフォールバック
