# LingBot-Depth Containerization

## 結論

`fluent_vision_ros2` と繋ぐ前提なら、分けるべきなのは **LingBot の依存** であって、`FluentVision` の ROS 2 全体ではない。

推奨構成:

1. `FluentVision` 側
   - `fv_realsense`
   - `fv_camera`
   - `fv_lingbot_depth` の薄い ROS bridge
2. `LingBot` 側
   - 専用 worker コンテナ
   - Torch / xformers / Jetson 依存を閉じ込める
3. 通信
   - HTTP (`worker_endpoint`)

## なぜ FluentVision の ROS 2 全体を分けないのか

- カメラ I/O と ROS topic 配線は FluentVision 側に既にまとまっている
- そこでさらに ROS 2 コンテナを分けても、得られる利益より配線コストが増える
- Intel / ARM 差分は主に Torch 周りなので、worker だけ分ければ十分

ROS 2 全体を別コンテナに切る意味があるのは次の場合だけ:

- FluentVision 本体と ROS distro を分けたい
- DDS 通信境界を明示したい
- RealSense や GPU を用途別に物理分離したい

通常はそこまでやる必要はない。

## Intel / ARM 両対応の考え方

- Intel:
  - upstream PyTorch wheel を使いやすい
  - Worker コンテナ化が比較的簡単
- ARM / Jetson:
  - JetPack 対応 PyTorch を選ぶ必要がある
  - `BASE_IMAGE` や `TORCH_PACKAGES` を実機に合わせて切り替える

この差分は worker コンテナの build args に閉じ込める。

## 実装済み

- `fv_lingbot_depth` に `backend=http` を追加
- `worker_endpoint` / `worker_timeout_sec` を追加
- `docker/lingbot_depth_worker/` に worker 雛形を追加
- `src/system/fluent_vision_system/config/fv_lingbot_depth_d405_http.yaml` を追加

## 起動手順の最小例

1. Worker を起動
2. `fv_lingbot_depth_d405_http.yaml` を使って `fv_lingbot_depth` を起動
3. `refined depth` / `mask` / `points` を FluentVision 既存 topic に流す

## 将来拡張

この構成は `LingBot-Map` にも流用できる。

- `fv_lingbot_map_bridge` を ROS 側に追加
- `lingbot-map-worker` を別コンテナ化
- pose / point cloud / trajectory を HTTP or gRPC で受け渡す
