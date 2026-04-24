# LingBot-Depth Worker

`fv_lingbot_depth` から HTTP で呼び出される専用 worker です。

## 目的

- LingBot の重い Python 依存を `fluent_vision_ros2` 本体から分離する
- Intel / ARM で Torch 周りの差分を worker コンテナ側に閉じ込める
- ROS 2 は FluentVision 側に残し、worker は非 ROS プロセスにする

## 構成

```text
fv_realsense / fv_camera
    -> fv_lingbot_depth (ROS 2 node, thin bridge)
    -> HTTP POST /infer
    -> lingbot_depth_worker (container)
    -> refined depth / mask / points
    -> fv_lingbot_depth publishes ROS topics
```

## 使い方

1. Worker コンテナを起動する
2. `fv_lingbot_depth` を `backend=http` で起動する

例:

```yaml
fv_lingbot_depth_d405:
  ros__parameters:
    backend: "http"
    worker_endpoint: "http://127.0.0.1:5540/infer"
    fallback_passthrough: true
```

## ビルド

簡単に試すなら:

```bash
cd docker/lingbot_depth_worker
./build.sh
```

`build.sh` は `x86_64` では `cu128` index を既定にし、`arm64` では PyPI の CPU wheel を既定にします。
Jetson GPU を使う場合は `TORCH_PACKAGES` を JetPack 対応 wheel に上書きしてください。

AMD64 の例:

```bash
docker build \
  -f docker/lingbot_depth_worker/Dockerfile \
  -t lingbot-depth-worker:amd64 \
  --build-arg TORCH_INDEX_URL=https://download.pytorch.org/whl/cu128 \
  --build-arg TORCH_PACKAGES="torch==2.6.0 torchvision" \
  /path/to/fluent_vision_ros2
```

Jetson / ARM64 の例:

```bash
docker build \
  -f docker/lingbot_depth_worker/Dockerfile \
  -t lingbot-depth-worker:jetson \
  --build-arg BASE_IMAGE=python:3.10-slim-bookworm \
  --build-arg TORCH_PACKAGES="/wheels/torch.whl /wheels/torchvision.whl" \
  /path/to/fluent_vision_ros2
```

Jetson は PyTorch の入れ方が JetPack 依存なので、`TORCH_PACKAGES` か `BASE_IMAGE` を実機に合わせて調整してください。
`lingbot-depth` 本体は `--no-deps` で入れているため、Torch は upstream 固定ではなく build arg 側で差し替えられます。

## 実行

簡単に試すなら:

```bash
cd docker/lingbot_depth_worker
./run.sh
```

CPU-only で起動確認する場合:

```bash
cd docker/lingbot_depth_worker
ENABLE_GPU=0 LOCAL_MODEL_PATH=/tmp/not-found ./run.sh
```

```bash
docker run --rm --net=host --gpus all \
  -e MODEL_ID=robbyant/lingbot-depth-pretrain-vitl-14-v0.5 \
  -e DEVICE=cuda \
  -e LISTEN_PORT=5540 \
  lingbot-depth-worker:amd64
```

## ヘルスチェック

```bash
curl http://127.0.0.1:5540/healthz
```

## `/infer` の smoke test

```bash
cd docker/lingbot_depth_worker
python3 smoke_test.py
```

## 注意

- worker は単体で起動できますが、初回は Hugging Face からモデルを取得するためネットワークが必要です
- `FALLBACK_PASSTHROUGH=1` なら、モデルロードに失敗してもヘルスチェックは通ります
- 本番では `MODEL_ID=robbyant/lingbot-depth-pretrain-vitl-14-v0.5` を推奨します
