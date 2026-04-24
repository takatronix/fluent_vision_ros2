# OpenPI Runtime

`openpi` の `pi0` / `pi05` を ROS 2 から分離して動かすための専用 runtime です。

## 目的

- `FluentVision` / `ROS 2` 本体から `openpi` の重い依存を分離する
- `pi0` と `pi05` を将来的に同一 runtime 内で切り替えられる土台を作る
- Intel / ARM で base image と JAX/Torch の差分を build 側に閉じ込める

## 構成

```text
fv_* bridge node
    -> HTTP /infer
    -> openpi-runtime container
    -> pi0 / pi05 policy worker
```

## ビルド

```bash
cd docker/openpi_runtime
./build.sh
```

既定値:

- `x86_64`: `python:3.11-slim-bookworm`, `torch==2.7.1`, `jax[cuda12]==0.5.3`
- `aarch64`: `nvcr.io/nvidia/pytorch:26.03-py3`, `jax==0.5.3`, Torch は base image 任せ

Jetson / ARM64 は upstream の `openpi` が `jax[cuda12]` と `torch==2.7.1` を強く前提にしているため、
そのままだと入りにくいです。`build.sh` はそこだけ緩めています。

## サーバ起動

```bash
cd docker/openpi_runtime
./run.sh
```

既定では `HTTP worker` を `8000` 番で起動します。

単一 policy を checkpoint 指定で起動する例:

```bash
OPENPI_POLICY_MODE=checkpoint \
OPENPI_POLICY_CONFIG=pi05_droid \
OPENPI_POLICY_DIR=gs://openpi-assets/checkpoints/pi05_droid \
./run.sh
```

複数 policy を起動時にロードする例:

```bash
OPENPI_POLICY_SPECS='pi0:pi0_aloha_sim:gs://openpi-assets/checkpoints/pi0_aloha_sim;pi05:pi05_droid:gs://openpi-assets/checkpoints/pi05_droid' \
./run.sh
```

この場合、`/infer` には `policy_id` を含めて送ります。

## Smoke Test

単体推論確認:

```bash
docker run --rm --net=host openpi-runtime:latest \
  uv run python /opt/openpi-runtime/smoke_test.py --config pi0_aloha_sim
```

`pi05` で試す場合:

```bash
docker run --rm --net=host openpi-runtime:latest \
  uv run python /opt/openpi-runtime/smoke_test.py --config pi05_droid
```

## 注意

- 初回 smoke test は checkpoint を取得するためネットワークが必要です
- upstream `openpi` は Ubuntu 22.04 / NVIDIA GPU 前提で、ARM64 は未保証です
- そのため `aarch64` では build/test がそのまま通らない可能性があります
- `fv_policy_runner` から使う場合は `http://<host>:8000/infer` を backend に指定します
