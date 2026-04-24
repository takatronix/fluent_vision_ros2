ARG BASE_IMAGE=nvcr.io/nvidia/pytorch:26.03-py3
FROM openpi-runtime:latest AS openpi_src
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_NO_CACHE_DIR=1
ENV PYTHONUNBUFFERED=1
ENV UV_CACHE_DIR=/root/.cache/uv
ENV UV_SYSTEM_PYTHON=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    ffmpeg \
    git \
    libgl1 \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

RUN python -m pip install --upgrade --ignore-installed pip setuptools wheel uv

COPY daihen-physical-ai/features/lerobot /opt/lerobot
COPY daihen-physical-ai/features/pyproject.toml /opt/percus_ai/pyproject.toml
COPY daihen-physical-ai/features/percus_ai /opt/percus_ai/percus_ai

RUN python - <<'PY'
from pathlib import Path

init_path = Path("/opt/lerobot/src/lerobot/policies/__init__.py")
text = init_path.read_text()
text = text.replace("from .groot.configuration_groot import GrootConfig as GrootConfig\n", "")
text = text.replace('    "GrootConfig",\n', "")
init_path.write_text(text)

factory_path = Path("/opt/lerobot/src/lerobot/policies/factory.py")
text = factory_path.read_text()
text = text.replace("from lerobot.policies.groot.configuration_groot import GrootConfig\n", "class GrootConfig:  # FluentVision DPEX runtime disables Groot imports.\n    pass\n")
factory_path.write_text(text)

robots_init_path = Path("/opt/lerobot/src/lerobot/robots/__init__.py")
text = robots_init_path.read_text()
text = text.replace("from . import mujoco_sim\n", "")
robots_init_path.write_text(text)
PY

RUN --mount=type=cache,target=/root/.cache/uv \
    uv pip install --system --break-system-packages \
    "numpy<2" \
    "opencv-python-headless>=4.9" \
    "pillow>=10" \
    "pyyaml>=6" \
    "safetensors>=0.4" \
    "sentencepiece>=0.2" \
    "transformers==4.53.2" \
    "accelerate>=1.0" \
    "av>=15.0" \
    "datasets==4.1.1" \
    "draccus==0.10.0" \
    "gymnasium>=1.2" \
    "jsonlines>=4.0" \
    "imageio>=2.37" \
    "mergedeep>=1.3" \
    "pyserial>=3.5" \
    "typing-inspect>=0.9" \
    "deepdiff>=8.6" \
    "termcolor>=3.0" \
    "msgpack>=1" \
    "pyzmq>=25"

COPY --from=openpi_src /opt/openpi/src/openpi/models_pytorch/transformers_replace /tmp/transformers_replace
RUN python - <<'PY'
import pathlib
import shutil

import transformers

src = pathlib.Path("/tmp/transformers_replace")
dst = pathlib.Path(transformers.__file__).resolve().parent
for item in src.iterdir():
    target = dst / item.name
    if item.is_dir():
        shutil.copytree(item, target, dirs_exist_ok=True)
    else:
        shutil.copy2(item, target)
print(f"patched transformers into {dst}")
PY

RUN --mount=type=cache,target=/root/.cache/uv \
    uv pip install --system --break-system-packages --no-deps -e /opt/percus_ai

RUN --mount=type=cache,target=/root/.cache/uv \
    uv pip install --system --break-system-packages --no-deps -e /opt/lerobot

WORKDIR /opt/fv-lerobot-policy-runtime
COPY ros2_ws/src/fluent_vision_ros2/docker/fv_lerobot_policy_runtime/server.py /opt/fv-lerobot-policy-runtime/server.py
COPY ros2_ws/src/fluent_vision_ros2/docker/fv_lerobot_policy_runtime/healthcheck.py /opt/fv-lerobot-policy-runtime/healthcheck.py
COPY ros2_ws/src/fluent_vision_ros2/docker/fv_lerobot_policy_runtime/entrypoint.sh /opt/fv-lerobot-policy-runtime/entrypoint.sh
COPY ros2_ws/src/fluent_vision_ros2/docker/fv_lerobot_policy_runtime/policy_models.yaml /opt/fv-lerobot-policy-runtime/policy_models.yaml

RUN chmod +x /opt/fv-lerobot-policy-runtime/entrypoint.sh

ENV PYTHONPATH=/opt/percus_ai:/opt/lerobot/src
ENV DPEX_PORT=8010
ENV DPEX_LOAD_ON_START=true
ENV DPEX_DEFAULT_POLICY_ID=pi05_week19_morikawa_all_20k
ENV DPEX_POLICY_SPECS=pi05_week19_morikawa_all_20k:pi05:/data/models/669ce87f-8cfa-5836-8f29-b9c3f6e650cf;pi05_week19_morikawa_all_10k:pi05:/data/models/1dc2af59-2d87-52f9-99af-fae7c6335bcf
ENV DPEX_CAMERA_ALIASES=cam_high=top_camera,top=top_camera,top_camera=top_camera,cam_left_wrist=arm_camera,cam_right_wrist=arm_camera,arm=arm_camera,arm_camera=arm_camera
ENV HF_HOME=/data/.cache/huggingface
ENV TRANSFORMERS_CACHE=/data/.cache/huggingface
ENV TORCH_HOME=/data/.cache/torch

EXPOSE 8010

HEALTHCHECK --interval=30s --timeout=5s --start-period=120s --retries=3 \
  CMD python /opt/fv-lerobot-policy-runtime/healthcheck.py

ENTRYPOINT ["/opt/fv-lerobot-policy-runtime/entrypoint.sh"]
