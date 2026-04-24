ARG BASE_IMAGE=python:3.11-slim-bookworm
FROM ${BASE_IMAGE}

ARG OPENPI_REPO=https://github.com/Physical-Intelligence/openpi.git
ARG OPENPI_REF=main
ARG OPENPI_INSTALL_PROFILE=default
ARG JAX_SPEC=jax[cuda12]==0.5.3
ARG TORCH_SPEC=torch==2.7.1
ARG JETSON_JAX_VERSION=0.10.0
ARG JETSON_JAX_INDEX_URL=https://pypi.jetson-ai-lab.io/sbsa/cu130

ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_NO_CACHE_DIR=1
ENV PYTHONUNBUFFERED=1
ENV UV_CACHE_DIR=/root/.cache/uv
ENV XLA_PYTHON_CLIENT_PREALLOCATE=false
ENV PATH=/opt/openpi/.venv/bin:${PATH}

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    build-essential \
    curl \
    ffmpeg \
    git \
    libgl1 \
    libgl1-mesa-dev \
    libglib2.0-0 \
    libglew-dev \
    libgomp1 \
    libosmesa6-dev \
    python3-dev \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

RUN python -m pip install --upgrade --ignore-installed pip setuptools wheel uv

WORKDIR /opt
RUN git clone --depth 1 --branch "${OPENPI_REF}" "${OPENPI_REPO}" openpi

WORKDIR /opt/openpi
RUN OPENPI_INSTALL_PROFILE="${OPENPI_INSTALL_PROFILE}" JAX_SPEC="${JAX_SPEC}" TORCH_SPEC="${TORCH_SPEC}" python - <<'PY'
import os
from pathlib import Path

path = Path("/opt/openpi/pyproject.toml")
text = path.read_text()
profile = os.environ["OPENPI_INSTALL_PROFILE"]

if profile == "jetson-thor":
    for dep in [
        "torch",
        "opencv-python",
        "transformers",
        "lerobot",
        "jax",
        "flax",
        "jaxtyping",
        "orbax-checkpoint",
        "equinox",
        "gym-aloha",
    ]:
        lines = text.splitlines(keepends=True)
        lines = [line for line in lines if f'"{dep}' not in line]
        text = "".join(lines)
else:
    text = text.replace('"jax[cuda12]==0.5.3"', f'"{os.environ["JAX_SPEC"]}"')
    torch_spec = os.environ["TORCH_SPEC"]
    if torch_spec == "__REMOVE_TORCH__":
        text = text.replace('    "torch==2.7.1",\n', "")
    else:
        text = text.replace('"torch==2.7.1"', f'"{torch_spec}"')
path.write_text(text)
PY

RUN cd /opt/openpi && uv venv --python /usr/bin/python3 --seed --system-site-packages

RUN if [ "${OPENPI_INSTALL_PROFILE}" = "jetson-thor" ]; then \
      cd /opt/openpi && \
      uv pip install --no-cache-dir --extra-index-url https://pypi.org/simple --index-url https://pypi.org/simple 'av>=12.0.5,<13.0.0' && \
      uv pip install 'transformers==4.53.2' && \
      uv pip install --no-deps 'flax==0.12.6' 'equinox>=0.11.8' 'orbax-checkpoint==0.11.20' 'jaxtyping==0.2.38' && \
      uv pip install --no-deps 'gym-aloha>=0.1.1' && \
      uv pip install --no-deps 'lerobot @ git+https://github.com/huggingface/lerobot@0cf864870cf29f4738d3ade893e6fd13fbd7cdb5' && \
      GIT_LFS_SKIP_SMUDGE=1 uv pip install -e packages/openpi-client && \
      GIT_LFS_SKIP_SMUDGE=1 uv pip install -e . && \
      uv pip install --index-url "${JETSON_JAX_INDEX_URL}" "jaxlib==${JETSON_JAX_VERSION}" jax_cuda13_plugin jax_cuda13_pjrt opt_einsum && \
      uv pip install --index-url https://pypi.org/simple --no-deps "jax==${JETSON_JAX_VERSION}" && \
      /opt/openpi/.venv/bin/pip install --index-url https://pypi.org/simple --upgrade 'ml-dtypes==0.5.3' && \
      uv pip install --index-url https://pypi.org/simple --no-deps \
        optax importlib_resources humanize tensorstore simplejson datasets pandas pyarrow pytz \
        multiprocess metadata dill xxhash jsonlines draccus mergedeep typing_inspect \
        mypy_extensions chex etils toolz nest_asyncio wadler-lindig safetensors aiofiles; \
    else \
      GIT_LFS_SKIP_SMUDGE=1 uv sync && \
      GIT_LFS_SKIP_SMUDGE=1 uv pip install -e .; \
    fi
RUN /opt/openpi/.venv/bin/python - <<'PY'
import importlib.util
import pathlib
import shutil

if importlib.util.find_spec("transformers") is None:
    print("transformers not installed; skipping patch")
    raise SystemExit(0)

import transformers

src = pathlib.Path("/opt/openpi/src/openpi/models_pytorch/transformers_replace")
dst = pathlib.Path(transformers.__file__).resolve().parent
for item in src.iterdir():
    target = dst / item.name
    if item.is_dir():
        shutil.copytree(item, target, dirs_exist_ok=True)
    else:
        shutil.copy2(item, target)
print(f"patched transformers into {dst}")
PY

RUN if [ "${OPENPI_INSTALL_PROFILE}" = "jetson-thor" ]; then \
      cd /opt/openpi && \
      /opt/openpi/.venv/bin/python -c 'import jax; print(f"JAX version: {jax.__version__}")' \
    ; fi

RUN /opt/openpi/.venv/bin/pip install gsutil

WORKDIR /opt/openpi-runtime
COPY docker/openpi_runtime/entrypoint.sh /opt/openpi-runtime/entrypoint.sh
COPY docker/openpi_runtime/healthcheck.py /opt/openpi-runtime/healthcheck.py
COPY docker/openpi_runtime/server.py /opt/openpi-runtime/server.py
COPY docker/openpi_runtime/smoke_test.py /opt/openpi-runtime/smoke_test.py

RUN chmod +x /opt/openpi-runtime/entrypoint.sh

ENV OPENPI_ENV=aloha_sim
ENV OPENPI_PORT=8000
ENV OPENPI_SERVER_MODE=http
ENV OPENPI_POLICY_MODE=default
ENV OPENPI_POLICY_CONFIG=pi0_aloha_sim
ENV OPENPI_POLICY_DIR=gs://openpi-assets/checkpoints/pi0_aloha_sim
ENV OPENPI_DEFAULT_PROMPT=
ENV XLA_FLAGS=--xla_gpu_autotune_level=0

EXPOSE 8000

HEALTHCHECK --interval=30s --timeout=5s --start-period=30s --retries=3 \
  CMD python /opt/openpi-runtime/healthcheck.py

ENTRYPOINT ["/opt/openpi-runtime/entrypoint.sh"]
CMD []
