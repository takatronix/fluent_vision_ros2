ARG BASE_IMAGE=python:3.10-slim-bookworm
FROM ${BASE_IMAGE}

ARG TORCH_INDEX_URL=
ARG TORCH_PACKAGES="torch torchvision"
ARG EXTRA_PIP_PACKAGES="click opencv-python-headless scipy trimesh pillow matplotlib requests"
ARG LINGBOT_DEPTH_REPO=https://github.com/Robbyant/lingbot-depth.git
ARG LINGBOT_DEPTH_REF=main
ARG INSTALL_XFORMERS=0
ARG XFORMERS_PACKAGES="xformers==0.0.29.post2"

ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_NO_CACHE_DIR=1
ENV PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    git \
    libglib2.0-0 \
    libgomp1 \
    && rm -rf /var/lib/apt/lists/*

RUN python -m pip install --upgrade --ignore-installed pip setuptools wheel && \
    python -m pip install "numpy<2" huggingface_hub ${EXTRA_PIP_PACKAGES}

RUN if [ -z "${TORCH_PACKAGES}" ]; then \
      python -c "import torch; print(torch.__version__); print(torch.cuda.is_available())"; \
    elif [ -n "${TORCH_INDEX_URL}" ]; then \
      python -m pip install --index-url "${TORCH_INDEX_URL}" ${TORCH_PACKAGES}; \
    else \
      python -m pip install ${TORCH_PACKAGES}; \
    fi

RUN if [ "${INSTALL_XFORMERS}" = "1" ]; then \
      python -m pip install ${XFORMERS_PACKAGES}; \
    fi

WORKDIR /opt
RUN git clone --depth 1 --branch "${LINGBOT_DEPTH_REF}" "${LINGBOT_DEPTH_REPO}" lingbot-depth
RUN cd /opt/lingbot-depth && python -m pip install --no-deps -e .

WORKDIR /opt/lingbot-depth-worker
COPY docker/lingbot_depth_worker/server.py /opt/lingbot-depth-worker/server.py

ENV MODEL_ID=robbyant/lingbot-depth-pretrain-vitl-14-v0.5
ENV LOCAL_MODEL_PATH=
ENV DEVICE=auto
ENV USE_FP16=1
ENV APPLY_MASK=1
ENV RESOLUTION_LEVEL=9
ENV LISTEN_HOST=0.0.0.0
ENV LISTEN_PORT=5540
ENV FALLBACK_PASSTHROUGH=1

EXPOSE 5540

HEALTHCHECK --interval=30s --timeout=5s --start-period=30s --retries=3 \
  CMD python -c "import os, urllib.request; port=os.environ.get('LISTEN_PORT', '5540'); resp=urllib.request.urlopen(f'http://127.0.0.1:{port}/healthz', timeout=3); body=resp.read().decode('utf-8'); raise SystemExit(0 if resp.status == 200 and body else 1)"

CMD ["python", "/opt/lingbot-depth-worker/server.py"]
