# FluentVision browser-webcam live demo — single-port container for free-tier
# hosts (Hugging Face Spaces Docker SDK, Railway, etc.). Target: x86_64.
#
#   visitor browser --JPEG/WS--> fv_browser_camera --> fv_object_detector (COCO, OpenVINO)
#                                                  \-> fv_yoloe (open-vocabulary, CPU)
#                    <--overlay JPEG/WS--
#
# Build:  docker build -f demo/browser_webcam/Dockerfile -t fv-demo .
# Run:    docker run --rm -p 7860:7860 fv-demo
# (build context = repository root)

FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-lc"]

# ---- system deps -----------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip python3-colcon-common-extensions \
        ros-humble-cv-bridge ros-humble-image-transport \
        ros-humble-vision-msgs libopencv-dev \
        build-essential cmake git curl ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# ---- python deps -----------------------------------------------------------
# CPU-only torch first so ultralytics doesn't pull the CUDA build (~2GB smaller).
RUN pip3 install --no-cache-dir torch torchvision --index-url https://download.pytorch.org/whl/cpu \
 && pip3 install --no-cache-dir ultralytics openvino aiohttp numpy

# The openvino pip wheel ships the C++ runtime + CMake config; point colcon at it
# so fv_object_detector's find_package(OpenVINO) succeeds without the apt repo.
RUN echo "export OpenVINO_DIR=$(python3 -c 'import openvino, pathlib; print(pathlib.Path(openvino.__file__).parent / \"cmake\")')" >> /etc/profile.d/openvino.sh

# ---- workspace -------------------------------------------------------------
WORKDIR /app
COPY src/common/fv_msgs            src/fv_msgs
COPY src/ai/fv_object_detector     src/fv_object_detector
COPY src/ai/fv_yoloe               src/fv_yoloe
COPY src/sensors/fv_browser_camera src/fv_browser_camera

RUN source /opt/ros/humble/setup.bash \
 && source /etc/profile.d/openvino.sh \
 && colcon build --packages-select fv_msgs fv_object_detector fv_yoloe fv_browser_camera \
 && test -f install/fv_object_detector/lib/fv_object_detector/fv_object_detector_node \
    || (echo 'fv_object_detector was skipped — OpenVINO not found' && exit 1)

# ---- models (baked at build so first visitor never waits) ------------------
COPY demo/browser_webcam/fetch_models.py demo/
RUN python3 demo/fetch_models.py /app/models

COPY demo/browser_webcam/config  demo/config
COPY demo/browser_webcam/start.sh demo/start.sh
RUN chmod +x demo/start.sh

EXPOSE 7860
ENV PORT=7860
CMD ["/app/demo/start.sh"]
