ARG BASE_IMAGE=ubuntu:24.04
FROM ${BASE_IMAGE}

ARG TARGETARCH
ARG ROS_DISTRO=jazzy
ARG ISAAC_ROS_RELEASE=4.1
ARG L4T_SUITE=r38.2
ARG INSTALL_EXAMPLES=1

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=${ROS_DISTRO}
ENV ISAAC_ROS_WS=/workspaces/isaac_ros-dev
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV PATH=/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/src/tensorrt/bin:${PATH}

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    git \
    git-lfs \
    gnupg \
    jq \
    locales \
    lsb-release \
    software-properties-common \
    tar \
    wget \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

RUN add-apt-repository universe

RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
      > /etc/apt/sources.list.d/ros2.list

RUN if [ "${TARGETARCH}" = "amd64" ]; then \
      wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb -O /tmp/cuda-keyring.deb; \
    elif [ "${TARGETARCH}" = "arm64" ]; then \
      wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/sbsa/cuda-keyring_1.1-1_all.deb -O /tmp/cuda-keyring.deb; \
    else \
      echo "Unsupported TARGETARCH=${TARGETARCH}" >&2; exit 1; \
    fi && \
    dpkg -i /tmp/cuda-keyring.deb && \
    rm -f /tmp/cuda-keyring.deb

RUN curl -fsSL https://repo.download.nvidia.com/jetson/jetson-ota-public.asc \
      | gpg --dearmor -o /usr/share/keyrings/nvidia-jetson.gpg && \
    if [ "${TARGETARCH}" = "amd64" ]; then \
      echo "deb [signed-by=/usr/share/keyrings/nvidia-jetson.gpg] https://repo.download.nvidia.com/jetson/x86_64/noble ${L4T_SUITE} main" \
        > /etc/apt/sources.list.d/nvidia-jetson.list; \
    elif [ "${TARGETARCH}" = "arm64" ]; then \
      printf '%s\n%s\n' \
        "deb [signed-by=/usr/share/keyrings/nvidia-jetson.gpg] https://repo.download.nvidia.com/jetson/common ${L4T_SUITE} main" \
        "deb [signed-by=/usr/share/keyrings/nvidia-jetson.gpg] https://repo.download.nvidia.com/jetson/som ${L4T_SUITE} main" \
        > /etc/apt/sources.list.d/nvidia-jetson.list; \
    else \
      echo "Unsupported TARGETARCH=${TARGETARCH}" >&2; exit 1; \
    fi

RUN curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key \
      | gpg --dearmor -o /usr/share/keyrings/nvidia-isaac-ros.gpg && \
    if [ "${TARGETARCH}" = "amd64" ]; then \
      echo "deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-${ISAAC_ROS_RELEASE} noble main" \
        > /etc/apt/sources.list.d/nvidia-isaac-ros.list; \
    elif [ "${TARGETARCH}" = "arm64" ]; then \
      echo "deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-${ISAAC_ROS_RELEASE} noble-jetpack main" \
        > /etc/apt/sources.list.d/nvidia-isaac-ros.list; \
    else \
      echo "Unsupported TARGETARCH=${TARGETARCH}" >&2; exit 1; \
    fi

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-vision-msgs-rviz-plugins \
    ros-${ROS_DISTRO}-isaac-ros-foundationpose \
    ros-${ROS_DISTRO}-isaac-ros-realsense \
    libnvinfer-bin \
    libnvinfer-dev \
    python3-libnvinfer \
    && if [ "${INSTALL_EXAMPLES}" = "1" ]; then \
         apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-isaac-ros-examples; \
       fi \
    && rm -rf /var/lib/apt/lists/*

RUN git lfs install --system && \
    mkdir -p ${ISAAC_ROS_WS}/src ${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose

WORKDIR /opt/foundationpose-runtime
COPY docker/foundationpose_runtime/entrypoint.sh /opt/foundationpose-runtime/entrypoint.sh
COPY docker/foundationpose_runtime/build_engines.sh /opt/foundationpose-runtime/build_engines.sh
COPY docker/foundationpose_runtime/run_foundationpose_example.sh /opt/foundationpose-runtime/run_foundationpose_example.sh

RUN chmod +x /opt/foundationpose-runtime/entrypoint.sh /opt/foundationpose-runtime/build_engines.sh /opt/foundationpose-runtime/run_foundationpose_example.sh

WORKDIR ${ISAAC_ROS_WS}

ENTRYPOINT ["/opt/foundationpose-runtime/entrypoint.sh"]
CMD ["bash"]
