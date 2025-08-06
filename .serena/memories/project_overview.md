# FluentVision ROS2 Project Overview

## Purpose
FluentVision ROS2 is a high-performance, lightweight vision system based on ROS2. It provides C++ implementations for fast image processing and real-time streaming capabilities.

## Key Features
- **High-speed processing**: All packages implemented in C++ for high performance
- **Multiple camera support**: RealSense depth cameras, USB cameras, network cameras
- **Real-time streaming**: RTMP, MJPEG, WebSocket, HTTP streaming
- **Modular design**: Independent packages for flexible configuration
- **Monitoring & management**: System monitoring and metrics collection

## Architecture
The project is organized into categories:
- **Sensors** (`src/sensors/`): Camera interfaces and drivers
  - fv_realsense: Intel RealSense depth camera support
  - sipeed_tof_ms_a010: TOF camera support
- **AI/ML** (`src/ai/`): AI processing modules
  - fv_face_recognizer: Face recognition system
- **Streaming** (`src/streaming/`): Various streaming servers
  - fv_mjpeg_server: MJPEG streaming
  - fv_websocket_server: WebSocket streaming
  - fv_image_distributor: HTTP distribution
  - fv_rtmp_server: RTMP streaming
  - fv_recorder: Recording functionality
- **Control** (`src/control/`): Robot control nodes (future)
- **Utils** (`src/utils/`): Common libraries and helpers

## Main Technologies
- **OS**: Ubuntu 22.04 (Jammy Jellyfish)
- **ROS2**: Humble Hawksbill
- **Language**: C++17
- **Build System**: CMake with ament_cmake
- **Package Format**: ROS2 packages with colcon build system

## Project Maintainer
- Takashi Otsuka (takatronix@gmail.com)