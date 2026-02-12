# fv_lingbot_depth

ROS2 node integrating [LingBot-Depth](https://github.com/Robbyant/lingbot-depth) into FluentVision.

## Features
- Synchronize `RGB + Depth + CameraInfo`
- Run LingBot-Depth inference
- Publish:
  - refined depth (`32FC1`)
  - validity mask (`mono8`)
  - point cloud (`PointCloud2`, XYZ)
- Optional passthrough fallback when model is unavailable

## Dependencies

ROS:
- `rclpy`, `message_filters`, `cv_bridge`, `sensor_msgs`, `std_msgs`

Python runtime:
- `torch`, `torchvision`, `xformers`
- `opencv-python`, `numpy`, `huggingface_hub`
- `mdm` package from LingBot-Depth repository

Example (inside your Python env):

```bash
pip install torch==2.6.0 torchvision xformers==0.0.29.post2 opencv-python numpy huggingface_hub
git clone https://github.com/Robbyant/lingbot-depth
cd lingbot-depth
pip install -e .
```

## Launch

```bash
ros2 launch fv_lingbot_depth fv_lingbot_depth.launch.py
```

Override config:

```bash
ros2 launch fv_lingbot_depth fv_lingbot_depth.launch.py \
  config_file:=/path/to/custom.yaml
```

