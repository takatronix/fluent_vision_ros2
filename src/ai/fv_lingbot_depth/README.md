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

RGB/depth/camera-info subscriptions are created only while a refined output
has a subscriber and the optional depth-source selector permits inference.
The 250 ms demand check removes subscriptions and clears synchronized frames
when demand disappears, so pausing also stops unused camera DDS traffic and
deserialization. Active downstream perception consumers retain the same inputs
and inference quality. The resident model stays loaded.

`~/capture` creates temporary input demand even without output subscribers,
including when the selector gate is disabled. Inputs are released after capture
or timeout; a timeout never reports raw depth as a successful refined capture.

The synthetic ROS lifecycle test runs in isolated domain 136:

```bash
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 -m pytest -q test/test_input_demand.py
```

## Dependencies

ROS:
- `rclpy`, `message_filters`, `cv_bridge`, `sensor_msgs`, `std_msgs`

Python runtime:
- `torch`, `torchvision`, `xformers`
- `opencv-python`, `numpy`, `huggingface_hub`
- `mdm` package from LingBot-Depth repository

Containerized worker mode:
- Set `backend=http` on the ROS node to keep LingBot dependencies out of the main FluentVision environment.
- The node will call an external worker at `worker_endpoint` and still publish the same ROS topics.
- Worker scaffolding lives under `docker/lingbot_depth_worker/`.

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

HTTP worker example:

```yaml
fv_lingbot_depth:
  ros__parameters:
    backend: "http"
    worker_endpoint: "http://127.0.0.1:5540/infer"
    fallback_passthrough: true
```
