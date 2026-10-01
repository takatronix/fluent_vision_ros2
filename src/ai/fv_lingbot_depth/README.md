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

### Selector-aware input demand and one-shot capture

A downstream reader starts continuous camera input only while this node is the
selected depth source. Selecting raw depth releases the image subscriptions even
if a reader remains on the refined output. An explicit `~/capture` still wakes
input and processes one frame when no output is subscribed, including when the
selector is already set to refined. Capture waits for the demand timer to update
subscriptions in the image callback group, avoiding cross-thread ROS entity
changes. Subscriber callbacks and synchronizer queues are discarded on pause.

`test/test_input_demand.py` checks pause/resume, selector changes, capture, and
timeout cleanup with tiny synthetic frames, a stubbed model, and ROS domain 136.
It does not contact the robot or load a neural model.
