# fv_visual_condition_detector

ROS2 node for visual condition detection from raw or compressed image topics.
The ASPA default consumes `/aspa/restamped/color_compressed` and loads
`~/.aspa/models/visual-condition-multihead.onnx`. The model is not committed to
this repository. Install `onnxruntime` or `onnxruntime-gpu` in the runtime
environment; startup fails when the model or runtime cannot be loaded.

```bash
ros2 launch fv_visual_condition_detector visual_condition_detector.launch.py \
  config:=/path/to/default.yaml
```

Outputs:

- `/visual_condition/status`: per-frame `worsen_score`, VAD-smoothed `bad`, `condition_top`, and `domain_top`
- `/visual_condition/events`: `start`/`stop` events after `min_on_sec`/`min_off_sec`
