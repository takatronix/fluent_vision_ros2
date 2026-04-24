# FV LeRobot Policy Runtime

HTTP runtime for LeRobot policy checkpoints. It keeps named policies loaded in GPU memory and exposes the same `/infer` response shape used by `fv_policy_runner`.

Build from `/home/aspa` so the image can copy both DPEX and FluentVision sources:

```bash
/home/aspa/ros2_ws/src/fluent_vision_ros2/docker/fv_lerobot_policy_runtime/build.sh
```

Run:

```bash
/home/aspa/ros2_ws/src/fluent_vision_ros2/docker/fv_lerobot_policy_runtime/run.sh
curl http://127.0.0.1:8010/models
```

The model registry is `policy_models.yaml`. It maps display names such as `Pi05-Week19-Morikawa-All_20k` to their UUID-backed DPEX model directories without creating filesystem aliases. The default test runtime loads `20k` and `10k` into memory; switch with `policy_id`.
