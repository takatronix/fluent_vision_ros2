# fv_insta360_x3

Dedicated ROS2 UVC camera node for Insta360 X3 webcam mode.

The node prefers the stable udev symlink:

```bash
/dev/v4l/by-id/usb-Amba_Insta360_X3-video-index0
```

It publishes:

```text
/x3/image_raw
/x3/image_raw/compressed
/x3/camera_info
```

Run:

```bash
ros2 launch fv_insta360_x3 fv_insta360_x3.launch.py
```

Run with the software stitcher:

```bash
ros2 launch fv_insta360_x3 fv_insta360_x3_with_stitch.launch.py
```

Additional stitcher outputs:

```text
/x3/equirectangular/image_raw
/x3/equirectangular/image_raw/compressed
/x3/view/image_raw
/x3/view/image_raw/compressed
/x3/view/camera_info
```

The 360-degree view interface is ROS parameters on
`/fv_insta360_x3_stitcher`:

```bash
ros2 param set /fv_insta360_x3_stitcher view.yaw_deg 90.0
ros2 param set /fv_insta360_x3_stitcher view.pitch_deg -20.0
ros2 param set /fv_insta360_x3_stitcher view.fov_deg 110.0
```

Use the directed camera view for simulator overlays:

```text
/x3/view/image_raw/compressed
```

The Web UI has PiPER presets named `piper_vlabor_x3_overlay` for recording
and inference. They render the VLAbor scene viewer with the X3 camera overlay
large on top of it. In Unity, select the same compressed topic in Camera View
and enlarge the panel.

Core parameters:

```yaml
view:
  yaw_deg: 0.0
  pitch_deg: 0.0
  roll_deg: 0.0
  fov_deg: 90.0

calibration:
  lens_fov_deg: 190.0
  front:
    center_x: 0.5
    center_y: 0.25
    radius_x: 0.5
    radius_y: 0.25
    yaw_deg: 0.0
    roll_deg: 0.0
  rear:
    center_x: 0.5
    center_y: 0.75
    radius_x: 0.5
    radius_y: 0.25
    yaw_deg: 180.0
    roll_deg: 0.0
```

Useful overrides:

```bash
ros2 launch fv_insta360_x3 fv_insta360_x3.launch.py width:=1920 height:=1080 fps:=30
ros2 launch fv_insta360_x3 fv_insta360_x3.launch.py device_path:=/dev/video8
```

Notes:

- The X3 appears as a UVC MJPEG camera in webcam mode.
- `/dev/video9` is not used for frames on the tested device.
- `camera_info` is an approximate pinhole model unless calibrated intrinsics are provided through parameters.
- The stitcher is an approximate dual-fisheye projection. Tune center,
  radius, roll, yaw, and lens FOV before treating the seam geometry as calibrated.
