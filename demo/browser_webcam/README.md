# 🌿 Browser Webcam Live Demo / ブラウザカメラ・ライブデモ

Visitors point their own webcam at the world and watch a **real ROS 2
perception pipeline** annotate it live — COCO detection (smooth) or YOLOE
open-vocabulary detection with a text prompt (type "red pen", it finds it).
Model switching is instant: both detectors stay resident and the bridge
routes frames to only the selected one.

訪問者が自分のWebカメラを向けると、**本物のROS 2認識パイプライン**がライブで
アノテーションを返します。COCO検出（滑らか）と、テキストプロンプト式の
YOLOEオープン語彙検出（"red pen"と打つと見つける）をページ上で即時切替。
両検出器は常駐し、ブリッジが選択中のモデルにだけフレームを流します。

```
browser --JPEG/WS--> fv_browser_camera --> fv_object_detector (COCO / OpenVINO CPU)
                                       \-> fv_yoloe (open-vocab / CPU)
        <--overlay JPEG/WS--
```

Everything is served on **one port** (`$PORT`, default 7860) — the constraint
of free-tier hosts. Frames are processed in memory only, never stored.

## Local build & run / ローカルでの確認

```bash
# from the repository root — x86_64 host
docker build -f demo/browser_webcam/Dockerfile -t fv-demo .
docker run --rm -p 7860:7860 fv-demo
# → open http://localhost:7860
```

> Note: `getUserMedia` requires HTTPS except on `localhost`. Hosted platforms
> (HF Spaces etc.) provide HTTPS automatically.
> 注: カメラ許可(`getUserMedia`)は`localhost`以外ではHTTPS必須です。
> HF Spaces等のホスティングは自動でHTTPSになります。

## Deploy to Hugging Face Spaces / HF Spacesへのデプロイ

1. Create a new Space → SDK: **Docker** (Blank template), CPU basic.
2. Push a repo with this layout (or mirror this repository and set the
   Space's Dockerfile path to `demo/browser_webcam/Dockerfile`):
   - the `src/` packages referenced by the Dockerfile
   - `demo/browser_webcam/` (Dockerfile, start.sh, fetch_models.py, config/)
3. The Space README needs this metadata header:

```yaml
---
title: FluentVision Live Demo
emoji: 🌿
colorFrom: green
colorTo: gray
sdk: docker
app_port: 7860
---
```

Free-tier notes / 無料枠の注意:
- The Space sleeps after ~48h of inactivity and wakes on the next visit
  (cold start = container boot, well under a minute; models are baked at
  build time so no download happens at wake).
- One container serves all visitors: the first WebSocket connection drives
  the camera, later ones watch. The page explains this.
- CPU inference: COCO lane targets ~10 fps, YOLOE lane ~1–3 fps.

## Files

| File | Role |
|---|---|
| `Dockerfile` | ros:humble-ros-base + OpenVINO(pip) + torch-cpu/ultralytics; builds the 4 packages; bakes models |
| `fetch_models.py` | yolov10n → OpenVINO IR (COCO), yoloe-11s-seg weight cache |
| `config/detector_coco.yaml` | detector params incl. the 80 COCO class names |
| `start.sh` | entrypoint: detector + YOLOE + bridge (bridge owns `$PORT`) |

The bridge node itself lives in the main tree:
[`src/sensors/fv_browser_camera/`](../../src/sensors/fv_browser_camera/).
