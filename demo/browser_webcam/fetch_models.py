#!/usr/bin/env python3
"""Fetch and convert the demo models at image-build time.

- yolov10n (COCO 80 classes)  -> OpenVINO IR for fv_object_detector
- yoloe-11s-seg               -> cached ultralytics weights for fv_yoloe

Usage: fetch_models.py <output_dir>
"""
import shutil
import sys
from pathlib import Path

from ultralytics import YOLO


def main() -> None:
    out = Path(sys.argv[1] if len(sys.argv) > 1 else 'models')
    (out / 'openvino').mkdir(parents=True, exist_ok=True)

    # --- COCO detector: yolov10n -> OpenVINO IR ---
    model = YOLO('yolov10n.pt')  # downloads on first use
    exported = Path(model.export(format='openvino', imgsz=640, half=False))
    # export returns .../yolov10n_openvino_model containing .xml/.bin
    for suffix in ('.xml', '.bin'):
        src = next(exported.glob(f'*{suffix}'))
        shutil.copy(src, out / 'openvino' / f'yolov10n_coco{suffix}')
    print(f'COCO model ready: {out}/openvino/yolov10n_coco.xml')

    # --- YOLOE weights: download into the ultralytics cache ---
    YOLO('yoloe-11s-seg.pt')
    print('YOLOE weights cached')


if __name__ == '__main__':
    main()
