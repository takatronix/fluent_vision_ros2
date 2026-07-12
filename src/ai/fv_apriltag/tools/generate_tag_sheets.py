"""Generate A4 multi-tag sheets: many tags per page with scissor cut
lines, at true scale (300 DPI).

Each cell is a light-gray CUT BOX of exactly (tag + 5 mm margin per
side) — the size the mount pockets expect — with the id/label caption
printed OUTSIDE the box in the gutter (readable before cutting,
discarded after). 50 mm tags tile 3×4 = 12 per sheet; 100 mm tags 1×2.

Run example:
    python3 generate_tag_sheets.py \
        --out-dir ~/ros2_ws/src/vlabor_ros2/docs/印刷物/field_tags/sheets
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import List, Tuple

from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parent))
from generate_cube_assets import fetch_tag_array, _font_for_box  # noqa: E402

DPI = 300.0
A4_MM = (210.0, 297.0)
# The tag36h11 artwork already CONTAINS its required white quiet ring
# (1 cell = tag_mm/10 per side), so the cut line sits ON the artwork
# edge: a "50 mm tag" cutout really measures 50 mm overall (black
# square = 40 mm). The white mounts' pocket rim supplies extra quiet
# zone; only when pasting directly onto a DARK surface leave extra
# white beyond the line. Mount pockets = tag_mm + 2*MARGIN_MM.
MARGIN_MM = 0.0
CUT_GRAY = 185         # light gray cut line — visible, not distracting
CUT_W_MM = 0.35


def _px(mm: float) -> int:
    return int(round(mm * DPI / 25.4))


def _paste_tag(canvas: Image.Image, draw: ImageDraw.ImageDraw,
               tag_id: int, tag_mm: float,
               x_mm: float, y_mm: float, label: str) -> None:
    paper = tag_mm + 2 * MARGIN_MM
    x0, y0 = _px(x_mm), _px(y_mm)
    x1, y1 = _px(x_mm + paper), _px(y_mm + paper)
    draw.rectangle([x0, y0, x1, y1], outline=CUT_GRAY,
                   width=max(1, _px(CUT_W_MM)))
    pattern = fetch_tag_array(tag_id)          # 10×10, 0=black
    tag_px = _px(tag_mm)
    tile = Image.fromarray((pattern * 255).astype('uint8'), 'L') \
        .resize((tag_px, tag_px), Image.NEAREST)
    canvas.paste(tile, (x0 + _px(MARGIN_MM), y0 + _px(MARGIN_MM)))
    caption = f'id{tag_id:03d} {label}'.strip()
    cap_h = _px(7.0)
    font = _font_for_box(caption, x1 - x0 - 8, cap_h - 4)
    bbox = font.getbbox(caption)
    tw = bbox[2] - bbox[0]
    draw.text(((x0 + x1 - tw) // 2 - bbox[0], y1 + _px(1.2) - bbox[1]),
              caption, fill=90, font=font)


def make_sheet(entries: List[Tuple[int, str]], tag_mm: float,
               out_stem: Path) -> None:
    """entries: [(tag_id, label), ...] — tiled row-major, A4 portrait."""
    w, h = _px(A4_MM[0]), _px(A4_MM[1])
    canvas = Image.new('L', (w, h), color=255)
    draw = ImageDraw.Draw(canvas)
    paper = tag_mm + 2 * MARGIN_MM
    if tag_mm <= 60:
        cols, gutter_x, top, pitch_y = 3, 6.0, 8.0, paper + 11.0
    else:
        cols, gutter_x, top, pitch_y = 1, 0.0, 10.0, paper + 13.0
    left = (A4_MM[0] - cols * paper - (cols - 1) * gutter_x) / 2.0
    rows = int((A4_MM[1] - top) // pitch_y)
    if len(entries) > cols * rows:
        raise ValueError(
            f'{len(entries)} tags do not fit {cols}x{rows} sheet')
    for i, (tag_id, label) in enumerate(entries):
        r, c = divmod(i, cols)
        _paste_tag(canvas, draw, tag_id, tag_mm,
                   left + c * (paper + gutter_x), top + r * pitch_y,
                   label)
    canvas.save(out_stem.with_suffix('.pdf'), 'PDF', resolution=DPI)
    canvas.save(out_stem.with_suffix('.png'), dpi=(DPI, DPI))
    print(f'sheet: {out_stem}.pdf  ({len(entries)} tags @ {tag_mm}mm, '
          f'{cols}col)')


FACE_NAMES = ('top', 'front', 'right', 'back', 'left', 'bottom')
CUBE_BASE_IDS = {'60A': 300, '60B': 306, '60C': 312,
                 '100A': 320, '100B': 326, '100C': 332}


def cube_entries(*cubes: str) -> List[Tuple[int, str]]:
    """Face tags of the given cubes (id layout per cube_kit MANIFEST:
    ids run top,front,right,back,left,bottom from the cube's base id).
    All cube faces use 50 mm tags regardless of cube size."""
    out: List[Tuple[int, str]] = []
    for cube in cubes:
        base = CUBE_BASE_IDS[cube]
        out += [(base + i, f'{cube} {f.upper()}')
                for i, f in enumerate(FACE_NAMES)]
    return out


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out-dir', required=True)
    p.add_argument('--cube-kit-out', default=None,
                   help='also generate the cube-face sheets here '
                        '(2 cubes = 12 faces per A4)')
    args = p.parse_args()
    out = Path(args.out_dir).expanduser()
    out.mkdir(parents=True, exist_ok=True)

    if args.cube_kit_out:
        ck = Path(args.cube_kit_out).expanduser()
        ck.mkdir(parents=True, exist_ok=True)
        make_sheet(cube_entries('60A', '60B'), 50.0,
                   ck / 'sheet_cube60_ab_id300-311_50mm')
        make_sheet(cube_entries('60C', '100A'), 50.0,
                   ck / 'sheet_cube60c_100a_id312-325_50mm')
        make_sheet(cube_entries('100B', '100C'), 50.0,
                   ck / 'sheet_cube100_bc_id326-337_50mm')

    # Calibration spares: 12× ID 0 (paper tags wear; recut, re-paste).
    make_sheet([(0, 'CALIB')] * 12, 50.0,
               out / 'sheet_calibration_id000_x12_50mm')
    # Surface corner anchors: distinct id per corner (registry 10-19;
    # 10-13 = first surface). Paper spares — the permanent install is
    # the two-colour printed plate (generate_tag_mounts --flat-plates).
    make_sheet([(i, f'CORNER ID{i}') for i in range(10, 14)], 50.0,
               out / 'sheet_surface_corners_id010-013_50mm')
    # Mother-stem marks: class id 100 for every stem (position tells
    # individuals apart; registry v2).
    make_sheet([(100, 'MOTHER STEM')] * 12, 50.0,
               out / 'sheet_mother_stem_id100_x12_50mm')
    # Zone/nav markers, 100 mm, 2 per sheet. Duplicates of one ID are
    # legitimate (several keep-out spots share the meaning).
    make_sheet([(50, 'KEEP OUT')] * 2, 100.0,
               out / 'sheet_keep_out_id050_x2_100mm')
    make_sheet([(51, 'STOP'), (52, 'SLOW')], 100.0,
               out / 'sheet_stop_slow_id051_052_100mm')
    make_sheet([(20, 'DOCK'), (20, 'DOCK')], 100.0,
               out / 'sheet_dock_id020_x2_100mm')
    make_sheet([(30, 'ROW N'), (31, 'ROW S')], 100.0,
               out / 'sheet_row_anchors_id030_031_100mm')
    return 0


if __name__ == '__main__':
    sys.exit(main())
