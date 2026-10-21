"""Generate A4 multi-tag sheets: many tags per page with scissor cut
lines, at true scale (300 DPI).

50 mm sheets (grid): each cell is a light-gray CUT BOX of exactly
(tag + margin) — the size the mount pockets expect — with the id/label
caption printed OUTSIDE the box in the gutter (readable before cutting,
discarded after). 50 mm tags tile 3×4 = 12 per sheet.

150 mm FIELD sheets (design v3, 2026-07-29): one tag per A4 with a
black NAME BANNER (white capitals — readable by humans from metres
away) under the tag. TWO cut lines: the inner line hugs the tag
artwork (drop-in for the mount/plate pockets), the outer line keeps
tag + banner together for direct pasting with the name attached.

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


BANNER_GAP_MM = 8.0    # white between tag artwork and banner — keeps the
                       # quiet zone generous even on the pasted-with-name cut
BANNER_H_MM = 26.0
OUTER_PAD_MM = 5.0


def _crop_marks(draw: ImageDraw.ImageDraw, x0: int, y0: int,
                x1: int, y1: int, len_px: int, w: int) -> None:
    """Corner crop marks (four L ticks) instead of a full cut rectangle.

    A full gray rectangle hugging the artwork becomes an aruco quad
    candidate; being bigger than the tag's black square and with
    corners within 5% of its perimeter, candidate merging KEEPS the
    rectangle and DROPS the real marker — the sheet stops decoding.
    Ticks never form a quad, and the cut still lands on the artwork
    edge (mount pockets expect exactly tag_mm)."""
    for cx, cy, sx, sy in ((x0, y0, 1, 1), (x1, y0, -1, 1),
                           (x1, y1, -1, -1), (x0, y1, 1, -1)):
        draw.line([(cx, cy), (cx + sx * len_px, cy)], fill=CUT_GRAY, width=w)
        draw.line([(cx, cy), (cx, cy + sy * len_px)], fill=CUT_GRAY, width=w)


def make_field_sheet(tag_id: int, label: str, out_stem: Path,
                     tag_mm: float = 150.0) -> None:
    """One 150 mm field tag per A4, design v3: name banner + dual cut.

    inner cut = tag artwork edge, marked by corner CROP MARKS (mount-
    pocket drop-in, name discarded — see _crop_marks for why not a box)
    outer cut = tag + banner + OUTER_PAD (paste with the name attached)
    The id/spec caption sits OUTSIDE the outer line (gutter, discarded).
    """
    w, h = _px(A4_MM[0]), _px(A4_MM[1])
    canvas = Image.new('L', (w, h), color=255)
    draw = ImageDraw.Draw(canvas)
    tag_x = (A4_MM[0] - tag_mm) / 2.0
    tag_y = 28.0
    cut_w = max(1, _px(CUT_W_MM))
    # tag artwork + crop marks on its edge
    pattern = fetch_tag_array(tag_id)          # 10×10, 0=black
    tag_px = _px(tag_mm)
    tile = Image.fromarray((pattern * 255).astype('uint8'), 'L') \
        .resize((tag_px, tag_px), Image.NEAREST)
    x0, y0 = _px(tag_x), _px(tag_y)
    canvas.paste(tile, (x0, y0))
    _crop_marks(draw, x0, y0, x0 + tag_px, y0 + tag_px,
                _px(6.0), cut_w)
    # name banner: black bar, white bold capitals, full tag width
    b_y0 = tag_y + tag_mm + BANNER_GAP_MM
    bx0, by0 = _px(tag_x), _px(b_y0)
    bx1, by1 = _px(tag_x + tag_mm), _px(b_y0 + BANNER_H_MM)
    draw.rectangle([bx0, by0, bx1, by1], fill=0)
    font = _font_for_box(label, bx1 - bx0 - _px(10.0),
                         by1 - by0 - _px(7.0))
    bbox = font.getbbox(label)
    tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
    draw.text(((bx0 + bx1 - tw) // 2 - bbox[0],
               (by0 + by1 - th) // 2 - bbox[1]), label, fill=255,
              font=font)
    # outer cut line: tag + banner stay one piece
    ox0, oy0 = _px(tag_x - OUTER_PAD_MM), _px(tag_y - OUTER_PAD_MM)
    ox1 = _px(tag_x + tag_mm + OUTER_PAD_MM)
    oy1 = _px(b_y0 + BANNER_H_MM + OUTER_PAD_MM)
    draw.rectangle([ox0, oy0, ox1, oy1], outline=CUT_GRAY, width=cut_w)
    # spec caption in the gutter below the outer line
    caption = (f'id{tag_id:03d}  {label}  tag36h11  '
               f'{tag_mm:.0f}mm (black {tag_mm * 0.8:.0f}mm)')
    cfont = _font_for_box(caption, ox1 - ox0 - 8, _px(5.0))
    cbox = cfont.getbbox(caption)
    draw.text((((ox0 + ox1) - (cbox[2] - cbox[0])) // 2 - cbox[0],
               oy1 + _px(2.0) - cbox[1]), caption, fill=90, font=cfont)
    canvas.save(out_stem.with_suffix('.pdf'), 'PDF', resolution=DPI)
    canvas.save(out_stem.with_suffix('.png'), dpi=(DPI, DPI))
    print(f'field sheet: {out_stem}.pdf  (id{tag_id:03d} {label} '
          f'@ {tag_mm:.0f}mm, banner)')


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

    # File naming: '<3-digit id>_<name>_...' so a directory listing
    # sorts by tag id (multi-id sheets use the id range).
    if args.cube_kit_out:
        ck = Path(args.cube_kit_out).expanduser()
        ck.mkdir(parents=True, exist_ok=True)
        make_sheet(cube_entries('60A', '60B'), 50.0,
                   ck / '300-311_cube60_ab_50mm')
        make_sheet(cube_entries('60C', '100A'), 50.0,
                   ck / '312-325_cube60c_100a_50mm')
        make_sheet(cube_entries('100B', '100C'), 50.0,
                   ck / '326-337_cube100_bc_50mm')

    # Calibration spares: 12× ID 0 (paper tags wear; recut, re-paste).
    make_sheet([(0, 'CALIB')] * 12, 50.0,
               out / '000_calib_x12_50mm')
    # Surface corner anchors: distinct id per corner (registry 10-19;
    # 10-13 = first surface). Paper spares — the permanent install is
    # the two-colour printed plate (generate_tag_mounts --flat-plates).
    make_sheet([(i, f'CORNER ID{i}') for i in range(10, 14)], 50.0,
               out / '010-013_corner_50mm')
    # Mother-stem marks: class id 100 for every stem (position tells
    # individuals apart; registry v2).
    make_sheet([(100, 'MOTHER STEM')] * 12, 50.0,
               out / '100_mother_stem_x12_50mm')
    # Field markers are 150mm unified (registry 2026-07-21): one tag
    # per A4 sheet, banner design v3. Print a sheet per installed spot
    # (duplicates of one ID are legitimate — several keep-out spots
    # share the meaning).
    make_field_sheet(20, 'HOME', out / '020_home_150mm')
    # lane anchors (registry v2.1): 'LANE<k> START'/'END', lanes 1-5
    for k in range(1, 6):
        sid = 28 + 2 * k
        make_field_sheet(sid, f'LANE{k} START',
                         out / f'{sid:03d}_lane{k}_start_150mm')
        make_field_sheet(sid + 1, f'LANE{k} END',
                         out / f'{sid + 1:03d}_lane{k}_end_150mm')
    make_field_sheet(50, 'KEEP OUT', out / '050_keep_out_150mm')
    make_field_sheet(51, 'STOP', out / '051_stop_150mm')
    make_field_sheet(52, 'SLOW', out / '052_slow_150mm')
    # localization anchors (registry 110-119): distinct id per point
    for i, letter in enumerate('ABCD'):
        make_field_sheet(110 + i, f'ANCHOR {letter}',
                         out / f'{110 + i}_anchor_{letter.lower()}_150mm')
    return 0


if __name__ == '__main__':
    sys.exit(main())
