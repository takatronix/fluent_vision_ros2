"""Generate 3D-printable AprilTag mounts: a ground PEG (field use) and a
flat STAND (bench/mat use).

Both carry a shallow rectangular pocket sized for a PAPER-printed tag
(cut along the white quiet-zone edge), so the paper self-aligns:

    pocket side = paper side + fit clearance, depth 0.6 mm

Paper sizes follow the print tools in generate_cube_assets.py
(tag + 5 mm white margin per side):

    50 mm tag -> 60 mm paper   (close range: arm camera 0.2-0.5 m)
    100 mm tag -> 110 mm paper (field range: nav camera 1-3 m)

The PEG holds the tag plate VERTICAL (side-facing — matches side-view
cameras and side-approach grasping) on a square stake with a pyramid
point. The STAND is a flat tile, tag facing up.

Meshes are overlapping closed boxes/prisms — slicers union them, same
convention as generate_cube_assets.py.

Run example:
    python3 generate_tag_mounts.py \
        --out-dir ~/ros2_ws/src/vlabor_ros2/docs/印刷物/cube_kit/stl/mounts
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import List, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent))
from generate_cube_assets import Vec3, _box_triangles, write_stl_ascii  # noqa: E402

Tri = Tuple[Vec3, Vec3, Vec3, Vec3]

POCKET_DEPTH_MM = 0.6
POCKET_CLEARANCE_MM = 0.6     # per side pair total — easy paper drop-in
BORDER_MM = 4.5               # pocket rim width
PLATE_T_MM = 4.0              # tag plate / tile thickness


def _tri(v1: Vec3, v2: Vec3, v3: Vec3) -> Tri:
    ux, uy, uz = (v2[0] - v1[0], v2[1] - v1[1], v2[2] - v1[2])
    wx, wy, wz = (v3[0] - v1[0], v3[1] - v1[1], v3[2] - v1[2])
    n = (uy * wz - uz * wy, uz * wx - ux * wz, ux * wy - uy * wx)
    ln = max((n[0] ** 2 + n[1] ** 2 + n[2] ** 2) ** 0.5, 1e-9)
    return ((n[0] / ln, n[1] / ln, n[2] / ln), v1, v2, v3)


def _pyramid_down(cx: float, cy: float, half: float,
                  z_top: float, z_tip: float) -> List[Tri]:
    """Closed square pyramid pointing DOWN (stake tip)."""
    a = (cx - half, cy - half, z_top)
    b = (cx + half, cy - half, z_top)
    c = (cx + half, cy + half, z_top)
    d = (cx - half, cy + half, z_top)
    tip = (cx, cy, z_tip)
    return [
        _tri(a, b, tip), _tri(b, c, tip), _tri(c, d, tip), _tri(d, a, tip),
        _tri(a, c, b), _tri(a, d, c),      # top square (closes the solid)
    ]


def _wedge_x(x0: float, x1: float, y0: float, y1: float,
             z0: float, z1: float) -> List[Tri]:
    """Right-triangle prism: full height at y0, tapering to z0 at y1.
    Used as a gusset rib between plate back and stake."""
    a0, b0 = (x0, y0, z0), (x0, y0, z1)
    c0 = (x0, y1, z0)
    a1, b1 = (x1, y0, z0), (x1, y0, z1)
    c1 = (x1, y1, z0)
    return [
        _tri(a0, b0, c0), _tri(a1, c1, b1),              # triangle ends
        _tri(a0, c0, c1), _tri(a0, c1, a1),              # bottom
        _tri(a0, a1, b1), _tri(a0, b1, b0),              # vertical face
        _tri(b0, b1, c1), _tri(b0, c1, c0),              # hypotenuse face
    ]


def _pocket_plate(w: float, h: float,
                  pocket_w: float, pocket_h: float) -> List[Tri]:
    """Plate w×h×PLATE_T lying in XY with a pocket recess on TOP
    (z = PLATE_T - POCKET_DEPTH .. PLATE_T), pocket centred."""
    tris: List[Tri] = []
    base_t = PLATE_T_MM - POCKET_DEPTH_MM
    tris += _box_triangles(0, 0, 0, w, h, base_t)
    bx = (w - pocket_w) / 2.0
    by = (h - pocket_h) / 2.0
    # four rim strips forming the pocket walls
    tris += _box_triangles(0, 0, base_t, w, by, PLATE_T_MM)
    tris += _box_triangles(0, h - by, base_t, w, h, PLATE_T_MM)
    tris += _box_triangles(0, by, base_t, bx, h - by, PLATE_T_MM)
    tris += _box_triangles(w - bx, by, base_t, w, h - by, PLATE_T_MM)
    return tris


def generate_stand(paper_mm: float, out: Path) -> None:
    pocket = paper_mm + POCKET_CLEARANCE_MM
    tile = pocket + 2 * BORDER_MM
    tris = _pocket_plate(tile, tile, pocket, pocket)
    write_stl_ascii(tris, out, name=f'fv_tag_stand_{int(paper_mm)}')
    print(f'stand: {out}  tile {tile:.1f}x{tile:.1f}x{PLATE_T_MM}mm '
          f'pocket {pocket:.1f}mm')


def generate_peg(paper_mm: float, stake_mm: float, out: Path) -> None:
    """Vertical tag plate on a square stake with pyramid point.

    Modelled in PRINT orientation: lying on its back (plate face up,
    +Z), stake running along +Y below the plate. In the field the part
    stands up: plate vertical, tag facing sideways.
    """
    pocket = paper_mm + POCKET_CLEARANCE_MM
    plate = pocket + 2 * BORDER_MM
    shaft = 22.0 if paper_mm <= 80 else 28.0
    tip = 28.0
    tris: List[Tri] = []
    # plate with pocket (top face at z=PLATE_T)
    tris += _pocket_plate(plate, plate, pocket, pocket)
    cx = plate / 2.0
    # stake shaft: from plate bottom edge (y=0) downward (−y)
    tris += _box_triangles(cx - shaft / 2, -stake_mm, 0,
                           cx + shaft / 2, 0, PLATE_T_MM)
    # pyramid tip continuing the shaft (in-plane wedge as printed —
    # closed pyramid oriented along −y): approximate with a wedge
    # tapering the shaft cross-section to a line, printable flat.
    tris += [
        _tri((cx - shaft / 2, -stake_mm, 0),
             (cx + shaft / 2, -stake_mm, 0),
             (cx, -stake_mm - tip, 0)),
        _tri((cx - shaft / 2, -stake_mm, PLATE_T_MM),
             (cx, -stake_mm - tip, PLATE_T_MM),
             (cx + shaft / 2, -stake_mm, PLATE_T_MM)),
        _tri((cx - shaft / 2, -stake_mm, 0),
             (cx, -stake_mm - tip, 0),
             (cx - shaft / 2, -stake_mm, PLATE_T_MM)),
        _tri((cx - shaft / 2, -stake_mm, PLATE_T_MM),
             (cx, -stake_mm - tip, 0),
             (cx, -stake_mm - tip, PLATE_T_MM)),
        _tri((cx + shaft / 2, -stake_mm, 0),
             (cx + shaft / 2, -stake_mm, PLATE_T_MM),
             (cx, -stake_mm - tip, 0)),
        _tri((cx + shaft / 2, -stake_mm, PLATE_T_MM),
             (cx, -stake_mm - tip, PLATE_T_MM),
             (cx, -stake_mm - tip, 0)),
        # close shaft junction (overlaps shaft box — slicer unions)
        _tri((cx - shaft / 2, -stake_mm, 0),
             (cx - shaft / 2, -stake_mm, PLATE_T_MM),
             (cx + shaft / 2, -stake_mm, 0)),
        _tri((cx + shaft / 2, -stake_mm, 0),
             (cx - shaft / 2, -stake_mm, PLATE_T_MM),
             (cx + shaft / 2, -stake_mm, PLATE_T_MM)),
    ]
    # gusset ribs where plate meets stake (stiffen against soil push)
    rib = 30.0
    tris += _wedge_x(cx - shaft / 2, cx - shaft / 2 + 4.0,
                     0, rib, PLATE_T_MM, PLATE_T_MM + 10.0)
    tris += _wedge_x(cx + shaft / 2 - 4.0, cx + shaft / 2,
                     0, rib, PLATE_T_MM, PLATE_T_MM + 10.0)
    # press shoulder above the plate to push/hammer by palm
    tris += _box_triangles(cx - 20, plate, 0, cx + 20, plate + 12,
                           PLATE_T_MM + 4.0)
    write_stl_ascii(tris, out, name=f'fv_tag_peg_{int(paper_mm)}')
    total = plate + 12 + stake_mm + tip
    print(f'peg: {out}  plate {plate:.1f}mm  stake {stake_mm:.0f}mm '
          f'tip {tip:.0f}mm  total {total:.0f}mm')


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out-dir', required=True)
    p.add_argument('--paper-sizes', default='60,110',
                   help='comma list of paper sizes [mm] (tag+margins)')
    p.add_argument('--stake-mm', type=float, default=150.0)
    args = p.parse_args()
    out = Path(args.out_dir).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    for s in [float(v) for v in args.paper_sizes.split(',')]:
        tag = int(s - 10)
        generate_stand(s, out / f'tag_stand_paper{int(s)}_tag{tag}.stl')
        generate_peg(s, args.stake_mm,
                     out / f'tag_peg_paper{int(s)}_tag{tag}.stl')
    return 0


if __name__ == '__main__':
    sys.exit(main())
