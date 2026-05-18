"""Generate AprilTag cube assets: printable tag PNGs + 3D-printable face panel STLs.

Output layout:
  <out_dir>/
    tag_pngs/  printable PNG per tag ID, scaled to <tag_size>mm at 300 dpi
    stl/       face panel STL per (cube_label, face, tag_id)

The STLs use a single-piece dual-extrusion-friendly design:
  - Base plate: <panel>×<panel>×<base_thickness>mm (intended white filament)
  - Raised pillars at every BLACK cell of the AprilTag pattern (intended
    black filament), rising <pillar_height>mm above the base
A slicer can be set to swap filament at Z = base_thickness for clean
two-colour prints; alternatively the panel can be printed mono and the
recess-shaped negative space sticker-applied.

Bit-pattern source: https://github.com/AprilRobotics/apriltag-imgs
(public, MIT licence). The 10×10 px PNGs are downloaded once and
cached under ~/.cache/fv_apriltag/tag_imgs/.

Run example:
    python3 generate_cube_assets.py \\
        --out-dir ~/ros2_ws/src/vlabor_ros2/docs/印刷物/cube_kit
"""
from __future__ import annotations

import argparse
import os
import sys
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
from PIL import Image


TAG_FAMILY = 'tag36h11'
TAG_IMG_URL = (
    'https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/master/'
    'tag36h11/tag36_11_{:05d}.png'
)
CACHE_DIR = Path.home() / '.cache' / 'fv_apriltag' / 'tag_imgs'
FACE_NAMES = ('top', 'front', 'right', 'back', 'left', 'bottom')


@dataclass
class CubeSpec:
    label: str            # e.g. "cube60_a"
    cube_size_mm: float   # e.g. 60.0
    tag_size_mm: float    # e.g. 50.0
    face_to_id: Dict[str, int]  # {face_name: tag_id}


def fetch_tag_array(tag_id: int) -> np.ndarray:
    """Return the 10×10 binary pattern for the given tag ID. 0=black, 1=white."""
    CACHE_DIR.mkdir(parents=True, exist_ok=True)
    local = CACHE_DIR / f'tag36_11_{tag_id:05d}.png'
    if not local.exists():
        url = TAG_IMG_URL.format(tag_id)
        urllib.request.urlretrieve(url, local)
    with Image.open(local) as img:
        arr = np.array(img.convert('L'))
    if arr.shape != (10, 10):
        raise ValueError(
            f'unexpected tag png shape {arr.shape} for id {tag_id} '
            f'(expected 10×10)'
        )
    return (arr > 127).astype(np.uint8)


def _render_tag_print_image(
    tag_id: int, tag_size_mm: float, dpi: int,
    white_margin_mm: float, label: str,
) -> Image.Image:
    """Render the printable tag as a PIL grayscale image with embedded DPI."""
    from PIL import ImageDraw, ImageFont

    pattern = fetch_tag_array(tag_id)
    px_per_mm = dpi / 25.4
    tag_px = int(round(tag_size_mm * px_per_mm))
    margin_px = int(round(white_margin_mm * px_per_mm))
    cell_px = tag_px // 10

    tag_img = np.zeros((tag_px, tag_px), dtype=np.uint8)
    for r in range(10):
        for c in range(10):
            v = 255 if pattern[r, c] else 0
            tag_img[r * cell_px:(r + 1) * cell_px,
                    c * cell_px:(c + 1) * cell_px] = v
    if cell_px * 10 < tag_px:
        tag_img[cell_px * 10:, :] = tag_img[cell_px * 10 - 1, :]
        tag_img[:, cell_px * 10:] = tag_img[:, cell_px * 10 - 1:cell_px * 10]

    label_band_px = int(round(8 * px_per_mm))
    full_w = tag_px + margin_px * 2
    full_h = tag_px + margin_px * 2 + label_band_px
    full = np.ones((full_h, full_w), dtype=np.uint8) * 255
    full[margin_px:margin_px + tag_px,
         margin_px:margin_px + tag_px] = tag_img

    out_img = Image.fromarray(full, mode='L')

    # Add a small caption under the tag so prints are self-identifying.
    draw = ImageDraw.Draw(out_img)
    try:
        font_size = int(round(3.5 * px_per_mm))
        font = ImageFont.truetype(
            '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', font_size,
        )
    except Exception:
        font = ImageFont.load_default()
    text_y = margin_px + tag_px + int(round(2 * px_per_mm))
    draw.text((margin_px, text_y), label, fill=0, font=font)
    return out_img


def generate_tag_print_png(
    tag_id: int, tag_size_mm: float, output_path: Path, dpi: int = 300,
    white_margin_mm: float = 5.0, label: str = '',
) -> None:
    """Write the printable tag as a PNG with embedded DPI metadata."""
    img = _render_tag_print_image(
        tag_id, tag_size_mm, dpi, white_margin_mm,
        label or f'tag36h11 id={tag_id}  size={tag_size_mm:g}mm',
    )
    img.save(output_path, dpi=(dpi, dpi))


def generate_tag_print_pdf(
    tag_id: int, tag_size_mm: float, output_path: Path, dpi: int = 300,
    white_margin_mm: float = 5.0, label: str = '',
) -> None:
    """Write the printable tag as a single-page PDF (PIL native PDF writer).

    PDF resolution is set to `dpi` so the tag prints at exactly
    `tag_size_mm`, independent of the receiving printer's default scale.
    """
    img = _render_tag_print_image(
        tag_id, tag_size_mm, dpi, white_margin_mm,
        label or f'tag36h11 id={tag_id}  size={tag_size_mm:g}mm',
    )
    img.save(output_path, 'PDF', resolution=float(dpi))


# --- STL geometry helpers ---------------------------------------------------

Vec3 = Tuple[float, float, float]


def _box_triangles(x0: float, y0: float, z0: float,
                   x1: float, y1: float, z1: float) -> List[Tuple[Vec3, Vec3, Vec3, Vec3]]:
    """Return 12 triangles (one box) as (normal, v1, v2, v3) tuples."""
    # 8 corners
    v000 = (x0, y0, z0); v100 = (x1, y0, z0)
    v010 = (x0, y1, z0); v110 = (x1, y1, z0)
    v001 = (x0, y0, z1); v101 = (x1, y0, z1)
    v011 = (x0, y1, z1); v111 = (x1, y1, z1)
    tris: List[Tuple[Vec3, Vec3, Vec3, Vec3]] = []
    # Bottom (-Z)
    n = (0.0, 0.0, -1.0)
    tris.append((n, v000, v110, v100))
    tris.append((n, v000, v010, v110))
    # Top (+Z)
    n = (0.0, 0.0, 1.0)
    tris.append((n, v001, v101, v111))
    tris.append((n, v001, v111, v011))
    # -Y
    n = (0.0, -1.0, 0.0)
    tris.append((n, v000, v100, v101))
    tris.append((n, v000, v101, v001))
    # +Y
    n = (0.0, 1.0, 0.0)
    tris.append((n, v010, v111, v110))
    tris.append((n, v010, v011, v111))
    # -X
    n = (-1.0, 0.0, 0.0)
    tris.append((n, v000, v001, v011))
    tris.append((n, v000, v011, v010))
    # +X
    n = (1.0, 0.0, 0.0)
    tris.append((n, v100, v110, v111))
    tris.append((n, v100, v111, v101))
    return tris


def write_stl_ascii(tris: List[Tuple[Vec3, Vec3, Vec3, Vec3]],
                    output_path: Path, name: str) -> None:
    lines: List[str] = [f'solid {name}']
    for n, a, b, c in tris:
        lines.append(f'  facet normal {n[0]:.6f} {n[1]:.6f} {n[2]:.6f}')
        lines.append('    outer loop')
        lines.append(f'      vertex {a[0]:.6f} {a[1]:.6f} {a[2]:.6f}')
        lines.append(f'      vertex {b[0]:.6f} {b[1]:.6f} {b[2]:.6f}')
        lines.append(f'      vertex {c[0]:.6f} {c[1]:.6f} {c[2]:.6f}')
        lines.append('    endloop')
        lines.append('  endfacet')
    lines.append(f'endsolid {name}')
    output_path.write_text('\n'.join(lines) + '\n')


def generate_tag_plate_stl(
    tag_id: int,
    tag_size_mm: float,
    base_path: Path,
    pattern_path: Path,
    plate_thickness_mm: float = 5.0,
    pattern_height_mm: float = 1.0,
) -> None:
    """Drop-in tag plate that fits into the cube body's face pocket.

    Two-piece multi-colour print:

    - <stem>_base.stl    : tag_size × tag_size × (plate_thickness -
                           pattern_height) flat plate
                           (intended white filament)
    - <stem>_pattern.stl : raised pillars at the AprilTag dark cells,
                           sitting on top of the base
                           (intended black filament)

    Both files share the same origin so the slicer can merge them as
    sub-parts and apply one filament per part. Total plate thickness
    = base + pattern = plate_thickness_mm, matched to the cube body's
    pocket depth.
    """
    pattern = fetch_tag_array(tag_id)  # 0 = black, 1 = white
    cell_size = tag_size_mm / 10.0
    base_thickness = plate_thickness_mm - pattern_height_mm
    if base_thickness <= 0.0:
        raise ValueError(
            f'plate_thickness ({plate_thickness_mm}) must exceed '
            f'pattern_height ({pattern_height_mm})'
        )

    base_tris: List[Tuple[Vec3, Vec3, Vec3, Vec3]] = _box_triangles(
        0.0, 0.0, 0.0,
        tag_size_mm, tag_size_mm, base_thickness,
    )
    write_stl_ascii(
        base_tris, base_path,
        name=f'fv_apriltag_plate_id{tag_id:03d}_base',
    )

    pattern_tris: List[Tuple[Vec3, Vec3, Vec3, Vec3]] = []
    for row in range(10):
        for col in range(10):
            if pattern[row, col] == 0:  # black
                x0 = col * cell_size
                y0 = (9 - row) * cell_size
                pattern_tris.extend(_box_triangles(
                    x0, y0, base_thickness,
                    x0 + cell_size, y0 + cell_size,
                    base_thickness + pattern_height_mm,
                ))
    write_stl_ascii(
        pattern_tris, pattern_path,
        name=f'fv_apriltag_plate_id{tag_id:03d}_pattern',
    )


def generate_cube_body_stl(
    cube_size_mm: float,
    output_path: Path,
    pocket_size_mm: float = 50.2,
    pocket_depth_mm: float = 5.0,
) -> None:
    """Solid cube body STL with 6 face pockets for the tag plates.

    The body is a solid `cube_size`³ block with a `pocket_size` ×
    `pocket_size` × `pocket_depth` rectangular pocket cut from the
    centre of each of the 6 faces. All six pockets — every face is
    detachable. Interior is solid in the STL so the slicer's infill
    carries the gripper-squeeze load.

    Implementation: split the cube along all relevant planes (pocket
    boundaries on each axis), then output any sub-cell whose centre
    lies outside every pocket. Works for any pocket_depth < cs/2,
    including the case where opposite pockets get close to each other
    or where pocket_depth exceeds the pocket-edge rim (cs - ps)/2.
    A 60 mm cube ends up roughly 50 sub-boxes / 600 triangles.
    """
    cs = cube_size_mm
    ps = pocket_size_mm
    pd = pocket_depth_mm
    if pd >= cs / 2.0:
        raise ValueError(
            f'pocket_depth ({pd}) must be smaller than cs/2 '
            f'({cs/2}) so opposite pockets do not punch through'
        )
    pmin = (cs - ps) / 2.0   # pocket-edge rim on each face (4.9 mm)
    pmax = cs - pmin

    # Define pockets in (xrange, yrange, zrange) form for the
    # solid-region test below.
    pockets = (
        ((pmin, pmax), (pmin, pmax), (cs - pd, cs)),  # +Z
        ((pmin, pmax), (pmin, pmax), (0.0, pd)),       # -Z
        ((cs - pd, cs), (pmin, pmax), (pmin, pmax)),   # +X
        ((0.0, pd),    (pmin, pmax), (pmin, pmax)),    # -X
        ((pmin, pmax), (cs - pd, cs), (pmin, pmax)),   # +Y
        ((pmin, pmax), (0.0, pd),    (pmin, pmax)),    # -Y
    )

    def in_any_pocket(x: float, y: float, z: float) -> bool:
        for (x0, x1), (y0, y1), (z0, z1) in pockets:
            if x0 < x < x1 and y0 < y < y1 and z0 < z < z1:
                return True
        return False

    # Sort and dedupe split planes per axis. The pocket geometry only
    # has variation along the cube's own axes, so a 1D split per axis
    # is sufficient.
    splits = sorted({0.0, pd, pmin, pmax, cs - pd, cs})

    tris: List[Tuple[Vec3, Vec3, Vec3, Vec3]] = []
    box_count = 0
    for i in range(len(splits) - 1):
        x0, x1 = splits[i], splits[i + 1]
        if x1 - x0 < 1e-6:
            continue
        for j in range(len(splits) - 1):
            y0, y1 = splits[j], splits[j + 1]
            if y1 - y0 < 1e-6:
                continue
            for k in range(len(splits) - 1):
                z0, z1 = splits[k], splits[k + 1]
                if z1 - z0 < 1e-6:
                    continue
                cx = (x0 + x1) * 0.5
                cy = (y0 + y1) * 0.5
                cz = (z0 + z1) * 0.5
                if in_any_pocket(cx, cy, cz):
                    continue
                tris.extend(_box_triangles(x0, y0, z0, x1, y1, z1))
                box_count += 1

    write_stl_ascii(
        tris, output_path,
        name=f'fv_apriltag_cube_body_{int(round(cs))}mm',
    )


# --- ID layout --------------------------------------------------------------

def default_cube_set() -> List[CubeSpec]:
    """Default ID layout for the vlabor cube kit.

    ID 0  : reserved for hand-eye calibration (separate single tag)
    ID 1-6  : cube60_a faces (top,front,right,back,left,bottom)
    ID 7-12 : cube60_b
    ID 13-18: cube60_c
    ID 21-26: cube100_a (skip 19-20 — buffer + legacy calibration ID 20)
    ID 27-32: cube100_b
    ID 33-38: cube100_c
    """
    def faces(start: int) -> Dict[str, int]:
        return {name: start + i for i, name in enumerate(FACE_NAMES)}

    return [
        CubeSpec('cube60_a',  60.0, 50.0, faces(1)),
        CubeSpec('cube60_b',  60.0, 50.0, faces(7)),
        CubeSpec('cube60_c',  60.0, 50.0, faces(13)),
        CubeSpec('cube100_a', 100.0, 50.0, faces(21)),
        CubeSpec('cube100_b', 100.0, 50.0, faces(27)),
        CubeSpec('cube100_c', 100.0, 50.0, faces(33)),
    ]


# --- Orchestration ----------------------------------------------------------

def generate_all(out_dir: Path) -> None:
    cubes = default_cube_set()
    tag_png_dir = out_dir / 'tag_pngs'
    tag_pdf_dir = out_dir / 'tag_pdfs'
    body_dir = out_dir / 'stl' / 'body'
    plate_dir = out_dir / 'stl' / 'plates'
    tag_png_dir.mkdir(parents=True, exist_ok=True)
    tag_pdf_dir.mkdir(parents=True, exist_ok=True)
    body_dir.mkdir(parents=True, exist_ok=True)
    plate_dir.mkdir(parents=True, exist_ok=True)

    def write_tag_assets(tag_id: int, tag_size_mm: float, stem: str) -> None:
        label = f'tag36h11 id={tag_id}  size={tag_size_mm:g}mm  ({stem})'
        generate_tag_print_png(
            tag_id, tag_size_mm, tag_png_dir / f'{stem}.png', label=label,
        )
        generate_tag_print_pdf(
            tag_id, tag_size_mm, tag_pdf_dir / f'{stem}.pdf', label=label,
        )

    # Cube body STLs — one per unique cube size, shared across the
    # cube60_{a,b,c} (or cube100_{a,b,c}) variants since the plates
    # decide the identity.
    body_sizes = sorted({c.cube_size_mm for c in cubes})
    for size_mm in body_sizes:
        body_path = body_dir / f'cube{int(round(size_mm))}_body.stl'
        generate_cube_body_stl(size_mm, body_path)
        print(f'wrote cube body {body_path.name} ({size_mm:g} mm)')

    # Calibration tag (ID 0). Single-page print, no STL plate needed —
    # operator mounts it on the existing 60×60 calibration plate.
    cal_id = 0
    write_tag_assets(cal_id, 50.0, f'tag36h11_id{cal_id:03d}_calibration_50mm')
    print(f'wrote calibration tag (id {cal_id})')

    # Cube tags + drop-in plate pairs. All 6 faces are now detachable
    # (the cube body has a pocket on every face including the bottom).
    used_ids: List[int] = [cal_id]
    for cube in cubes:
        plates_written = 0
        for face_name, tag_id in cube.face_to_id.items():
            if tag_id in used_ids:
                raise RuntimeError(
                    f'tag id {tag_id} reused — fix default_cube_set'
                )
            used_ids.append(tag_id)

            stem = (
                f'tag36h11_id{tag_id:03d}_{cube.label}_{face_name}_50mm'
            )
            write_tag_assets(tag_id, cube.tag_size_mm, stem)

            stl_stem = plate_dir / (
                f'{cube.label}_{face_name}_id{tag_id:03d}'
            )
            generate_tag_plate_stl(
                tag_id=tag_id,
                tag_size_mm=cube.tag_size_mm,
                base_path=stl_stem.with_name(stl_stem.name + '_base.stl'),
                pattern_path=stl_stem.with_name(stl_stem.name + '_pattern.stl'),
            )
            plates_written += 1
        print(
            f'cube {cube.label}: '
            f'IDs {sorted(cube.face_to_id.values())} '
            f'-> {plates_written} plate pairs (all 6 faces)'
        )

    # Manifest for downstream tooling.
    manifest = out_dir / 'MANIFEST.txt'
    lines = ['# fv_apriltag cube kit manifest', '']
    lines.append(f'calibration_tag_id: {cal_id}')
    lines.append('')
    for cube in cubes:
        lines.append(f'[{cube.label}]')
        lines.append(f'  cube_size_mm: {cube.cube_size_mm}')
        lines.append(f'  tag_size_mm:  {cube.tag_size_mm}')
        for face_name, tag_id in cube.face_to_id.items():
            lines.append(f'  {face_name:<7s} -> id {tag_id}')
        lines.append('')
    manifest.write_text('\n'.join(lines))
    print(f'wrote {manifest}')


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        '--out-dir', required=True,
        help='output directory (will contain tag_pngs/ and stl/)',
    )
    args = p.parse_args()
    generate_all(Path(args.out_dir).expanduser())
    return 0


if __name__ == '__main__':
    sys.exit(main())
