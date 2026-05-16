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


def generate_face_panel_stl(
    tag_id: int,
    panel_size_mm: float,
    tag_size_mm: float,
    output_path: Path,
    base_thickness_mm: float = 3.0,
    pillar_height_mm: float = 0.6,
) -> None:
    """Single-piece face panel STL with raised pillars at AprilTag black cells.

    The 10×10 raw tag pattern includes a 1-cell black border, so the
    pillars naturally include the AprilTag-required outer black frame.
    """
    pattern = fetch_tag_array(tag_id)  # 0 = black, 1 = white
    cell_size = tag_size_mm / 10.0
    tag_offset = (panel_size_mm - tag_size_mm) / 2.0

    tris: List[Tuple[Vec3, Vec3, Vec3, Vec3]] = []
    # Base plate.
    tris.extend(_box_triangles(
        0.0, 0.0, 0.0,
        panel_size_mm, panel_size_mm, base_thickness_mm,
    ))

    # Raised pillars for every black cell.
    # AprilTag PNGs convention: row 0 is the top of the tag pattern, but
    # we treat the panel surface with row 0 at +Y (panel "up").
    # Result: panel orientation when looking at it matches the PNG image.
    for row in range(10):
        for col in range(10):
            if pattern[row, col] == 0:  # black
                x0 = tag_offset + col * cell_size
                y0 = tag_offset + (9 - row) * cell_size  # flip Y so row 0 = +Y
                tris.extend(_box_triangles(
                    x0, y0, base_thickness_mm,
                    x0 + cell_size, y0 + cell_size,
                    base_thickness_mm + pillar_height_mm,
                ))

    write_stl_ascii(
        tris, output_path,
        name=f'fv_apriltag_panel_id{tag_id:03d}',
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
    stl_dir = out_dir / 'stl'
    tag_png_dir.mkdir(parents=True, exist_ok=True)
    tag_pdf_dir.mkdir(parents=True, exist_ok=True)
    stl_dir.mkdir(parents=True, exist_ok=True)

    def write_tag_assets(tag_id: int, tag_size_mm: float, stem: str) -> None:
        label = f'tag36h11 id={tag_id}  size={tag_size_mm:g}mm  ({stem})'
        generate_tag_print_png(
            tag_id, tag_size_mm, tag_png_dir / f'{stem}.png', label=label,
        )
        generate_tag_print_pdf(
            tag_id, tag_size_mm, tag_pdf_dir / f'{stem}.pdf', label=label,
        )

    # Calibration tag (ID 0). Single-page print, no STL panel needed —
    # operator mounts it on the existing 60×60 calibration plate.
    cal_id = 0
    write_tag_assets(cal_id, 50.0, f'tag36h11_id{cal_id:03d}_calibration_50mm')
    print(f'wrote calibration tag (id {cal_id})')

    # Cube tags + face panels.
    used_ids: List[int] = [cal_id]
    for cube in cubes:
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

            stl_path = stl_dir / (
                f'{cube.label}_{face_name}_id{tag_id:03d}.stl'
            )
            generate_face_panel_stl(
                tag_id=tag_id,
                panel_size_mm=cube.cube_size_mm,
                tag_size_mm=cube.tag_size_mm,
                output_path=stl_path,
            )
        print(
            f'cube {cube.label}: '
            f'IDs {sorted(cube.face_to_id.values())} '
            f'-> {len(cube.face_to_id)} panels'
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
