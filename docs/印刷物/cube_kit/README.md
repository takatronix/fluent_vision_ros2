# fv_apriltag cube kit

3D-printable AprilTag cube assets for the fv_apriltag detector.

Three tag sets per size class (stackable for individual 6DOF tracking).
**2026-07-12: the printed 60 mm cube bodies are RETIRED — the "cube60"
sets are now paper tags pasted directly onto the REAL 50 mm cubes**
(a 50 mm cutout fits a 50 mm face edge-to-edge; the artwork's built-in
white ring is the quiet zone). Set names are kept for topic/config
compatibility.

| set       | physical cube | tag nominal (black) | tag IDs |
| --------- | ------------- | ------------------- | ------- |
| cube60_a  | 50 mm (real cube, paste-on) | 50 mm (black 40) | 1–6     |
| cube60_b  | 50 mm (real cube, paste-on) | 50 mm (black 40) | 7–12    |
| cube60_c  | 50 mm (real cube, paste-on) | 50 mm (black 40) | 13–18   |
| cube100_a | 100 mm (printed body) | 50 mm (black 40) | 21–26   |
| cube100_b | 100 mm (printed body) | 50 mm (black 40) | 27–32   |
| cube100_c | 100 mm (printed body) | 50 mm (black 40) | 33–38   |

The hand-eye calibration tag (ID 0) lives in
`../field_tags/sheets/000_calib_x12_50mm.pdf`.

IDs 19 and 20 are deliberately skipped (buffer + legacy calibration
ID 20 still referenced in some scene_viewer labels at time of
writing).

## Directory layout

```
cube_kit/
├── MANIFEST.txt        machine-readable ID → cube/face map
├── sheets/             A4 cut-line sheets, 12 faces (= 2 sets) per page
│                       (300 DPI; cut piece = 50 mm incl. the artwork's
│                       own white quiet ring; BLACK square = 40 mm)
└── stl/
    ├── body/           hollow cube bodies (60 mm body retired; 100 mm in use)
    ├── plates/         drop-in tag plates for printed bodies
    └── mounts/         tag pegs / stands (paper-pocket, A1-mini sized)
```

Print sheets at 100% scale, cut along the light-gray lines (each cell
captions `idNNN <SET> <FACE>` outside the cut box), and paste onto the
cube faces. Detector configs use the BLACK edge: `tag_size: 0.040`;
cube edge sizes live in `cubes_piper.yaml` (`size:`).

## How the cube goes together

The kit is **cartridge-style**: print one cube body, print 6 tag
plates, then drop the plates into the body's face pockets. The body
is a solid block with a 50.2 × 50.2 × 3 mm pocket cut from every
face, so all 6 sides are detectable and detachable. Tag plates are
held in by friction fit (0.1 mm clearance per side).

```
   stl/body/cube60_body.stl    one 60 × 60 × 60 mm solid block
                               with a pocket cut from every face;
                               solid interior → gripper-safe.
                               Slicer infill (30 %) fills the body.
   stl/plates/<cube>_<face>_<id>_base.stl        +  2 mm white base
   stl/plates/<cube>_<face>_<id>_pattern.stl     +  1 mm black tag
                                                    pattern
                                                  = 3 mm plate that
                                                    fits the pocket
                                                    flush with the
                                                    cube surface
```

Each cube needs **6 plate pairs** (one per face). The generator
emits 6 plate pairs per cube label (cube60_a etc.). Plate files are
named `<cube>_<face>_id<NNN>_{base,pattern}.stl` so a mis-routed
plate is obvious.

## Printing the cube body

1. Drag `stl/body/cube60_body.stl` (or `cube100_body.stl`) into Bambu
   Studio.
2. Single colour (white PLA), 3 walls, **30 % gyroid infill**,
   supports ON (Organic / Tree). Because every face has a pocket,
   the cube can't lay flat on a solid base — supports carry the
   pocket overhangs. Tree supports leave the rim faces clean.
3. Orientation: stand the cube on a corner (or face) — Bambu Studio
   defaults to corner-on-bed for objects with no obvious flat face.
   Either is OK; tree supports handle both.
4. Solid interior = gripper-safe. Slicer infill (30 % gyroid) gives
   ~270 g of plastic for a 60 mm cube and easily survives a 50 N
   gripper squeeze.

## Printing the tag plates (Bambu Studio, two-colour with AMS)

### Recommended: 3MF drop-in

1. **Drag `cube60_a_top_id001.3mf` (or any other `*.3mf`) into Bambu
   Studio.** It arrives as a single composite object with three
   parts (`base`, `pattern`, `text`).
2. The materials come pre-declared: part `base` is the WhitePLA
   slot, `pattern` and `text` are the BlackPLA slot.
3. Confirm the filament assignments match your AMS slots (or
   swap them via *Filaments*).
4. Slice and print.

### Fallback: STL trio with manual assembly

1. Drag `*_base.stl`, `*_pattern.stl`, and `*_text.stl` into the
   slicer.
2. Select all three → right-click → *Group selected* (or drag
   pattern + text under base in the object tree).
3. Assign filaments: base → white, pattern → black, text → black.
4. Slice and print. Plate is 50 × 50 × 3 mm so a build plate holds
   many at once — print all 6 plates of one cube as a batch.

For monochrome printers: print only `_base.stl` in white, then paint
the cells (use `_pattern.stl` as a depth reference) or stick a
printout of the matching PNG/PDF onto the base.

## Assembling

1. Print the body. Print all 6 plates of cube N. Total time ≈ 3-4 h
   per cube on a Bambu X1 / P1S at 0.2 mm layers (body ~2.5 h, six
   plates ~1 h batched together).
2. Drop each plate into its matching pocket from the outside. The
   plate is 50.0 × 50.0 × 3 mm; the pocket is 50.2 × 50.2 × 3 mm,
   so the plate slides in with 0.1 mm clearance per side and sits
   flush with the cube face. Friction holds it in place.
3. (Optional) tiny dab of CA glue at one corner if a particular
   plate is too loose. Leave most un-glued so IDs can be swapped
   by re-printing one plate.

The body is shared across cube60_a/b/c (and cube100_a/b/c) — only
the plates encode cube identity. Reprint one plate to change a face's
ID; the body stays.

## Regenerating

```
cd ros2_ws/src/fluent_vision_ros2/src/ai/fv_apriltag/tools
python3 generate_cube_assets.py \
    --out-dir ros2_ws/src/vlabor_ros2/docs/印刷物/cube_kit
```

The generator downloads the raw 10 × 10 tag images from
github.com/AprilRobotics/apriltag-imgs (cached under
`~/.cache/fv_apriltag/tag_imgs/`) so the first run needs internet.

Editing the ID layout: change `default_cube_set()` in the generator
and re-run. Then sync the IDs into
`fv_apriltag/config/cubes_piper.yaml` and `cubes_daihen.yaml`.
