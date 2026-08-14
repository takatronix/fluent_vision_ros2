// contents_def.h — the single source of truth for content METADATA: the
// public Scene name of every content type, its fields (name, kind, required,
// documented default), and which shared style keys it accepts.
//
// scene/schema.hpp includes this file with different definitions of the
// macros to derive, from this one list:
//
//   - the Scene YAML vocabulary        (content: { rect: { size: [340, 96] } })
//   - unknown-key / type validation
//   - the capability catalog           (fvsc describe --json, §13-1)
//   - the canonical formatter ordering (fvsc fmt, §13-8)
//
// Behavior is NOT derived from here: the compiler builds layers through the
// same Layer API a C++ author uses (stage.rect(...).cornerRadius(...)), and
// applies only the fields that are explicitly present — so every default
// lives once, in the C++ creation call. The numeric defaults recorded here
// are documentation, and scene_tests verifies they match the C++ structs.
//
// Field kinds (FS_K_*) are ValueKind names from scene/schema.hpp.
// Style fields (FS_CSTYLE) are declared once in the style table below and
// referenced by name; they map to the shared Layer style setters (§7).
//
// No include guard: this file is included multiple times by design.
//
//   FS_STYLE(name, KIND, summary)                    — shared style key
//   FS_CONTENT(name, summary)                        — content type
//   FS_CFIELD(name, KIND, required, def, summary)    — content field
//                                                      (def: brace-free token,
//                                                       FS_D* helpers below)
//   FS_CSTYLE(name)                                  — accepted style key
//   FS_CONTENT_END(name)

// ---- shared style keys (each maps to one Layer setter) ---------------------
// Defaults are content-dependent (each drawing call picks its own — a line
// starts at thickness 3, a rect filled), so none are recorded here.

FS_STYLE(color, Color, "content color (shapes, text, box outlines)")
FS_STYLE(thickness, F32, "stroke width in logical units; 0 fills closed shapes")
FS_STYLE(dash, F32, "dash length in logical units (equal on/off); 0 = solid")
FS_STYLE(cap, EnumCap, "line end and joint style: round | butt")
FS_STYLE(corner_radius, F32, "rounds rect/boxes content and the layer clip")
FS_STYLE(smoothing, F32, "boxes temporal smoothing time constant in seconds; 0 disables")
FS_STYLE(show_label, Bool, "draw label + score above each box")

// ---- content types (§7 catalog order) --------------------------------------

FS_CONTENT(image, "a borrowed image (camera frame) fitted into the layer bounds")
FS_CFIELD(source, Input, 1, FS_DNONE, "input reference ($inputs.<name>, type image.rgba8)")
FS_CFIELD(fit, EnumFit, 0, FS_DSTR(contain), "mapping into bounds: contain | cover | fill")
FS_CFIELD(source_rect, Rect, 0, FS_DRECT(0, 0, 0, 0),
          "source crop in pixels; zero size = whole image")
FS_CONTENT_END(image)

FS_CONTENT(text, "a run of UTF-8 text (HarfBuzz-shaped, Japanese included)")
FS_CFIELD(text, Str, 0, FS_DNONE, "the text (exactly one of text | source)")
FS_CFIELD(source, Input, 0, FS_DNONE, "input reference (type text.utf8) for live text")
FS_CFIELD(position, Vec2, 0, FS_DVEC2(0, 0), "reference point in layer space")
FS_CFIELD(size, F32, 0, FS_DF32(26), "font size in logical units")
FS_CFIELD(align, EnumAlign, 0, FS_DSTR(left),
          "edge the position refers to: left | center | right")
FS_CSTYLE(color)
FS_CONTENT_END(text)

FS_CONTENT(line, "a straight line")
FS_CFIELD(from, Vec2, 1, FS_DNONE, "start point")
FS_CFIELD(to, Vec2, 1, FS_DNONE, "end point")
FS_CSTYLE(color)
FS_CSTYLE(thickness)
FS_CSTYLE(dash)
FS_CSTYLE(cap)
FS_CONTENT_END(line)

FS_CONTENT(polyline, "an open path with round joints")
FS_CFIELD(points, Points, 0, FS_DNONE, "path points (exactly one of points | source)")
FS_CFIELD(source, Input, 0, FS_DNONE, "input reference (type sequence<vec2, N>)")
FS_CSTYLE(color)
FS_CSTYLE(thickness)
FS_CSTYLE(dash)
FS_CSTYLE(cap)
FS_CONTENT_END(polyline)

FS_CONTENT(polygon, "a closed path: filled at thickness 0, outlined otherwise")
FS_CFIELD(points, Points, 0, FS_DNONE, "path points (exactly one of points | source)")
FS_CFIELD(source, Input, 0, FS_DNONE, "input reference (type sequence<vec2, N>)")
FS_CSTYLE(color)
FS_CSTYLE(thickness)
FS_CSTYLE(dash)
FS_CSTYLE(cap)
FS_CONTENT_END(polygon)

FS_CONTENT(rect, "a rectangle (thickness 0 fills; corner_radius rounds it)")
FS_CFIELD(rect, Rect, 0, FS_DNONE, "the rectangle in layer space (or use size)")
FS_CFIELD(size, Vec2, 0, FS_DNONE, "shorthand for rect: [0, 0, w, h]")
FS_CSTYLE(color)
FS_CSTYLE(thickness)
FS_CSTYLE(corner_radius)
FS_CONTENT_END(rect)

FS_CONTENT(circle, "a circle (thickness 0 fills; otherwise a ring)")
FS_CFIELD(center, Vec2, 1, FS_DNONE, "center")
FS_CFIELD(radius, F32, 1, FS_DNONE, "radius in logical units")
FS_CSTYLE(color)
FS_CSTYLE(thickness)
FS_CONTENT_END(circle)

FS_CONTENT(circles, "markers at each point — the point-cloud content")
FS_CFIELD(points, Points, 0, FS_DNONE, "marker centers (exactly one of points | source)")
FS_CFIELD(source, Input, 0, FS_DNONE, "input reference (type sequence<vec2, N>)")
FS_CFIELD(radius, F32, 0, FS_DF32(6), "marker radius in logical units")
FS_CSTYLE(color)
FS_CSTYLE(thickness)
FS_CONTENT_END(circles)

FS_CONTENT(arc, "a circular arc (degrees; 0 deg = +x, clockwise on screen)")
FS_CFIELD(center, Vec2, 1, FS_DNONE, "center")
FS_CFIELD(radius, F32, 1, FS_DNONE, "radius in logical units")
FS_CFIELD(start_deg, F32, 1, FS_DNONE, "start angle in degrees")
FS_CFIELD(end_deg, F32, 1, FS_DNONE, "end angle in degrees")
FS_CSTYLE(color)
FS_CSTYLE(thickness)
FS_CSTYLE(cap)
FS_CONTENT_END(arc)

FS_CONTENT(arrow, "an arrow with a filled triangular head")
FS_CFIELD(from, Vec2, 1, FS_DNONE, "tail")
FS_CFIELD(to, Vec2, 1, FS_DNONE, "tip")
FS_CFIELD(head_size, F32, 0, FS_DF32(0),
          "head length in logical units; 0 = auto from thickness")
FS_CSTYLE(color)
FS_CSTYLE(thickness)
FS_CONTENT_END(arrow)

FS_CONTENT(crosshair, "four ticks around a center with a central gap")
FS_CFIELD(center, Vec2, 1, FS_DNONE, "center")
FS_CFIELD(size, F32, 0, FS_DF32(24), "overall half-extent from the center")
FS_CSTYLE(color)
FS_CSTYLE(thickness)
FS_CONTENT_END(crosshair)

FS_CONTENT(grid, "a debug/calibration grid across the whole layer")
FS_CFIELD(spacing, F32, 0, FS_DF32(100), "line spacing in logical units")
FS_CSTYLE(color)
FS_CSTYLE(thickness)
FS_CONTENT_END(grid)

FS_CONTENT(boxes, "data-driven detection boxes with optional temporal smoothing")
FS_CFIELD(source, Input, 1, FS_DNONE,
          "input reference (type sequence<detection2d, N>)")
FS_CSTYLE(color)
FS_CSTYLE(thickness)
FS_CSTYLE(corner_radius)
FS_CSTYLE(smoothing)
FS_CSTYLE(show_label)
FS_CONTENT_END(boxes)
