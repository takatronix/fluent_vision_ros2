// attributes_def.h — the single source of truth for layer-attribute METADATA
// in the Scene vocabulary (§6.2 of the design): the YAML key, the value kind,
// whether §9 implicit animation can interpolate it, and a one-line summary.
//
// scene/schema.hpp includes this file with different macro definitions to
// derive the AttrId enum, the runtime spec table (validation, describe
// --json, fmt ordering), and nothing else — behavior stays in the compiler,
// which applies each attribute through the same Layer setter a C++ author
// calls, so defaults and semantics live once, in layer.hpp.
//
// Structural keys (id, role, protected, content, sublayers, filters,
// transition, states) are not attributes and are handled by the parser
// directly.
//
// Kinds are ValueKind names from scene/schema.hpp. `scale` is Vec2 but the
// parser promotes a bare scalar s to [s, s]. Shadow/Border map fields are
// described by shadowFields()/borderFields() in schema.hpp, whose defaults
// are read from the C++ structs themselves.
//
// No include guard: included multiple times by design.
//
//   FS_ATTR(Ident, yaml_name, KIND, animatable, summary)

FS_ATTR(Bounds, bounds, Rect, 1, "the layer's own coordinate space rectangle")
FS_ATTR(Position, position, Vec2, 1,
        "where the anchor lands in the parent's space (CALayer.position)")
FS_ATTR(Anchor, anchor, Vec2, 0,
        "pivot for rotation/scale as a normalized point in bounds; default [0.5, 0.5]")
FS_ATTR(Frame, frame, Rect, 1,
        "sugar: axis-aligned placement in the parent; derives bounds.size and position")
FS_ATTR(Rotation, rotation, F32, 1,
        "rotation in degrees around the anchor (clockwise on screen)")
FS_ATTR(Scale, scale, Vec2, 1,
        "scale around the anchor; negative mirrors; a bare scalar s means [s, s]")
FS_ATTR(Transform, transform, Mat23, 0,
        "full affine [a, b, c, d, tx, ty] composed with the simple attributes")
FS_ATTR(Opacity, opacity, F32, 1, "opacity 0-1; on a group applies to the composited result")
FS_ATTR(Hidden, hidden, Bool, 0, "skips rendering without removing from the tree")
FS_ATTR(MasksToBounds, masks_to_bounds, Bool, 0,
        "clips content and sublayers to bounds (corner_radius included); default false")
FS_ATTR(CornerRadius, corner_radius, F32, 0,
        "rounds the layer clip shape, background, border, and rect/boxes content")
FS_ATTR(Background, background, Color, 0,
        "a background plate behind the content, filling bounds; default transparent")
FS_ATTR(Blend, blend, EnumBlend, 0,
        "blend against what is behind: normal | add | multiply | screen")
FS_ATTR(Shadow, shadow, MapShadow, 0,
        "drop shadow from the layer silhouette; {} alone looks right")
FS_ATTR(Border, border, MapBorder, 0,
        "border stroked along the clip shape (corner radius included)")
