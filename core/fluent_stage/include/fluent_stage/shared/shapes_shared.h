// shapes_shared.h — the single source of truth for every shape's signed
// distance function, written in the GLSL∩C++ subset (like filters_shared.h):
//
//   - the CPU reference includes it through shared/glsl_compat.hpp;
//   - the Vulkan backend (Phase L1) will #include it into its shape shaders.
//
// Conventions:
//   - all coordinates are in the layer's local logical units, top-left
//     origin, +y down (the one coordinate system of the whole library);
//   - distances are signed: negative inside, positive outside, zero on the
//     edge — so `fill coverage` and `stroke coverage` (below) are the only
//     two ways a shape becomes pixels, and antialiasing is uniform;
//   - angles are radians here; the public API speaks degrees and converts
//     at the boundary (0° = +x, increasing clockwise on screen because +y
//     points down);
//   - variable-length shapes (polyline, polygon, circles) are loops the
//     renderer drives over these per-element functions; the per-element
//     math lives here, the iteration is backend structure.

// ---- coverage: distance -> alpha -------------------------------------------

// Fill coverage with an antialiasing footprint `aa` (the size of one output
// pixel in local units). 1 well inside, 0 well outside, smooth over ~1px.
float sd_fill(float d, float aa) { return clamp(0.5 - d / max(aa, 1e-6), 0.0, 1.0); }

// Stroke coverage: a band of `width` centered on the d=0 contour.
float sd_stroke(float d, float width, float aa) {
    return sd_fill(abs(d) - width * 0.5, aa);
}

// ---- primitive distances ---------------------------------------------------

float sd_circle(vec2 p, vec2 center, float radius) {
    return length(p - center) - radius;
}

// Rectangle centered at `center` with half extents `half_size` and corner
// radius `radius` (callers clamp radius to min(half_size)).
float sd_rounded_rect(vec2 p, vec2 center, vec2 half_size, float radius) {
    vec2 q = abs(p - center) - (half_size - vec2(radius, radius));
    return length(max(q, vec2(0.0, 0.0))) + min(max(q.x, q.y), 0.0) - radius;
}

// Distance to the segment a–b (stroking this with sd_stroke gives a capsule:
// round caps and round joints for free).
float sd_segment(vec2 p, vec2 a, vec2 b) {
    vec2 pa = p - a;
    vec2 ba = b - a;
    float h = clamp(dot(pa, ba) / max(dot(ba, ba), 1e-8), 0.0, 1.0);
    return length(pa - ba * h);
}

// Oriented box of half width `half_width` along segment a–b: a butt-capped
// stroke (ends squared exactly at the endpoints). Already a filled distance —
// render with sd_fill, not sd_stroke.
float sd_segment_butt(vec2 p, vec2 a, vec2 b, float half_width) {
    float l = max(length(b - a), 1e-8);
    vec2 dirv = (b - a) * (1.0 / l);
    vec2 q = p - (a + b) * 0.5;
    q = abs(vec2(dot(q, dirv), dot(q, vec2(-dirv.y, dirv.x))));
    q = q - vec2(l * 0.5, half_width);
    return length(max(q, vec2(0.0, 0.0))) + min(max(q.x, q.y), 0.0);
}

// Distance to the centerline of a circular arc around `center` from angle
// `a0` to `a1` (radians, a1 > a0, sweep <= 2π). Stroking gives round caps.
float sd_arc(vec2 p, vec2 center, float radius, float a0, float a1) {
    vec2 d = p - center;
    float sweep = a1 - a0;
    float rel = atan(d.y, d.x) - a0;
    rel = rel - floor(rel / 6.28318530718) * 6.28318530718;
    if (rel <= sweep) {
        return abs(length(d) - radius);
    }
    vec2 e0 = center + vec2(cos(a0), sin(a0)) * radius;
    vec2 e1 = center + vec2(cos(a1), sin(a1)) * radius;
    return min(length(p - e0), length(p - e1));
}

float sd_cross2(vec2 a, vec2 b) { return a.x * b.y - a.y * b.x; }

// Signed distance to a triangle (negative inside), any winding.
float sd_triangle(vec2 p, vec2 p0, vec2 p1, vec2 p2) {
    vec2 e0 = p1 - p0;
    vec2 e1 = p2 - p1;
    vec2 e2 = p0 - p2;
    vec2 v0 = p - p0;
    vec2 v1 = p - p1;
    vec2 v2 = p - p2;
    vec2 pq0 = v0 - e0 * clamp(dot(v0, e0) / max(dot(e0, e0), 1e-8), 0.0, 1.0);
    vec2 pq1 = v1 - e1 * clamp(dot(v1, e1) / max(dot(e1, e1), 1e-8), 0.0, 1.0);
    vec2 pq2 = v2 - e2 * clamp(dot(v2, e2) / max(dot(e2, e2), 1e-8), 0.0, 1.0);
    float s = sd_cross2(e0, e2) < 0.0 ? -1.0 : 1.0;
    vec2 dmin = min(min(vec2(dot(pq0, pq0), s * sd_cross2(v0, e0)),
                        vec2(dot(pq1, pq1), s * sd_cross2(v1, e1))),
                    vec2(dot(pq2, pq2), s * sd_cross2(v2, e2)));
    return -sqrt(dmin.x) * (dmin.y > 0.0 ? 1.0 : -1.0);
}

// Distance to the nearest line of an axis-aligned grid with the given
// spacing (lines pass through the local origin).
float sd_grid(vec2 p, float spacing) {
    float s = max(spacing, 1e-3);
    vec2 q = p - floor(p * (1.0 / s) + vec2(0.5, 0.5)) * s;
    return min(abs(q.x), abs(q.y));
}
