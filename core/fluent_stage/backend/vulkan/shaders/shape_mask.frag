// shape_mask.frag — evaluates one shape part's SDF coverage into the R16F
// mask (MAX blending unions overlapping parts, exactly like the CPU
// reference's coverage mask). The shape math is the shared single source.

#version 450
#extension GL_GOOGLE_include_directive : require
#include "push_common.glsl"
#include "fluent_stage/shared/shapes_shared.h"
#include "shape_modes.h"

layout(std430, set = 0, binding = 0) readonly buffer Points { vec2 pts[]; };

layout(location = 0) out float o_cov;

// Signed distance to a polygon (negative inside, any winding) — the loop is
// backend structure, the per-edge math mirrors sd_segment (see
// shapes_shared.h's note on variable-length shapes).
float polygon_distance(int offset, int count, vec2 p) {
    float d2 = 1e30;
    float sgn = 1.0;
    for (int i = 0, j = count - 1; i < count; j = i++) {
        vec2 vi = pts[offset + i];
        vec2 vj = pts[offset + j];
        vec2 e = vj - vi;
        vec2 w = p - vi;
        float t = clamp(dot(w, e) / max(dot(e, e), 1e-12), 0.0, 1.0);
        vec2 b = w - e * t;
        d2 = min(d2, dot(b, b));
        bool c0 = p.y >= vi.y;
        bool c1 = p.y < vj.y;
        bool c2 = e.x * w.y > e.y * w.x;
        if ((c0 && c1 && c2) || (!c0 && !c1 && !c2)) {
            sgn = -sgn;
        }
    }
    return sgn * sqrt(d2);
}

void main() {
    vec2 local = u_local(gl_FragCoord.xy);
    float aa = u.meta.z;
    int mode = u_mode();
    float cov = 0.0;
    if (mode == SM_ROUNDED_RECT) {
        float d = sd_rounded_rect(local, u.pa.xy, u.pa.zw, u.pb.x);
        cov = u.pb.y > 0.0 ? sd_stroke(d, u.pb.y, aa) : sd_fill(d, aa);
    } else if (mode == SM_CIRCLE) {
        float d = sd_circle(local, u.pa.xy, u.pa.z);
        cov = u.pa.w > 0.0 ? sd_stroke(d, u.pa.w, aa) : sd_fill(d, aa);
    } else if (mode == SM_SEGMENT_ROUND) {
        cov = sd_stroke(sd_segment(local, u.pa.xy, u.pa.zw), u.pb.x, aa);
    } else if (mode == SM_SEGMENT_BUTT) {
        cov = sd_fill(sd_segment_butt(local, u.pa.xy, u.pa.zw, u.pb.x), aa);
    } else if (mode == SM_ARC) {
        float d = sd_arc(local, u.pa.xy, u.pa.z, u.pa.w, u.pb.x);
        cov = sd_fill(d - u.pb.y * 0.5, aa);
    } else if (mode == SM_TRIANGLE) {
        cov = sd_fill(sd_triangle(local, u.pa.xy, u.pa.zw, u.pb.xy), aa);
    } else if (mode == SM_GRID) {
        float lines = sd_stroke(sd_grid(local, u.pa.x), u.pa.y, aa);
        float inside = sd_fill(sd_rounded_rect(local, u.pb.xy, u.pb.zw, 0.0), aa);
        cov = lines * inside;
    } else if (mode == SM_POLYGON) {
        cov = sd_fill(polygon_distance(int(u.pa.x + 0.5), int(u.pa.y + 0.5), local), aa);
    }
    o_cov = cov;
}
