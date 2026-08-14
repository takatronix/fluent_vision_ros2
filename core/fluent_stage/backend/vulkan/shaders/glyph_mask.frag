// glyph_mask.frag — one text glyph's coverage from the R8 atlas into the
// mask (MAX blending; glyphs union like every other multi-part content).
// pa = glyph rect in local units (x, y, w, h); pb = atlas uv (u0, v0, u1, v1).

#version 450
#extension GL_GOOGLE_include_directive : require
#include "push_common.glsl"

layout(set = 0, binding = 0) uniform sampler2D atlas_tex;

layout(location = 0) out float o_cov;

void main() {
    vec2 local = u_local(gl_FragCoord.xy);
    vec2 f = (local - u.pa.xy) / u.pa.zw;
    if (f.x < 0.0 || f.y < 0.0 || f.x >= 1.0 || f.y >= 1.0) {
        o_cov = 0.0;
        return;
    }
    vec2 uv = u.pb.xy + f * (u.pb.zw - u.pb.xy);
    o_cov = texture(atlas_tex, uv).r;
}
