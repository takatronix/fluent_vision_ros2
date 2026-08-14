// filter.frag — the single-source filter bodies running on the GPU: this
// file #includes the very same filters_shared.h the CPU reference compiles.
// Fullscreen pass over a premultiplied source; bodies see straight rgb.
// meta.w = FS_* mode; pa.xyzw + pb.x = the five parameter slots (already
// Length-scaled by the recorder via plan::scaleFilterValues).

#version 450
#extension GL_GOOGLE_include_directive : require
#include "push_common.glsl"

layout(set = 0, binding = 0) uniform sampler2D src_tex;

layout(location = 0) in vec2 v_uv;
layout(location = 0) out vec4 o_color;

// texelFetch with the CPU reference's exact index math (int truncation +
// clamp) — texture() nearest rounds differently at texel centers, which
// shifts whole blocks in resampling filters like pixelate.
vec3 fs_sample_straight(vec2 uv) {
    ivec2 sz = textureSize(src_tex, 0);
    ivec2 p = clamp(ivec2(uv * vec2(sz)), ivec2(0), sz - 1);
    vec4 c = texelFetch(src_tex, p, 0);
    return c.a > 0.0 ? c.rgb / c.a : vec3(0.0);
}

#define FS_SAMPLE(uv) fs_sample_straight(uv)
#define FS_TEXEL (vec2(1.0) / vec2(textureSize(src_tex, 0)))
#include "fluent_stage/shared/filters_shared.h"

void main() {
    int mode = u_mode();
    vec4 res = fs_apply(mode, v_uv, u.pa.x, u.pa.y, u.pa.z, u.pa.w, u.pb.x);
    // Filters keep the source's alpha shape; FS_OPACITY scales it (the
    // same rule as the CPU reference).
    float src_a = texture(src_tex, v_uv).a;
    float a = mode == FS_OPACITY ? src_a * res.a : src_a;
    o_color = vec4(res.rgb * a, a);
}
