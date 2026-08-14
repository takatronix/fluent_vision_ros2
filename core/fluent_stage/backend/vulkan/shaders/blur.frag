// blur.frag — one direction of the separable gaussian, on premultiplied
// pixels, with the identical weight formula as the CPU reference (sigma =
// radius/2, half-width = ceil(2.5 sigma), normalized in-loop).
// pa = (dir_x, dir_y, sigma, half_width).

#version 450
#extension GL_GOOGLE_include_directive : require
#include "push_common.glsl"

layout(set = 0, binding = 0) uniform sampler2D src_tex;

layout(location = 0) in vec2 v_uv;
layout(location = 0) out vec4 o_color;

void main() {
    vec2 texel = vec2(1.0) / vec2(textureSize(src_tex, 0));
    float sigma = max(u.pa.z, 1e-4);
    int half_width = int(u.pa.w + 0.5);
    vec4 accum = vec4(0.0);
    float weight_sum = 0.0;
    for (int i = -half_width; i <= half_width; ++i) {
        float w = exp(-0.5 * (float(i) / sigma) * (float(i) / sigma));
        accum += texture(src_tex, v_uv + u.pa.xy * texel * float(i)) * w;
        weight_sum += w;
    }
    o_color = accum / weight_sum;
}
