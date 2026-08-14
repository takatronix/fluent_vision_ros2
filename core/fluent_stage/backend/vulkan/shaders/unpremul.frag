// unpremul.frag — final conversion: premultiplied float target → straight
// alpha for the RGBA8 readback surface (the same math as the CPU
// reference's output loop; UNORM store rounds).

#version 450
#extension GL_GOOGLE_include_directive : require
#include "push_common.glsl"

layout(set = 0, binding = 0) uniform sampler2D src_tex;

layout(location = 0) in vec2 v_uv;
layout(location = 0) out vec4 o_color;

void main() {
    vec4 p = texture(src_tex, v_uv);
    float a = clamp(p.a, 0.0, 1.0);
    vec3 rgb = a > 0.0 ? clamp(p.rgb / a, vec3(0.0), vec3(1.0)) : vec3(0.0);
    o_color = vec4(rgb, a);
}
