// mask_composite.frag — turns an accumulated coverage mask into color:
// out = premul(color) × coverage × fold_alpha (pa.x), blended into the
// target with the layer's blend mode (fixed-function factors).

#version 450
#extension GL_GOOGLE_include_directive : require
#include "push_common.glsl"

layout(set = 0, binding = 0) uniform sampler2D mask_tex;

layout(location = 0) out vec4 o_color;

void main() {
    vec2 uv = (gl_FragCoord.xy - u.rect.xy) / u.rect.zw;
    float cov = texture(mask_tex, uv).r;
    float a = u.color.a * u.pa.x * cov;
    o_color = vec4(u.color.rgb * a, a);
}
