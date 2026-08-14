// shape_color.frag — direct single-part rounded rect (layer backgrounds
// and borders): SDF → premultiplied color in one pass, like the CPU
// reference's drawRoundedRect. pa=(cx,cy,hx,hy) pb=(corner, stroke_w);
// px2.x = fold alpha.

#version 450
#extension GL_GOOGLE_include_directive : require
#include "push_common.glsl"
#include "fluent_stage/shared/shapes_shared.h"

layout(location = 0) out vec4 o_color;

void main() {
    vec2 local = u_local(gl_FragCoord.xy);
    float aa = u.meta.z;
    float d = sd_rounded_rect(local, u.pa.xy, u.pa.zw, u.pb.x);
    float cov = u.pb.y > 0.0 ? sd_stroke(d, u.pb.y, aa) : sd_fill(d, aa);
    float a = u.color.a * u.px2.x * cov;
    o_color = vec4(u.color.rgb * a, a);
}
