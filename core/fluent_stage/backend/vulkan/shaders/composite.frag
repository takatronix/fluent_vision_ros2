// composite.frag — draws an offscreen buffer into its parent target
// through the layer transform: bilinear sample (transparent border),
// optional rounded-rect mask evaluated crisply in local space, optional
// tint mode for shadows (color × sampled alpha).
//
// meta.w flags: bit0 = rounded-rect mask, bit1 = tint (shadow).
// pa = (origin_x, origin_y, scale, opacity)  — buffer pixel = (local − origin)·scale
// pb = mask rounded rect (cx, cy, hx, hy) in local units
// px2 = (mask corner radius, unused, unused, unused)
// color = tint color (straight; alpha = shadow opacity × fold)

#version 450
#extension GL_GOOGLE_include_directive : require
#include "push_common.glsl"
#include "fluent_stage/shared/shapes_shared.h"

layout(set = 0, binding = 0) uniform sampler2D buf_tex;

layout(location = 0) out vec4 o_color;

void main() {
    vec2 local = u_local(gl_FragCoord.xy);
    vec2 p = (local - u.pa.xy) * u.pa.z;
    vec4 smp = texture(buf_tex, p / vec2(textureSize(buf_tex, 0)));
    int flags = u_mode();
    if ((flags & 2) != 0) {  // tint: shadow from the buffer's silhouette
        float a = u.color.a * u.pa.w * smp.a;
        o_color = vec4(u.color.rgb * a, a);
        return;
    }
    float a = u.pa.w;
    if ((flags & 1) != 0) {  // rounded-rect clip, evaluated vector-crisp
        a *= sd_fill(sd_rounded_rect(local, u.pb.xy, u.pb.zw, u.px2.x), u.meta.z);
    }
    o_color = smp * a;
}
