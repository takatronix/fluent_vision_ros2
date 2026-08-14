// image.frag — image content: inverse-map the fragment into layer space,
// reject outside the destination rect, sample the (premultiplied) source
// window bilinearly with the CPU reference's window clamp.
// pa = dest rect (x,y,w,h) in local units; pb = source window in pixels
// (x,y,w,h); px2 = (tex_w, tex_h, fold_alpha, 0).

#version 450
#extension GL_GOOGLE_include_directive : require
#include "push_common.glsl"

layout(set = 0, binding = 0) uniform sampler2D image_tex;

layout(location = 0) out vec4 o_color;

// One bilinear tap with the CPU reference's integer window clamp.
vec4 tap(int sx, int sy) {
    sx = clamp(sx, int(u.pb.x), int(u.pb.x + u.pb.z) - 1);
    sy = clamp(sy, int(u.pb.y), int(u.pb.y + u.pb.w) - 1);
    return texelFetch(image_tex, ivec2(sx, sy), 0);
}

void main() {
    vec2 local = u_local(gl_FragCoord.xy);
    if (local.x < u.pa.x || local.y < u.pa.y || local.x >= u.pa.x + u.pa.z ||
        local.y >= u.pa.y + u.pa.w) {
        o_color = vec4(0.0);
        return;
    }
    // Manual bilinear, mirroring the CPU reference tap for tap: hardware
    // filtering quantizes the fraction and clamps differently at the
    // source-window edges.
    vec2 srcf = u.pb.xy + (local - u.pa.xy) * (u.pb.zw / u.pa.zw) - 0.5;
    vec2 base = floor(srcf);
    vec2 t = srcf - base;
    int sx0 = int(base.x);
    int sy0 = int(base.y);
    vec4 smp = (tap(sx0, sy0) * (1.0 - t.x) + tap(sx0 + 1, sy0) * t.x) * (1.0 - t.y) +
               (tap(sx0, sy0 + 1) * (1.0 - t.x) + tap(sx0 + 1, sy0 + 1) * t.x) * t.y;
    o_color = smp * u.px2.z;
}
