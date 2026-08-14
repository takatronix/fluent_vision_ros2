// quad.vert — a 4-vertex triangle-strip quad covering u.rect in target
// pixels. Fragments use gl_FragCoord (pixel centers), matching the CPU
// reference's x+0.5 convention exactly.

#version 450
#extension GL_GOOGLE_include_directive : require
#include "push_common.glsl"

void main() {
    vec2 c = vec2(float(gl_VertexIndex & 1), float(gl_VertexIndex >> 1));
    vec2 px = u.rect.xy + c * u.rect.zw;
    gl_Position = vec4(px / u.meta.xy * 2.0 - 1.0, 0.0, 1.0);
}
