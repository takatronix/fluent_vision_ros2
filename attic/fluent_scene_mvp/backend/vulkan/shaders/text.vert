#version 450
// Glyph quads are expanded on the CPU (6 vertices per glyph); the shadow pass
// re-draws the same vertex range with an offset and shadow color.
layout(push_constant) uniform PC {
    vec4 color;
    vec2 viewport;
    vec2 offset;
} pc;

layout(location = 0) in vec2 in_pos;  // output pixels
layout(location = 1) in vec2 in_uv;   // normalized atlas coords

layout(location = 0) out vec2 v_uv;

void main() {
    v_uv = in_uv;
    vec2 px = in_pos + pc.offset;
    gl_Position = vec4(px / pc.viewport * 2.0 - 1.0, 0.0, 1.0);
}
