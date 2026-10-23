#version 450
// Fitted image quad; destination rectangle comes from the CPU-side fit
// computation (contain/cover/fill), so the shader stays trivial.
layout(push_constant) uniform PC {
    vec4 dst_rect;   // x, y, w, h in output pixels
    vec2 viewport;   // output size in pixels
    vec2 pad;
} pc;

layout(location = 0) out vec2 v_uv;

void main() {
    const vec2 corners[6] =
        vec2[](vec2(0, 0), vec2(1, 0), vec2(1, 1), vec2(0, 0), vec2(1, 1), vec2(0, 1));
    vec2 c = corners[gl_VertexIndex];
    v_uv = c;
    vec2 px = pc.dst_rect.xy + c * pc.dst_rect.zw;
    gl_Position = vec4(px / pc.viewport * 2.0 - 1.0, 0.0, 1.0);
}
