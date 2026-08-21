#version 450
// Instanced polyline segments: one oriented quad per segment; the fragment
// shader evaluates a capsule SDF (distance to the segment), which yields round
// caps and joins for free.
layout(push_constant) uniform PC {
    vec4 color;
    vec2 viewport;
    float thickness;
    float pad;
} pc;

layout(location = 0) in vec4 in_segment;  // a.xy, b.xy in output pixels (per instance)

layout(location = 0) out vec2 v_pos;  // pixel position of this fragment
layout(location = 1) out vec4 v_seg;  // segment endpoints (constant per instance)

void main() {
    vec2 a = in_segment.xy;
    vec2 b = in_segment.zw;
    vec2 axis = b - a;
    float len = max(length(axis), 1e-4);
    vec2 dir = axis / len;
    vec2 nrm = vec2(-dir.y, dir.x);
    float half_w = pc.thickness * 0.5 + 1.5;  // AA + round-cap margin

    const vec2 params[6] = vec2[](vec2(0, -1), vec2(1, -1), vec2(1, 1),
                                  vec2(0, -1), vec2(1, 1), vec2(0, 1));
    vec2 p = params[gl_VertexIndex];
    vec2 base = mix(a - dir * half_w, b + dir * half_w, p.x);
    vec2 px = base + nrm * (p.y * half_w);

    v_pos = px;
    v_seg = in_segment;
    gl_Position = vec4(px / pc.viewport * 2.0 - 1.0, 0.0, 1.0);
}
