#version 450
// Instanced circles: one quad per point, shape evaluated as a signed distance
// field in the fragment shader (anti-aliasing falls out of the distance).
layout(push_constant) uniform PC {
    vec4 color;
    vec2 viewport;
    float radius;
    float thickness;  // 0 = filled disc, > 0 = ring outline
} pc;

layout(location = 0) in vec2 in_center;  // output pixels (per instance)

layout(location = 0) out vec2 v_offset;  // pixel offset from the center

void main() {
    const vec2 corners[6] =
        vec2[](vec2(-1, -1), vec2(1, -1), vec2(1, 1), vec2(-1, -1), vec2(1, 1), vec2(-1, 1));
    float half_size = pc.radius + pc.thickness * 0.5 + 1.5;  // AA margin
    vec2 c = corners[gl_VertexIndex];
    v_offset = c * half_size;
    vec2 px = in_center + v_offset;
    gl_Position = vec4(px / pc.viewport * 2.0 - 1.0, 0.0, 1.0);
}
