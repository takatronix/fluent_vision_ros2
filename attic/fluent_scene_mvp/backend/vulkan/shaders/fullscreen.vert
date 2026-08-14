#version 450
// Fullscreen pass for image-space effects: two triangles covering NDC.
layout(location = 0) out vec2 v_uv;

void main() {
    const vec2 corners[6] =
        vec2[](vec2(0, 0), vec2(1, 0), vec2(1, 1), vec2(0, 0), vec2(1, 1), vec2(0, 1));
    vec2 c = corners[gl_VertexIndex];
    v_uv = c;
    gl_Position = vec4(c * 2.0 - 1.0, 0.0, 1.0);
}
