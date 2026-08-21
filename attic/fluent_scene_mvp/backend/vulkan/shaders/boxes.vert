#version 450
// Instanced detection boxes: one quad per instance, outline drawn in the
// fragment shader from the pixel-space distance to the box edge.
layout(push_constant) uniform PC {
    vec4 color;
    vec2 viewport;
    float thickness;
    float pad;
} pc;

layout(location = 0) in vec4 in_bbox;  // x, y, w, h in output pixels (per instance)

layout(location = 0) out vec2 v_local;  // pixel position inside the box
layout(location = 1) out vec2 v_size;   // box size (constant per instance)

void main() {
    const vec2 corners[6] =
        vec2[](vec2(0, 0), vec2(1, 0), vec2(1, 1), vec2(0, 0), vec2(1, 1), vec2(0, 1));
    vec2 c = corners[gl_VertexIndex];
    v_local = c * in_bbox.zw;
    v_size = in_bbox.zw;
    vec2 px = in_bbox.xy + c * in_bbox.zw;
    gl_Position = vec4(px / pc.viewport * 2.0 - 1.0, 0.0, 1.0);
}
