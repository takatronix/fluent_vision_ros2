#version 450
layout(push_constant) uniform PC {
    vec4 color;
    vec2 viewport;
    float thickness;
    float pad;
} pc;

layout(location = 0) in vec2 v_local;
layout(location = 1) in vec2 v_size;
layout(location = 0) out vec4 o_color;

void main() {
    vec2 d = min(v_local, v_size - v_local);
    float edge = min(d.x, d.y);
    if (edge > pc.thickness) {
        discard;
    }
    o_color = pc.color;
}
