#version 450
layout(push_constant) uniform PC {
    vec4 dst_rect;
    vec2 viewport;
    vec2 pad;
} pc;

layout(set = 0, binding = 0) uniform sampler2D u_tex;
layout(location = 0) in vec2 v_uv;
layout(location = 0) out vec4 o_color;

void main() {
    o_color = vec4(texture(u_tex, v_uv).rgb, 1.0);
}
