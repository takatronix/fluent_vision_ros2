#version 450
layout(push_constant) uniform PC {
    vec4 color;
    vec2 viewport;
    vec2 offset;
} pc;

layout(set = 0, binding = 0) uniform sampler2D u_atlas;
layout(location = 0) in vec2 v_uv;
layout(location = 0) out vec4 o_color;

void main() {
    float coverage = texture(u_atlas, v_uv).r;
    o_color = vec4(pc.color.rgb, pc.color.a * coverage);
}
