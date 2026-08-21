#version 450
layout(push_constant) uniform PC {
    vec4 color;
    vec2 viewport;
    float thickness;
    float pad;
} pc;

layout(location = 0) in vec2 v_pos;
layout(location = 1) in vec4 v_seg;
layout(location = 0) out vec4 o_color;

void main() {
    vec2 a = v_seg.xy;
    vec2 ba = v_seg.zw - a;
    float h = clamp(dot(v_pos - a, ba) / max(dot(ba, ba), 1e-6), 0.0, 1.0);
    float d = length(v_pos - a - ba * h) - pc.thickness * 0.5;  // capsule SDF
    float alpha = 1.0 - smoothstep(-0.75, 0.75, d);
    if (alpha <= 0.0) {
        discard;
    }
    o_color = vec4(pc.color.rgb, pc.color.a * alpha);
}
