#version 450
layout(push_constant) uniform PC {
    vec4 color;
    vec2 viewport;
    float radius;
    float thickness;
} pc;

layout(location = 0) in vec2 v_offset;
layout(location = 0) out vec4 o_color;

void main() {
    float d = length(v_offset) - pc.radius;        // signed distance to the circle edge
    if (pc.thickness > 0.0) {
        d = abs(d) - pc.thickness * 0.5;           // ring centered on the edge
    }
    float alpha = 1.0 - smoothstep(-0.75, 0.75, d);
    if (alpha <= 0.0) {
        discard;
    }
    o_color = vec4(pc.color.rgb, pc.color.a * alpha);
}
