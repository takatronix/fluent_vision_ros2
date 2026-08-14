#version 450
// Per-pixel color transform: brightness (add), contrast (scale around 0.5),
// saturation (mix with luma), gamma (power curve).
layout(push_constant) uniform PC {
    float brightness;
    float contrast;
    float saturation;
    float gamma;
} pc;

layout(set = 0, binding = 0) uniform sampler2D u_src;
layout(location = 0) in vec2 v_uv;
layout(location = 0) out vec4 o_color;

void main() {
    vec3 c = texture(u_src, v_uv).rgb;
    c = (c - 0.5) * pc.contrast + 0.5 + pc.brightness;
    float luma = dot(c, vec3(0.2126, 0.7152, 0.0722));
    c = mix(vec3(luma), c, pc.saturation);
    c = pow(clamp(c, 0.0, 1.0), vec3(1.0 / max(pc.gamma, 0.05)));
    o_color = vec4(c, 1.0);
}
