#version 450
// One direction of a separable Gaussian blur. The kernel is statically
// bounded (13 taps); sigma follows the declared radius. Run twice
// (horizontal then vertical) for the full effect.
layout(push_constant) uniform PC {
    vec2 texel;      // 1/size of the source, oriented along the blur direction
    float radius;    // declared blur radius in pixels (clamped by the registry)
    float pad0;
} pc;

layout(set = 0, binding = 0) uniform sampler2D u_src;
layout(location = 0) in vec2 v_uv;
layout(location = 0) out vec4 o_color;

void main() {
    float sigma = max(pc.radius * 0.5, 0.3);
    float two_sigma2 = 2.0 * sigma * sigma;
    vec3 sum = vec3(0.0);
    float weight_sum = 0.0;
    for (int i = -6; i <= 6; ++i) {
        float offset = float(i) * max(pc.radius / 6.0, 0.001);
        float weight = exp(-(offset * offset) / two_sigma2);
        sum += texture(u_src, v_uv + pc.texel * offset).rgb * weight;
        weight_sum += weight;
    }
    o_color = vec4(sum / weight_sum, 1.0);
}
