// filters_shared.h — the single source of truth for every single-pass filter
// BODY. Each filter is ONE named function with NAMED parameters, written in
// the GLSL∩C++ subset:
//
//   - the Vulkan backend (Phase L1) #includes this (glslc -I) and the GPU
//     runs these exact bodies;
//   - the CPU reference includes it through shared/glsl_compat.hpp and runs
//     the very same bodies.
//
// Its sibling filters_def.h is the single source of the filter *metadata*
// (public names, parameter names/defaults/units); the parameter order
// declared there IS the slot order p0..p4 used by fs_apply below. Adding a
// filter = one function + one dispatch line here + one block in
// filters_def.h. Mode ids never appear anywhere else.
//
// The includer must provide:
//   FS_SAMPLE(uv)  -> vec3   sample the source image at normalized uv
//   FS_TEXEL       -> vec2   1 / source size
// Function bodies are only compiled when FS_SAMPLE is defined, so metadata
// consumers can include this header without a sampler.

// ---- stable mode ids (shared by GLSL and C++) ------------------------------
const int FS_GRAYSCALE = 1;
const int FS_SEPIA = 2;
const int FS_INVERT = 3;
const int FS_HUE = 4;
const int FS_EXPOSURE = 5;
const int FS_WHITE_BALANCE = 6;
const int FS_LEVELS = 7;
const int FS_THRESHOLD = 8;
const int FS_POSTERIZE = 9;
const int FS_VIGNETTE = 10;
const int FS_PIXELATE = 11;
const int FS_SHARPEN = 12;
const int FS_EMBOSS = 13;
const int FS_EDGE_SOBEL = 14;
const int FS_SKETCH = 15;
const int FS_TOON = 16;
const int FS_SWIRL = 17;
const int FS_SOLARIZE = 18;
const int FS_MOTION_BLUR = 19;
const int FS_ZOOM_BLUR = 20;
const int FS_RGB = 21;
const int FS_OPACITY = 22;
const int FS_HAZE = 23;
const int FS_HALFTONE = 24;
const int FS_COLOR_TRANSFORM = 25;
const int FS_BILATERAL = 26;
const int FS_MEDIAN = 27;
// Separable gaussian blur is multi-pass, so it has a mode id here (the ids
// are the shared vocabulary) but no body in fs_apply — renderers implement
// it as two 1-D passes. Radius is in logical units like every other length.
const int FS_BLUR = 28;

#ifdef FS_SAMPLE

float fs_luma(vec3 c) { return dot(c, vec3(0.2126, 0.7152, 0.0722)); }

float fs_sobel_magnitude(vec2 uv) {
    float tl = fs_luma(FS_SAMPLE(uv + vec2(-FS_TEXEL.x, -FS_TEXEL.y)));
    float tc = fs_luma(FS_SAMPLE(uv + vec2(0.0, -FS_TEXEL.y)));
    float tr = fs_luma(FS_SAMPLE(uv + vec2(FS_TEXEL.x, -FS_TEXEL.y)));
    float ml = fs_luma(FS_SAMPLE(uv + vec2(-FS_TEXEL.x, 0.0)));
    float mr = fs_luma(FS_SAMPLE(uv + vec2(FS_TEXEL.x, 0.0)));
    float bl = fs_luma(FS_SAMPLE(uv + vec2(-FS_TEXEL.x, FS_TEXEL.y)));
    float bc = fs_luma(FS_SAMPLE(uv + vec2(0.0, FS_TEXEL.y)));
    float br = fs_luma(FS_SAMPLE(uv + vec2(FS_TEXEL.x, FS_TEXEL.y)));
    float gx = -tl - 2.0 * ml - bl + tr + 2.0 * mr + br;
    float gy = -tl - 2.0 * tc - tr + bl + 2.0 * bc + br;
    return length(vec2(gx, gy));
}

// ---- color filters ---------------------------------------------------------

vec3 fs_grayscale(vec3 c) { return vec3(fs_luma(c)); }

vec3 fs_sepia(vec3 c, float intensity) {
    vec3 sepia = vec3(dot(c, vec3(0.393, 0.769, 0.189)),
                      dot(c, vec3(0.349, 0.686, 0.168)),
                      dot(c, vec3(0.272, 0.534, 0.131)));
    return mix(c, sepia, clamp(intensity, 0.0, 1.0));
}

vec3 fs_invert(vec3 c) { return 1.0 - c; }

vec3 fs_hue_rotate(vec3 c, float angle_degrees) {
    float a = radians(angle_degrees);
    float yy = dot(c, vec3(0.299, 0.587, 0.114));
    float ii = dot(c, vec3(0.596, -0.274, -0.322));
    float qq = dot(c, vec3(0.211, -0.523, 0.312));
    float hue = atan(qq, ii) + a;
    float chroma = length(vec2(ii, qq));
    ii = chroma * cos(hue);
    qq = chroma * sin(hue);
    return vec3(yy + 0.956 * ii + 0.621 * qq,
                yy - 0.272 * ii - 0.647 * qq,
                yy - 1.106 * ii + 1.703 * qq);
}

vec3 fs_exposure(vec3 c, float stops) { return c * exp2(stops); }

vec3 fs_white_balance(vec3 c, float temperature, float tint) {
    return vec3(c.x * (1.0 + 0.25 * temperature),
                c.y * (1.0 + 0.15 * tint),
                c.z * (1.0 - 0.25 * temperature));
}

vec3 fs_levels(vec3 c, float in_min, float in_max, float gamma) {
    c = clamp((c - vec3(in_min)) * (1.0 / max(in_max - in_min, 1e-4)), 0.0, 1.0);
    return pow(c, vec3(1.0 / max(gamma, 0.05)));
}

vec3 fs_threshold(vec3 c, float threshold) { return vec3(step(threshold, fs_luma(c))); }

vec3 fs_posterize(vec3 c, float levels) {
    float n = max(levels, 2.0);
    return floor(c * n + vec3(0.5)) / n;
}

vec3 fs_vignette(vec3 c, vec2 uv, float start, float end) {
    float d = length(uv - vec2(0.5)) * 1.414;
    return c * (1.0 - smoothstep(start, end, d));
}

vec3 fs_solarize(vec3 c, float threshold) {
    return fs_luma(c) < threshold ? c : 1.0 - c;
}

vec3 fs_rgb(vec3 c, float red, float green, float blue) {
    return c * vec3(red, green, blue);
}

vec3 fs_haze(vec3 c, vec2 uv, float distance_, float slope) {
    float d = distance_ + slope * (1.0 - uv.y);
    return clamp((c - vec3(d)) * (1.0 / max(1.0 - d, 0.05)), 0.0, 1.0);
}

vec3 fs_color_transform(vec3 c, float brightness, float contrast, float saturation, float gamma) {
    c = (c - vec3(0.5)) * contrast + vec3(0.5 + brightness);
    c = mix(vec3(fs_luma(c)), c, saturation);
    return pow(clamp(c, 0.0, 1.0), vec3(1.0 / max(gamma, 0.05)));
}

// ---- kernel filters --------------------------------------------------------

vec3 fs_sharpen(vec2 uv, float strength) {
    vec3 c = FS_SAMPLE(uv);
    vec3 neighbors = FS_SAMPLE(uv + vec2(FS_TEXEL.x, 0.0)) +
                     FS_SAMPLE(uv - vec2(FS_TEXEL.x, 0.0)) +
                     FS_SAMPLE(uv + vec2(0.0, FS_TEXEL.y)) +
                     FS_SAMPLE(uv - vec2(0.0, FS_TEXEL.y));
    return c * (1.0 + 4.0 * strength) - neighbors * strength;
}

vec3 fs_emboss(vec2 uv, float strength) {
    vec3 d = FS_SAMPLE(uv) - FS_SAMPLE(uv - FS_TEXEL);
    return vec3(0.5 + fs_luma(d) * strength * 4.0);
}

vec3 fs_edge_sobel(vec2 uv, float strength) {
    return vec3(clamp(fs_sobel_magnitude(uv) * strength, 0.0, 1.0));
}

vec3 fs_sketch(vec2 uv, float strength) {
    return vec3(1.0 - clamp(fs_sobel_magnitude(uv) * strength, 0.0, 1.0));
}

vec3 fs_toon(vec2 uv, float levels, float edge_threshold) {
    vec3 c = fs_posterize(FS_SAMPLE(uv), levels);
    return fs_sobel_magnitude(uv) > edge_threshold ? vec3(0.05) : c;
}

// Edge-preserving smoothing: 7x7 sampled grid, weights fall off with both
// spatial distance and color distance. This is the denoiser of choice when
// edges must survive.
vec3 fs_bilateral(vec2 uv, float radius, float sigma_color) {
    vec3 center = FS_SAMPLE(uv);
    float range_norm = 2.0 * max(sigma_color, 0.01) * max(sigma_color, 0.01);
    float grid = max(radius / 3.0, 1.0);
    vec3 accum = vec3(0.0);
    float weight_sum = 0.0;
    for (int j = -3; j <= 3; ++j) {
        for (int i = -3; i <= 3; ++i) {
            vec3 s = FS_SAMPLE(uv + FS_TEXEL * vec2(float(i), float(j)) * grid);
            vec3 d = s - center;
            float w = exp(-float(i * i + j * j) / 4.5) * exp(-dot(d, d) / range_norm);
            accum += s * w;
            weight_sum += w;
        }
    }
    return accum * (1.0 / max(weight_sum, 1e-5));
}

// 3x3 median via a sorting network; kills salt-and-pepper noise.
vec3 fs_median(vec2 uv) {
    vec3 p0 = FS_SAMPLE(uv + FS_TEXEL * vec2(-1.0, -1.0));
    vec3 p1 = FS_SAMPLE(uv + FS_TEXEL * vec2(0.0, -1.0));
    vec3 p2 = FS_SAMPLE(uv + FS_TEXEL * vec2(1.0, -1.0));
    vec3 p3 = FS_SAMPLE(uv + FS_TEXEL * vec2(-1.0, 0.0));
    vec3 p4 = FS_SAMPLE(uv);
    vec3 p5 = FS_SAMPLE(uv + FS_TEXEL * vec2(1.0, 0.0));
    vec3 p6 = FS_SAMPLE(uv + FS_TEXEL * vec2(-1.0, 1.0));
    vec3 p7 = FS_SAMPLE(uv + FS_TEXEL * vec2(0.0, 1.0));
    vec3 p8 = FS_SAMPLE(uv + FS_TEXEL * vec2(1.0, 1.0));
    vec3 t;
#define FS_SORT2(a, b) t = min(a, b); b = max(a, b); a = t;
    FS_SORT2(p1, p2) FS_SORT2(p4, p5) FS_SORT2(p7, p8)
    FS_SORT2(p0, p1) FS_SORT2(p3, p4) FS_SORT2(p6, p7)
    FS_SORT2(p1, p2) FS_SORT2(p4, p5) FS_SORT2(p7, p8)
    FS_SORT2(p0, p3) FS_SORT2(p5, p8) FS_SORT2(p4, p7)
    FS_SORT2(p3, p6) FS_SORT2(p1, p4) FS_SORT2(p2, p5)
    FS_SORT2(p4, p7) FS_SORT2(p4, p2) FS_SORT2(p6, p4)
    FS_SORT2(p4, p2)
#undef FS_SORT2
    return p4;
}

// ---- resampling filters (change where we read, then what we read) ----------

vec3 fs_pixelate(vec2 uv, float block_size) {
    float size = max(block_size, 1.0);
    vec2 px = uv / FS_TEXEL;
    return FS_SAMPLE((floor(px / size) * size + vec2(size * 0.5)) * FS_TEXEL);
}

vec3 fs_swirl(vec2 uv, float radius, float angle) {
    vec2 d = uv - vec2(0.5);
    float r = length(d);
    if (r < radius) {
        float k = (radius - r) / max(radius, 1e-4);
        float theta = k * k * angle;
        float cs = cos(theta);
        float sn = sin(theta);
        d = vec2(d.x * cs - d.y * sn, d.x * sn + d.y * cs);
    }
    return FS_SAMPLE(vec2(0.5) + d);
}

vec3 fs_motion_blur(vec2 uv, float angle_degrees, float distance_px) {
    float a = radians(angle_degrees);
    vec2 stride = vec2(cos(a), sin(a)) * FS_TEXEL * (distance_px / 4.0);
    vec3 accum = vec3(0.0);
    for (int i = -4; i <= 4; ++i) {
        accum += FS_SAMPLE(uv + stride * float(i));
    }
    return accum / 9.0;
}

vec3 fs_zoom_blur(vec2 uv, float strength) {
    vec2 stride = (vec2(0.5) - uv) * (strength / 8.0);
    vec3 accum = vec3(0.0);
    for (int i = 0; i < 9; ++i) {
        accum += FS_SAMPLE(uv + stride * float(i));
    }
    return accum / 9.0;
}

vec3 fs_halftone(vec2 uv, float spacing) {
    float s = max(spacing, 2.0);
    vec2 px = uv / FS_TEXEL;
    vec2 cell = (floor(px / s) + vec2(0.5)) * s;
    float l = fs_luma(FS_SAMPLE(cell * FS_TEXEL));
    float radius = (1.0 - l) * s * 0.6;
    return vec3(step(radius, length(px - cell)));
}

// ---- dispatch --------------------------------------------------------------
// The only place slot values (p0..p4, in table order) meet parameter names.
// Returns rgba; alpha is 1 except for filters that define it.

vec4 fs_apply(int mode, vec2 uv, float p0, float p1, float p2, float p3, float p4) {
    vec3 c = FS_SAMPLE(uv);
    float alpha = 1.0;
    if (mode == FS_GRAYSCALE)            c = fs_grayscale(c);
    else if (mode == FS_SEPIA)           c = fs_sepia(c, p0);
    else if (mode == FS_INVERT)          c = fs_invert(c);
    else if (mode == FS_HUE)             c = fs_hue_rotate(c, p0);
    else if (mode == FS_EXPOSURE)        c = fs_exposure(c, p0);
    else if (mode == FS_WHITE_BALANCE)   c = fs_white_balance(c, p0, p1);
    else if (mode == FS_LEVELS)          c = fs_levels(c, p0, p1, p2);
    else if (mode == FS_THRESHOLD)       c = fs_threshold(c, p0);
    else if (mode == FS_POSTERIZE)       c = fs_posterize(c, p0);
    else if (mode == FS_VIGNETTE)        c = fs_vignette(c, uv, p0, p1);
    else if (mode == FS_PIXELATE)        c = fs_pixelate(uv, p0);
    else if (mode == FS_SHARPEN)         c = fs_sharpen(uv, p0);
    else if (mode == FS_EMBOSS)          c = fs_emboss(uv, p0);
    else if (mode == FS_EDGE_SOBEL)      c = fs_edge_sobel(uv, p0);
    else if (mode == FS_SKETCH)          c = fs_sketch(uv, p0);
    else if (mode == FS_TOON)            c = fs_toon(uv, p0, p1);
    else if (mode == FS_SWIRL)           c = fs_swirl(uv, p0, p1);
    else if (mode == FS_SOLARIZE)        c = fs_solarize(c, p0);
    else if (mode == FS_MOTION_BLUR)     c = fs_motion_blur(uv, p0, p1);
    else if (mode == FS_ZOOM_BLUR)       c = fs_zoom_blur(uv, p0);
    else if (mode == FS_RGB)             c = fs_rgb(c, p0, p1, p2);
    else if (mode == FS_OPACITY)         alpha = clamp(p0, 0.0, 1.0);
    else if (mode == FS_HAZE)            c = fs_haze(c, uv, p0, p1);
    else if (mode == FS_HALFTONE)        c = fs_halftone(uv, p0);
    else if (mode == FS_COLOR_TRANSFORM) c = fs_color_transform(c, p0, p1, p2, p3);
    else if (mode == FS_BILATERAL)       c = fs_bilateral(uv, p0, p1);
    else if (mode == FS_MEDIAN)          c = fs_median(uv);
    return vec4(clamp(c, 0.0, 1.0), alpha);
}

#endif  // FS_SAMPLE

