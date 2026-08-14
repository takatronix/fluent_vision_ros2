#pragma once

// C++ implementations of the GLSL vector types and intrinsics used by
// fluent_stage/shared/filters_shared.h, so every filter body is written exactly once in
// the GLSL∩C++ subset and compiled for both the GPU and the CPU reference.
// Only the operations the shared filters use are provided — this is a
// compatibility shim, not a math library.

#include <cmath>

namespace fluent_stage {
namespace glsl {

struct vec2 {
    float x = 0, y = 0;
    vec2() = default;
    vec2(float v) : x(v), y(v) {}
    vec2(float x_, float y_) : x(x_), y(y_) {}
};

struct vec3 {
    float x = 0, y = 0, z = 0;
    vec3() = default;
    vec3(float v) : x(v), y(v), z(v) {}
    vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

struct vec4 {
    float x = 0, y = 0, z = 0, w = 0;
    vec4() = default;
    vec4(float v) : x(v), y(v), z(v), w(v) {}
    vec4(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}
    vec4(const vec3& v, float w_) : x(v.x), y(v.y), z(v.z), w(w_) {}
};

inline vec2 operator+(vec2 a, vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline vec2 operator-(vec2 a, vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline vec2 operator*(vec2 a, vec2 b) { return {a.x * b.x, a.y * b.y}; }
inline vec2 operator*(vec2 a, float s) { return {a.x * s, a.y * s}; }
inline vec2 operator*(float s, vec2 a) { return a * s; }
inline vec2 operator/(vec2 a, vec2 b) { return {a.x / b.x, a.y / b.y}; }
inline vec2 operator/(vec2 a, float s) { return {a.x / s, a.y / s}; }

inline vec3 operator+(vec3 a, vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline vec3 operator-(vec3 a, vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline vec3 operator-(float s, vec3 a) { return {s - a.x, s - a.y, s - a.z}; }
inline vec3 operator*(vec3 a, vec3 b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; }
inline vec3 operator*(vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline vec3 operator*(float s, vec3 a) { return a * s; }
inline vec3 operator/(vec3 a, float s) { return {a.x / s, a.y / s, a.z / s}; }
inline vec3& operator+=(vec3& a, vec3 b) { a = a + b; return a; }

inline float dot(vec2 a, vec2 b) { return a.x * b.x + a.y * b.y; }
inline float dot(vec3 a, vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline float length(vec2 a) { return std::sqrt(dot(a, a)); }
inline float length(vec3 a) { return std::sqrt(dot(a, a)); }

inline float clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline vec2 clamp(vec2 v, vec2 lo, vec2 hi) {
    return {clamp(v.x, lo.x, hi.x), clamp(v.y, lo.y, hi.y)};
}
inline vec3 clamp(vec3 v, float lo, float hi) {
    return {clamp(v.x, lo, hi), clamp(v.y, lo, hi), clamp(v.z, lo, hi)};
}
inline float mix(float a, float b, float t) { return a + (b - a) * t; }
inline vec3 mix(vec3 a, vec3 b, float t) { return a + (b - a) * t; }
inline float step(float edge, float v) { return v < edge ? 0.0f : 1.0f; }
inline float smoothstep(float e0, float e1, float v) {
    const float t = clamp((v - e0) / (e1 - e0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}
inline float floorf_(float v) { return std::floor(v); }
inline vec3 floor(vec3 v) { return {std::floor(v.x), std::floor(v.y), std::floor(v.z)}; }
inline vec2 floor(vec2 v) { return {std::floor(v.x), std::floor(v.y)}; }
inline float pow_(float b, float e) { return std::pow(b, e); }
inline vec3 pow(vec3 b, vec3 e) {
    return {std::pow(b.x, e.x), std::pow(b.y, e.y), std::pow(b.z, e.z)};
}
inline float exp_(float v) { return std::exp(v); }
inline float exp2_(float v) { return std::exp2(v); }
inline float atan_(float y, float x) { return std::atan2(y, x); }
inline float cos_(float v) { return std::cos(v); }
inline float sin_(float v) { return std::sin(v); }
inline float min_(float a, float b) { return a < b ? a : b; }
inline float max_(float a, float b) { return a > b ? a : b; }
inline vec3 min_(vec3 a, vec3 b) { return {min_(a.x, b.x), min_(a.y, b.y), min_(a.z, b.z)}; }
inline vec3 max_(vec3 a, vec3 b) { return {max_(a.x, b.x), max_(a.y, b.y), max_(a.z, b.z)}; }
inline float radians_(float deg) { return deg * 0.01745329252f; }
inline float abs_(float v) { return v < 0 ? -v : v; }
inline vec2 abs_(vec2 v) { return {abs_(v.x), abs_(v.y)}; }
inline vec2 min2_(vec2 a, vec2 b) { return {min_(a.x, b.x), min_(a.y, b.y)}; }
inline vec2 max2_(vec2 a, vec2 b) { return {max_(a.x, b.x), max_(a.y, b.y)}; }

// GLSL spellings for the names that collide with <cmath>.
inline float pow(float b, float e) { return pow_(b, e); }
inline float exp(float v) { return exp_(v); }
inline float exp2(float v) { return exp2_(v); }
inline float atan(float y, float x) { return atan_(y, x); }
inline float cos(float v) { return cos_(v); }
inline float sin(float v) { return sin_(v); }
inline float min(float a, float b) { return min_(a, b); }
inline float max(float a, float b) { return max_(a, b); }
inline vec3 min(vec3 a, vec3 b) { return min_(a, b); }
inline vec3 max(vec3 a, vec3 b) { return max_(a, b); }
inline float radians(float deg) { return radians_(deg); }
inline float floor(float v) { return floorf_(v); }
inline float abs(float v) { return abs_(v); }
inline float sqrt(float v) { return std::sqrt(v); }
inline vec2 abs(vec2 v) { return abs_(v); }
inline vec2 min(vec2 a, vec2 b) { return min2_(a, b); }
inline vec2 max(vec2 a, vec2 b) { return max2_(a, b); }

}  // namespace glsl
}  // namespace fluent_stage
