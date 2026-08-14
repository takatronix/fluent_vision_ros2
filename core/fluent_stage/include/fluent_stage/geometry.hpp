#pragma once

/// \file geometry.hpp
/// \brief Value types for the fluent_stage coordinate world.
///
/// One coordinate system everywhere: **origin at the top-left, +x right,
/// +y down**, in the Stage's logical units. There is no flipped variant and
/// no normalized (0–1) variant anywhere in the public API.

#include <cmath>

namespace fluent_stage {

/// A 2D point or vector in logical units.
struct Vec2 {
    float x = 0;  ///< Horizontal component (+x is right).
    float y = 0;  ///< Vertical component (+y is down).

    constexpr Vec2() = default;
    constexpr Vec2(float x_, float y_) : x(x_), y(y_) {}

    constexpr Vec2 operator+(Vec2 o) const { return {x + o.x, y + o.y}; }
    constexpr Vec2 operator-(Vec2 o) const { return {x - o.x, y - o.y}; }
    constexpr Vec2 operator*(float s) const { return {x * s, y * s}; }
};

/// An axis-aligned rectangle: origin (top-left corner) plus size.
struct Rect {
    float x = 0;  ///< Left edge.
    float y = 0;  ///< Top edge.
    float w = 0;  ///< Width.
    float h = 0;  ///< Height.

    constexpr Rect() = default;
    constexpr Rect(float x_, float y_, float w_, float h_) : x(x_), y(y_), w(w_), h(h_) {}

    constexpr Vec2 origin() const { return {x, y}; }
    constexpr Vec2 size() const { return {w, h}; }
    constexpr Vec2 center() const { return {x + w * 0.5f, y + h * 0.5f}; }
    constexpr bool contains(Vec2 p) const {
        return p.x >= x && p.y >= y && p.x < x + w && p.y < y + h;
    }
};

/// A 2D affine transform (column-major 2×3 matrix):
///
///     | a c tx |   | x |
///     | b d ty | · | y |
///                  | 1 |
///
/// Composes translation, rotation, scale, and shear. Layer attributes
/// (`position` / `rotation` / `scale`) are conveniences that resolve into one
/// of these; `Layer::transform()` accepts one directly.
struct Mat23 {
    float a = 1, b = 0, c = 0, d = 1, tx = 0, ty = 0;

    static constexpr Mat23 identity() { return {}; }

    static Mat23 translation(Vec2 t) { return {1, 0, 0, 1, t.x, t.y}; }

    /// Rotation by \p degrees, clockwise on screen (because +y points down).
    static Mat23 rotationDeg(float degrees) {
        const float r = degrees * 0.01745329252f;
        const float cs = std::cos(r), sn = std::sin(r);
        return {cs, sn, -sn, cs, 0, 0};
    }

    static Mat23 scaling(Vec2 s) { return {s.x, 0, 0, s.y, 0, 0}; }

    /// this ∘ other — apply \p other first, then this.
    Mat23 operator*(const Mat23& o) const {
        return {a * o.a + c * o.b,        b * o.a + d * o.b,
                a * o.c + c * o.d,        b * o.c + d * o.d,
                a * o.tx + c * o.ty + tx, b * o.tx + d * o.ty + ty};
    }

    Vec2 apply(Vec2 p) const { return {a * p.x + c * p.y + tx, b * p.x + d * p.y + ty}; }

    /// Inverse transform. Degenerate matrices (determinant ≈ 0) return
    /// identity; callers treat such layers as invisible.
    Mat23 inverse() const {
        const float det = a * d - b * c;
        if (det > -1e-8f && det < 1e-8f) {
            return {};
        }
        const float inv = 1.0f / det;
        Mat23 m{d * inv, -b * inv, -c * inv, a * inv, 0, 0};
        m.tx = -(m.a * tx + m.c * ty);
        m.ty = -(m.b * tx + m.d * ty);
        return m;
    }

    bool isIdentity() const {
        return a == 1 && b == 0 && c == 0 && d == 1 && tx == 0 && ty == 0;
    }
    bool isTranslationOnly() const { return a == 1 && b == 0 && c == 0 && d == 1; }
};

/// An sRGB color with straight (non-premultiplied) alpha, each channel 0–1.
struct Color {
    float r = 1, g = 1, b = 1, a = 1;

    constexpr Color() = default;
    constexpr Color(float r_, float g_, float b_, float a_ = 1.0f)
        : r(r_), g(g_), b(b_), a(a_) {}

    /// Same color with alpha multiplied by \p alpha.
    constexpr Color faded(float alpha) const { return {r, g, b, a * alpha}; }

    static const Color White;
    static const Color Black;
    static const Color Red;
    static const Color Green;
    static const Color Blue;
    static const Color Yellow;
    static const Color Orange;
    static const Color Teal;
    static const Color Magenta;
    static const Color Gray;
    static const Color Transparent;
};

inline constexpr Color Color::White{1, 1, 1, 1};
inline constexpr Color Color::Black{0, 0, 0, 1};
inline constexpr Color Color::Red{0.92f, 0.26f, 0.21f, 1};
inline constexpr Color Color::Green{0.20f, 0.78f, 0.35f, 1};
inline constexpr Color Color::Blue{0.22f, 0.53f, 0.90f, 1};
inline constexpr Color Color::Yellow{0.98f, 0.80f, 0.18f, 1};
inline constexpr Color Color::Orange{1.00f, 0.62f, 0.15f, 1};
inline constexpr Color Color::Teal{0.10f, 0.90f, 0.70f, 1};
inline constexpr Color Color::Magenta{0.91f, 0.34f, 0.76f, 1};
inline constexpr Color Color::Gray{0.55f, 0.58f, 0.60f, 1};
inline constexpr Color Color::Transparent{0, 0, 0, 0};

}  // namespace fluent_stage
