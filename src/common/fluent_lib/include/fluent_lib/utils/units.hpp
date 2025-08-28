#pragma once

#include <cmath>
#include <opencv2/core.hpp>

namespace fluent {
namespace units {

// 角度
constexpr inline double degToRad(double deg) { return deg * M_PI / 180.0; }
constexpr inline double radToDeg(double rad) { return rad * 180.0 / M_PI; }

// 長さ
constexpr inline double mToCm(double m) { return m * 100.0; }
constexpr inline double cmToM(double cm) { return cm / 100.0; }
constexpr inline double mToMm(double m) { return m * 1000.0; }
constexpr inline double mmToM(double mm) { return mm / 1000.0; }

// 時間
constexpr inline double sToMs(double s) { return s * 1000.0; }
constexpr inline double msToS(double ms) { return ms / 1000.0; }

// RealSense等の16U深度スケール → [m]
// 例: D405=0.0001, D415=0.001
constexpr inline double depth16UToMeters(uint16_t raw, double unit_m_16u) {
    return (raw == 0) ? 0.0 : static_cast<double>(raw) * unit_m_16u;
}

// カメラ内参
struct Intrinsics {
    double fx {0.0};
    double fy {0.0};
    double cx {0.0};
    double cy {0.0};
};

// 3D(camera) → 2D(pixel)
inline cv::Point2f projectPixel(const Intrinsics& K, const cv::Point3d& P) {
    double u = (P.x * K.fx) / P.z + K.cx;
    double v = (P.y * K.fy) / P.z + K.cy;
    return cv::Point2f(static_cast<float>(u), static_cast<float>(v));
}

// 2D(pixel) + Z[m] → 3D(camera)
inline cv::Point3d backProjectPixel(const Intrinsics& K, const cv::Point2f& uv, double Z) {
    double x = (static_cast<double>(uv.x) - K.cx) * Z / K.fx;
    double y = (static_cast<double>(uv.y) - K.cy) * Z / K.fy;
    return cv::Point3d(x, y, Z);
}

} // namespace units
} // namespace fluent


