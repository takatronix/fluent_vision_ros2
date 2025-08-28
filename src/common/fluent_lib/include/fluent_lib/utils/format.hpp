#pragma once

#include <string>
#include <cmath>
#include <cstdio>

namespace fluent {
namespace fmt {

// NaN/無効値なら "--" を返し、そうでなければ固定小数点で出力
inline std::string fixed(double v, int digits) {
    if (!std::isfinite(v)) return std::string("--");
    char buf[64];
    std::snprintf(buf, sizeof(buf), ("%." + std::to_string(digits) + "f").c_str(), v);
    return std::string(buf);
}

// 1桁小数のヘルパ
inline std::string one(double v) { return fixed(v, 1); }

// 単位付きの簡易フォーマッタ
inline std::string cm1(double meters) {
    if (!std::isfinite(meters)) return std::string("--");
    double cm = meters * 100.0;
    return one(cm);
}

inline std::string ms1(double milliseconds) {
    return one(milliseconds);
}

// 端数丸めだけほしい場合（1桁）
inline double round1(double v) {
    return std::round(v * 10.0) / 10.0;
}

} // namespace fmt
} // namespace fluent


