#pragma once

#include <cmath>

namespace fv_realsense {

// Number of samples visited by `for (int i = 0; i < extent; i += step)`.
// Avoid extent + step - 1, which can overflow for a large decimation parameter.
inline int sampledExtent(int extent, int step) {
    return extent > 0 && step > 0 ? 1 + (extent - 1) / step : 0;
}

// Validate before converting to int: NaN, infinity and very large projections
// must never reach a pixel access (or an out-of-range float-to-int conversion).
inline bool projectedColorPixel(float x, float y, int width, int height, int& u, int& v) {
    if (!std::isfinite(x) || !std::isfinite(y) ||
        x < 0.0f || y < 0.0f || x >= width || y >= height) {
        return false;
    }
    u = static_cast<int>(x);
    v = static_cast<int>(y);
    return true;
}

} // namespace fv_realsense
