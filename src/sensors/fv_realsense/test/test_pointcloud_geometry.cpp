#include "fv_realsense/pointcloud_geometry.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <climits>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <vector>

int main() {
    using fv_realsense::sampledExtent;
    using fv_realsense::projectedColorPixel;
    // Match the writer's iteration domain, including non-divisors and a step
    // larger than the image. ASan checks the actual writes, not just arithmetic.
    for (int width : {1, 7, 640, 848, 1280}) {
        for (int height : {1, 5, 480, 720}) {
            for (int step : {1, 2, 3, 7, 10000, INT_MAX}) {
                for (int point_step : {12, 16}) {
                    const size_t size = static_cast<size_t>(sampledExtent(width, step)) *
                        sampledExtent(height, step) * point_step;
                    std::vector<uint8_t> data(size);
                    uint8_t* dst = data.data();
                    const float value = 1.0f;
                    for (int v = 0; v < height; v += step) {
                        for (int u = 0; u < width; u += step) {
                            for (int offset = 0; offset < point_step; offset += 4) {
                                std::memcpy(dst + offset, &value, sizeof(value));
                            }
                            dst += point_step;
                        }
                    }
                    assert(dst == data.data() + data.size());
                }
            }
        }
    }
    assert(sampledExtent(640, 3) == 214);
    assert(sampledExtent(480, 3) == 160);
    assert(sampledExtent(0, 1) == 0);
    assert(sampledExtent(1, 0) == 0);
    int u = -1, v = -1;
    assert(projectedColorPixel(0, 0, 640, 480, u, v) && u == 0 && v == 0);
    assert(projectedColorPixel(639.9f, 479.9f, 640, 480, u, v) && u == 639 && v == 479);
    for (float bad : {-1.0f, 640.0f, 1e30f,
                      std::numeric_limits<float>::infinity(),
                      std::numeric_limits<float>::quiet_NaN()}) {
        assert(!projectedColorPixel(bad, 0, 640, 480, u, v));
    }
    for (float bad : {-1.0f, 480.0f, 1e30f,
                      std::numeric_limits<float>::infinity(),
                      std::numeric_limits<float>::quiet_NaN()}) {
        assert(!projectedColorPixel(0, bad, 640, 480, u, v));
    }
    // Smaller color image: reject a raw depth pixel outside its raster.
    assert(!projectedColorPixel(1000, 600, 640, 480, u, v));
    std::cout << "240 organized layouts and projection boundary cases passed\n";
}
