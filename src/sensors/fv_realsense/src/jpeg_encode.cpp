// JPEG を作るのだけをこのファイルに置き、CMakeLists でこのファイルだけ最適化 (-O3) して組む (2026-10-30、fv_camera の jpeg_encode.cpp と同じ)。
// 機体の colcon build は build type を付けず、fv_realsense は最適化なし (-O0) で組まれる (aspa2・aspa3 の CMakeCache)。hw_jpeg.hpp は
// header-only なので、BGR→I420 の自前ループが呼ぶ側の最適化のまま動く (aspa2 の fv_camera で 1080x1920 が 19.1 ms、-O3 で 3.4 ms)。
// パッケージ全体の build type は変えない (最適化で前からの競合を表に出さないため)。
#include "fv_realsense/jpeg_encode.hpp"

namespace fv_realsense {

bool encode_jpeg(fluent::hwjpeg::Encoder& encoder, const cv::Mat& bgr, int quality, std::vector<uint8_t>& out)
{
    return encoder.encode(bgr, quality, out);
}

}  // namespace fv_realsense
