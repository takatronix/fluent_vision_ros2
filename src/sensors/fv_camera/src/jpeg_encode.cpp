// JPEG を作るのだけをこのファイルに置き、CMakeLists でこのファイルだけ最適化 (-O3) して組む (2026-10-30)。
// 機体の colcon build は build type を付けず、fv_camera は最適化なし (-O0) で組まれる。hw_jpeg.hpp は header-only なので、
// BGR→I420 の自前ループが呼ぶ側の最適化のまま動き、aspa2 で 1080x1920 を作る CPU が 19.1 ms (OpenCV 9.2 ms) になった
// (-O3 で 3.4 ms)。パッケージ全体は最適化しない: 撮影スレッドと設定サービスの間に前からの競合があり、表に出しうるため。
#include "fv_camera/jpeg_encode.hpp"

namespace fv_camera {

bool encode_jpeg(fluent::hwjpeg::Encoder& encoder, const cv::Mat& bgr, int quality, std::vector<uint8_t>& out)
{
    return encoder.encode(bgr, quality, out);
}

}  // namespace fv_camera
