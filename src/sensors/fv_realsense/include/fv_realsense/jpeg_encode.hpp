#pragma once
// JPEG を作る (fluent_lib/hw_jpeg.hpp の Encoder) のを、最適化して組む別のファイル (src/jpeg_encode.cpp) に置く (2026-10-30)。
// fv_realsense_node.cpp からは Encoder::encode を直接呼ばず、必ずこれを通す (呼ぶと最適化なしの inline の写しができる)。
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>
#include <fluent_lib/hw_jpeg.hpp>

namespace fv_realsense {

bool encode_jpeg(fluent::hwjpeg::Encoder& encoder, const cv::Mat& bgr, int quality, std::vector<uint8_t>& out);

}  // namespace fv_realsense
