#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

namespace cvx {
/// @brief 入力画像を指定チャンネル数・サイズ・正規化で前処理しCHW float配列へ（16ch, 3ch, 1ch対応）
std::vector<float> preprocessToCHW(const cv::Mat& src, int out_ch, int out_h, int out_w);
}
