#pragma once
#include "cvx/cvx_text.hpp"
#include "cvx/cvx_image.hpp"
#include "cvx/cvx_image_filter.hpp"
#include "cvx/cvx_image_utils.hpp"
#include "cvx/cvx_draw.hpp"
#include <vector>
#include <opencv2/opencv.hpp>   

namespace cvx {
// 2値画像から輪郭抽出
std::vector<std::vector<cv::Point>> findContours(const cv::Mat& binary, int mode = cv::RETR_EXTERNAL, int method = cv::CHAIN_APPROX_SIMPLE);
}

