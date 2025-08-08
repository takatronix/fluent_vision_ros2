#pragma once

#include <opencv2/opencv.hpp>
#include <string>

namespace fluent {
namespace text {

// 日本語対応のテキスト描画関数
// cvx::drawTextのラッパーとして機能
void draw(cv::Mat& img, const std::string& text, cv::Point org,
          cv::Scalar color = cv::Scalar(255,255,255),
          double font_scale = 0.5,
          int thickness = 1,
          int baseline_offset = 0);

// 影付きテキスト描画
void drawShadow(cv::Mat& img, const std::string& text, cv::Point org,
                cv::Scalar color = cv::Scalar(255,255,255),
                cv::Scalar shadow_color = cv::Scalar(0,0,0),
                double font_scale = 0.5,
                int thickness = 1,
                int baseline_offset = 0);

} // namespace text
} // namespace fluent