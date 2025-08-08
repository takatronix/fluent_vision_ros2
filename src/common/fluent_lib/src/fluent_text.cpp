#include "fluent_text.hpp"
#include "cvx/cvx_text.hpp"

namespace fluent {
namespace text {

void draw(cv::Mat& img, const std::string& text, cv::Point org,
          cv::Scalar color, double font_scale, int thickness, int baseline_offset) {
    // cvx::drawTextを呼び出す（自動的に日本語対応）
    cvx::drawText(img, text, org, color, font_scale, thickness, baseline_offset);
}

void drawShadow(cv::Mat& img, const std::string& text, cv::Point org,
                cv::Scalar color, cv::Scalar shadow_color,
                double font_scale, int thickness, int baseline_offset) {
    // cvx::drawShadowTextを呼び出す
    cvx::drawShadowText(img, text, org, color, shadow_color, font_scale, thickness, baseline_offset);
}

} // namespace text
} // namespace fluent