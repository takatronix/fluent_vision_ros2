#pragma once
#include <opencv2/opencv.hpp>

namespace fluent_ui {
namespace alpha {

inline void blend_rect(cv::Mat &image, const cv::Rect &rect, const cv::Scalar &color_bgr, double alpha)
{
    if (image.empty() || rect.width <= 0 || rect.height <= 0) return;
    cv::Rect r = rect & cv::Rect(0, 0, image.cols, image.rows);
    if (r.area() <= 0) return;
    cv::Mat roi = image(r);
    cv::Mat overlay; roi.copyTo(overlay);
    cv::rectangle(overlay, cv::Rect(0, 0, roi.cols, roi.rows), color_bgr, cv::FILLED);
    cv::addWeighted(overlay, std::clamp(alpha, 0.0, 1.0), roi, 1.0 - std::clamp(alpha, 0.0, 1.0), 0.0, roi);
}

inline void blend_polygon(cv::Mat &image, const std::vector<cv::Point> &poly, const cv::Scalar &color_bgr, double alpha)
{
    if (image.empty() || poly.size() < 3) return;
    std::vector<std::vector<cv::Point>> polys{poly};
    cv::Mat overlay = image.clone();
    cv::fillPoly(overlay, polys, color_bgr, cv::LINE_AA);
    cv::addWeighted(overlay, std::clamp(alpha, 0.0, 1.0), image, 1.0 - std::clamp(alpha, 0.0, 1.0), 0.0, image);
}

} // namespace alpha
} // namespace fluent_ui


