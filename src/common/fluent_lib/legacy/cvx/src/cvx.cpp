#include "cvx/cvx.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/freetype.hpp>

namespace cvx {

std::vector<std::vector<cv::Point>> findContours(const cv::Mat& binary, int mode = cv::RETR_EXTERNAL, int method = cv::CHAIN_APPROX_SIMPLE) {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary, contours, hierarchy, mode, method);
    return contours;
}

} // namespace cvx

