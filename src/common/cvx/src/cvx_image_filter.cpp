#include "cvx/cvx_image_filter.hpp"
#include <opencv2/imgproc.hpp>

namespace cvx {
cv::Mat blur(const cv::Mat& src, int ksize) {
    cv::Mat dst;
    cv::blur(src, dst, cv::Size(ksize, ksize));
    return dst;
}
cv::Mat gaussianBlur(const cv::Mat& src, int ksize, double sigma) {
    cv::Mat dst;
    cv::GaussianBlur(src, dst, cv::Size(ksize, ksize), sigma);
    return dst;
}
cv::Mat medianBlur(const cv::Mat& src, int ksize) {
    cv::Mat dst;
    cv::medianBlur(src, dst, ksize);
    return dst;
}
cv::Mat bilateralFilter(const cv::Mat& src, int d, double sigmaColor, double sigmaSpace) {
    cv::Mat dst;
    cv::bilateralFilter(src, dst, d, sigmaColor, sigmaSpace);
    return dst;
}
cv::Mat laplacian(const cv::Mat& src, int ksize) {
    cv::Mat dst;
    cv::Laplacian(src, dst, CV_16S, ksize);
    cv::convertScaleAbs(dst, dst);
    return dst;
}
cv::Mat sobel(const cv::Mat& src, int dx, int dy, int ksize) {
    cv::Mat dst;
    cv::Sobel(src, dst, CV_16S, dx, dy, ksize);
    cv::convertScaleAbs(dst, dst);
    return dst;
}
cv::Mat scharr(const cv::Mat& src, int dx, int dy) {
    cv::Mat dst;
    cv::Scharr(src, dst, CV_16S, dx, dy);
    cv::convertScaleAbs(dst, dst);
    return dst;
}
cv::Mat canny(const cv::Mat& src, double th1, double th2) {
    cv::Mat gray, edges;
    if (src.channels() == 3) {
        cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = src;
    }
    cv::Canny(gray, edges, th1, th2);
    return edges;
}
cv::Mat threshold(const cv::Mat& src, double thresh, double maxval, int type) {
    cv::Mat gray, bin;
    if (src.channels() == 3) {
        cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = src;
    }
    cv::threshold(gray, bin, thresh, maxval, type);
    return bin;
}
cv::Mat adaptiveThreshold(const cv::Mat& src, double maxval, int method, int type, int blockSize, double C) {
    cv::Mat gray, bin;
    if (src.channels() == 3) {
        cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = src;
    }
    cv::adaptiveThreshold(gray, bin, maxval, method, type, blockSize, C);
    return bin;
}
cv::Mat dilate(const cv::Mat& src, int ksize) {
    cv::Mat dst;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(ksize, ksize));
    cv::dilate(src, dst, kernel);
    return dst;
}
cv::Mat erode(const cv::Mat& src, int ksize) {
    cv::Mat dst;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(ksize, ksize));
    cv::erode(src, dst, kernel);
    return dst;
}
cv::Mat morphologyOpen(const cv::Mat& src, int ksize) {
    cv::Mat dst;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(ksize, ksize));
    cv::morphologyEx(src, dst, cv::MORPH_OPEN, kernel);
    return dst;
}
cv::Mat morphologyClose(const cv::Mat& src, int ksize) {
    cv::Mat dst;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(ksize, ksize));
    cv::morphologyEx(src, dst, cv::MORPH_CLOSE, kernel);
    return dst;
}
cv::Mat warpAffine(const cv::Mat& src, const cv::Mat& M, const cv::Size& dsize) {
    cv::Mat dst;
    cv::warpAffine(src, dst, M, dsize);
    return dst;
}
cv::Mat warpPerspective(const cv::Mat& src, const cv::Mat& M, const cv::Size& dsize) {
    cv::Mat dst;
    cv::warpPerspective(src, dst, M, dsize);
    return dst;
}
} // namespace cvx
