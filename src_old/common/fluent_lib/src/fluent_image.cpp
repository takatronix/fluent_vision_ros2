#include "fluent.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

// =============================================================================
// Image実装
// =============================================================================

Image Image::load(const std::string& path) {
    cv::Mat img = cv::imread(path);
    if (img.empty()) {
        throw std::runtime_error("画像を読み込めませんでした: " + path);
    }
    return Image(img);
}

Image Image::from(const cv::Mat& mat) {
    return Image(mat.clone());
}

Image Image::create(int width, int height, int type) {
    cv::Mat img = cv::Mat::zeros(height, width, type);
    return Image(img);
}

Image& Image::resize(int width, int height) {
    cv::resize(img_, img_, cv::Size(width, height));
    return *this;
}

Image& Image::resize(double scale) {
    cv::resize(img_, img_, cv::Size(), scale, scale);
    return *this;
}

Image& Image::crop(int x, int y, int w, int h) {
    cv::Rect roi(x, y, w, h);
    img_ = img_(roi);
    return *this;
}

Image& Image::flip(int code) {
    cv::flip(img_, img_, code);
    return *this;
}

Image& Image::rotate(double angle) {
    cv::Point2f center(img_.cols / 2.0, img_.rows / 2.0);
    cv::Mat rotation_matrix = cv::getRotationMatrix2D(center, angle, 1.0);
    cv::warpAffine(img_, img_, rotation_matrix, img_.size());
    return *this;
}

Image& Image::blur(int size) {
    if (size % 2 == 0) size++; // 奇数にする
    cv::GaussianBlur(img_, img_, cv::Size(size, size), 0);
    return *this;
}

Image& Image::sharp() {
    cv::Mat kernel = (cv::Mat_<float>(3, 3) << 
        0, -1, 0,
        -1, 5, -1,
        0, -1, 0);
    cv::filter2D(img_, img_, -1, kernel);
    return *this;
}

Image& Image::bright(double alpha) {
    img_.convertTo(img_, -1, alpha, 0);
    return *this;
}

Image& Image::contrast(double beta) {
    img_.convertTo(img_, -1, 1.0, beta);
    return *this;
}

Image& Image::gamma(double gamma) {
    cv::Mat lookupTable(1, 256, CV_8U);
    uchar* p = lookupTable.ptr();
    for (int i = 0; i < 256; ++i) {
        p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
    }
    cv::LUT(img_, lookupTable, img_);
    return *this;
}

Image& Image::text(const std::string& txt, int x, int y) {
    cv::putText(img_, txt, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 
                0.7, Colors::WHITE, 2);
    return *this;
}

Image& Image::circle(int x, int y, int radius, cv::Scalar color) {
    cv::circle(img_, cv::Point(x, y), radius, color, 2);
    return *this;
}

Image& Image::rect(int x, int y, int w, int h, cv::Scalar color) {
    cv::rectangle(img_, cv::Rect(x, y, w, h), color, 2);
    return *this;
}

Image& Image::line(int x1, int y1, int x2, int y2, cv::Scalar color) {
    cv::line(img_, cv::Point(x1, y1), cv::Point(x2, y2), color, 2);
    return *this;
}

void Image::save(const std::string& path) {
    cv::imwrite(path, img_);
}

void Image::show(const std::string& window) {
    cv::imshow(window, img_);
    cv::waitKey(0);
}