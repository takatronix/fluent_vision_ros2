#include "fv_image_filter/fluent_vision.hpp"
#include <cvx/cvx.hpp>

namespace fluent_vision {

// 色定数の定義
const cv::Scalar FluentVision::RED = cv::Scalar(0, 0, 255);
const cv::Scalar FluentVision::GREEN = cv::Scalar(0, 255, 0);
const cv::Scalar FluentVision::BLUE = cv::Scalar(255, 0, 0);
const cv::Scalar FluentVision::YELLOW = cv::Scalar(0, 255, 255);
const cv::Scalar FluentVision::CYAN = cv::Scalar(255, 255, 0);
const cv::Scalar FluentVision::MAGENTA = cv::Scalar(255, 0, 255);
const cv::Scalar FluentVision::WHITE = cv::Scalar(255, 255, 255);
const cv::Scalar FluentVision::BLACK = cv::Scalar(0, 0, 0);

// コンストラクタ
FluentVision::FluentVision(const cv::Mat& image) : image_(image.clone()) {}
FluentVision::FluentVision(cv::Mat&& image) : image_(std::move(image)) {}

// ファクトリメソッド
FluentVision FluentVision::from(const cv::Mat& image) {
    return FluentVision(image);
}

FluentVision FluentVision::from(cv::Mat&& image) {
    return FluentVision(std::move(image));
}

// フィルタ操作
FluentVision& FluentVision::blur(double radius) {
    int kernel_size = static_cast<int>(radius * 2) | 1; // 奇数にする
    cv::blur(image_, image_, cv::Size(kernel_size, kernel_size));
    return *this;
}

FluentVision& FluentVision::gaussianBlur(int kernel_size, double sigma) {
    if (sigma <= 0) {
        sigma = 0.3 * ((kernel_size - 1) * 0.5 - 1) + 0.8;
    }
    cv::GaussianBlur(image_, image_, cv::Size(kernel_size, kernel_size), sigma);
    return *this;
}

FluentVision& FluentVision::medianBlur(int kernel_size) {
    cv::medianBlur(image_, image_, kernel_size);
    return *this;
}

FluentVision& FluentVision::bilateralFilter(int d, double sigma_color, double sigma_space) {
    cv::Mat temp;
    cv::bilateralFilter(image_, temp, d, sigma_color, sigma_space);
    image_ = temp;
    return *this;
}

// エッジ検出
FluentVision& FluentVision::edge(double threshold1, double threshold2) {
    if (threshold2 == 0) {
        threshold2 = threshold1 * 3;
    }
    return canny(threshold1, threshold2);
}

FluentVision& FluentVision::canny(double threshold1, double threshold2) {
    cv::Mat gray;
    if (image_.channels() > 1) {
        cv::cvtColor(image_, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image_;
    }
    cv::Canny(gray, image_, threshold1, threshold2);
    return *this;
}

FluentVision& FluentVision::sobel(int dx, int dy, int kernel_size) {
    cv::Mat grad;
    cv::Sobel(image_, grad, CV_16S, dx, dy, kernel_size);
    cv::convertScaleAbs(grad, image_);
    return *this;
}

FluentVision& FluentVision::laplacian(int kernel_size) {
    cv::Mat lap;
    cv::Laplacian(image_, lap, CV_16S, kernel_size);
    cv::convertScaleAbs(lap, image_);
    return *this;
}

// 色調整
FluentVision& FluentVision::color(double brightness, double contrast, double saturation) {
    this->brightness(brightness);
    this->contrast(contrast);
    if (saturation != 1.0 && image_.channels() >= 3) {
        this->saturation(saturation);
    }
    return *this;
}

FluentVision& FluentVision::brightness(double factor) {
    image_.convertTo(image_, -1, 1.0, (factor - 1.0) * 127);
    return *this;
}

FluentVision& FluentVision::contrast(double factor) {
    image_.convertTo(image_, -1, factor, 0);
    return *this;
}

FluentVision& FluentVision::saturation(double factor) {
    if (image_.channels() < 3) return *this;
    
    cv::Mat hsv;
    cv::cvtColor(image_, hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    channels[1] *= factor;
    cv::merge(channels, hsv);
    cv::cvtColor(hsv, image_, cv::COLOR_HSV2BGR);
    return *this;
}

FluentVision& FluentVision::gamma(double gamma) {
    cv::Mat lookUpTable(1, 256, CV_8U);
    uchar* p = lookUpTable.ptr();
    for (int i = 0; i < 256; ++i) {
        p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
    }
    cv::LUT(image_, lookUpTable, image_);
    return *this;
}

// 描画操作
FluentVision& FluentVision::drawRectangle(const cv::Rect& rect, const cv::Scalar& color, int thickness) {
    cv::rectangle(image_, rect, color, thickness);
    return *this;
}

FluentVision& FluentVision::drawLine(const cv::Point& pt1, const cv::Point& pt2, const cv::Scalar& color, int thickness) {
    cv::line(image_, pt1, pt2, color, thickness);
    return *this;
}

FluentVision& FluentVision::drawCircle(const cv::Point& center, int radius, const cv::Scalar& color, int thickness) {
    cv::circle(image_, center, radius, color, thickness);
    return *this;
}

FluentVision& FluentVision::drawText(const std::string& text, const cv::Point& pos, const cv::Scalar& color, 
                                     double scale, int thickness) {
    cv::putText(image_, text, pos, cv::FONT_HERSHEY_SIMPLEX, scale, color, thickness);
    return *this;
}

// ブレンド・合成
FluentVision& FluentVision::blend(const cv::Mat& overlay, double alpha) {
    cv::addWeighted(image_, 1.0 - alpha, overlay, alpha, 0, image_);
    return *this;
}

FluentVision& FluentVision::addWeighted(const cv::Mat& src2, double alpha, double beta, double gamma) {
    cv::addWeighted(image_, alpha, src2, beta, gamma, image_);
    return *this;
}

// 変換
FluentVision& FluentVision::resize(const cv::Size& size, int interpolation) {
    cv::resize(image_, image_, size, 0, 0, interpolation);
    return *this;
}

FluentVision& FluentVision::rotate(double angle, const cv::Point2f& center) {
    cv::Point2f rotation_center = center;
    if (center.x < 0 || center.y < 0) {
        rotation_center = cv::Point2f(image_.cols / 2.0f, image_.rows / 2.0f);
    }
    cv::Mat M = cv::getRotationMatrix2D(rotation_center, angle, 1.0);
    cv::warpAffine(image_, image_, M, image_.size());
    return *this;
}

FluentVision& FluentVision::flip(int flip_code) {
    cv::flip(image_, image_, flip_code);
    return *this;
}

FluentVision& FluentVision::crop(const cv::Rect& roi) {
    image_ = image_(roi).clone();
    return *this;
}

// モルフォロジー演算
FluentVision& FluentVision::erode(int kernel_size, int iterations) {
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
    cv::erode(image_, image_, kernel, cv::Point(-1, -1), iterations);
    return *this;
}

FluentVision& FluentVision::dilate(int kernel_size, int iterations) {
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
    cv::dilate(image_, image_, kernel, cv::Point(-1, -1), iterations);
    return *this;
}

FluentVision& FluentVision::morphologyEx(int op, int kernel_size, int iterations) {
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
    cv::morphologyEx(image_, image_, op, kernel, cv::Point(-1, -1), iterations);
    return *this;
}

// 色空間変換
FluentVision& FluentVision::toGray() {
    if (image_.channels() > 1) {
        cv::cvtColor(image_, image_, cv::COLOR_BGR2GRAY);
    }
    return *this;
}

FluentVision& FluentVision::toBGR() {
    if (image_.channels() == 1) {
        cv::cvtColor(image_, image_, cv::COLOR_GRAY2BGR);
    }
    return *this;
}

FluentVision& FluentVision::toRGB() {
    if (image_.channels() == 1) {
        cv::cvtColor(image_, image_, cv::COLOR_GRAY2RGB);
    } else if (image_.channels() >= 3) {
        cv::cvtColor(image_, image_, cv::COLOR_BGR2RGB);
    }
    return *this;
}

FluentVision& FluentVision::toHSV() {
    if (image_.channels() >= 3) {
        cv::cvtColor(image_, image_, cv::COLOR_BGR2HSV);
    }
    return *this;
}

FluentVision& FluentVision::toLab() {
    if (image_.channels() >= 3) {
        cv::cvtColor(image_, image_, cv::COLOR_BGR2Lab);
    }
    return *this;
}

// カスタムフィルタ
FluentVision& FluentVision::custom(std::function<void(cv::Mat&)> filter) {
    filter(image_);
    return *this;
}

// 出力
cv::Mat FluentVision::toMat() const {
    return image_;
}

cv::Mat FluentVision::clone() const {
    return image_.clone();
}

void FluentVision::copyTo(cv::Mat& dst) const {
    image_.copyTo(dst);
}

} // namespace fluent_vision