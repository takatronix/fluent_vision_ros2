#include "cvx/cvx_image.hpp"
#include <opencv2/imgproc.hpp>
#include "cvx/cvx_image_filter.hpp"

namespace cvx {
// --- cvx::Mat 実装 ---
#include <iostream>
#include <iomanip>

void Mat::printInfo(const std::string& label) const {
    std::cout << (label.empty() ? "[cvx::Mat]" : label) << " ";
    std::cout << "size=" << mat_.cols << "x" << mat_.rows;
    std::cout << ", channels=" << mat_.channels();
    std::cout << ", type=" << typeStr(mat_.type());
    std::cout << ", color=" << colorStr(mat_.type(), mat_.channels());
    double minVal, maxVal;
    cv::minMaxLoc(mat_, &minVal, &maxVal);
    std::cout << ", min=" << minVal << ", max=" << maxVal;
    cv::Scalar mean = cv::mean(mat_);
    std::cout << ", mean=[";
    for(int i=0;i<mat_.channels();++i) {
        std::cout << std::fixed << std::setprecision(2) << mean[i];
        if(i+1<mat_.channels()) std::cout << ",";
    }
    std::cout << "]" << std::endl;
}

std::string Mat::typeStr(int type) {
    std::string r;
    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);
    switch (depth) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
    }
    r = "CV_" + r + "C" + std::to_string(chans);
    return r;
}

std::string Mat::colorStr(int /*type*/, int channels) {
    // 色空間推定
    if (channels == 1) return "GRAY";
    if (channels == 2) return "GRAY_ALPHA";
    if (channels == 3) {
        // 型推定: BGR/RGB/Lab/HSVなど用途によるがOpenCV標準はBGR
        return "BGR";
    }
    if (channels == 4) return "BGRA";
    return "UNKNOWN";
}

Mat::Mat() : mat_() {}
Mat::Mat(const cv::Mat& m) : mat_(m.clone()) {}
Mat::Mat(cv::Mat&& m) : mat_(std::move(m)) {}
Mat Mat::clone() const { return Mat(mat_.clone()); }
cv::Size Mat::size() const { return mat_.size(); }
int Mat::type() const { return mat_.type(); }
int Mat::channels() const { return mat_.channels(); }
Mat Mat::toGray() const {
    if (mat_.channels() == 1) return Mat(mat_);
    cv::Mat gray;
    cv::cvtColor(mat_, gray, cv::COLOR_BGR2GRAY);
    return Mat(gray);
}
Mat Mat::toColor() const {
    if (mat_.channels() == 3) return Mat(mat_);
    cv::Mat color;
    cv::cvtColor(mat_, color, cv::COLOR_GRAY2BGR);
    return Mat(color);
}
Mat Mat::toBGR() const {
    if (mat_.channels() == 3) return Mat(mat_);
    if (mat_.channels() == 1) return toColor();
    cv::Mat bgr;
    cv::cvtColor(mat_, bgr, cv::COLOR_RGB2BGR);
    return Mat(bgr);
}
Mat Mat::toRGB() const {
    if (mat_.channels() == 3) {
        cv::Mat rgb;
        cv::cvtColor(mat_, rgb, cv::COLOR_BGR2RGB);
        return Mat(rgb);
    }
    if (mat_.channels() == 1) {
        cv::Mat rgb;
        cv::cvtColor(mat_, rgb, cv::COLOR_GRAY2RGB);
        return Mat(rgb);
    }
    return Mat(mat_);
}
Mat Mat::toHSV() const {
    cv::Mat hsv;
    cv::cvtColor(mat_, hsv, cv::COLOR_BGR2HSV);
    return Mat(hsv);
}
Mat Mat::toLab() const {
    cv::Mat lab;
    cv::cvtColor(mat_, lab, cv::COLOR_BGR2Lab);
    return Mat(lab);
}
Mat Mat::toBGRA() const {
    cv::Mat bgra;
    if (mat_.channels() == 3) {
        cv::cvtColor(mat_, bgra, cv::COLOR_BGR2BGRA);
    } else if (mat_.channels() == 1) {
        cv::cvtColor(mat_, bgra, cv::COLOR_GRAY2BGRA);
    } else {
        bgra = mat_;
    }
    return Mat(bgra);
}
Mat Mat::toRGBA() const {
    cv::Mat rgba;
    if (mat_.channels() == 3) {
        cv::cvtColor(mat_, rgba, cv::COLOR_BGR2RGBA);
    } else if (mat_.channels() == 1) {
        cv::cvtColor(mat_, rgba, cv::COLOR_GRAY2RGBA);
    } else {
        rgba = mat_;
    }
    return Mat(rgba);
}
Mat Mat::toYCrCb() const {
    cv::Mat ycrcb;
    cv::cvtColor(mat_, ycrcb, cv::COLOR_BGR2YCrCb);
    return Mat(ycrcb);
}
Mat Mat::toXYZ() const {
    cv::Mat xyz;
    cv::cvtColor(mat_, xyz, cv::COLOR_BGR2XYZ);
    return Mat(xyz);
}
Mat Mat::toLuv() const {
    cv::Mat luv;
    cv::cvtColor(mat_, luv, cv::COLOR_BGR2Luv);
    return Mat(luv);
}
Mat Mat::toHLS() const {
    cv::Mat hls;
    cv::cvtColor(mat_, hls, cv::COLOR_BGR2HLS);
    return Mat(hls);
}
Mat Mat::toGray16() const {
    cv::Mat gray16;
    if (mat_.channels() == 1 && mat_.depth() == CV_16U) return Mat(mat_);
    cv::Mat gray = toGray();
    gray.convertTo(gray16, CV_16U, 256.0);
    return Mat(gray16);
}
Mat Mat::toGray32() const {
    cv::Mat gray32;
    cv::Mat gray = toGray();
    gray.convertTo(gray32, CV_32F, 1.0/255.0);
    return Mat(gray32);
}
Mat Mat::toFloat() const {
    cv::Mat f;
    mat_.convertTo(f, CV_32F);
    return Mat(f);
}
Mat Mat::toU8() const {
    cv::Mat u8;
    mat_.convertTo(u8, CV_8U);
    return Mat(u8);
}
Mat Mat::toU16() const {
    cv::Mat u16;
    mat_.convertTo(u16, CV_16U);
    return Mat(u16);
}
Mat Mat::toS16() const {
    cv::Mat s16;
    mat_.convertTo(s16, CV_16S);
    return Mat(s16);
}
Mat Mat::to32F() const {
    cv::Mat f32;
    mat_.convertTo(f32, CV_32F);
    return Mat(f32);
}
Mat Mat::to64F() const {
    cv::Mat f64;
    mat_.convertTo(f64, CV_64F);
    return Mat(f64);
}

Mat Mat::blur(int ksize) const {
    return Mat(cvx::blur(mat_, ksize));
}
Mat Mat::gaussianBlur(int ksize, double sigma) const {
    return Mat(cvx::gaussianBlur(mat_, ksize, sigma));
}
Mat Mat::medianBlur(int ksize) const {
    return Mat(cvx::medianBlur(mat_, ksize));
}
Mat Mat::bilateralFilter(int d, double sigmaColor, double sigmaSpace) const {
    return Mat(cvx::bilateralFilter(mat_, d, sigmaColor, sigmaSpace));
}
Mat Mat::laplacian(int ksize) const {
    return Mat(cvx::laplacian(mat_, ksize));
}
Mat Mat::sobel(int dx, int dy, int ksize) const {
    return Mat(cvx::sobel(mat_, dx, dy, ksize));
}
Mat Mat::scharr(int dx, int dy) const {
    return Mat(cvx::scharr(mat_, dx, dy));
}
Mat Mat::canny(double th1, double th2) const {
    return Mat(cvx::canny(mat_, th1, th2));
}
Mat Mat::threshold(double thresh, double maxval, int type) const {
    return Mat(cvx::threshold(mat_, thresh, maxval, type));
}
Mat Mat::adaptiveThreshold(double maxval, int method, int type, int blockSize, double C) const {
    return Mat(cvx::adaptiveThreshold(mat_, maxval, method, type, blockSize, C));
}
Mat Mat::dilate(int ksize) const {
    return Mat(cvx::dilate(mat_, ksize));
}
Mat Mat::erode(int ksize) const {
    return Mat(cvx::erode(mat_, ksize));
}
Mat Mat::morphologyOpen(int ksize) const {
    return Mat(cvx::morphologyOpen(mat_, ksize));
}
Mat Mat::morphologyClose(int ksize) const {
    return Mat(cvx::morphologyClose(mat_, ksize));
}
Mat Mat::warpAffine(const cv::Mat& M, const cv::Size& dsize) const {
    return Mat(cvx::warpAffine(mat_, M, dsize));
}
Mat Mat::warpPerspective(const cv::Mat& M, const cv::Size& dsize) const {
    return Mat(cvx::warpPerspective(mat_, M, dsize));
}
Mat Mat::quantize(int levels) const {
    // 色数levelsに減らす（単純量子化）
    cv::Mat q;
    double div = 256.0 / levels;
    mat_.convertTo(q, CV_32F);
    q = q / div;
    cv::Mat qi = q.clone();
    qi.forEach<float>([](float& v, const int*){ v = std::floor(v); });
    qi = qi * div + div / 2;
    qi.convertTo(qi, mat_.type());
    return Mat(qi);
}
Mat Mat::add(const Mat& other, double alpha, double beta, double gamma) const {
    cv::Mat dst;
    cv::addWeighted(mat_, alpha, other.mat_, beta, gamma, dst);
    return Mat(dst);
}
Mat Mat::subtract(const Mat& other) const {
    cv::Mat dst;
    cv::subtract(mat_, other.mat_, dst);
    return Mat(dst);
}
Mat Mat::multiply(const Mat& other, double scale) const {
    cv::Mat dst;
    cv::multiply(mat_, other.mat_, dst, scale);
    return Mat(dst);
}
Mat Mat::blend(const Mat& other, double alpha) const {
    // alpha: 本画像寄与率, (1-alpha): other寄与率
    cv::Mat dst;
    cv::addWeighted(mat_, alpha, other.mat_, 1.0-alpha, 0.0, dst);
    return Mat(dst);
}
Mat Mat::bitwiseAnd(const Mat& other) const {
    cv::Mat dst;
    cv::bitwise_and(mat_, other.mat_, dst);
    return Mat(dst);
}
Mat Mat::bitwiseOr(const Mat& other) const {
    cv::Mat dst;
    cv::bitwise_or(mat_, other.mat_, dst);
    return Mat(dst);
}
Mat Mat::bitwiseXor(const Mat& other) const {
    cv::Mat dst;
    cv::bitwise_xor(mat_, other.mat_, dst);
    return Mat(dst);
}
Mat Mat::bitwiseNot() const {
    cv::Mat dst;
    cv::bitwise_not(mat_, dst);
    return Mat(dst);
}

Mat Mat::normalize(double min, double max) const {
    cv::Mat f, norm;
    mat_.convertTo(f, CV_32F);
    double minVal, maxVal;
    cv::minMaxLoc(f, &minVal, &maxVal);
    if (maxVal - minVal < 1e-6) {
        return Mat(cv::Mat::zeros(mat_.size(), CV_32F));
    }
    f = (f - minVal) / (maxVal - minVal);
    norm = f * (max - min) + min;
    return Mat(norm);
}
Mat Mat::mask(const cv::Mat& mask) const {
    // 入力チェック - 無効なマスクや空の画像の場合は元画像を返す
    if (mask.empty() || mat_.empty()) {
        return *this; // 元画像をそのまま返す
    }
    
    try {
        cv::Mat mask8u, mask_resized;
        if (mask.channels() != 1) {
            cv::cvtColor(mask, mask8u, cv::COLOR_BGR2GRAY);
        } else {
            mask8u = mask.clone();  // 修正: mask_resizedではなくmask8uに代入
        }
        
        // 8ビット型に変換
        if (mask8u.type() != CV_8U) {
            mask8u.convertTo(mask8u, CV_8U);
        }
        
        // サイズが異なる場合はリサイズ
        if (mask8u.size() != mat_.size()) {
            cv::resize(mask8u, mask_resized, mat_.size(), 0, 0, cv::INTER_NEAREST);
        } else {
            mask_resized = mask8u;
        }
        
        cv::Mat masked;
        cv::bitwise_and(mat_, mat_, masked, mask_resized);
        return Mat(masked);
    } catch (const cv::Exception& e) {
        // OpenCVの例外が発生した場合は元画像を返す
        std::cerr << "mask error: " << e.what() << std::endl;
        return *this;
    }
}

Mat Mat::applyMask(const Mat& mask_image) const {
    // 無効なマスク画像のチェック
    if (mask_image.get().empty()) {
        return *this; // 元画像をそのまま返す
    }
    return applyMask(mask_image.get());
}

Mat Mat::applyMask(const cv::Mat& mask_image) const {
    // 入力チェック - 無効なマスクや空の画像の場合は元画像を返す
    if (mask_image.empty() || mat_.empty()) {
        return *this; // 元画像をそのまま返す
    }
    
    // マスク画像の前処理（グレースケール化、サイズ調整、型変換）
    cv::Mat mask8u, mask_resized;
    
    try {
        // マスクがカラーの場合はグレースケールに変換
        if (mask_image.channels() != 1) {
            cv::cvtColor(mask_image, mask8u, cv::COLOR_BGR2GRAY);
        } else {
            mask8u = mask_image.clone();
        }
        
        // 8ビット型に変換
        if (mask8u.type() != CV_8U) {
            mask8u.convertTo(mask8u, CV_8U);
        }
        
        // サイズが異なる場合はリサイズ
        if (mask8u.size() != mat_.size()) {
            cv::resize(mask8u, mask_resized, mat_.size(), 0, 0, cv::INTER_NEAREST);
        } else {
            mask_resized = mask8u;
        }
        
        // 深度画像の場合は閾値処理をスキップ（グレースケールを維持）
        bool is_depth = (mat_.type() == CV_16U || mat_.type() == CV_32F || mat_.type() == CV_64F);
        
        if (!is_depth) {
            // 通常の画像の場合は閾値処理で2値化
            cv::threshold(mask_resized, mask_resized, 127, 255, cv::THRESH_BINARY);
        }
        
        // マスク適用（マスク部分のみ残す）
        cv::Mat result = cv::Mat::zeros(mat_.size(), mat_.type());
        mat_.copyTo(result, mask_resized);
        
        return Mat(result);
    } catch (const cv::Exception& e) {
        // OpenCVの例外が発生した場合は元画像を返す
        std::cerr << "applyMask error: " << e.what() << std::endl;
        return *this;
    }
}

cv::Mat toGray(const cv::Mat& src) {
    if (src.channels() == 1) return src.clone();
    cv::Mat gray;
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    return gray;
}
cv::Mat toColor(const cv::Mat& src) {
    if (src.channels() == 3) return src.clone();
    cv::Mat color;
    cv::cvtColor(src, color, cv::COLOR_GRAY2BGR);
    return color;
}
cv::Mat normalize(const cv::Mat& src, double min, double max) {
    cv::Mat f, norm;
    src.convertTo(f, CV_32F);
    double minVal, maxVal;
    cv::minMaxLoc(f, &minVal, &maxVal);
    if (maxVal - minVal < 1e-6) {
        return cv::Mat::zeros(src.size(), CV_32F);
    }
    f = (f - minVal) / (maxVal - minVal);
    norm = f * (max - min) + min;
    return norm;
}
cv::Mat resizeTo(const cv::Mat& src, const cv::Size& size) {
    cv::Mat dst;
    cv::resize(src, dst, size, 0, 0, cv::INTER_LINEAR);
    return dst;
}
cv::Mat mask(const cv::Mat& src, const cv::Mat& mask) {
    cv::Mat mask8u, mask_resized;
    if (mask.channels() != 1) {
        cv::cvtColor(mask, mask8u, cv::COLOR_BGR2GRAY);
    } else {
        mask8u = mask;
    }
    if (mask8u.type() != CV_8U) {
        mask8u.convertTo(mask8u, CV_8U);
    }
    if (mask8u.size() != src.size()) {
        cv::resize(mask8u, mask_resized, src.size(), 0, 0, cv::INTER_NEAREST);
    } else {
        mask_resized = mask8u;
    }
    cv::Mat masked;
    cv::bitwise_and(src, src, masked, mask_resized);
    return masked;
}
} // namespace cvx
