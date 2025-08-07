#ifndef FLUENT_VISION_HPP
#define FLUENT_VISION_HPP

#include <opencv2/opencv.hpp>
#include <memory>
#include <functional>
#include <vector>

namespace fluent_vision {

class FluentVision {
public:
    // ファクトリメソッド
    static FluentVision from(const cv::Mat& image);
    static FluentVision from(cv::Mat&& image);
    
    // フィルタ操作（チェーン可能）
    FluentVision& blur(double radius);
    FluentVision& gaussianBlur(int kernel_size, double sigma = 0);
    FluentVision& medianBlur(int kernel_size);
    FluentVision& bilateralFilter(int d, double sigma_color, double sigma_space);
    
    // エッジ検出
    FluentVision& edge(double threshold1, double threshold2 = 0);
    FluentVision& canny(double threshold1, double threshold2);
    FluentVision& sobel(int dx = 1, int dy = 0, int kernel_size = 3);
    FluentVision& laplacian(int kernel_size = 3);
    
    // 色調整
    FluentVision& color(double brightness = 1.0, double contrast = 1.0, double saturation = 1.0);
    FluentVision& brightness(double factor);
    FluentVision& contrast(double factor);
    FluentVision& saturation(double factor);
    FluentVision& gamma(double gamma);
    
    // 描画操作
    FluentVision& drawRectangle(const cv::Rect& rect, const cv::Scalar& color, int thickness = 1);
    FluentVision& drawLine(const cv::Point& pt1, const cv::Point& pt2, const cv::Scalar& color, int thickness = 1);
    FluentVision& drawCircle(const cv::Point& center, int radius, const cv::Scalar& color, int thickness = 1);
    FluentVision& drawText(const std::string& text, const cv::Point& pos, const cv::Scalar& color, 
                          double scale = 1.0, int thickness = 1);
    
    // ブレンド・合成
    FluentVision& blend(const cv::Mat& overlay, double alpha);
    FluentVision& addWeighted(const cv::Mat& src2, double alpha, double beta, double gamma = 0);
    
    // 変換
    FluentVision& resize(const cv::Size& size, int interpolation = cv::INTER_LINEAR);
    FluentVision& rotate(double angle, const cv::Point2f& center = cv::Point2f(-1, -1));
    FluentVision& flip(int flip_code); // 0: vertical, 1: horizontal, -1: both
    FluentVision& crop(const cv::Rect& roi);
    
    // モルフォロジー演算
    FluentVision& erode(int kernel_size = 3, int iterations = 1);
    FluentVision& dilate(int kernel_size = 3, int iterations = 1);
    FluentVision& morphologyEx(int op, int kernel_size = 3, int iterations = 1);
    
    // 色空間変換
    FluentVision& toGray();
    FluentVision& toBGR();
    FluentVision& toRGB();
    FluentVision& toHSV();
    FluentVision& toLab();
    
    // カスタムフィルタ
    FluentVision& custom(std::function<void(cv::Mat&)> filter);
    
    // 出力
    cv::Mat toMat() const;
    cv::Mat clone() const;
    void copyTo(cv::Mat& dst) const;
    
    // 便利な定数
    static const cv::Scalar RED;
    static const cv::Scalar GREEN;
    static const cv::Scalar BLUE;
    static const cv::Scalar YELLOW;
    static const cv::Scalar CYAN;
    static const cv::Scalar MAGENTA;
    static const cv::Scalar WHITE;
    static const cv::Scalar BLACK;
    
private:
    cv::Mat image_;
    
    FluentVision(const cv::Mat& image);
    FluentVision(cv::Mat&& image);
};

} // namespace fluent_vision

#endif // FLUENT_VISION_HPP