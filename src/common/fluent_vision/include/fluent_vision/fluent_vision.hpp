#ifndef FLUENT_VISION_HPP
#define FLUENT_VISION_HPP

#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cvx/cvx.hpp>  // cvxライブラリの美しいAPI
#include <memory>
#include <functional>
#include <vector>
#include <string>

namespace fluent_vision {

// Forward declarations
class Pipeline;
class GPUBackend;

/**
 * @brief FluentVision - チェーン可能な画像処理ライブラリ
 * 
 * 使用例:
 * auto result = FluentVision::from(image_msg)
 *     .blur(2.0)
 *     .edge(0.2)
 *     .color(1.2, 1.1)
 *     .drawText("Processed", Point(10, 30))
 *     .toImageMsg();
 */
class FluentVision {
public:
    // ========== ファクトリメソッド ==========
    // OpenCV Matから
    static FluentVision from(const cv::Mat& image);
    static FluentVision from(cv::Mat&& image);
    
    // ROS2 Image messageから
    static FluentVision from(const sensor_msgs::msg::Image::SharedPtr& msg);
    static FluentVision from(const sensor_msgs::msg::Image& msg);
    
    // ========== フィルタ操作 (チェーン可能) ==========
    // ぼかし系
    FluentVision& blur(double radius);
    FluentVision& gaussianBlur(double sigma);
    FluentVision& medianBlur(int kernel_size = 5);
    FluentVision& bilateralFilter(double sigma_color = 50, double sigma_space = 50);
    FluentVision& motionBlur(double angle, int kernel_size = 15);
    
    // エッジ検出系
    FluentVision& edge(double threshold = 0.2);
    FluentVision& canny(double low_threshold, double high_threshold);
    FluentVision& sobel(double scale = 1.0);
    FluentVision& laplacian(double scale = 1.0);
    FluentVision& scharr(double scale = 1.0);
    
    // 色調整系
    FluentVision& color(double brightness = 1.0, double contrast = 1.0, double saturation = 1.0);
    FluentVision& brightness(double factor);
    FluentVision& contrast(double factor);
    FluentVision& saturation(double factor);
    FluentVision& hue(double shift);
    FluentVision& gamma(double gamma);
    FluentVision& autoWhiteBalance();
    FluentVision& histogram_equalization();
    
    // 描画系
    FluentVision& drawRectangle(const cv::Rect& rect, const cv::Scalar& color, int thickness = 2);
    FluentVision& drawLine(const cv::Point& pt1, const cv::Point& pt2, const cv::Scalar& color, int thickness = 2);
    FluentVision& drawCircle(const cv::Point& center, int radius, const cv::Scalar& color, int thickness = 2);
    FluentVision& drawText(const std::string& text, const cv::Point& pos, 
                          const cv::Scalar& color = cv::Scalar(255, 255, 255), 
                          double scale = 1.0, int thickness = 2);
    FluentVision& drawPolyline(const std::vector<cv::Point>& points, const cv::Scalar& color, int thickness = 2);
    FluentVision& fillPolygon(const std::vector<cv::Point>& points, const cv::Scalar& color);
    
    // 変換系
    FluentVision& resize(const cv::Size& size);
    FluentVision& resize(double scale);
    FluentVision& rotate(double angle);
    FluentVision& flip(bool horizontal = true, bool vertical = false);
    FluentVision& crop(const cv::Rect& roi);
    FluentVision& smartCrop(int width = 0, int height = 0);  // 自動クロップ！
    FluentVision& warpPerspective(const std::vector<cv::Point2f>& src, const std::vector<cv::Point2f>& dst);
    
    // ブレンド・合成系
    FluentVision& blend(const cv::Mat& overlay, double alpha = 0.5);
    FluentVision& blend(const FluentVision& other, double alpha = 0.5);
    FluentVision& addWeighted(const cv::Mat& src2, double alpha, double beta);
    FluentVision& overlay(const cv::Mat& overlay, const cv::Point& position);
    
    // モルフォロジー演算
    FluentVision& erode(int kernel_size = 3);
    FluentVision& dilate(int kernel_size = 3);
    FluentVision& morphologyOpen(int kernel_size = 3);
    FluentVision& morphologyClose(int kernel_size = 3);
    FluentVision& morphologyGradient(int kernel_size = 3);
    
    // 色空間変換
    FluentVision& toGray();
    FluentVision& toBGR();
    FluentVision& toRGB();
    FluentVision& toHSV();
    FluentVision& toLab();
    FluentVision& toYCrCb();
    
    // 特殊効果
    FluentVision& denoise(double h = 10);
    FluentVision& cartoon(double sigma_s = 60, double sigma_r = 0.07);
    FluentVision& pencilSketch(double sigma_s = 60, double sigma_r = 0.07);
    FluentVision& oilPainting(int size = 7, int dynRatio = 1);
    
    // AI/ML統合
    FluentVision& detectFaces();
    FluentVision& detectObjects(const std::string& model_name = "yolo");
    FluentVision& segmentObjects(const std::string& model_name = "unet");
    
    // カスタムフィルタ
    FluentVision& custom(std::function<void(cv::Mat&)> filter);
    FluentVision& customAsync(std::function<void(cv::Mat&)> filter);
    
    // アニメーション効果
    FluentVision& animate(class FluentAnimation animation);
    FluentVision& spotlight(cv::Point2f center, float radius, float duration);
    FluentVision& fadeBackground(float darkness, float duration);
    FluentVision& pulseGlow(float min_intensity, float max_intensity, float duration);
    FluentVision& when_detected(std::function<bool()> condition, 
                                std::function<void(FluentVision&)> action);
    
    // ========== GPU処理 ==========
    FluentVision& useGPU(bool enable = true);
    FluentVision& useCUDA();
    FluentVision& useOpenCL();
    bool isGPUEnabled() const;
    
    // ========== パイプライン操作 ==========
    FluentVision& cache(const std::string& key);
    FluentVision& restore(const std::string& key);
    FluentVision& branch(std::function<FluentVision(FluentVision)> branch_func);
    FluentVision& parallel(std::vector<std::function<FluentVision(FluentVision)>> funcs);
    
    // ========== 出力 ==========
    cv::Mat toMat() const;
    sensor_msgs::msg::Image::SharedPtr toImageMsg() const;
    sensor_msgs::msg::Image toImageMsgCopy() const;
    void copyTo(cv::Mat& dst) const;
    bool save(const std::string& filename) const;
    
    // ========== ユーティリティ ==========
    cv::Size size() const;
    int width() const;
    int height() const;
    int channels() const;
    int depth() const;
    bool empty() const;
    
    // デバッグ
    FluentVision& show(const std::string& window_name = "FluentVision", int delay = 0);
    FluentVision& print(const std::string& label = "");
    
    // ========== 便利な色定数 ==========
    static const cv::Scalar RED;
    static const cv::Scalar GREEN;
    static const cv::Scalar BLUE;
    static const cv::Scalar YELLOW;
    static const cv::Scalar CYAN;
    static const cv::Scalar MAGENTA;
    static const cv::Scalar WHITE;
    static const cv::Scalar BLACK;
    static const cv::Scalar ORANGE;
    static const cv::Scalar PURPLE;
    
private:
    cvx::Mat image_;  // cvxのMatクラスを使用
    std::shared_ptr<Pipeline> pipeline_;
    std::shared_ptr<GPUBackend> gpu_backend_;
    bool use_gpu_ = false;
    
    // Private constructors
    FluentVision(const cv::Mat& image);
    FluentVision(cv::Mat&& image);
    FluentVision(const cvx::Mat& image);
    FluentVision(cvx::Mat&& image);
    
    // Helper methods
    void ensureChannels(int required_channels);
    void applyPipeline();
};

// ========== パイプライン最適化 ==========
class Pipeline {
public:
    void addOperation(std::function<void(cv::Mat&)> op);
    void optimize();
    void execute(cv::Mat& image);
    void clear();
    
private:
    std::vector<std::function<void(cv::Mat&)>> operations_;
};

} // namespace fluent_vision

#endif // FLUENT_VISION_HPP