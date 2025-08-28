#include "fluent_vision/fluent_vision.hpp"
#include <cv_bridge/cv_bridge.h>
#include <cvx/cvx.hpp>

namespace fluent_vision {

// ========== 色定数の定義 ==========
const cv::Scalar FluentVision::RED = cv::Scalar(0, 0, 255);
const cv::Scalar FluentVision::GREEN = cv::Scalar(0, 255, 0);
const cv::Scalar FluentVision::BLUE = cv::Scalar(255, 0, 0);
const cv::Scalar FluentVision::YELLOW = cv::Scalar(0, 255, 255);
const cv::Scalar FluentVision::CYAN = cv::Scalar(255, 255, 0);
const cv::Scalar FluentVision::MAGENTA = cv::Scalar(255, 0, 255);
const cv::Scalar FluentVision::WHITE = cv::Scalar(255, 255, 255);
const cv::Scalar FluentVision::BLACK = cv::Scalar(0, 0, 0);
const cv::Scalar FluentVision::ORANGE = cv::Scalar(0, 165, 255);
const cv::Scalar FluentVision::PURPLE = cv::Scalar(128, 0, 128);

// ========== コンストラクタ ==========
FluentVision::FluentVision(const cv::Mat& image) 
    : image_(cvx::Mat(image)), 
      pipeline_(std::make_shared<Pipeline>()) {}

FluentVision::FluentVision(cv::Mat&& image) 
    : image_(cvx::Mat(std::move(image))), 
      pipeline_(std::make_shared<Pipeline>()) {}

FluentVision::FluentVision(const cvx::Mat& image) 
    : image_(image), 
      pipeline_(std::make_shared<Pipeline>()) {}

FluentVision::FluentVision(cvx::Mat&& image) 
    : image_(std::move(image)), 
      pipeline_(std::make_shared<Pipeline>()) {}

// ========== ファクトリメソッド ==========
FluentVision FluentVision::from(const cv::Mat& image) {
    return FluentVision(image);
}

FluentVision FluentVision::from(cv::Mat&& image) {
    return FluentVision(std::move(image));
}

FluentVision FluentVision::from(const sensor_msgs::msg::Image::SharedPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    return FluentVision(cv_ptr->image);
}

FluentVision FluentVision::from(const sensor_msgs::msg::Image& msg) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg.encoding);
    return FluentVision(cv_ptr->image);
}

// ========== ぼかし系フィルタ ==========
FluentVision& FluentVision::blur(double radius) {
    int kernel_size = static_cast<int>(radius * 2) | 1;
    image_ = image_.blur(kernel_size);  // cvxの美しいAPI
    return *this;
}

FluentVision& FluentVision::gaussianBlur(double sigma) {
    int kernel_size = static_cast<int>(sigma * 6) | 1;
    image_ = image_.gaussianBlur(kernel_size, sigma);  // cvxのチェーン可能API
    return *this;
}

FluentVision& FluentVision::medianBlur(int kernel_size) {
    image_ = image_.medianBlur(kernel_size);  // cvxのAPI
    return *this;
}

FluentVision& FluentVision::bilateralFilter(double sigma_color, double sigma_space) {
    image_ = image_.bilateralFilter(9, sigma_color, sigma_space);  // cvxのAPI
    return *this;
}

FluentVision& FluentVision::motionBlur(double angle, int kernel_size) {
    cv::Mat kernel = cv::Mat::zeros(kernel_size, kernel_size, CV_32F);
    
    // Create motion blur kernel
    double radian = angle * CV_PI / 180.0;
    int cx = kernel_size / 2;
    int cy = kernel_size / 2;
    
    for (int i = 0; i < kernel_size; ++i) {
        int x = static_cast<int>(cx + (i - cx) * cos(radian));
        int y = static_cast<int>(cy + (i - cy) * sin(radian));
        if (x >= 0 && x < kernel_size && y >= 0 && y < kernel_size) {
            kernel.at<float>(y, x) = 1.0f;
        }
    }
    
    kernel /= cv::sum(kernel)[0];
    cv::Mat temp = cv::Mat(image_);
    cv::filter2D(temp, temp, -1, kernel);
    image_ = cvx::Mat(temp);
    return *this;
}

// ========== エッジ検出系 ==========
FluentVision& FluentVision::edge(double threshold) {
    return canny(threshold * 100, threshold * 300);
}

FluentVision& FluentVision::canny(double low_threshold, double high_threshold) {
    // cvxのチェーン可能なAPIを使用
    if (image_.channels() > 1) {
        image_ = image_.toGray().canny(low_threshold, high_threshold);
    } else {
        image_ = image_.canny(low_threshold, high_threshold);
    }
    return *this;
}

FluentVision& FluentVision::sobel(double scale) {
    // cvxのsobelメソッドを使用
    image_ = image_.sobel(1, 1, 3);  // dx=1, dy=1で両方向のエッジ
    return *this;
}

FluentVision& FluentVision::laplacian(double scale) {
    image_ = image_.laplacian(3);  // cvxのAPI
    return *this;
}

FluentVision& FluentVision::scharr(double scale) {
    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y;
    
    cv::Mat temp = cv::Mat(image_);
    cv::Scharr(temp, grad_x, CV_16S, 1, 0, scale);
    cv::Scharr(temp, grad_y, CV_16S, 0, 1, scale);
    
    cv::convertScaleAbs(grad_x, abs_grad_x);
    cv::convertScaleAbs(grad_y, abs_grad_y);
    
    cv::Mat result;
    cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, result);
    image_ = cvx::Mat(result);
    return *this;
}

// ========== 色調整系 ==========
FluentVision& FluentVision::color(double brightness, double contrast, double saturation) {
    this->brightness(brightness);
    this->contrast(contrast);
    if (saturation != 1.0 && image_.channels() >= 3) {
        this->saturation(saturation);
    }
    return *this;
}

FluentVision& FluentVision::brightness(double factor) {
    cv::Mat temp = cv::Mat(image_);
    temp.convertTo(temp, -1, 1.0, (factor - 1.0) * 127);
    image_ = cvx::Mat(temp);
    return *this;
}

FluentVision& FluentVision::contrast(double factor) {
    cv::Mat temp = cv::Mat(image_);
    temp.convertTo(temp, -1, factor, 0);
    image_ = cvx::Mat(temp);
    return *this;
}

FluentVision& FluentVision::saturation(double factor) {
    if (image_.channels() < 3) return *this;
    
    cv::Mat hsv;
    cv::Mat temp = cv::Mat(image_);
    cv::cvtColor(temp, hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    channels[1] *= factor;
    cv::merge(channels, hsv);
    cv::Mat result;
    cv::cvtColor(hsv, result, cv::COLOR_HSV2BGR);
    image_ = cvx::Mat(result);
    return *this;
}

FluentVision& FluentVision::hue(double shift) {
    if (image_.channels() < 3) return *this;
    
    cv::Mat hsv;
    cv::Mat temp = cv::Mat(image_);
    cv::cvtColor(temp, hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    channels[0] += shift;
    cv::merge(channels, hsv);
    cv::Mat result;
    cv::cvtColor(hsv, result, cv::COLOR_HSV2BGR);
    image_ = cvx::Mat(result);
    return *this;
}

FluentVision& FluentVision::gamma(double gamma) {
    cv::Mat lookUpTable(1, 256, CV_8U);
    uchar* p = lookUpTable.ptr();
    for (int i = 0; i < 256; ++i) {
        p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
    }
    cv::Mat temp = cv::Mat(image_);
    cv::Mat result;
    cv::LUT(temp, lookUpTable, result);
    image_ = cvx::Mat(result);
    return *this;
}

FluentVision& FluentVision::autoWhiteBalance() {
    if (image_.channels() < 3) return *this;
    
    std::vector<cv::Mat> channels;
    cv::Mat temp = cv::Mat(image_);
    cv::split(temp, channels);
    
    for (auto& channel : channels) {
        cv::equalizeHist(channel, channel);
    }
    
    cv::Mat result;
    cv::merge(channels, result);
    image_ = cvx::Mat(result);
    return *this;
}

FluentVision& FluentVision::histogram_equalization() {
    if (image_.channels() == 1) {
        cv::Mat temp = cv::Mat(image_);
        cv::equalizeHist(temp, temp);
        image_ = cvx::Mat(temp);
    } else {
        cv::Mat ycrcb;
        cv::Mat temp = cv::Mat(image_);
        cv::cvtColor(temp, ycrcb, cv::COLOR_BGR2YCrCb);
        std::vector<cv::Mat> channels;
        cv::split(ycrcb, channels);
        cv::equalizeHist(channels[0], channels[0]);
        cv::merge(channels, ycrcb);
        cv::Mat result;
        cv::cvtColor(ycrcb, result, cv::COLOR_YCrCb2BGR);
        image_ = cvx::Mat(result);
    }
    return *this;
}

// ========== 描画系 ==========
FluentVision& FluentVision::drawRectangle(const cv::Rect& rect, const cv::Scalar& color, int thickness) {
    cv::Mat temp = cv::Mat(image_);
    cv::rectangle(temp, rect, color, thickness);
    image_ = cvx::Mat(temp);
    return *this;
}

FluentVision& FluentVision::drawLine(const cv::Point& pt1, const cv::Point& pt2, const cv::Scalar& color, int thickness) {
    cv::Mat temp = cv::Mat(image_);
    cv::line(temp, pt1, pt2, color, thickness);
    image_ = cvx::Mat(temp);
    return *this;
}

FluentVision& FluentVision::drawCircle(const cv::Point& center, int radius, const cv::Scalar& color, int thickness) {
    cv::Mat temp = cv::Mat(image_);
    cv::circle(temp, center, radius, color, thickness);
    image_ = cvx::Mat(temp);
    return *this;
}

FluentVision& FluentVision::drawText(const std::string& text, const cv::Point& pos, 
                                     const cv::Scalar& color, double scale, int thickness) {
    cv::Mat temp = cv::Mat(image_);
    cv::putText(temp, text, pos, cv::FONT_HERSHEY_SIMPLEX, scale, color, thickness);
    image_ = cvx::Mat(temp);
    return *this;
}

FluentVision& FluentVision::drawPolyline(const std::vector<cv::Point>& points, const cv::Scalar& color, int thickness) {
    cv::Mat temp = cv::Mat(image_);
    const cv::Point* pts = points.data();
    int npts = static_cast<int>(points.size());
    cv::polylines(temp, &pts, &npts, 1, false, color, thickness);
    image_ = cvx::Mat(temp);
    return *this;
}

FluentVision& FluentVision::fillPolygon(const std::vector<cv::Point>& points, const cv::Scalar& color) {
    cv::Mat temp = cv::Mat(image_);
    const cv::Point* pts = points.data();
    int npts = static_cast<int>(points.size());
    cv::fillPoly(temp, &pts, &npts, 1, color);
    image_ = cvx::Mat(temp);
    return *this;
}

// ========== 変換系 ==========
FluentVision& FluentVision::resize(const cv::Size& size) {
    cv::Mat temp = cv::Mat(image_);
    cv::resize(temp, temp, size);
    image_ = cvx::Mat(temp);
    return *this;
}

FluentVision& FluentVision::resize(double scale) {
    cv::Mat temp = cv::Mat(image_);
    cv::resize(temp, temp, cv::Size(), scale, scale);
    image_ = cvx::Mat(temp);
    return *this;
}

FluentVision& FluentVision::rotate(double angle) {
    cv::Point2f center(image_.size().width / 2.0f, image_.size().height / 2.0f);
    cv::Mat M = cv::getRotationMatrix2D(center, angle, 1.0);
    cv::Mat temp = cv::Mat(image_);
    cv::warpAffine(temp, temp, M, image_.size());
    image_ = cvx::Mat(temp);
    return *this;
}

FluentVision& FluentVision::flip(bool horizontal, bool vertical) {
    int flip_code = horizontal ? (vertical ? -1 : 1) : 0;
    cv::Mat temp = cv::Mat(image_);
    cv::flip(temp, temp, flip_code);
    image_ = cvx::Mat(temp);
    return *this;
}

FluentVision& FluentVision::crop(const cv::Rect& roi) {
    cv::Mat temp = cv::Mat(image_);
    image_ = cvx::Mat(temp(roi).clone());
    return *this;
}

FluentVision& FluentVision::smartCrop(int width, int height) {
    // デフォルトサイズ（指定なしの場合は正方形に）
    if (width == 0 || height == 0) {
        int min_dim = std::min(image_.size().width, image_.size().height);
        width = height = min_dim;
    }
    
    cv::Mat img = image_.get();
    
    // 1. エッジ検出で重要な領域を見つける
    cv::Mat gray, edges;
    if (img.channels() > 1) {
        cv::Mat temp_img = cv::Mat(img);
        cv::cvtColor(temp_img, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = img;
    }
    cv::Canny(gray, edges, 50, 150);
    
    // 2. エッジの密度マップを作成
    cv::Mat edge_density;
    cv::blur(edges, edge_density, cv::Size(img.cols/10, img.rows/10));
    
    // 3. 最も密度の高い領域を探す
    cv::Point max_loc;
    cv::minMaxLoc(edge_density, nullptr, nullptr, nullptr, &max_loc);
    
    // 4. Rule of Thirdsも考慮
    std::vector<cv::Point> thirds_points = {
        cv::Point(img.cols/3, img.rows/3),
        cv::Point(2*img.cols/3, img.rows/3),
        cv::Point(img.cols/3, 2*img.rows/3),
        cv::Point(2*img.cols/3, 2*img.rows/3)
    };
    
    // 最も近い三分割点を選ぶ
    double min_dist = std::numeric_limits<double>::max();
    cv::Point best_point = max_loc;
    for (const auto& pt : thirds_points) {
        double dist = cv::norm(pt - max_loc);
        if (dist < min_dist && dist < img.cols/4) {  // 近すぎる場合のみ採用
            min_dist = dist;
            best_point = pt;
        }
    }
    
    // 5. クロップ領域を計算（境界チェック付き）
    int x = std::max(0, best_point.x - width/2);
    int y = std::max(0, best_point.y - height/2);
    x = std::min(x, img.cols - width);
    y = std::min(y, img.rows - height);
    
    cv::Rect roi(x, y, width, height);
    image_ = cvx::Mat(img(roi).clone());
    
    return *this;
}

// ========== 色空間変換 ==========
FluentVision& FluentVision::toGray() {
    if (image_.channels() > 1) {
        cv::Mat temp = cv::Mat(image_);
        cv::cvtColor(temp, temp, cv::COLOR_BGR2GRAY);
        image_ = cvx::Mat(temp);
    }
    return *this;
}

FluentVision& FluentVision::toBGR() {
    if (image_.channels() == 1) {
        cv::Mat temp = cv::Mat(image_);
        cv::cvtColor(temp, temp, cv::COLOR_GRAY2BGR);
        image_ = cvx::Mat(temp);
    }
    return *this;
}

FluentVision& FluentVision::toRGB() {
    if (image_.channels() == 1) {
        cv::Mat temp = cv::Mat(image_);
        cv::cvtColor(temp, temp, cv::COLOR_GRAY2RGB);
        image_ = cvx::Mat(temp);
    } else if (image_.channels() >= 3) {
        cv::Mat temp = cv::Mat(image_);
        cv::cvtColor(temp, temp, cv::COLOR_BGR2RGB);
        image_ = cvx::Mat(temp);
    }
    return *this;
}

// ========== 出力 ==========
cv::Mat FluentVision::toMat() const {
    return image_.get();  // cvx::Matからcv::Matへ変換
}

sensor_msgs::msg::Image::SharedPtr FluentVision::toImageMsg() const {
    cv_bridge::CvImage cv_image;
    cv_image.image = image_;
    cv_image.encoding = image_.channels() == 1 ? "mono8" : "bgr8";
    return cv_image.toImageMsg();
}

// ========== ユーティリティ ==========
cv::Size FluentVision::size() const {
    return image_.size();
}

int FluentVision::width() const {
    return image_.size().width;
}

int FluentVision::height() const {
    return image_.size().height;
}

int FluentVision::channels() const {
    return image_.channels();
}

bool FluentVision::empty() const {
    return image_.get().empty();
}

// ========== デバッグ ==========
FluentVision& FluentVision::show(const std::string& window_name, int delay) {
    cv::Mat temp = cv::Mat(image_);
    cv::imshow(window_name, temp);
    if (delay >= 0) {
        cv::waitKey(delay);
    }
    return *this;
}

FluentVision& FluentVision::print(const std::string& label) {
    std::cout << label << " - Size: " << image_.size() 
              << ", Channels: " << image_.channels() 
              << ", Type: " << image_.type() << std::endl;
    return *this;
}

// ========== Pipeline実装 ==========
void Pipeline::addOperation(std::function<void(cv::Mat&)> op) {
    operations_.push_back(op);
}

void Pipeline::optimize() {
    // TODO: パイプライン最適化の実装
    // 例: 連続するblurを統合、不要な色空間変換を削除など
}

void Pipeline::execute(cv::Mat& image) {
    for (const auto& op : operations_) {
        op(image);
    }
}

void Pipeline::clear() {
    operations_.clear();
}

} // namespace fluent_vision