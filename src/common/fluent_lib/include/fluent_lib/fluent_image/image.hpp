#pragma once

// Minimal samples:
//  - From ROS Image → draw → publish
//    fluent_image::Image img(*msg); img = img.to_bgr8();
//    img.text({20,40}, "Hello", {0,255,0});
//    pub->publish(sensor_msgs::msg::Image(img));

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <utility>
#include <type_traits>
#include <cv_bridge/cv_bridge.h>

namespace fluent_image {

// Thin wrapper around cv::Mat with automatic conversions to/from ROS Image
class Image {
public:
  Image() = default;
  Image(const cv::Mat &mat, std::string encoding = "bgr8")
    : mat_(mat), encoding_(std::move(encoding)) {}
  Image(cv::Mat &&mat, std::string encoding = "BGR8")
    : mat_(std::move(mat)), encoding_(std::move(encoding)) {}

  // Construct from ROS Image (cv_bridge inside). Copies data.
  explicit Image(const sensor_msgs::msg::Image &msg) {
    auto cv = cv_bridge::toCvCopy(msg, msg.encoding.empty() ? sensor_msgs::image_encodings::BGR8 : msg.encoding);
    mat_ = cv->image;
    encoding_ = cv->encoding;
  }

  // Implicit conversions (be cautious but match user's expectation of automatic conversion)
  operator cv::Mat&() { return mat_; }
  operator const cv::Mat&() const { return mat_; }

  operator sensor_msgs::msg::Image() const {
    // Header left default; caller can set if needed
    return *cv_bridge::CvImage(std_msgs::msg::Header{}, encoding_, mat_).toImageMsg();
  }

  // Accessors
  int w() const { return mat_.cols; }
  int h() const { return mat_.rows; }
  const std::string &encoding() const { return encoding_; }

  // Encoding helpers
  Image& assume_bgr8() { encoding_ = "bgr8"; return *this; }
  Image& assume_gray8() { encoding_ = "mono8"; return *this; }

  // Auto conversions (return new Image)
  Image to_bgr8() const {
    if (mat_.empty()) return *this;
    cv::Mat out;
    auto enc = lower(encoding_);
    if (enc == "bgr8" && mat_.type() == CV_8UC3) return Image(mat_.clone(), "bgr8");
    if (enc == "rgb8" && mat_.type() == CV_8UC3) {
      cv::cvtColor(mat_, out, cv::COLOR_RGB2BGR); return Image(out, "bgr8"); }
    if ((enc == "bgra8" || enc == "rgba8") && mat_.type() == CV_8UC4) {
      int code = (enc == "bgra8") ? cv::COLOR_BGRA2BGR : cv::COLOR_RGBA2BGR;
      cv::cvtColor(mat_, out, code); return Image(out, "bgr8"); }
    if ((enc == "mono8" || enc == "8uc1") && mat_.type() == CV_8UC1) {
      cv::cvtColor(mat_, out, cv::COLOR_GRAY2BGR); return Image(out, "bgr8"); }
    // Fallback: try convert by type
    if (mat_.type() == CV_8UC3) return Image(mat_.clone(), "bgr8");
    if (mat_.type() == CV_8UC1) { cv::cvtColor(mat_, out, cv::COLOR_GRAY2BGR); return Image(out, "bgr8"); }
    return Image(mat_.clone(), "bgr8");
  }

  Image to_gray8() const {
    if (mat_.empty()) return *this;
    cv::Mat out;
    auto enc = lower(encoding_);
    if ((enc == "mono8" || enc == "8uc1") && mat_.type() == CV_8UC1) return Image(mat_.clone(), "mono8");
    if (enc == "bgr8" && mat_.type() == CV_8UC3) { cv::cvtColor(mat_, out, cv::COLOR_BGR2GRAY); return Image(out, "mono8"); }
    if (enc == "rgb8" && mat_.type() == CV_8UC3) { cv::cvtColor(mat_, out, cv::COLOR_RGB2GRAY); return Image(out, "mono8"); }
    if ((enc == "bgra8" || enc == "rgba8") && mat_.type() == CV_8UC4) {
      int code = (enc == "bgra8") ? cv::COLOR_BGRA2GRAY : cv::COLOR_RGBA2GRAY; cv::cvtColor(mat_, out, code); return Image(out, "mono8"); }
    if (mat_.type() == CV_8UC3) { cv::cvtColor(mat_, out, cv::COLOR_BGR2GRAY); return Image(out, "mono8"); }
    return Image(mat_.clone(), "mono8");
  }

  // Normalize depth to 32F meters (depth_unit_m applied if input is 16U)
  Image to_depth32f(float depth_unit_m) const {
    if (mat_.empty()) return *this;
    cv::Mat out;
    if (mat_.type() == CV_16UC1) {
      mat_.convertTo(out, CV_32FC1, static_cast<double>(depth_unit_m));
      return Image(out, "32FC1");
    }
    if (mat_.type() == CV_32FC1) return Image(mat_.clone(), "32FC1");
    return Image(mat_.clone(), "32FC1");
  }

  // Basic draw helpers (chainable)
  Image& rect(const cv::Rect &r, const cv::Scalar &color, int thickness=2) {
    if (!mat_.empty()) cv::rectangle(mat_, r, color, thickness);
    return *this;
  }
  Image& text(const cv::Point &org, const std::string &s, const cv::Scalar &color,
              double scale=0.6, int th=2) {
    if (!mat_.empty()) cv::putText(mat_, s, org, cv::FONT_HERSHEY_SIMPLEX, scale, color, th);
    return *this;
  }

  // Encode to CompressedImage (jpeg/png). Quality: 1..100 for jpeg, 0..9 for png (mapped from 1..100)
  sensor_msgs::msg::CompressedImage to_compressed(int quality = 85, const std::string &format = "jpeg") const {
    sensor_msgs::msg::CompressedImage msg;
    if (mat_.empty()) { msg.format = format; return msg; }
    std::vector<int> params;
    std::string f = lower(format);
    if (f == "jpeg" || f == "jpg") {
      msg.format = "jpeg";
      params = {cv::IMWRITE_JPEG_QUALITY, std::clamp(quality, 1, 100)};
      cv::imencode(".jpg", mat_, msg.data, params);
    } else if (f == "png") {
      msg.format = "png";
      int level = std::clamp(quality, 1, 100) / 11; // 0..9 roughly from 1..100
      params = {cv::IMWRITE_PNG_COMPRESSION, level};
      cv::imencode(".png", mat_, msg.data, params);
    } else {
      msg.format = "jpeg";
      params = {cv::IMWRITE_JPEG_QUALITY, std::clamp(quality, 1, 100)};
      cv::imencode(".jpg", mat_, msg.data, params);
    }
    return msg;
  }

  sensor_msgs::msg::CompressedImage to_compressed(const std_msgs::msg::Header &hdr, int quality = 85, const std::string &format = "jpeg") const {
    auto msg = to_compressed(quality, format);
    msg.header = hdr;
    return msg;
  }

private:
  cv::Mat mat_;
  std::string encoding_ {"BGR8"};

  static std::string lower(const std::string &s) {
    std::string t = s; std::transform(t.begin(), t.end(), t.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); }); return t;
  }
};

// Pipe operator to enable function-style transforms: img | f
template <typename F>
auto operator|(const Image &img, F &&f) -> decltype(f(img)) { return f(img); }

// Overloads for common sources (cv::Mat, ROS Image)
inline Image make(const cv::Mat &m, const std::string &enc = "BGR8") { return Image(m, enc); }
inline Image make(const sensor_msgs::msg::Image &msg) { return Image(msg); }

} // namespace fluent_image

namespace fluent_image {
// Helper to attach header when creating Image msg
inline sensor_msgs::msg::Image to_msg(const Image &img, const std_msgs::msg::Header &hdr) {
  sensor_msgs::msg::Image out = static_cast<sensor_msgs::msg::Image>(img);
  out.header = hdr;
  return out;
}
}


