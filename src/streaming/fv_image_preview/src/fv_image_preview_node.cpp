#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace
{

bool is_yuyv(const std::string & encoding)
{
  return encoding == "yuv422_yuy2" || encoding == "yuyv" || encoding == "yuyv422" ||
         encoding == "YUYV" || encoding == "YUY2";
}

cv::Mat resize_preview(const cv::Mat & image, int max_width)
{
  if (image.empty() || max_width <= 0 || image.cols <= max_width) {
    return image;
  }
  const double scale = static_cast<double>(max_width) / static_cast<double>(image.cols);
  cv::Mat out;
  cv::resize(
    image, out,
    cv::Size(max_width, std::max(1, static_cast<int>(image.rows * scale))),
    0.0, 0.0, cv::INTER_AREA);
  return out;
}

}  // namespace

class FVImagePreviewNode : public rclcpp::Node
{
public:
  FVImagePreviewNode()
  : Node("fv_image_preview_node")
  {
    input_topics_ = declare_parameter<std::vector<std::string>>("input_topics", std::vector<std::string>{});
    output_topics_ = declare_parameter<std::vector<std::string>>("output_topics", std::vector<std::string>{});
    labels_ = declare_parameter<std::vector<std::string>>("labels", std::vector<std::string>{});
    max_width_ = std::max(160, static_cast<int>(declare_parameter<int>("max_width", 480)));
    max_fps_ = std::max(0.2, declare_parameter<double>("max_fps", 2.0));
    jpeg_quality_ = std::clamp(static_cast<int>(declare_parameter<int>("jpeg_quality", 45)), 1, 100);
    diagnostics_interval_sec_ = std::max(0.0, declare_parameter<double>("diagnostics_interval_sec", 5.0));
    depth_min_mm_ = std::max(0.0, declare_parameter<double>("depth_min_mm", 200.0));
    depth_max_mm_ = std::max(depth_min_mm_ + 1.0, declare_parameter<double>("depth_max_mm", 5000.0));

    if (input_topics_.size() != output_topics_.size()) {
      throw std::runtime_error("input_topics and output_topics must have the same length");
    }
    if (labels_.size() < input_topics_.size()) {
      labels_.resize(input_topics_.size());
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    channels_.reserve(input_topics_.size());
    for (size_t i = 0; i < input_topics_.size(); ++i) {
      Channel channel;
      channel.label = labels_[i].empty() ? input_topics_[i] : labels_[i];
      channel.input_topic = input_topics_[i];
      channel.output_topic = output_topics_[i];
      channel.pub = create_publisher<sensor_msgs::msg::CompressedImage>(channel.output_topic, qos);
      channels_.push_back(channel);
    }

    for (size_t i = 0; i < channels_.size(); ++i) {
      channels_[i].sub = create_subscription<sensor_msgs::msg::Image>(
        channels_[i].input_topic, qos,
        [this, i](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          handle_image(i, msg);
        });
      RCLCPP_INFO(
        get_logger(), "preview channel: %s -> %s",
        channels_[i].input_topic.c_str(), channels_[i].output_topic.c_str());
    }

    RCLCPP_INFO(
      get_logger(),
      "fv_image_preview ready: channels=%zu max_width=%d max_fps=%.1f quality=%d",
      channels_.size(), max_width_, max_fps_, jpeg_quality_);

    if (diagnostics_interval_sec_ > 0.0) {
      diagnostics_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double>(diagnostics_interval_sec_)),
        [this]() { log_diagnostics(); });
    }
  }

private:
  struct Channel
  {
    std::string label;
    std::string input_topic;
    std::string output_topic;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;
    std::chrono::steady_clock::time_point last_input;
    std::chrono::steady_clock::time_point last_pub;
    uint64_t raw_frames{0};
    uint64_t encoded_frames{0};
    uint64_t skipped_no_sub{0};
    uint64_t conversion_errors{0};
  };

  void handle_image(size_t index, const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    auto & channel = channels_[index];
    const auto steady_now = std::chrono::steady_clock::now();
    channel.raw_frames++;
    channel.last_input = steady_now;
    if (channel.pub->get_subscription_count() == 0) {
      channel.skipped_no_sub++;
      return;
    }

    if (channel.last_pub.time_since_epoch().count() > 0) {
      const double dt = std::chrono::duration<double>(steady_now - channel.last_pub).count();
      if (dt < (1.0 / max_fps_)) {
        return;
      }
    }

    cv::Mat bgr;
    if (!to_bgr(*msg, channel.label, bgr)) {
      channel.conversion_errors++;
      return;
    }
    bgr = resize_preview(bgr, max_width_);
    if (bgr.empty()) {
      return;
    }

    std::vector<uchar> jpg;
    const std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
    if (!cv::imencode(".jpg", bgr, jpg, params)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "JPEG encode failed");
      return;
    }

    sensor_msgs::msg::CompressedImage out;
    out.header = msg->header;
    std::ostringstream format;
    format << "jpeg"
           << "; width=" << bgr.cols
           << "; height=" << bgr.rows
           << "; source_width=" << msg->width
           << "; source_height=" << msg->height
           << "; encoding=" << msg->encoding
           << "; label=" << channel.label;
    out.format = format.str();
    out.data.assign(jpg.begin(), jpg.end());
    channel.pub->publish(std::move(out));
    channel.last_pub = steady_now;
    channel.encoded_frames++;
  }

  bool to_bgr(const sensor_msgs::msg::Image & msg, const std::string & label, cv::Mat & bgr)
  {
    try {
      const int width = static_cast<int>(msg.width);
      const int height = static_cast<int>(msg.height);
      if (width <= 0 || height <= 0 || msg.data.empty()) {
        return false;
      }

      if (msg.encoding == "bgr8") {
        bgr = cv::Mat(height, width, CV_8UC3, const_cast<uint8_t *>(msg.data.data()), msg.step).clone();
        return true;
      }
      if (msg.encoding == "rgb8") {
        cv::Mat rgb(height, width, CV_8UC3, const_cast<uint8_t *>(msg.data.data()), msg.step);
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
        return true;
      }
      if (is_yuyv(msg.encoding)) {
        cv::Mat yuyv(height, width, CV_8UC2, const_cast<uint8_t *>(msg.data.data()), msg.step);
        cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUY2);
        return true;
      }
      if (msg.encoding == "mono8" || msg.encoding == "8UC1") {
        cv::Mat gray(height, width, CV_8UC1, const_cast<uint8_t *>(msg.data.data()), msg.step);
        cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
        return true;
      }
      if (msg.encoding == "16UC1" || msg.encoding == "mono16") {
        cv::Mat depth(height, width, CV_16UC1, const_cast<uint8_t *>(msg.data.data()), msg.step);
        cv::Mat scaled;
        const double scale = 255.0 / (depth_max_mm_ - depth_min_mm_);
        depth.convertTo(scaled, CV_8U, scale, -depth_min_mm_ * scale);
        cv::applyColorMap(scaled, bgr, cv::COLORMAP_TURBO);
        return true;
      }
      if (msg.encoding == "32FC1") {
        cv::Mat depth(height, width, CV_32FC1, const_cast<uint8_t *>(msg.data.data()), msg.step);
        cv::Mat clean = depth.clone();
        cv::patchNaNs(clean, 0.0);
        cv::Mat scaled;
        const double min_m = depth_min_mm_ / 1000.0;
        const double max_m = depth_max_mm_ / 1000.0;
        const double scale = 255.0 / (max_m - min_m);
        clean.convertTo(scaled, CV_8U, scale, -min_m * scale);
        cv::applyColorMap(scaled, bgr, cv::COLORMAP_TURBO);
        return true;
      }

      const size_t expected = static_cast<size_t>(width) * static_cast<size_t>(height);
      if (msg.data.size() >= expected) {
        cv::Mat gray(height, width, CV_8UC1, const_cast<uint8_t *>(msg.data.data()), msg.step);
        cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "%s unsupported encoding '%s'; using first byte plane", label.c_str(), msg.encoding.c_str());
        return true;
      }
    } catch (const cv::Exception & exc) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "%s OpenCV conversion failed: %s", label.c_str(), exc.what());
    }
    return false;
  }

  void log_diagnostics()
  {
    const auto now = std::chrono::steady_clock::now();
    for (const auto & channel : channels_) {
      const double input_age = channel.last_input.time_since_epoch().count() == 0
        ? -1.0
        : std::chrono::duration<double>(now - channel.last_input).count();
      const double pub_age = channel.last_pub.time_since_epoch().count() == 0
        ? -1.0
        : std::chrono::duration<double>(now - channel.last_pub).count();
      RCLCPP_INFO(
        get_logger(),
        "%s stats: sub=%zu raw=%llu encoded=%llu no_sub=%llu conversion_errors=%llu input_age=%.1fs pub_age=%.1fs",
        channel.label.c_str(),
        channel.pub->get_subscription_count(),
        static_cast<unsigned long long>(channel.raw_frames),
        static_cast<unsigned long long>(channel.encoded_frames),
        static_cast<unsigned long long>(channel.skipped_no_sub),
        static_cast<unsigned long long>(channel.conversion_errors),
        input_age,
        pub_age);
    }
  }

  std::vector<std::string> input_topics_;
  std::vector<std::string> output_topics_;
  std::vector<std::string> labels_;
  std::vector<Channel> channels_;
  int max_width_{480};
  double max_fps_{2.0};
  int jpeg_quality_{45};
  double diagnostics_interval_sec_{5.0};
  double depth_min_mm_{200.0};
  double depth_max_mm_{5000.0};
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FVImagePreviewNode>());
  rclcpp::shutdown();
  return 0;
}
