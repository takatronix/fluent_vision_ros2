#pragma once
/**
 * ImageFilterBase — Abstract base class for chainable image filter ROS2 nodes.
 *
 * Derived classes implement:
 *   on_parameters_changed()  — update cached values on live param change
 *   apply_filter()           — the actual cv::Mat → cv::Mat processing
 *
 * Derived constructors must:
 *   1) Declare filter-specific parameters using declare_and_get()
 *   2) Call init_filter() as the LAST step
 *
 * Chaining:  ros2 run ... --ros-args -p input_topic:=/cam/color -p output_topic:=/filtered
 *            Next node: -p input_topic:=/filtered  -p output_topic:=/final
 */

#include <string>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <fluent_lib/cv_bridge_compat.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>

// ── Convenience macro: each filter .cpp ends with this ──────────────
#define FV_FILTER_MAIN(FilterClass)                         \
    int main(int argc, char** argv) {                       \
        rclcpp::init(argc, argv);                           \
        rclcpp::spin(std::make_shared<FilterClass>());      \
        rclcpp::shutdown();                                 \
        return 0;                                           \
    }

// ── Base class ──────────────────────────────────────────────────────
class ImageFilterBase : public rclcpp::Node {
public:
    explicit ImageFilterBase(const std::string& node_name)
        : Node(node_name), node_name_(node_name) {}

protected:
    /// Call this at the END of the derived constructor, after declaring all params.
    void init_filter() {
        input_topic_  = declare_and_get<std::string>("input_topic",  "~/input");
        output_topic_ = declare_and_get<std::string>("output_topic", "~/output");

        // Register live-update callback
        param_cb_handle_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& params)
            -> rcl_interfaces::msg::SetParametersResult {
                std::lock_guard<std::mutex> lock(param_mutex_);
                on_parameters_changed(params);
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                return result;
            });

        auto qos = rclcpp::SensorDataQoS();
        sub_ = create_subscription<sensor_msgs::msg::Image>(
            input_topic_, qos,
            [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { on_image(msg); });

        pub_ = create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);

        RCLCPP_INFO(get_logger(), "%s: %s -> %s",
                    node_name_.c_str(), input_topic_.c_str(), output_topic_.c_str());
    }

    /// Called whenever parameters are changed at runtime (already under mutex).
    virtual void on_parameters_changed(const std::vector<rclcpp::Parameter>& params) = 0;

    /// Core processing — return the filtered image.
    virtual cv::Mat apply_filter(const cv::Mat& input) = 0;

    /// Override to customise output encoding (default: auto-detect from mat type).
    virtual std::string output_encoding(const cv::Mat& result,
                                        const std::string& input_encoding) {
        (void)input_encoding;
        return encoding_from_mat(result);
    }

    // ── Helpers ─────────────────────────────────────────────────────
    template <typename T>
    T declare_and_get(const std::string& name, const T& default_val) {
        declare_parameter<T>(name, default_val);
        return get_parameter(name).template get_value<T>();
    }

    /// Ensure a kernel size is positive and odd.
    static int ensure_odd(int k) { return (k < 1) ? 1 : (k | 1); }

    /// Convert BGR input to grayscale if needed, returning true if conversion happened.
    static bool to_gray(const cv::Mat& src, cv::Mat& gray) {
        if (src.channels() == 1) { gray = src; return false; }
        cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
        return true;
    }

    std::mutex param_mutex_;

private:
    void on_image(sensor_msgs::msg::Image::ConstSharedPtr msg) {
        cv_bridge::CvImageConstPtr cv_img;
        try {
            cv_img = cv_bridge::toCvShare(msg);
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "cv_bridge error: %s", e.what());
            return;
        }

        cv::Mat result;
        {
            std::lock_guard<std::mutex> lock(param_mutex_);
            try {
                result = apply_filter(cv_img->image);
            } catch (const cv::Exception& e) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "OpenCV error: %s", e.what());
                return;
            }
        }

        if (result.empty()) return;

        auto enc = output_encoding(result, cv_img->encoding);
        auto out_msg = cv_bridge::CvImage(msg->header, enc, result).toImageMsg();
        pub_->publish(*out_msg);
    }

    static std::string encoding_from_mat(const cv::Mat& mat) {
        switch (mat.type()) {
            case CV_8UC1:  return "mono8";
            case CV_8UC3:  return "bgr8";
            case CV_8UC4:  return "bgra8";
            case CV_16UC1: return "16UC1";
            case CV_32FC1: return "32FC1";
            default:       return "bgr8";
        }
    }

    std::string node_name_;
    std::string input_topic_, output_topic_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};
