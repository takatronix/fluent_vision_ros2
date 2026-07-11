#ifdef __has_include
#  if __has_include(<cv_bridge/cv_bridge.hpp>)
#    include <cv_bridge/cv_bridge.hpp>
#  elif __has_include(<cv_bridge/cv_bridge.h>)
#    include <cv_bridge/cv_bridge.h>
#  else
#    error "cv_bridge header not found"
#  endif
#else
#  include <cv_bridge/cv_bridge.hpp>
#endif

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace
{
constexpr const char *kDefaultX3ById =
    "/dev/v4l/by-id/usb-Amba_Insta360_X3-video-index0";

bool path_exists(const std::string &path)
{
    if (path.empty()) {
        return false;
    }
    std::error_code ec;
    return std::filesystem::exists(path, ec);
}

std::string normalize_pixel_format(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return value;
}

}  // namespace

class FVInsta360X3Node : public rclcpp::Node
{
public:
    FVInsta360X3Node()
        : Node("fv_insta360_x3")
    {
        loadParameters();
        initializePublishers();

        running_ = true;
        capture_thread_ = std::thread(&FVInsta360X3Node::captureLoop, this);

        RCLCPP_INFO(get_logger(), "fv_insta360_x3 started");
    }

    ~FVInsta360X3Node() override
    {
        running_ = false;
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
        closeCamera();
    }

private:
    void loadParameters()
    {
        device_path_ = declare_parameter<std::string>("device.path", kDefaultX3ById);
        fallback_device_path_ = declare_parameter<std::string>("device.fallback_path", "/dev/video8");
        reconnect_delay_ms_ = declare_parameter<int>("device.reconnect_delay_ms", 1000);
        read_fail_reopen_threshold_ =
            declare_parameter<int>("device.read_fail_reopen_threshold", 30);

        width_ = declare_parameter<int>("camera.width", 1280);
        height_ = declare_parameter<int>("camera.height", 720);
        fps_ = declare_parameter<int>("camera.fps", 30);
        pixel_format_ = declare_parameter<std::string>("camera.pixel_format", "MJPG");
        rotate_180_ = declare_parameter<bool>("camera.rotate_180", false);

        raw_enabled_ = declare_parameter<bool>("streams.raw_enabled", true);
        compressed_enabled_ = declare_parameter<bool>("streams.compressed_enabled", true);
        camera_info_enabled_ = declare_parameter<bool>("streams.camera_info_enabled", true);

        image_topic_ = declare_parameter<std::string>("topics.image_raw", "/x3/image_raw");
        compressed_topic_ = declare_parameter<std::string>(
            "topics.image_raw_compressed", "/x3/image_raw/compressed");
        camera_info_topic_ =
            declare_parameter<std::string>("topics.camera_info", "/x3/camera_info");

        frame_id_ = declare_parameter<std::string>(
            "camera_info.frame_id", "x3_camera_optical_frame");
        compressed_quality_ = declare_parameter<int>("camera_info.compressed_quality", 85);
        fx_ = declare_parameter<double>("camera_info.fx", -1.0);
        fy_ = declare_parameter<double>("camera_info.fy", -1.0);
        cx_ = declare_parameter<double>("camera_info.cx", -1.0);
        cy_ = declare_parameter<double>("camera_info.cy", -1.0);
        distortion_ = declare_parameter<std::vector<double>>(
            "camera_info.distortion", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});

        reconnect_delay_ms_ = std::max(100, reconnect_delay_ms_);
        read_fail_reopen_threshold_ = std::max(1, read_fail_reopen_threshold_);
        fps_ = std::max(1, fps_);
        compressed_quality_ = std::max(1, std::min(100, compressed_quality_));
    }

    void initializePublishers()
    {
        const auto qos = rclcpp::SensorDataQoS();

        if (raw_enabled_) {
            image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, qos);
            RCLCPP_INFO(get_logger(), "image publisher: %s", image_topic_.c_str());
        }

        if (compressed_enabled_) {
            compressed_pub_ =
                create_publisher<sensor_msgs::msg::CompressedImage>(compressed_topic_, qos);
            RCLCPP_INFO(get_logger(), "compressed publisher: %s", compressed_topic_.c_str());
        }

        if (camera_info_enabled_) {
            camera_info_pub_ =
                create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, qos);
            RCLCPP_INFO(get_logger(), "camera_info publisher: %s", camera_info_topic_.c_str());
        }
    }

    std::string discoverDevicePath() const
    {
        std::vector<std::string> candidates;
        candidates.push_back(device_path_);

        std::error_code ec;
        const std::filesystem::path by_id_dir("/dev/v4l/by-id");
        if (std::filesystem::exists(by_id_dir, ec)) {
            for (const auto &entry : std::filesystem::directory_iterator(by_id_dir, ec)) {
                const std::string path = entry.path().string();
                const std::string name = entry.path().filename().string();
                if (name.find("Insta360_X3") != std::string::npos &&
                    name.find("video-index0") != std::string::npos) {
                    candidates.push_back(path);
                }
            }
        }

        candidates.push_back(fallback_device_path_);

        for (const auto &candidate : candidates) {
            if (path_exists(candidate)) {
                return candidate;
            }
        }

        return "";
    }

    bool openCamera()
    {
        closeCamera();

        const std::string selected_path = discoverDevicePath();
        if (selected_path.empty()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Insta360 X3 UVC device not found. Expected %s",
                device_path_.c_str());
            return false;
        }

        RCLCPP_INFO(get_logger(), "opening Insta360 X3 at %s", selected_path.c_str());
        // Some OpenCV builds reject V4L2 open-by-path ("can't be used to
        // capture by name") — resolve the by-id symlink to /dev/videoN and
        // open by index first, keeping the path forms as fallback.
        int device_index = -1;
        std::error_code ec;
        const auto resolved = std::filesystem::canonical(selected_path, ec);
        if (!ec) {
            const std::string real = resolved.string();
            const auto pos = real.rfind("video");
            if (pos != std::string::npos) {
                device_index = std::atoi(real.c_str() + pos + 5);
            }
        }
        const bool opened =
            (device_index >= 0 && camera_.open(device_index, cv::CAP_V4L2)) ||
            camera_.open(selected_path, cv::CAP_V4L2) ||
            camera_.open(selected_path);
        if (!opened) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "failed to open Insta360 X3 at %s (index %d)",
                selected_path.c_str(), device_index);
            return false;
        }

        applyCameraSettings();
        updateCameraInfo(width_, height_);

        const int actual_width = static_cast<int>(camera_.get(cv::CAP_PROP_FRAME_WIDTH));
        const int actual_height = static_cast<int>(camera_.get(cv::CAP_PROP_FRAME_HEIGHT));
        const double actual_fps = camera_.get(cv::CAP_PROP_FPS);
        RCLCPP_INFO(
            get_logger(),
            "Insta360 X3 opened: requested=%dx%d@%d actual=%dx%d@%.1f format=%s",
            width_, height_, fps_, actual_width, actual_height, actual_fps,
            pixel_format_.c_str());
        return true;
    }

    void closeCamera()
    {
        if (camera_.isOpened()) {
            camera_.release();
        }
    }

    void applyCameraSettings()
    {
        const std::string fmt = normalize_pixel_format(pixel_format_);
        if (fmt == "MJPG" || fmt == "MJPEG") {
            camera_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        } else if (fmt == "YUYV") {
            camera_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
        }

        if (width_ > 0 && height_ > 0) {
            camera_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
            camera_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
        }
        if (fps_ > 0) {
            camera_.set(cv::CAP_PROP_FPS, fps_);
        }
    }

    void captureLoop()
    {
        cv::Mat frame;
        int read_fail_streak = 0;
        int frame_count = 0;
        auto last_stats_time = std::chrono::steady_clock::now();

        while (running_) {
            const auto loop_start = std::chrono::steady_clock::now();

            if (!camera_.isOpened()) {
                if (!openCamera()) {
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(reconnect_delay_ms_));
                    continue;
                }
            }

            if (!camera_.read(frame) || frame.empty()) {
                read_fail_streak += 1;
                if (read_fail_streak == 1 || read_fail_streak % read_fail_reopen_threshold_ == 0) {
                    RCLCPP_WARN(
                        get_logger(),
                        "failed to read Insta360 X3 frame (streak=%d)",
                        read_fail_streak);
                }

                if (read_fail_streak >= read_fail_reopen_threshold_) {
                    RCLCPP_WARN(get_logger(), "reopening Insta360 X3 after read failures");
                    closeCamera();
                    read_fail_streak = 0;
                }

                std::this_thread::sleep_for(100ms);
                continue;
            }
            read_fail_streak = 0;

            if (rotate_180_) {
                cv::rotate(frame, frame, cv::ROTATE_180);
            }

            publishFrame(frame);

            frame_count += 1;
            const auto now = std::chrono::steady_clock::now();
            const auto elapsed_sec =
                std::chrono::duration_cast<std::chrono::seconds>(now - last_stats_time).count();
            if (elapsed_sec >= 10) {
                RCLCPP_INFO(
                    get_logger(), "Insta360 X3 frame rate: %.1f FPS",
                    static_cast<double>(frame_count) / static_cast<double>(elapsed_sec));
                frame_count = 0;
                last_stats_time = now;
            }

            const auto target_period = std::chrono::milliseconds(1000 / fps_);
            const auto elapsed_loop = std::chrono::steady_clock::now() - loop_start;
            if (elapsed_loop < target_period) {
                std::this_thread::sleep_for(target_period - elapsed_loop);
            }
        }
    }

    void publishFrame(const cv::Mat &frame)
    {
        const auto stamp = now();
        updateCameraInfo(frame.cols, frame.rows);

        if (image_pub_) {
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            msg->header.stamp = stamp;
            msg->header.frame_id = frame_id_;
            image_pub_->publish(*msg);
        }

        if (compressed_pub_ && compressed_pub_->get_subscription_count() > 0) {
            std::vector<uchar> jpeg_data;
            const std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, compressed_quality_};
            if (cv::imencode(".jpg", frame, jpeg_data, params)) {
                sensor_msgs::msg::CompressedImage msg;
                msg.header.stamp = stamp;
                msg.header.frame_id = frame_id_;
                msg.format = "jpeg";
                msg.data = std::move(jpeg_data);
                compressed_pub_->publish(msg);
            } else {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000,
                    "failed to JPEG-encode Insta360 X3 frame");
            }
        }

        if (camera_info_pub_) {
            camera_info_msg_.header.stamp = stamp;
            camera_info_msg_.header.frame_id = frame_id_;
            camera_info_pub_->publish(camera_info_msg_);
        }
    }

    void updateCameraInfo(int width, int height)
    {
        if (width <= 0 || height <= 0) {
            return;
        }

        camera_info_msg_.header.frame_id = frame_id_;
        camera_info_msg_.width = static_cast<uint32_t>(width);
        camera_info_msg_.height = static_cast<uint32_t>(height);
        camera_info_msg_.distortion_model = "plumb_bob";
        camera_info_msg_.d = distortion_.empty()
                                 ? std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0}
                                 : distortion_;

        const double fx = fx_ > 0.0 ? fx_ : static_cast<double>(width);
        const double fy = fy_ > 0.0 ? fy_ : static_cast<double>(width);
        const double cx = cx_ > 0.0 ? cx_ : static_cast<double>(width) / 2.0;
        const double cy = cy_ > 0.0 ? cy_ : static_cast<double>(height) / 2.0;

        camera_info_msg_.k = {
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0,
        };
        camera_info_msg_.r = {
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        };
        camera_info_msg_.p = {
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        };
    }

    std::string device_path_;
    std::string fallback_device_path_;
    std::string pixel_format_;
    std::string frame_id_;
    std::string image_topic_;
    std::string compressed_topic_;
    std::string camera_info_topic_;
    int reconnect_delay_ms_ = 1000;
    int read_fail_reopen_threshold_ = 30;
    int width_ = 1280;
    int height_ = 720;
    int fps_ = 30;
    int compressed_quality_ = 85;
    bool rotate_180_ = false;
    bool raw_enabled_ = true;
    bool compressed_enabled_ = true;
    bool camera_info_enabled_ = true;
    double fx_ = -1.0;
    double fy_ = -1.0;
    double cx_ = -1.0;
    double cy_ = -1.0;
    std::vector<double> distortion_;

    cv::VideoCapture camera_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

    std::atomic<bool> running_{false};
    std::thread capture_thread_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FVInsta360X3Node>());
    rclcpp::shutdown();
    return 0;
}
