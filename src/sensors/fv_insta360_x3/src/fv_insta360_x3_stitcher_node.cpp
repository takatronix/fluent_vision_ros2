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

#include <builtin_interfaces/msg/time.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

namespace
{
constexpr double kPi = 3.14159265358979323846;

double deg2rad(double deg)
{
    return deg * kPi / 180.0;
}

double clamp(double value, double lo, double hi)
{
    return std::max(lo, std::min(hi, value));
}

cv::Vec3d normalize(const cv::Vec3d &v)
{
    const double n = cv::norm(v);
    if (n <= std::numeric_limits<double>::epsilon()) {
        return cv::Vec3d(1.0, 0.0, 0.0);
    }
    return v / n;
}

double dot(const cv::Vec3d &a, const cv::Vec3d &b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

cv::Vec3d rotateX(const cv::Vec3d &v, double angle_rad)
{
    const double c = std::cos(angle_rad);
    const double s = std::sin(angle_rad);
    return cv::Vec3d(v[0], c * v[1] - s * v[2], s * v[1] + c * v[2]);
}

cv::Vec3d rotateY(const cv::Vec3d &v, double angle_rad)
{
    const double c = std::cos(angle_rad);
    const double s = std::sin(angle_rad);
    return cv::Vec3d(c * v[0] - s * v[2], v[1], s * v[0] + c * v[2]);
}

cv::Vec3d rotateZ(const cv::Vec3d &v, double angle_rad)
{
    const double c = std::cos(angle_rad);
    const double s = std::sin(angle_rad);
    return cv::Vec3d(c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]);
}

}  // namespace

class FVInsta360X3StitcherNode : public rclcpp::Node
{
public:
    FVInsta360X3StitcherNode()
        : Node("fv_insta360_x3_stitcher")
    {
        loadParameters();
        initializeRosInterfaces();
        parameter_callback_handle_ = add_on_set_parameters_callback(
            std::bind(&FVInsta360X3StitcherNode::onSetParameters, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "fv_insta360_x3_stitcher started");
    }

private:
    struct LensConfig
    {
        double center_x = 0.5;
        double center_y = 0.25;
        double radius_x = 0.5;
        double radius_y = 0.25;
        double yaw_deg = 0.0;
        double roll_deg = 0.0;
    };

    struct Projection
    {
        bool valid = false;
        double score = -2.0;
        float x = -1.0F;
        float y = -1.0F;
    };

    void loadParameters()
    {
        input_topic_ = declare_parameter<std::string>("input_topic", "/x3/image_raw");
        input_layout_ = declare_parameter<std::string>("input_layout", "top_bottom");

        lens_fov_deg_ = declare_parameter<double>("calibration.lens_fov_deg", 190.0);
        front_lens_ = declare_parameter<std::string>("calibration.front_lens", "top");

        const bool front_is_bottom =
            input_layout_ == "top_bottom" && front_lens_ == "bottom";
        const bool front_is_right =
            input_layout_ == "left_right" && front_lens_ == "right";

        const double front_default_x = front_is_right ? 0.75 : 0.5;
        const double rear_default_x = front_is_right ? 0.25 : 0.5;
        const double front_default_y = front_is_bottom ? 0.75 : 0.25;
        const double rear_default_y = front_is_bottom ? 0.25 : 0.75;

        front_lens_config_.center_x =
            declare_parameter<double>("calibration.front.center_x", front_default_x);
        front_lens_config_.center_y =
            declare_parameter<double>("calibration.front.center_y", front_default_y);
        front_lens_config_.radius_x =
            declare_parameter<double>("calibration.front.radius_x", input_layout_ == "left_right" ? 0.25 : 0.5);
        front_lens_config_.radius_y =
            declare_parameter<double>("calibration.front.radius_y", input_layout_ == "left_right" ? 0.5 : 0.25);
        front_lens_config_.yaw_deg =
            declare_parameter<double>("calibration.front.yaw_deg", 0.0);
        front_lens_config_.roll_deg =
            declare_parameter<double>("calibration.front.roll_deg", 0.0);

        rear_lens_config_.center_x =
            declare_parameter<double>("calibration.rear.center_x", rear_default_x);
        rear_lens_config_.center_y =
            declare_parameter<double>("calibration.rear.center_y", rear_default_y);
        rear_lens_config_.radius_x =
            declare_parameter<double>("calibration.rear.radius_x", input_layout_ == "left_right" ? 0.25 : 0.5);
        rear_lens_config_.radius_y =
            declare_parameter<double>("calibration.rear.radius_y", input_layout_ == "left_right" ? 0.5 : 0.25);
        rear_lens_config_.yaw_deg =
            declare_parameter<double>("calibration.rear.yaw_deg", 180.0);
        rear_lens_config_.roll_deg =
            declare_parameter<double>("calibration.rear.roll_deg", 0.0);

        equirect_enabled_ =
            declare_parameter<bool>("equirectangular.enabled", true);
        equirect_topic_ =
            declare_parameter<std::string>("equirectangular.topic", "/x3/equirectangular/image_raw");
        equirect_compressed_topic_ =
            declare_parameter<std::string>("equirectangular.compressed_topic", "/x3/equirectangular/image_raw/compressed");
        equirect_width_ =
            declare_parameter<int>("equirectangular.width", 1280);
        equirect_height_ =
            declare_parameter<int>("equirectangular.height", 640);

        view_enabled_ = declare_parameter<bool>("view.enabled", true);
        view_image_topic_ =
            declare_parameter<std::string>("view.image_topic", "/x3/view/image_raw");
        view_compressed_topic_ =
            declare_parameter<std::string>("view.compressed_topic", "/x3/view/image_raw/compressed");
        view_camera_info_topic_ =
            declare_parameter<std::string>("view.camera_info_topic", "/x3/view/camera_info");
        view_frame_id_ =
            declare_parameter<std::string>("view.frame_id", "x3_view_optical_frame");
        view_width_ = declare_parameter<int>("view.width", 640);
        view_height_ = declare_parameter<int>("view.height", 480);
        view_yaw_deg_ = declare_parameter<double>("view.yaw_deg", 0.0);
        view_pitch_deg_ = declare_parameter<double>("view.pitch_deg", 0.0);
        view_roll_deg_ = declare_parameter<double>("view.roll_deg", 0.0);
        view_fov_deg_ = declare_parameter<double>("view.fov_deg", 90.0);
        compressed_quality_ = declare_parameter<int>("view.compressed_quality", 85);

        sanitizeParameters();
    }

    void sanitizeParameters()
    {
        lens_fov_deg_ = clamp(lens_fov_deg_, 1.0, 359.0);
        equirect_width_ = std::max(16, equirect_width_);
        equirect_height_ = std::max(8, equirect_height_);
        view_width_ = std::max(16, view_width_);
        view_height_ = std::max(8, view_height_);
        view_fov_deg_ = clamp(view_fov_deg_, 1.0, 179.0);
        compressed_quality_ = std::max(1, std::min(100, compressed_quality_));
    }

    void initializeRosInterfaces()
    {
        const auto qos = rclcpp::SensorDataQoS();

        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            input_topic_, qos,
            std::bind(&FVInsta360X3StitcherNode::onImage, this, std::placeholders::_1));

        if (equirect_enabled_) {
            equirect_pub_ = create_publisher<sensor_msgs::msg::Image>(equirect_topic_, qos);
            equirect_compressed_pub_ =
                create_publisher<sensor_msgs::msg::CompressedImage>(equirect_compressed_topic_, qos);
            RCLCPP_INFO(get_logger(), "equirectangular publisher: %s", equirect_topic_.c_str());
            RCLCPP_INFO(get_logger(), "equirectangular compressed publisher: %s", equirect_compressed_topic_.c_str());
        }

        if (view_enabled_) {
            view_pub_ = create_publisher<sensor_msgs::msg::Image>(view_image_topic_, qos);
            view_compressed_pub_ =
                create_publisher<sensor_msgs::msg::CompressedImage>(view_compressed_topic_, qos);
            view_camera_info_pub_ =
                create_publisher<sensor_msgs::msg::CameraInfo>(view_camera_info_topic_, qos);
            RCLCPP_INFO(get_logger(), "view publisher: %s", view_image_topic_.c_str());
            RCLCPP_INFO(get_logger(), "view compressed publisher: %s", view_compressed_topic_.c_str());
            RCLCPP_INFO(get_logger(), "view camera_info publisher: %s", view_camera_info_topic_.c_str());
        }

        RCLCPP_INFO(get_logger(), "input topic: %s", input_topic_.c_str());
    }

    rcl_interfaces::msg::SetParametersResult onSetParameters(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);

        for (const auto &param : parameters) {
            const std::string &name = param.get_name();
            if (name == "calibration.lens_fov_deg") {
                lens_fov_deg_ = param.as_double();
            } else if (name == "calibration.front.center_x") {
                front_lens_config_.center_x = param.as_double();
            } else if (name == "calibration.front.center_y") {
                front_lens_config_.center_y = param.as_double();
            } else if (name == "calibration.front.radius_x") {
                front_lens_config_.radius_x = param.as_double();
            } else if (name == "calibration.front.radius_y") {
                front_lens_config_.radius_y = param.as_double();
            } else if (name == "calibration.front.yaw_deg") {
                front_lens_config_.yaw_deg = param.as_double();
            } else if (name == "calibration.front.roll_deg") {
                front_lens_config_.roll_deg = param.as_double();
            } else if (name == "calibration.rear.center_x") {
                rear_lens_config_.center_x = param.as_double();
            } else if (name == "calibration.rear.center_y") {
                rear_lens_config_.center_y = param.as_double();
            } else if (name == "calibration.rear.radius_x") {
                rear_lens_config_.radius_x = param.as_double();
            } else if (name == "calibration.rear.radius_y") {
                rear_lens_config_.radius_y = param.as_double();
            } else if (name == "calibration.rear.yaw_deg") {
                rear_lens_config_.yaw_deg = param.as_double();
            } else if (name == "calibration.rear.roll_deg") {
                rear_lens_config_.roll_deg = param.as_double();
            } else if (name == "equirectangular.width") {
                equirect_width_ = static_cast<int>(param.as_int());
            } else if (name == "equirectangular.height") {
                equirect_height_ = static_cast<int>(param.as_int());
            } else if (name == "view.width") {
                view_width_ = static_cast<int>(param.as_int());
            } else if (name == "view.height") {
                view_height_ = static_cast<int>(param.as_int());
            } else if (name == "view.yaw_deg") {
                view_yaw_deg_ = param.as_double();
            } else if (name == "view.pitch_deg") {
                view_pitch_deg_ = param.as_double();
            } else if (name == "view.roll_deg") {
                view_roll_deg_ = param.as_double();
            } else if (name == "view.fov_deg") {
                view_fov_deg_ = param.as_double();
            } else if (name == "view.compressed_quality") {
                compressed_quality_ = static_cast<int>(param.as_int());
            }
        }

        sanitizeParameters();
        maps_dirty_ = true;

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    void onImage(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "failed to convert X3 input image to bgr8: %s", e.what());
            return;
        }

        const cv::Mat &input = cv_ptr->image;
        if (input.empty()) {
            return;
        }

        cv::Mat equirect_map_x;
        cv::Mat equirect_map_y;
        cv::Mat view_map_x;
        cv::Mat view_map_y;
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            ensureMapsLocked(input.cols, input.rows);
            equirect_map_x = equirect_map_x_;
            equirect_map_y = equirect_map_y_;
            view_map_x = view_map_x_;
            view_map_y = view_map_y_;
        }

        if (equirect_pub_ && !equirect_map_x.empty()) {
            cv::Mat equirect;
            cv::remap(input, equirect, equirect_map_x, equirect_map_y,
                      cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar::all(0));
            auto out = cv_bridge::CvImage(msg->header, "bgr8", equirect).toImageMsg();
            out->header.frame_id = "x3_equirectangular";
            equirect_pub_->publish(*out);
            publishCompressed(equirect, out->header, equirect_compressed_pub_);
        }

        if (view_pub_ && !view_map_x.empty()) {
            cv::Mat view;
            cv::remap(input, view, view_map_x, view_map_y,
                      cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar::all(0));
            auto out = cv_bridge::CvImage(msg->header, "bgr8", view).toImageMsg();
            out->header.frame_id = view_frame_id_;
            view_pub_->publish(*out);
            publishCompressed(view, out->header, view_compressed_pub_);

            if (view_camera_info_pub_) {
                auto info = makeViewCameraInfo(msg->header.stamp);
                view_camera_info_pub_->publish(info);
            }
        }
    }

    void publishCompressed(
        const cv::Mat &image,
        const std_msgs::msg::Header &header,
        const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr &publisher)
    {
        if (!publisher || publisher->get_subscription_count() == 0) {
            return;
        }

        std::vector<uchar> jpeg_data;
        const std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, compressed_quality_};
        if (!cv::imencode(".jpg", image, jpeg_data, params)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "failed to JPEG-encode stitched X3 frame");
            return;
        }

        sensor_msgs::msg::CompressedImage msg;
        msg.header = header;
        msg.format = "jpeg";
        msg.data = std::move(jpeg_data);
        publisher->publish(msg);
    }

    void ensureMapsLocked(int input_width, int input_height)
    {
        if (!maps_dirty_ && input_width == mapped_input_width_ &&
            input_height == mapped_input_height_) {
            return;
        }

        if (equirect_pub_) {
            buildEquirectMap(input_width, input_height);
        }
        if (view_pub_) {
            buildViewMap(input_width, input_height);
        }

        mapped_input_width_ = input_width;
        mapped_input_height_ = input_height;
        maps_dirty_ = false;
        RCLCPP_INFO(
            get_logger(),
            "rebuilt X3 stitch maps input=%dx%d equirect=%dx%d view=%dx%d",
            input_width, input_height, equirect_width_, equirect_height_,
            view_width_, view_height_);
    }

    void buildEquirectMap(int input_width, int input_height)
    {
        equirect_map_x_ = cv::Mat(equirect_height_, equirect_width_, CV_32FC1);
        equirect_map_y_ = cv::Mat(equirect_height_, equirect_width_, CV_32FC1);

        for (int y = 0; y < equirect_height_; ++y) {
            const double v = (static_cast<double>(y) + 0.5) / equirect_height_;
            const double lat = (0.5 - v) * kPi;
            const double cos_lat = std::cos(lat);

            for (int x = 0; x < equirect_width_; ++x) {
                const double u = (static_cast<double>(x) + 0.5) / equirect_width_;
                const double lon = (u - 0.5) * 2.0 * kPi;
                const cv::Vec3d dir(
                    cos_lat * std::cos(lon),
                    cos_lat * std::sin(lon),
                    std::sin(lat));

                float src_x = -1.0F;
                float src_y = -1.0F;
                if (projectDirection(dir, input_width, input_height, src_x, src_y)) {
                    equirect_map_x_.at<float>(y, x) = src_x;
                    equirect_map_y_.at<float>(y, x) = src_y;
                } else {
                    equirect_map_x_.at<float>(y, x) = -1.0F;
                    equirect_map_y_.at<float>(y, x) = -1.0F;
                }
            }
        }
    }

    void buildViewMap(int input_width, int input_height)
    {
        view_map_x_ = cv::Mat(view_height_, view_width_, CV_32FC1);
        view_map_y_ = cv::Mat(view_height_, view_width_, CV_32FC1);

        const double fov_rad = deg2rad(view_fov_deg_);
        const double tan_half = std::tan(fov_rad / 2.0);
        const double aspect = static_cast<double>(view_width_) / static_cast<double>(view_height_);
        const double yaw = deg2rad(view_yaw_deg_);
        const double pitch = deg2rad(view_pitch_deg_);
        const double roll = deg2rad(view_roll_deg_);

        for (int y = 0; y < view_height_; ++y) {
            const double py = 2.0 * (static_cast<double>(y) + 0.5) / view_height_ - 1.0;
            for (int x = 0; x < view_width_; ++x) {
                const double px = 2.0 * (static_cast<double>(x) + 0.5) / view_width_ - 1.0;

                cv::Vec3d dir(1.0, px * tan_half, -py * tan_half / aspect);
                dir = normalize(dir);
                dir = rotateX(dir, roll);
                dir = rotateY(dir, pitch);
                dir = rotateZ(dir, yaw);
                dir = normalize(dir);

                float src_x = -1.0F;
                float src_y = -1.0F;
                if (projectDirection(dir, input_width, input_height, src_x, src_y)) {
                    view_map_x_.at<float>(y, x) = src_x;
                    view_map_y_.at<float>(y, x) = src_y;
                } else {
                    view_map_x_.at<float>(y, x) = -1.0F;
                    view_map_y_.at<float>(y, x) = -1.0F;
                }
            }
        }
    }

    bool projectDirection(
        const cv::Vec3d &dir,
        int input_width,
        int input_height,
        float &src_x,
        float &src_y) const
    {
        const Projection front =
            projectWithLens(dir, front_lens_config_, input_width, input_height);
        const Projection rear =
            projectWithLens(dir, rear_lens_config_, input_width, input_height);

        const Projection best = (front.score >= rear.score) ? front : rear;
        if (!best.valid) {
            return false;
        }

        src_x = best.x;
        src_y = best.y;
        return true;
    }

    Projection projectWithLens(
        const cv::Vec3d &dir,
        const LensConfig &lens,
        int input_width,
        int input_height) const
    {
        Projection result;

        const double yaw = deg2rad(lens.yaw_deg);
        const double roll = deg2rad(lens.roll_deg);
        const cv::Vec3d forward(std::cos(yaw), std::sin(yaw), 0.0);
        const cv::Vec3d base_right(-std::sin(yaw), std::cos(yaw), 0.0);
        const cv::Vec3d base_up(0.0, 0.0, 1.0);
        const cv::Vec3d right =
            normalize(base_right * std::cos(roll) + base_up * std::sin(roll));
        const cv::Vec3d up =
            normalize(base_up * std::cos(roll) - base_right * std::sin(roll));

        const double score = dot(dir, forward);
        const double theta = std::acos(clamp(score, -1.0, 1.0));
        const double half_fov = deg2rad(lens_fov_deg_ / 2.0);
        if (theta > half_fov) {
            return result;
        }

        const double sin_theta = std::sin(theta);
        double plane_x = 0.0;
        double plane_y = 0.0;
        if (std::abs(sin_theta) > 1e-6) {
            plane_x = dot(dir, right) / sin_theta;
            plane_y = dot(dir, up) / sin_theta;
        }

        const double rho = theta / std::max(half_fov, 1e-6);
        const double cx = lens.center_x * input_width;
        const double cy = lens.center_y * input_height;
        const double rx = std::max(1.0, lens.radius_x * input_width);
        const double ry = std::max(1.0, lens.radius_y * input_height);

        const double src_x = cx + plane_x * rho * rx;
        const double src_y = cy - plane_y * rho * ry;
        if (src_x < 0.0 || src_y < 0.0 ||
            src_x >= static_cast<double>(input_width - 1) ||
            src_y >= static_cast<double>(input_height - 1)) {
            return result;
        }

        result.valid = true;
        result.score = score;
        result.x = static_cast<float>(src_x);
        result.y = static_cast<float>(src_y);
        return result;
    }

    sensor_msgs::msg::CameraInfo makeViewCameraInfo(
        const builtin_interfaces::msg::Time &stamp) const
    {
        sensor_msgs::msg::CameraInfo msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = view_frame_id_;
        msg.width = static_cast<uint32_t>(view_width_);
        msg.height = static_cast<uint32_t>(view_height_);
        msg.distortion_model = "plumb_bob";
        msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};

        const double fx =
            (static_cast<double>(view_width_) * 0.5) / std::tan(deg2rad(view_fov_deg_) * 0.5);
        const double fy = fx;
        const double cx = static_cast<double>(view_width_) * 0.5;
        const double cy = static_cast<double>(view_height_) * 0.5;

        msg.k = {
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0,
        };
        msg.r = {
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        };
        msg.p = {
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        };
        return msg;
    }

    std::string input_topic_;
    std::string input_layout_;
    std::string front_lens_;
    std::string equirect_topic_;
    std::string equirect_compressed_topic_;
    std::string view_image_topic_;
    std::string view_compressed_topic_;
    std::string view_camera_info_topic_;
    std::string view_frame_id_;

    LensConfig front_lens_config_;
    LensConfig rear_lens_config_;
    double lens_fov_deg_ = 190.0;

    bool equirect_enabled_ = true;
    int equirect_width_ = 1280;
    int equirect_height_ = 640;

    bool view_enabled_ = true;
    int view_width_ = 640;
    int view_height_ = 480;
    double view_yaw_deg_ = 0.0;
    double view_pitch_deg_ = 0.0;
    double view_roll_deg_ = 0.0;
    double view_fov_deg_ = 90.0;
    int compressed_quality_ = 85;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr equirect_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr equirect_compressed_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr view_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr view_compressed_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr view_camera_info_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

    std::mutex map_mutex_;
    bool maps_dirty_ = true;
    int mapped_input_width_ = -1;
    int mapped_input_height_ = -1;
    cv::Mat equirect_map_x_;
    cv::Mat equirect_map_y_;
    cv::Mat view_map_x_;
    cv::Mat view_map_y_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FVInsta360X3StitcherNode>());
    rclcpp::shutdown();
    return 0;
}
