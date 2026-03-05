/**
 * fv_pointcloud_node -- Depth image to PointCloud2 converter
 *
 * Subscribes to a depth image and camera_info, deprojects each pixel
 * to 3D (camera frame) and publishes as sensor_msgs/PointCloud2.
 *
 * Designed for VLA (Vision-Language-Action) model training pipelines
 * where point cloud input significantly outperforms 2D depth
 * representations (HHA, surface normals, raw depth images).
 *
 * Pipeline:  filtered depth + camera_info -> deproject -> PointCloud2
 *
 * No PCL dependency -- PointCloud2 is constructed manually for minimal
 * binary size and fast compilation.
 */

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/bool.hpp>
#if __has_include(<cv_bridge/cv_bridge.h>)
#include <cv_bridge/cv_bridge.h>
#elif __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge/cv_bridge.hpp>
#else
#error "cv_bridge header not found"
#endif
#include <opencv2/core.hpp>

template <typename T>
static T get_param(rclcpp::Node& node, const std::string& name, const T& def) {
    node.declare_parameter<T>(name, def);
    return node.get_parameter(name).template get_value<T>();
}

class FvPointcloudNode : public rclcpp::Node {
public:
    FvPointcloudNode() : Node("fv_pointcloud_node") {
        color_topic_      = get_param(*this, "color_topic",       std::string(""));
        depth_topic_      = get_param(*this, "depth_topic",       std::string("~/depth/image_rect_raw"));
        info_topic_       = get_param(*this, "camera_info_topic", std::string("~/camera_info"));
        cloud_topic_      = get_param(*this, "pointcloud_topic",  std::string("~/pointcloud_topic"));
        depth_scale_      = get_param(*this, "depth_scale",       0.001);
        min_depth_        = get_param(*this, "min_depth",         0.05);
        max_depth_        = get_param(*this, "max_depth",         3.0);
        stride_           = get_param(*this, "stride",            1);
        frame_id_         = get_param(*this, "frame_id",          std::string(""));
        color_timeout_ms_ = get_param(*this, "color_timeout_ms",  1000);

        if (stride_ < 1) stride_ = 1;

        auto qos = rclcpp::SensorDataQoS();
        if (!color_topic_.empty()) {
            sub_color_ = create_subscription<sensor_msgs::msg::Image>(
                color_topic_, qos,
                [this](sensor_msgs::msg::Image::ConstSharedPtr m) { on_color(m); });
        }
        sub_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            info_topic_, qos,
            [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) { on_info(m); });
        sub_depth_ = create_subscription<sensor_msgs::msg::Image>(
            depth_topic_, qos,
            [this](sensor_msgs::msg::Image::ConstSharedPtr m) { on_depth(m); });

        // Point cloud is for high-rate visualization/preview: prefer BEST_EFFORT
        // to avoid backpressure and reduce latency on weak links.
        pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, qos);

        // Runtime enable/disable via Bool topic (dashboard toggle)
        sub_enable_ = create_subscription<std_msgs::msg::Bool>(
            "~/enable", rclcpp::QoS(1).reliable(),
            [this](std_msgs::msg::Bool::ConstSharedPtr m) {
                enabled_.store(m->data);
                RCLCPP_INFO(get_logger(), "Pointcloud processing %s",
                            m->data ? "ENABLED" : "DISABLED");
            });

        RCLCPP_INFO(get_logger(),
            "FvPointcloudNode: color=%s depth=%s info=%s out=%s scale=%.4f range=[%.2f,%.2f] stride=%d color_timeout_ms=%d",
            color_topic_.empty() ? "(none)" : color_topic_.c_str(),
            depth_topic_.c_str(), info_topic_.c_str(), cloud_topic_.c_str(),
            depth_scale_, min_depth_, max_depth_, stride_, color_timeout_ms_);
    }

private:
    void on_color(sensor_msgs::msg::Image::ConstSharedPtr msg) {
        try {
            auto cv_color = cv_bridge::toCvCopy(msg, "bgr8");
            if (!cv_color || cv_color->image.empty()) return;

            std::lock_guard<std::mutex> lk(color_mtx_);
            latest_color_bgr_ = cv_color->image.clone();
            latest_color_stamp_ = rclcpp::Time(msg->header.stamp);
            has_color_ = true;
        } catch (...) {
            // Ignore bad color frame and keep using the latest valid one.
        }
    }

    void on_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        fx_ = msg->k[0];
        fy_ = msg->k[4];
        cx_ = msg->k[2];
        cy_ = msg->k[5];
        has_info_ = (fx_ > 0 && fy_ > 0);
    }

    void on_depth(sensor_msgs::msg::Image::ConstSharedPtr msg) {
        // Skip conversion work when nobody subscribes to point cloud output.
        if (!pub_cloud_ || pub_cloud_->get_subscription_count() == 0) {
            return;
        }
        // Skip when disabled by dashboard toggle
        if (!enabled_.load()) {
            return;
        }

        double fx, fy, cx, cy;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (!has_info_) return;
            fx = fx_; fy = fy_; cx = cx_; cy = cy_;
        }

        cv_bridge::CvImageConstPtr cv_depth;
        try {
            cv_depth = cv_bridge::toCvShare(msg);
        } catch (...) { return; }

        // Convert to float metres
        cv::Mat depth_m;
        if (cv_depth->image.type() == CV_16UC1) {
            cv_depth->image.convertTo(depth_m, CV_32F, depth_scale_);
        } else if (cv_depth->image.type() == CV_32FC1) {
            depth_m = cv_depth->image;
        } else {
            return;
        }

        const int h = depth_m.rows;
        const int w = depth_m.cols;
        if (h <= 0 || w <= 0) return;

        // Count valid points for pre-allocation
        const int sh = (h + stride_ - 1) / stride_;
        const int sw = (w + stride_ - 1) / stride_;

        cv::Mat color_bgr;
        bool has_fresh_color = false;
        const bool include_rgb = !color_topic_.empty();
        if (include_rgb) {
            std::lock_guard<std::mutex> lk(color_mtx_);
            if (has_color_ && !latest_color_bgr_.empty()) {
                const int64_t depth_stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
                const int64_t color_stamp_ns = latest_color_stamp_.nanoseconds();
                const int64_t dt_ms = std::llabs((depth_stamp_ns - color_stamp_ns) / 1000000);
                if (color_timeout_ms_ < 0 || dt_ms <= static_cast<int64_t>(color_timeout_ms_)) {
                    color_bgr = latest_color_bgr_;
                    has_fresh_color = (color_bgr.type() == CV_8UC3);
                }
            }
        }
        const int color_h = has_fresh_color ? color_bgr.rows : 0;
        const int color_w = has_fresh_color ? color_bgr.cols : 0;

        // Build point cloud as float array: XYZ (3) or XYZRGB-packed-float (4).
        std::vector<float> points;
        const size_t floats_per_point = include_rgb ? 4u : 3u;
        points.reserve(static_cast<size_t>(sh * sw) * floats_per_point);

        for (int r = 0; r < h; r += stride_) {
            const float* dptr = depth_m.ptr<float>(r);
            const float rf = static_cast<float>(r);
            for (int c = 0; c < w; c += stride_) {
                float z = dptr[c];
                if (z <= 0.0f || !std::isfinite(z)) continue;
                if (z < static_cast<float>(min_depth_) ||
                    z > static_cast<float>(max_depth_)) continue;

                float cf = static_cast<float>(c);
                float x = (cf - static_cast<float>(cx)) * z / static_cast<float>(fx);
                float y = (rf - static_cast<float>(cy)) * z / static_cast<float>(fy);

                points.push_back(x);
                points.push_back(y);
                points.push_back(z);

                if (include_rgb) {
                    uint32_t packed_rgb = 0;
                    if (has_fresh_color && color_h > 0 && color_w > 0) {
                        const int rr = std::min(color_h - 1, (r * color_h) / h);
                        const int cc = std::min(color_w - 1, (c * color_w) / w);
                        const cv::Vec3b bgr = color_bgr.at<cv::Vec3b>(rr, cc);
                        packed_rgb =
                            (static_cast<uint32_t>(bgr[2]) << 16) |
                            (static_cast<uint32_t>(bgr[1]) << 8) |
                            static_cast<uint32_t>(bgr[0]);
                    } else {
                        // If color frame is temporarily unavailable, keep schema stable
                        // with depth-based grayscale as fallback.
                        const float t = std::clamp(
                            (z - static_cast<float>(min_depth_)) /
                            std::max(0.001f, static_cast<float>(max_depth_ - min_depth_)),
                            0.0f, 1.0f);
                        const uint8_t g = static_cast<uint8_t>((1.0f - t) * 255.0f);
                        packed_rgb =
                            (static_cast<uint32_t>(g) << 16) |
                            (static_cast<uint32_t>(g) << 8) |
                            static_cast<uint32_t>(g);
                    }
                    float rgb_as_float = 0.0f;
                    std::memcpy(&rgb_as_float, &packed_rgb, sizeof(float));
                    points.push_back(rgb_as_float);
                }
            }
        }

        const uint32_t num_points = static_cast<uint32_t>(points.size() / floats_per_point);
        if (num_points == 0) return;

        // Build PointCloud2 message (no PCL dependency)
        auto cloud = sensor_msgs::msg::PointCloud2();
        cloud.header = msg->header;
        if (!frame_id_.empty()) cloud.header.frame_id = frame_id_;

        cloud.height = 1;
        cloud.width  = num_points;
        cloud.is_bigendian = false;
        cloud.is_dense = true;
        cloud.point_step = include_rgb ? 16 : 12;
        cloud.row_step   = cloud.point_step * num_points;

        // Field descriptors
        sensor_msgs::msg::PointField fx_field, fy_field, fz_field;
        fx_field.name = "x"; fx_field.offset = 0;
        fx_field.datatype = sensor_msgs::msg::PointField::FLOAT32; fx_field.count = 1;
        fy_field.name = "y"; fy_field.offset = 4;
        fy_field.datatype = sensor_msgs::msg::PointField::FLOAT32; fy_field.count = 1;
        fz_field.name = "z"; fz_field.offset = 8;
        fz_field.datatype = sensor_msgs::msg::PointField::FLOAT32; fz_field.count = 1;
        if (include_rgb) {
            sensor_msgs::msg::PointField frgb_field;
            frgb_field.name = "rgb"; frgb_field.offset = 12;
            frgb_field.datatype = sensor_msgs::msg::PointField::FLOAT32; frgb_field.count = 1;
            cloud.fields = {fx_field, fy_field, fz_field, frgb_field};
        } else {
            cloud.fields = {fx_field, fy_field, fz_field};
        }

        // Copy data
        const size_t data_size = points.size() * sizeof(float);
        cloud.data.resize(data_size);
        std::memcpy(cloud.data.data(), points.data(), data_size);

        pub_cloud_->publish(cloud);
    }

    // Parameters
    std::string color_topic_, depth_topic_, info_topic_, cloud_topic_, frame_id_;
    double depth_scale_, min_depth_, max_depth_;
    int stride_;
    int color_timeout_ms_;

    // Camera intrinsics
    std::mutex mtx_;
    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
    bool has_info_ = false;

    // Latest color frame cache
    std::mutex color_mtx_;
    cv::Mat latest_color_bgr_;
    rclcpp::Time latest_color_stamp_{0, 0, RCL_ROS_TIME};
    bool has_color_ = false;

    // Enable/disable control
    std::atomic<bool> enabled_{true};
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_color_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FvPointcloudNode>());
    rclcpp::shutdown();
    return 0;
}
