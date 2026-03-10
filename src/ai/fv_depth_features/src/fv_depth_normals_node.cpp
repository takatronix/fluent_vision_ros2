/**
 * fv_depth_normals_node -- C++ port of vlabor depth_normals_node.py
 *
 * Subscribes to color, depth, and camera_info topics.
 * Publishes:
 *   - Surface normals as 32FC3 image
 *   - RGBD (color + depth fused) as 32FC4 image
 *   - (optional) Normals preview as CompressedImage (JPEG BGR8)
 *
 * This replaces the Python depth_normals_node with native C++ for
 * better throughput on Jetson / ARM platforms.
 */

#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
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
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

template <typename T>
static T get_param(rclcpp::Node& node, const std::string& name, const T& def) {
    node.declare_parameter<T>(name, def);
    return node.get_parameter(name).template get_value<T>();
}

class FvDepthNormalsNode : public rclcpp::Node {
public:
    FvDepthNormalsNode() : Node("fv_depth_normals_node") {
        color_topic_   = get_param(*this, "color_topic",       std::string("~/color/image_raw"));
        depth_topic_   = get_param(*this, "depth_topic",       std::string("~/depth/image_rect_raw"));
        info_topic_    = get_param(*this, "camera_info_topic", std::string("~/camera_info"));
        rgbd_topic_    = get_param(*this, "rgbd_topic",        std::string("~/rgbd_topic"));
        normals_topic_ = get_param(*this, "normals_topic",     std::string("~/normals_topic"));
        depth_scale_   = get_param(*this, "depth_scale",       0.001);
        stride_        = std::max(1, get_param(*this, "stride", 2));
        publish_rgbd_  = get_param(*this, "publish_rgbd",      true);
        publish_normals_ = get_param(*this, "publish_normals", true);
        frame_id_      = get_param(*this, "frame_id",          std::string(""));
        normals_preview_topic_ = get_param(*this, "normals_preview_topic", std::string(""));
        normals_preview_quality_ = std::max(1, std::min(100, get_param(*this, "normals_preview_quality", 75)));

        auto qos = rclcpp::SensorDataQoS();
        sub_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            info_topic_, qos, [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m){ on_info(m); });
        sub_color_ = create_subscription<sensor_msgs::msg::Image>(
            color_topic_, qos, [this](sensor_msgs::msg::Image::ConstSharedPtr m){ on_color(m); });
        sub_depth_ = create_subscription<sensor_msgs::msg::Image>(
            depth_topic_, qos, [this](sensor_msgs::msg::Image::ConstSharedPtr m){ on_depth(m); });

        if (publish_rgbd_)
            pub_rgbd_ = create_publisher<sensor_msgs::msg::Image>(rgbd_topic_, 10);
        if (publish_normals_)
            pub_normals_ = create_publisher<sensor_msgs::msg::Image>(normals_topic_, 10);
        if (!normals_preview_topic_.empty())
            pub_normals_preview_ = create_publisher<sensor_msgs::msg::CompressedImage>(normals_preview_topic_, 10);

        RCLCPP_INFO(get_logger(),
            "FvDepthNormalsNode: color=%s depth=%s info=%s rgbd=%s normals=%s preview=%s stride=%d",
            color_topic_.c_str(), depth_topic_.c_str(), info_topic_.c_str(),
            rgbd_topic_.c_str(), normals_topic_.c_str(),
            normals_preview_topic_.empty() ? "(none)" : normals_preview_topic_.c_str(),
            stride_);
    }

private:
    void on_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        fx_ = msg->k[0];
        fy_ = msg->k[4];
        cx_ = msg->k[2];
        cy_ = msg->k[5];
        has_info_ = true;
    }

    void on_color(sensor_msgs::msg::Image::ConstSharedPtr msg) {
        cv_bridge::CvImageConstPtr cv_color;
        try {
            cv_color = cv_bridge::toCvShare(msg, "bgr8");
        } catch (...) { return; }
        std::lock_guard<std::mutex> lk(mtx_);
        last_color_ = cv_color->image.clone();
        has_color_ = true;
    }

    void on_depth(sensor_msgs::msg::Image::ConstSharedPtr msg) {
        double fx, fy, cx, cy;
        cv::Mat color;
        bool has_color;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (!has_info_) return;
            fx = fx_; fy = fy_; cx = cx_; cy = cy_;
            has_color = has_color_;
            if (has_color_) color = last_color_.clone();
        }

        cv_bridge::CvImageConstPtr cv_depth;
        try {
            cv_depth = cv_bridge::toCvShare(msg);
        } catch (...) { return; }

        cv::Mat depth_m;
        if (cv_depth->image.type() == CV_16UC1 || cv_depth->image.type() == CV_16SC1) {
            cv_depth->image.convertTo(depth_m, CV_32F, depth_scale_);
        } else if (cv_depth->image.type() == CV_32FC1) {
            depth_m = cv_depth->image;
        } else {
            return;
        }

        const int h = depth_m.rows;
        const int w = depth_m.cols;

        // --- RGBD ---
        if (publish_rgbd_ && pub_rgbd_ && has_color) {
            if (color.rows != h || color.cols != w) {
                cv::resize(color, color, cv::Size(w, h));
            }
            cv::Mat rgb_f;
            color.convertTo(rgb_f, CV_32FC3, 1.0 / 255.0);

            // Split BGR channels and combine with depth
            std::vector<cv::Mat> bgr;
            cv::split(rgb_f, bgr);
            cv::Mat rgbd;
            std::vector<cv::Mat> channels = {bgr[0], bgr[1], bgr[2], depth_m};
            cv::merge(channels, rgbd);

            auto out = cv_bridge::CvImage(msg->header, "32FC4", rgbd).toImageMsg();
            if (!frame_id_.empty()) out->header.frame_id = frame_id_;
            pub_rgbd_->publish(*out);
        }

        // --- Surface normals ---
        if ((publish_normals_ && pub_normals_) || pub_normals_preview_) {
            cv::Mat normals = compute_normals(depth_m,
                static_cast<float>(fx), static_cast<float>(fy),
                static_cast<float>(cx), static_cast<float>(cy), stride_);

            if (publish_normals_ && pub_normals_) {
                auto out = cv_bridge::CvImage(msg->header, "32FC3", normals).toImageMsg();
                if (!frame_id_.empty()) out->header.frame_id = frame_id_;
                pub_normals_->publish(*out);
            }

            // --- Normals preview (CompressedImage JPEG) ---
            if (pub_normals_preview_) {
                // normals range [-1, 1] → BGR8 [0, 255]
                cv::Mat vis;
                normals.convertTo(vis, CV_8UC3, 127.5, 127.5);
                std::vector<uchar> buf;
                std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, normals_preview_quality_};
                cv::imencode(".jpg", vis, buf, params);

                auto comp = sensor_msgs::msg::CompressedImage();
                comp.header = msg->header;
                if (!frame_id_.empty()) comp.header.frame_id = frame_id_;
                comp.format = "jpeg";
                comp.data = std::move(buf);
                pub_normals_preview_->publish(comp);
            }
        }
    }

    static cv::Mat compute_normals(const cv::Mat& depth_m, float fx, float fy, float cx, float cy, int stride) {
        const int h = depth_m.rows;
        const int w = depth_m.cols;
        cv::Mat normals(h, w, CV_32FC3, cv::Scalar(0, 0, 0));

        for (int r = 0; r < h - 1; ++r) {
            const float* d0 = depth_m.ptr<float>(r);
            const float* d1 = depth_m.ptr<float>(r + 1);
            cv::Vec3f* nptr = normals.ptr<cv::Vec3f>(r);

            for (int c = 0; c < w - 1; ++c) {
                if (stride > 1 && (r % stride != 0 || c % stride != 0)) continue;

                float z = d0[c];
                if (z <= 0.0f || std::isnan(z)) continue;
                float z_r = d0[c + 1];
                float z_d = d1[c];
                if (z_r <= 0.0f || z_d <= 0.0f || std::isnan(z_r) || std::isnan(z_d)) continue;

                float cf = static_cast<float>(c);
                float rf = static_cast<float>(r);

                float x  = (cf - cx) / fx * z;
                float y  = (rf - cy) / fy * z;
                float xr = (cf + 1.0f - cx) / fx * z_r;
                float yr = (rf - cy) / fy * z_r;
                float xd = (cf - cx) / fx * z_d;
                float yd = (rf + 1.0f - cy) / fy * z_d;

                cv::Vec3f v1(xr - x, yr - y, z_r - z);
                cv::Vec3f v2(xd - x, yd - y, z_d - z);
                cv::Vec3f n = v1.cross(v2);
                float mag = cv::norm(n);
                if (mag > 1e-6f) {
                    nptr[c] = n / mag;
                }
            }
        }
        return normals;
    }

    // Parameters
    std::string color_topic_, depth_topic_, info_topic_;
    std::string rgbd_topic_, normals_topic_, frame_id_;
    std::string normals_preview_topic_;
    int normals_preview_quality_;
    double depth_scale_;
    int stride_;
    bool publish_rgbd_, publish_normals_;

    // State
    std::mutex mtx_;
    bool has_info_ = false, has_color_ = false;
    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
    cv::Mat last_color_;

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_color_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_rgbd_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_normals_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_normals_preview_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FvDepthNormalsNode>());
    rclcpp::shutdown();
    return 0;
}
