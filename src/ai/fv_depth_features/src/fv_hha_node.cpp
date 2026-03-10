/**
 * fv_hha_node -- HHA depth-color fusion encoder
 *
 * Subscribes to color, depth, and camera_info topics and publishes an
 * HHA-encoded image (Gupta et al. 2014).
 *
 * HHA channels (published as BGR8):
 *   B = Horizontal disparity  (1/depth, normalised to 0-255)
 *   G = Height above ground   (camera-frame Y, normalised)
 *   R = Angle with gravity    (surface-normal vs gravity, 0-pi -> 0-255)
 */

#include <cmath>
#include <mutex>
#include <string>

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

class FvHhaNode : public rclcpp::Node {
public:
    FvHhaNode() : Node("fv_hha_node") {
        color_topic_    = get_param(*this, "color_topic",       std::string("~/color/image_raw"));
        depth_topic_    = get_param(*this, "depth_topic",       std::string("~/depth/image_rect_raw"));
        info_topic_     = get_param(*this, "camera_info_topic", std::string("~/camera_info"));
        hha_topic_      = get_param(*this, "hha_topic",         std::string("~/hha_topic"));
        depth_scale_    = get_param(*this, "depth_scale",       0.001);
        max_depth_      = get_param(*this, "max_depth",         3.0);
        min_depth_      = get_param(*this, "min_depth",         0.05);
        height_min_     = get_param(*this, "height_min",        -1.0);
        height_max_     = get_param(*this, "height_max",        1.0);
        gravity_axis_   = get_param(*this, "gravity_axis",      std::string("+y"));
        frame_id_       = get_param(*this, "frame_id",          std::string(""));
        hha_preview_topic_ = get_param(*this, "hha_preview_topic", std::string(""));
        hha_preview_quality_ = std::max(1, std::min(100, get_param(*this, "hha_preview_quality", 75)));

        auto qos = rclcpp::SensorDataQoS();
        sub_info_  = create_subscription<sensor_msgs::msg::CameraInfo>(
            info_topic_, qos, [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m){ on_info(m); });
        sub_color_ = create_subscription<sensor_msgs::msg::Image>(
            color_topic_, qos, [this](sensor_msgs::msg::Image::ConstSharedPtr m){ on_color(m); });
        sub_depth_ = create_subscription<sensor_msgs::msg::Image>(
            depth_topic_, qos, [this](sensor_msgs::msg::Image::ConstSharedPtr m){ on_depth(m); });

        pub_hha_ = create_publisher<sensor_msgs::msg::Image>(hha_topic_, 10);
        if (!hha_preview_topic_.empty())
            pub_hha_preview_ = create_publisher<sensor_msgs::msg::CompressedImage>(hha_preview_topic_, 10);

        RCLCPP_INFO(get_logger(),
            "FvHhaNode: color=%s depth=%s info=%s hha=%s preview=%s gravity=%s",
            color_topic_.c_str(), depth_topic_.c_str(), info_topic_.c_str(),
            hha_topic_.c_str(),
            hha_preview_topic_.empty() ? "(none)" : hha_preview_topic_.c_str(),
            gravity_axis_.c_str());
    }

private:
    // ---- Callbacks ----
    void on_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        fx_ = msg->k[0];
        fy_ = msg->k[4];
        cx_ = msg->k[2];
        cy_ = msg->k[5];
        has_info_ = true;
    }

    void on_color(sensor_msgs::msg::Image::ConstSharedPtr msg) {
        // Color is not used for HHA computation but reserved for future RGBD-HHA fusion
        (void)msg;
    }

    void on_depth(sensor_msgs::msg::Image::ConstSharedPtr msg) {
        double fx, fy, cx, cy;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (!has_info_) return;
            fx = fx_; fy = fy_; cx = cx_; cy = cy_;
        }

        // Convert depth to CV_32F in metres
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

        // Validity mask
        cv::Mat valid = (depth_m > static_cast<float>(min_depth_)) &
                        (depth_m < static_cast<float>(max_depth_));

        // --- Channel 1: Horizontal disparity ---
        cv::Mat disparity = cv::Mat::zeros(h, w, CV_32F);
        const float max_disp = static_cast<float>(1.0 / min_depth_);
        for (int r = 0; r < h; ++r) {
            const float* dptr = depth_m.ptr<float>(r);
            const uchar* vptr = valid.ptr<uchar>(r);
            float* optr = disparity.ptr<float>(r);
            for (int c = 0; c < w; ++c) {
                if (vptr[c]) {
                    optr[c] = std::min(1.0f / dptr[c] / max_disp, 1.0f);
                }
            }
        }

        // --- Channel 2: Height above ground ---
        cv::Mat height_norm = cv::Mat::zeros(h, w, CV_32F);
        const float h_range = static_cast<float>(height_max_ - height_min_);
        const float h_min_f = static_cast<float>(height_min_);
        const float fy_f = static_cast<float>(fy);
        const float cy_f = static_cast<float>(cy);
        for (int r = 0; r < h; ++r) {
            const float* dptr = depth_m.ptr<float>(r);
            const uchar* vptr = valid.ptr<uchar>(r);
            float* optr = height_norm.ptr<float>(r);
            const float v_off = (static_cast<float>(r) - cy_f) / fy_f;
            for (int c = 0; c < w; ++c) {
                if (vptr[c]) {
                    // Camera Y down, height = -y_3d
                    float height = -(v_off * dptr[c]);
                    optr[c] = std::clamp((height - h_min_f) / h_range, 0.0f, 1.0f);
                }
            }
        }

        // --- Channel 3: Angle with gravity ---
        // Compute surface normals from depth
        cv::Mat normals = compute_normals(depth_m, static_cast<float>(fx), fy_f, static_cast<float>(cx), cy_f);

        // Gravity direction
        cv::Vec3f gravity;
        if (gravity_axis_ == "y" || gravity_axis_ == "+y")        gravity = {0.0f, 1.0f, 0.0f};
        else if (gravity_axis_ == "-y")  gravity = {0.0f, -1.0f, 0.0f};
        else if (gravity_axis_ == "z" || gravity_axis_ == "+z")   gravity = {0.0f, 0.0f, 1.0f};
        else if (gravity_axis_ == "-z")  gravity = {0.0f, 0.0f, -1.0f};
        else                             gravity = {0.0f, 1.0f, 0.0f};

        cv::Mat angle_norm = cv::Mat::zeros(h, w, CV_32F);
        for (int r = 0; r < h; ++r) {
            const cv::Vec3f* nptr = normals.ptr<cv::Vec3f>(r);
            const uchar* vptr = valid.ptr<uchar>(r);
            float* optr = angle_norm.ptr<float>(r);
            for (int c = 0; c < w; ++c) {
                if (!vptr[c]) continue;
                const cv::Vec3f& n = nptr[c];
                float mag = cv::norm(n);
                if (mag < 1e-6f) continue;
                float cos_a = n.dot(gravity) / mag;
                cos_a = std::clamp(cos_a, -1.0f, 1.0f);
                optr[c] = std::acos(cos_a) / static_cast<float>(M_PI);
            }
        }

        // Assemble 3-channel uint8 BGR image
        cv::Mat ch_b, ch_g, ch_r;
        disparity.convertTo(ch_b, CV_8U, 255.0);
        height_norm.convertTo(ch_g, CV_8U, 255.0);
        angle_norm.convertTo(ch_r, CV_8U, 255.0);

        cv::Mat hha;
        std::vector<cv::Mat> channels = {ch_b, ch_g, ch_r};
        cv::merge(channels, hha);

        // Publish raw
        auto out = cv_bridge::CvImage(msg->header, "bgr8", hha).toImageMsg();
        if (!frame_id_.empty()) {
            out->header.frame_id = frame_id_;
        }
        pub_hha_->publish(*out);

        // Publish compressed preview
        if (pub_hha_preview_) {
            std::vector<uchar> buf;
            std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, hha_preview_quality_};
            cv::imencode(".jpg", hha, buf, params);
            auto comp = sensor_msgs::msg::CompressedImage();
            comp.header = out->header;
            comp.format = "jpeg";
            comp.data.assign(buf.begin(), buf.end());
            pub_hha_preview_->publish(comp);
        }
    }

    // ---- Surface normal computation (image-based) ----
    static cv::Mat compute_normals(const cv::Mat& depth_m, float fx, float fy, float cx, float cy) {
        const int h = depth_m.rows;
        const int w = depth_m.cols;
        cv::Mat normals(h, w, CV_32FC3, cv::Scalar(0, 0, 0));

        for (int r = 0; r < h - 1; ++r) {
            const float* d0 = depth_m.ptr<float>(r);
            const float* d1 = depth_m.ptr<float>(r + 1);
            cv::Vec3f* nptr = normals.ptr<cv::Vec3f>(r);
            for (int c = 0; c < w - 1; ++c) {
                float z = d0[c];
                if (z <= 0.0f) continue;
                float z_r = d0[c + 1];
                float z_d = d1[c];
                if (z_r <= 0.0f || z_d <= 0.0f) continue;

                // 3D points
                float x  = (static_cast<float>(c) - cx) / fx * z;
                float y  = (static_cast<float>(r) - cy) / fy * z;
                float xr = (static_cast<float>(c + 1) - cx) / fx * z_r;
                float yr = (static_cast<float>(r) - cy) / fy * z_r;
                float xd = (static_cast<float>(c) - cx) / fx * z_d;
                float yd = (static_cast<float>(r + 1) - cy) / fy * z_d;

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

    // ---- Members ----
    std::string color_topic_, depth_topic_, info_topic_, hha_topic_;
    std::string hha_preview_topic_;
    int hha_preview_quality_;
    std::string gravity_axis_, frame_id_;
    double depth_scale_, max_depth_, min_depth_;
    double height_min_, height_max_;

    std::mutex mtx_;
    bool has_info_ = false;
    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_color_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_hha_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_hha_preview_;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FvHhaNode>());
    rclcpp::shutdown();
    return 0;
}
