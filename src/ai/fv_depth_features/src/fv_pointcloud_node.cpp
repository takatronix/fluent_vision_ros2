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

#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

template <typename T>
static T get_param(rclcpp::Node& node, const std::string& name, const T& def) {
    node.declare_parameter<T>(name, def);
    return node.get_parameter(name).template get_value<T>();
}

class FvPointcloudNode : public rclcpp::Node {
public:
    FvPointcloudNode() : Node("fv_pointcloud_node") {
        depth_topic_      = get_param(*this, "depth_topic",       std::string("~/depth/image_rect_raw"));
        info_topic_       = get_param(*this, "camera_info_topic", std::string("~/camera_info"));
        cloud_topic_      = get_param(*this, "pointcloud_topic",  std::string("~/pointcloud_topic"));
        depth_scale_      = get_param(*this, "depth_scale",       0.001);
        min_depth_        = get_param(*this, "min_depth",         0.05);
        max_depth_        = get_param(*this, "max_depth",         3.0);
        stride_           = get_param(*this, "stride",            1);
        frame_id_         = get_param(*this, "frame_id",          std::string(""));

        if (stride_ < 1) stride_ = 1;

        auto qos = rclcpp::SensorDataQoS();
        sub_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            info_topic_, qos,
            [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) { on_info(m); });
        sub_depth_ = create_subscription<sensor_msgs::msg::Image>(
            depth_topic_, qos,
            [this](sensor_msgs::msg::Image::ConstSharedPtr m) { on_depth(m); });

        pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, 10);

        RCLCPP_INFO(get_logger(),
            "FvPointcloudNode: depth=%s info=%s out=%s scale=%.4f range=[%.2f,%.2f] stride=%d",
            depth_topic_.c_str(), info_topic_.c_str(), cloud_topic_.c_str(),
            depth_scale_, min_depth_, max_depth_, stride_);
    }

private:
    void on_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        fx_ = msg->k[0];
        fy_ = msg->k[4];
        cx_ = msg->k[2];
        cy_ = msg->k[5];
        has_info_ = (fx_ > 0 && fy_ > 0);
    }

    void on_depth(sensor_msgs::msg::Image::ConstSharedPtr msg) {
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

        // Count valid points for pre-allocation
        const int sh = (h + stride_ - 1) / stride_;
        const int sw = (w + stride_ - 1) / stride_;

        // Build point cloud (XYZ float32, 12 bytes per point)
        std::vector<float> points;
        points.reserve(static_cast<size_t>(sh * sw) * 3);

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
            }
        }

        const uint32_t num_points = static_cast<uint32_t>(points.size() / 3);
        if (num_points == 0) return;

        // Build PointCloud2 message (no PCL dependency)
        auto cloud = sensor_msgs::msg::PointCloud2();
        cloud.header = msg->header;
        if (!frame_id_.empty()) cloud.header.frame_id = frame_id_;

        cloud.height = 1;
        cloud.width  = num_points;
        cloud.is_bigendian = false;
        cloud.is_dense = true;
        cloud.point_step = 12;  // 3 * float32
        cloud.row_step   = cloud.point_step * num_points;

        // Field descriptors
        sensor_msgs::msg::PointField fx_field, fy_field, fz_field;
        fx_field.name = "x"; fx_field.offset = 0;
        fx_field.datatype = sensor_msgs::msg::PointField::FLOAT32; fx_field.count = 1;
        fy_field.name = "y"; fy_field.offset = 4;
        fy_field.datatype = sensor_msgs::msg::PointField::FLOAT32; fy_field.count = 1;
        fz_field.name = "z"; fz_field.offset = 8;
        fz_field.datatype = sensor_msgs::msg::PointField::FLOAT32; fz_field.count = 1;
        cloud.fields = {fx_field, fy_field, fz_field};

        // Copy data
        const size_t data_size = points.size() * sizeof(float);
        cloud.data.resize(data_size);
        std::memcpy(cloud.data.data(), points.data(), data_size);

        pub_cloud_->publish(cloud);
    }

    // Parameters
    std::string depth_topic_, info_topic_, cloud_topic_, frame_id_;
    double depth_scale_, min_depth_, max_depth_;
    int stride_;

    // Camera intrinsics
    std::mutex mtx_;
    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
    bool has_info_ = false;

    // ROS
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
