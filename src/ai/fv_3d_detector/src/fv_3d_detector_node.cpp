// fv_3d_detector_node.cpp
// 3D cube detection: binary mask + detections + depth → 3D centroid, OBB, distance
// Overlay image with 3D info, distance/size filters
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <unordered_map>

#include "fv_msgs/msg/detection_array.hpp"
#include "fv_msgs/msg/object3_d.hpp"
#include "fv_msgs/msg/object3_d_array.hpp"

#include "fluent_lib/fluent_cloud/pipeline.hpp"

namespace fv_3d_detector {

using Cloud    = pcl::PointCloud<pcl::PointXYZRGB>;
using CloudPtr = Cloud::Ptr;
using Obj3D    = fv_msgs::msg::Object3D;

static geometry_msgs::msg::Quaternion axes_to_quaternion(
    const Eigen::Vector3f &ax0, const Eigen::Vector3f &ax1, const Eigen::Vector3f &ax2)
{
    Eigen::Matrix3f rot;
    rot.col(0) = ax0; rot.col(1) = ax1; rot.col(2) = ax2;
    if (rot.determinant() < 0.0f) rot.col(2) = -rot.col(2);
    Eigen::Quaternionf q(rot);
    q.normalize();
    geometry_msgs::msg::Quaternion msg;
    msg.x = q.x(); msg.y = q.y(); msg.z = q.z(); msg.w = q.w();
    return msg;
}

// Label color mapping
static cv::Scalar label_color(const std::string &label) {
    if (label == "red")   return cv::Scalar(0, 0, 255);
    if (label == "green") return cv::Scalar(0, 200, 0);
    if (label == "blue")  return cv::Scalar(255, 100, 0);
    if (label == "white") return cv::Scalar(200, 200, 200);
    return cv::Scalar(128, 128, 128);
}

class Detector3DNode : public rclcpp::Node {
public:
    explicit Detector3DNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("fv_3d_detector_node", options)
    {
        this->declare_parameter<std::string>("mask_topic", "/d405_cube/mask");
        this->declare_parameter<std::string>("detections_topic", "/d405_cube/detections");
        this->declare_parameter<std::string>("depth_topic", "/d405_depth/filtered");
        this->declare_parameter<std::string>("color_topic", "/d405_color/image_raw");
        this->declare_parameter<std::string>("camera_info_topic", "/d405_depth/camera_info");
        this->declare_parameter<std::string>("output_topic", "~/objects_3d");
        this->declare_parameter<bool>("publish_markers", true);
        this->declare_parameter<std::string>("marker_topic", "~/markers");
        this->declare_parameter<bool>("publish_overlay", true);
        this->declare_parameter<int>("min_points", 10);
        this->declare_parameter<double>("depth_scale", 0.001);
        // Distance filter
        this->declare_parameter<double>("min_depth_m", 0.05);
        this->declare_parameter<double>("max_depth_m", 3.0);
        // Size filter (extent in meters, 0 = disabled)
        this->declare_parameter<double>("filter.min_size_m", 0.0);
        this->declare_parameter<double>("filter.max_size_m", 0.0);
        // Distance filter on 3D centroid (0 = disabled)
        this->declare_parameter<double>("filter.min_distance_m", 0.0);
        this->declare_parameter<double>("filter.max_distance_m", 0.0);
        // Performance
        this->declare_parameter<double>("max_fps", 10.0);
        this->declare_parameter<int>("point_stride", 2);

        readParams();

        auto qos_be = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

        det_sub_ = this->create_subscription<fv_msgs::msg::DetectionArray>(
            det_topic_, qos_be,
            std::bind(&Detector3DNode::onDetections, this, std::placeholders::_1));
        mask_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            mask_topic_, qos_be,
            std::bind(&Detector3DNode::onMask, this, std::placeholders::_1));
        info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic_, qos_be,
            std::bind(&Detector3DNode::onCameraInfo, this, std::placeholders::_1));
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            depth_topic_, qos_be,
            std::bind(&Detector3DNode::onDepth, this, std::placeholders::_1));
        color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            color_topic_, qos_be,
            std::bind(&Detector3DNode::onColor, this, std::placeholders::_1));

        obj_pub_ = this->create_publisher<fv_msgs::msg::Object3DArray>(output_topic_, rclcpp::QoS(10));
        cloud_out_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "~/cloud", rclcpp::QoS(1).best_effort());
        if (publish_markers_) {
            marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                marker_topic_, rclcpp::QoS(10));
        }
        if (publish_overlay_) {
            overlay_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
                "~/overlay/compressed", rclcpp::QoS(1).best_effort());
        }

        RCLCPP_INFO(get_logger(), "fv_3d_detector_node ready: mask=%s det=%s → %s (overlay=%s)",
            mask_topic_.c_str(), det_topic_.c_str(), output_topic_.c_str(),
            publish_overlay_ ? "on" : "off");
    }

private:
    struct Intrinsics {
        double fx{0}, fy{0}, cx{0}, cy{0};
        bool valid() const { return fx > 0.0 && fy > 0.0; }
    };

    void readParams() {
        mask_topic_        = this->get_parameter("mask_topic").as_string();
        det_topic_         = this->get_parameter("detections_topic").as_string();
        depth_topic_       = this->get_parameter("depth_topic").as_string();
        color_topic_       = this->get_parameter("color_topic").as_string();
        camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
        output_topic_      = this->get_parameter("output_topic").as_string();
        publish_markers_   = this->get_parameter("publish_markers").as_bool();
        marker_topic_      = this->get_parameter("marker_topic").as_string();
        publish_overlay_   = this->get_parameter("publish_overlay").as_bool();
        min_points_        = std::max(3, static_cast<int>(this->get_parameter("min_points").as_int()));
        depth_scale_       = this->get_parameter("depth_scale").as_double();
        min_depth_m_       = static_cast<float>(this->get_parameter("min_depth_m").as_double());
        max_depth_m_       = static_cast<float>(this->get_parameter("max_depth_m").as_double());
        filter_min_size_   = static_cast<float>(this->get_parameter("filter.min_size_m").as_double());
        filter_max_size_   = static_cast<float>(this->get_parameter("filter.max_size_m").as_double());
        filter_min_dist_   = static_cast<float>(this->get_parameter("filter.min_distance_m").as_double());
        filter_max_dist_   = static_cast<float>(this->get_parameter("filter.max_distance_m").as_double());
        max_fps_           = this->get_parameter("max_fps").as_double();
        point_stride_      = std::max(1, static_cast<int>(this->get_parameter("point_stride").as_int()));
        if (max_fps_ > 0.0) min_interval_ns_ = static_cast<int64_t>(1.0e9 / max_fps_);
    }

    void onDepth(const sensor_msgs::msg::Image::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        last_depth_ = msg;
    }
    void onColor(const sensor_msgs::msg::Image::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        last_color_ = msg;
    }
    void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        intr_.fx = msg->k[0]; intr_.fy = msg->k[4];
        intr_.cx = msg->k[2]; intr_.cy = msg->k[5];
        camera_frame_ = msg->header.frame_id;
    }
    void onDetections(const fv_msgs::msg::DetectionArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        last_det_ = msg;
    }

    bool passFilters(float distance_m, float max_extent) const {
        if (filter_min_dist_ > 0.0f && distance_m < filter_min_dist_) return false;
        if (filter_max_dist_ > 0.0f && distance_m > filter_max_dist_) return false;
        if (filter_min_size_ > 0.0f && max_extent < filter_min_size_) return false;
        if (filter_max_size_ > 0.0f && max_extent > filter_max_size_) return false;
        return true;
    }

    void onMask(const sensor_msgs::msg::Image::SharedPtr mask_msg) {
        // FPS throttle
        if (min_interval_ns_ > 0) {
            auto now = std::chrono::steady_clock::now().time_since_epoch().count();
            if (now - last_process_ns_ < min_interval_ns_) return;
            last_process_ns_ = now;
        }

        fv_msgs::msg::DetectionArray::SharedPtr det_msg;
        sensor_msgs::msg::Image::SharedPtr depth_msg, color_msg;
        Intrinsics intr;
        std::string frame_id;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            det_msg   = last_det_;
            depth_msg = last_depth_;
            color_msg = last_color_;
            intr      = intr_;
            frame_id  = camera_frame_;
        }
        if (!det_msg || det_msg->detections.empty()) return;
        if (!depth_msg || !intr.valid()) return;
        if (mask_msg->encoding != "mono8" && mask_msg->encoding != "8UC1") return;

        const uint32_t mask_w = mask_msg->width;
        const uint32_t mask_h = mask_msg->height;
        const uint8_t *mask_data = mask_msg->data.data();
        if (frame_id.empty()) frame_id = depth_msg->header.frame_id;

        const uint16_t *depth_ptr = reinterpret_cast<const uint16_t*>(depth_msg->data.data());
        const uint32_t depth_w = depth_msg->width;
        const uint32_t depth_h = depth_msg->height;
        const uint32_t depth_step = depth_msg->step / sizeof(uint16_t);

        const uint8_t *color_ptr = nullptr;
        uint32_t color_w = 0, color_step = 0;
        int color_channels = 3;
        bool color_bgr = false;
        if (color_msg && !color_msg->data.empty()) {
            color_ptr = color_msg->data.data();
            color_w   = color_msg->width;
            color_step = color_msg->step;
            color_channels = (color_msg->encoding == "rgba8" || color_msg->encoding == "bgra8") ? 4 : 3;
            color_bgr = (color_msg->encoding == "bgr8" || color_msg->encoding == "bgra8");
        }

        std::unordered_map<int32_t, fv_msgs::msg::Detection2D> det_map;
        for (const auto &d : det_msg->detections) det_map[d.id] = d;

        fv_msgs::msg::Object3DArray out;
        out.header.stamp = mask_msg->header.stamp;
        out.header.frame_id = frame_id;

        visualization_msgs::msg::MarkerArray markers;
        int marker_id = 0;
        CloudPtr agg_cloud(new Cloud);

        // Overlay image (only when someone is subscribed)
        bool do_overlay = publish_overlay_ && color_msg && !color_msg->data.empty()
                          && overlay_pub_->get_subscription_count() > 0;
        cv::Mat overlay;
        if (do_overlay) {
            overlay = cv::Mat(color_msg->height, color_msg->width, CV_8UC3,
                const_cast<uint8_t*>(color_msg->data.data())).clone();
            if (!color_bgr) cv::cvtColor(overlay, overlay, cv::COLOR_RGB2BGR);
        }

        for (const auto &[det_id, det] : det_map) {
            int x_min = std::max(0, static_cast<int>(det.bbox_min.x));
            int y_min = std::max(0, static_cast<int>(det.bbox_min.y));
            int x_max = std::min(static_cast<int>(mask_w) - 1, static_cast<int>(det.bbox_max.x));
            int y_max = std::min(static_cast<int>(mask_h) - 1, static_cast<int>(det.bbox_max.y));
            if (x_min >= x_max || y_min >= y_max) continue;

            CloudPtr sub(new Cloud);
            sub->points.reserve(static_cast<size_t>((x_max - x_min) * (y_max - y_min) / 4));

            for (int py = y_min; py <= y_max; py += point_stride_) {
                for (int px = x_min; px <= x_max; px += point_stride_) {
                    if (mask_data[py * mask_msg->step + px] == 0) continue;
                    if (static_cast<uint32_t>(px) >= depth_w || static_cast<uint32_t>(py) >= depth_h) continue;
                    uint16_t dv = depth_ptr[py * depth_step + px];
                    if (dv == 0) continue;
                    float z = static_cast<float>(dv) * static_cast<float>(depth_scale_);
                    if (z < min_depth_m_ || z > max_depth_m_) continue;
                    float x = static_cast<float>((static_cast<double>(px) - intr.cx) * z / intr.fx);
                    float y = static_cast<float>((static_cast<double>(py) - intr.cy) * z / intr.fy);
                    pcl::PointXYZRGB pt;
                    pt.x = x; pt.y = y; pt.z = z;
                    if (color_ptr && static_cast<uint32_t>(px) < color_w) {
                        const uint8_t *cp = color_ptr + py * color_step + px * color_channels;
                        if (color_bgr) { pt.b = cp[0]; pt.g = cp[1]; pt.r = cp[2]; }
                        else           { pt.r = cp[0]; pt.g = cp[1]; pt.b = cp[2]; }
                    }
                    sub->points.push_back(pt);
                }
            }

            sub->width = sub->points.size(); sub->height = 1; sub->is_dense = false;
            if (static_cast<int>(sub->points.size()) < min_points_) continue;

            auto obb = fluent_cloud::compute_obb_metrics(sub);
            if (obb.num_points < static_cast<uint32_t>(min_points_)) continue;

            float distance = obb.center.norm();
            float max_extent = std::max({2.0f * obb.extents[0], 2.0f * obb.extents[1], 2.0f * obb.extents[2]});

            // Apply distance/size filters
            if (!passFilters(distance, max_extent)) continue;

            fv_msgs::msg::Object3D obj;
            obj.header = out.header;
            obj.id = det_id;
            obj.class_id   = det.class_id;
            obj.label      = det.label;
            obj.confidence = det.conf_fused;
            obj.centroid.x = obb.center.x();
            obj.centroid.y = obb.center.y();
            obj.centroid.z = obb.center.z();
            obj.orientation = axes_to_quaternion(obb.axes[0], obb.axes[1], obb.axes[2]);
            obj.extent.x = 2.0 * obb.extents[0];
            obj.extent.y = 2.0 * obb.extents[1];
            obj.extent.z = 2.0 * obb.extents[2];
            obj.distance_m = distance;
            obj.mean_r = obb.mean_r; obj.mean_g = obb.mean_g; obj.mean_b = obb.mean_b;
            obj.color  = Obj3D::COLOR_UNKNOWN;
            obj.shape  = Obj3D::SHAPE_CUBE;
            obj.num_points = obb.num_points;
            obj.eigen_ratio_01 = (obb.eigenvalues[0] > 1e-12f) ? obb.eigenvalues[1] / obb.eigenvalues[0] : 0.0f;
            obj.eigen_ratio_02 = (obb.eigenvalues[0] > 1e-12f) ? obb.eigenvalues[2] / obb.eigenvalues[0] : 0.0f;

            out.objects.push_back(obj);
            agg_cloud->points.insert(agg_cloud->points.end(), sub->points.begin(), sub->points.end());

            // --- Overlay ---
            if (do_overlay && !overlay.empty()) {
                cv::Scalar col = label_color(det.label);
                cv::rectangle(overlay, cv::Point(x_min, y_min), cv::Point(x_max, y_max), col, 2);

                // Project 3D centroid to 2D pixel
                int cx_px = static_cast<int>(intr.fx * obb.center.x() / obb.center.z() + intr.cx);
                int cy_px = static_cast<int>(intr.fy * obb.center.y() / obb.center.z() + intr.cy);

                // Yellow dot at centroid
                if (cx_px >= 0 && cx_px < static_cast<int>(mask_w) &&
                    cy_px >= 0 && cy_px < static_cast<int>(mask_h)) {
                    cv::circle(overlay, cv::Point(cx_px, cy_px), 5, cv::Scalar(0, 255, 255), cv::FILLED);
                    cv::circle(overlay, cv::Point(cx_px, cy_px), 5, cv::Scalar(0, 0, 0), 1);
                }

                // Line 1: label + distance mm
                char line1[80];
                std::snprintf(line1, sizeof(line1), "%s  %.0fmm",
                    det.label.c_str(),
                    static_cast<double>(distance) * 1000.0);

                // Line 2: xyz in mm
                char line2[80];
                std::snprintf(line2, sizeof(line2), "(%.0f, %.0f, %.0f)",
                    static_cast<double>(obb.center.x()) * 1000.0,
                    static_cast<double>(obb.center.y()) * 1000.0,
                    static_cast<double>(obb.center.z()) * 1000.0);

                auto drawText = [&](const char* text, int tx, int ty) {
                    int baseline = 0;
                    cv::Size tsz = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.42, 1, &baseline);
                    cv::rectangle(overlay,
                        cv::Point(tx - 1, ty - tsz.height - 2),
                        cv::Point(tx + tsz.width + 2, ty + baseline + 2),
                        cv::Scalar(0, 0, 0), cv::FILLED);
                    cv::putText(overlay, text, cv::Point(tx, ty),
                        cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
                };

                int tx = x_min;
                int ty = y_min - 4;
                if (ty - 24 < 0) ty = y_max + 16;
                drawText(line1, tx, ty);
                drawText(line2, tx, ty + 14);
            }

            // --- RViz markers ---
            if (publish_markers_) {
                visualization_msgs::msg::Marker m;
                m.header = out.header;
                m.ns = "fv_3d_detector";
                m.id = marker_id++;
                m.type = visualization_msgs::msg::Marker::CUBE;
                m.action = visualization_msgs::msg::Marker::ADD;
                m.pose.position = obj.centroid;
                m.pose.orientation = obj.orientation;
                m.scale = obj.extent;
                if (m.scale.x < 0.001) m.scale.x = 0.001;
                if (m.scale.y < 0.001) m.scale.y = 0.001;
                if (m.scale.z < 0.001) m.scale.z = 0.001;
                m.color.r = obb.mean_r / 255.0f;
                m.color.g = obb.mean_g / 255.0f;
                m.color.b = obb.mean_b / 255.0f;
                m.color.a = 0.4f;
                m.lifetime = rclcpp::Duration::from_seconds(0.5);
                markers.markers.push_back(m);

                visualization_msgs::msg::Marker txt;
                txt.header = out.header;
                txt.ns = "fv_3d_labels";
                txt.id = marker_id++;
                txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                txt.action = visualization_msgs::msg::Marker::ADD;
                txt.pose.position = obj.centroid;
                txt.pose.position.z += obj.extent.z * 0.5 + 0.02;
                txt.scale.z = 0.015;
                txt.color.r = 1.0f; txt.color.g = 1.0f; txt.color.b = 1.0f; txt.color.a = 1.0f;
                txt.lifetime = rclcpp::Duration::from_seconds(0.5);
                char buf[160];
                std::snprintf(buf, sizeof(buf), "#%d %s %.0fcm",
                    obj.id, obj.label.c_str(),
                    static_cast<double>(obj.distance_m) * 100.0);
                txt.text = buf;
                markers.markers.push_back(txt);
            }
        }

        // Cleanup old markers
        if (publish_markers_) {
            for (int del = marker_id; del < prev_marker_count_; ++del) {
                for (const char *ns : {"fv_3d_detector", "fv_3d_labels"}) {
                    visualization_msgs::msg::Marker dm;
                    dm.header = out.header;
                    dm.ns = ns;
                    dm.id = del;
                    dm.action = visualization_msgs::msg::Marker::DELETE;
                    markers.markers.push_back(dm);
                }
            }
            prev_marker_count_ = marker_id;
            marker_pub_->publish(markers);
        }

        obj_pub_->publish(out);

        // Publish aggregated cloud
        if (!agg_cloud->points.empty()) {
            agg_cloud->width = agg_cloud->points.size();
            agg_cloud->height = 1;
            agg_cloud->is_dense = false;
            sensor_msgs::msg::PointCloud2 cloud_out;
            pcl::toROSMsg(*agg_cloud, cloud_out);
            cloud_out.header = out.header;
            cloud_out_pub_->publish(cloud_out);
        }

        // Publish overlay as JPEG compressed
        if (do_overlay && !overlay.empty()) {
            std::vector<uint8_t> jpeg_buf;
            cv::imencode(".jpg", overlay, jpeg_buf, {cv::IMWRITE_JPEG_QUALITY, 80});
            sensor_msgs::msg::CompressedImage comp_msg;
            comp_msg.header = mask_msg->header;
            comp_msg.format = "jpeg";
            comp_msg.data = std::move(jpeg_buf);
            overlay_pub_->publish(comp_msg);
        }
    }

    std::string mask_topic_, det_topic_, depth_topic_, color_topic_;
    std::string camera_info_topic_, output_topic_, marker_topic_;
    bool publish_markers_{true}, publish_overlay_{true};
    int min_points_{10};
    double depth_scale_{0.001};
    float min_depth_m_{0.05f}, max_depth_m_{3.0f};
    float filter_min_size_{0.0f}, filter_max_size_{0.0f};
    float filter_min_dist_{0.0f}, filter_max_dist_{0.0f};
    double max_fps_{10.0};
    int point_stride_{2};
    int64_t min_interval_ns_{0};
    int64_t last_process_ns_{0};
    int prev_marker_count_{0};

    std::mutex mutex_;
    Intrinsics intr_;
    std::string camera_frame_;
    fv_msgs::msg::DetectionArray::SharedPtr last_det_;
    sensor_msgs::msg::Image::SharedPtr last_depth_, last_color_;

    rclcpp::Subscription<fv_msgs::msg::DetectionArray>::SharedPtr det_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_, color_sub_;

    rclcpp::Publisher<fv_msgs::msg::Object3DArray>::SharedPtr obj_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_out_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr overlay_pub_;
};

}  // namespace fv_3d_detector

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<fv_3d_detector::Detector3DNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
