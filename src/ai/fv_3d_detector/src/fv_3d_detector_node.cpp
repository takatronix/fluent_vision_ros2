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
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <std_msgs/msg/string.hpp>
#include <atomic>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>

#include "fv_msgs/msg/detection_array.hpp"
#include "fv_msgs/msg/object3_d.hpp"
#include "fv_msgs/msg/object3_d_array.hpp"

#include "fluent_lib/fluent_cloud/pipeline.hpp"

// tf2 — used to optionally re-publish detections in a downstream
// frame (typically `base_link`) so consumers don't have to chase the
// camera pose themselves. Lookup uses the original detection stamp so
// arm motion between detection and publish doesn't smear positions.
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

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
        // Optional switchable secondary depth source (e.g. LingBot refined
        // depth). Both stay subscribed; depth_source_topic
        // (std_msgs/String, transient_local) selects the active one:
        // "default"/"raw" = depth_topic, anything else = alt. The alt sub
        // is RELIABLE: large fragmented Images starve BEST_EFFORT delivery.
        this->declare_parameter<std::string>("alt_depth_topic", "");
        this->declare_parameter<std::string>("depth_source_topic", "");
        this->declare_parameter<std::string>("color_topic", "/d405_color/image_raw");
        this->declare_parameter<std::string>("camera_info_topic", "/d405_depth/camera_info");
        this->declare_parameter<std::string>("output_topic", "~/objects_3d");
        this->declare_parameter<bool>("publish_markers", true);
        this->declare_parameter<std::string>("marker_topic", "~/markers");
        this->declare_parameter<bool>("publish_overlay", true);
        this->declare_parameter<int>("min_points", 10);
        // Depth-dropout hold: when a live 2D track fails 3D extraction
        // (typically stereo depth holes on low-texture faces - red cubes
        // are the worst case), reuse its last successful OBB for up to
        // this many seconds instead of dropping the object for the frame.
        // Bounded, gated on the 2D track still existing, and logged.
        // 0 = off.
        this->declare_parameter<double>("depth_hold_sec", 0.0);
        this->declare_parameter<int>("mask.erode_px", 0);
        this->declare_parameter<double>("depth_scale", 0.001);
        // Distance filter
        this->declare_parameter<double>("min_depth_m", 0.05);
        this->declare_parameter<double>("max_depth_m", 3.0);
        // Size filter (extent in meters, 0 = disabled)
        this->declare_parameter<double>("filter.min_size_m", 0.0);
        this->declare_parameter<double>("filter.max_size_m", 0.0);
        // Aspect ratio filter (longest_edge / shortest_edge, 0 = disabled)
        this->declare_parameter<double>("filter.max_aspect", 0.0);
        // Same-object NMS in 3D — when the segmentation classifier
        // outputs two boxes for the same physical cube (e.g. "blue"
        // and "unknown_color") their centroids end up within a few
        // cm of each other. Greedy-keep the highest-confidence one
        // per cluster. 0 = disabled.
        this->declare_parameter<double>("filter.nms_dist_m", 0.0);
        // Distance filter on 3D centroid (0 = disabled)
        this->declare_parameter<double>("filter.min_distance_m", 0.0);
        this->declare_parameter<double>("filter.max_distance_m", 0.0);
        // Robust per-detection depth gate. 0 = disabled. Useful for
        // open-vocabulary masks that include table/background speckles.
        this->declare_parameter<double>("filter.depth_median_abs_m", 0.0);
        this->declare_parameter<int>("filter.sor_mean_k", 0);
        this->declare_parameter<double>("filter.sor_stddev_mul", 1.0);
        this->declare_parameter<double>("filter.cluster_tolerance_m", 0.0);
        this->declare_parameter<int>("filter.cluster_min_points", 0);
        // Performance
        this->declare_parameter<double>("max_fps", 10.0);
        this->declare_parameter<int>("point_stride", 2);

        // Optional output frame. When non-empty (typical: "base_link"),
        // every centroid + orientation in the published Object3DArray is
        // transformed from the camera/depth frame into target_frame
        // using a stamped tf2 lookup. Markers are published in the same
        // frame so RViz / scene_viewer render them correctly without
        // each consumer redoing the math. Empty string = legacy
        // behaviour (publish in source frame).
        this->declare_parameter<std::string>("target_frame", "");
        // How long to wait for the source→target tf to be available
        // when handling each detection batch. The buffer is normally
        // warm (TFs flow at >>10 Hz); only the first frame after node
        // start tends to need this.
        this->declare_parameter<double>("tf_lookup_timeout_sec", 0.2);

        readParams();

        // Initialise tf2 buffer + listener even when target_frame is
        // empty — it costs almost nothing and lets us flip the param
        // at runtime without restart.
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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
            [this](const sensor_msgs::msg::Image::SharedPtr m) {
                if (use_alt_depth_.load()) return;
                onDepth(m);
            });
        if (!alt_depth_topic_.empty()) {
            depth_alt_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                alt_depth_topic_, rclcpp::QoS(1).reliable(),
                [this](const sensor_msgs::msg::Image::SharedPtr m) {
                    if (!use_alt_depth_.load()) return;
                    onDepth(m);
                });
            if (!depth_source_topic_.empty()) {
                depth_source_sub_ = this->create_subscription<std_msgs::msg::String>(
                    depth_source_topic_,
                    rclcpp::QoS(1).reliable().transient_local(),
                    [this](std_msgs::msg::String::ConstSharedPtr m) {
                        const bool alt = (m->data != "default" && m->data != "raw");
                        if (alt != use_alt_depth_.load()) {
                            use_alt_depth_.store(alt);
                            RCLCPP_INFO(this->get_logger(), "depth source -> %s",
                                        alt ? alt_depth_topic_.c_str() : depth_topic_.c_str());
                        }
                    });
            }
        }
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
        alt_depth_topic_   = this->get_parameter("alt_depth_topic").as_string();
        depth_source_topic_ = this->get_parameter("depth_source_topic").as_string();
        color_topic_       = this->get_parameter("color_topic").as_string();
        camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
        output_topic_      = this->get_parameter("output_topic").as_string();
        publish_markers_   = this->get_parameter("publish_markers").as_bool();
        marker_topic_      = this->get_parameter("marker_topic").as_string();
        publish_overlay_   = this->get_parameter("publish_overlay").as_bool();
        min_points_        = std::max(3, static_cast<int>(this->get_parameter("min_points").as_int()));
        depth_hold_sec_    = std::max(0.0, this->get_parameter("depth_hold_sec").as_double());
        mask_erode_px_     = std::max(0, static_cast<int>(this->get_parameter("mask.erode_px").as_int()));
        depth_scale_       = this->get_parameter("depth_scale").as_double();
        min_depth_m_       = static_cast<float>(this->get_parameter("min_depth_m").as_double());
        max_depth_m_       = static_cast<float>(this->get_parameter("max_depth_m").as_double());
        filter_min_size_   = static_cast<float>(this->get_parameter("filter.min_size_m").as_double());
        filter_max_size_   = static_cast<float>(this->get_parameter("filter.max_size_m").as_double());
        filter_max_aspect_ = static_cast<float>(this->get_parameter("filter.max_aspect").as_double());
        filter_nms_dist_   = static_cast<float>(this->get_parameter("filter.nms_dist_m").as_double());
        filter_min_dist_   = static_cast<float>(this->get_parameter("filter.min_distance_m").as_double());
        filter_max_dist_   = static_cast<float>(this->get_parameter("filter.max_distance_m").as_double());
        filter_depth_median_abs_ = static_cast<float>(this->get_parameter("filter.depth_median_abs_m").as_double());
        filter_sor_mean_k_ = std::max(0, static_cast<int>(this->get_parameter("filter.sor_mean_k").as_int()));
        filter_sor_stddev_mul_ = static_cast<float>(this->get_parameter("filter.sor_stddev_mul").as_double());
        filter_cluster_tolerance_ = static_cast<float>(this->get_parameter("filter.cluster_tolerance_m").as_double());
        filter_cluster_min_points_ = std::max(0, static_cast<int>(this->get_parameter("filter.cluster_min_points").as_int()));
        max_fps_           = this->get_parameter("max_fps").as_double();
        point_stride_      = std::max(1, static_cast<int>(this->get_parameter("point_stride").as_int()));
        target_frame_      = this->get_parameter("target_frame").as_string();
        tf_lookup_timeout_sec_ = this->get_parameter("tf_lookup_timeout_sec").as_double();
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

    bool passFilters(float distance_m, float max_extent, float mid_extent) const {
        if (filter_min_dist_ > 0.0f && distance_m < filter_min_dist_) return false;
        if (filter_max_dist_ > 0.0f && distance_m > filter_max_dist_) return false;
        if (filter_min_size_ > 0.0f && max_extent < filter_min_size_) return false;
        if (filter_max_size_ > 0.0f && max_extent > filter_max_size_) return false;
        // Aspect is judged on the two LARGEST extents (the visible face),
        // not max/min: a cube seen face-on by the wrist camera images as a
        // ~40x30x3mm slab (only its front face has depth), so a max/min
        // ratio compares face width against viewing-angle-dependent
        // thickness and strobes with depth noise (white cube measured
        // 39/26/3mm -> ratio 13, rejected every frame; it only passed when
        // depth noise happened to thicken the slab). Sliver ghosts are
        // still rejected: a thin line has a tiny mid extent, and oversized
        // background bleed is caught by filter.max_size_m above.
        if (filter_max_aspect_ > 0.0f && mid_extent > 1e-4f &&
            (max_extent / mid_extent) > filter_max_aspect_) return false;
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
        if (!depth_msg || !intr.valid()) return;
        if (mask_msg->encoding != "mono8" && mask_msg->encoding != "8UC1") return;

        // No detections: publish passthrough overlay and return
        if (!det_msg || det_msg->detections.empty()) {
            if (publish_overlay_ && color_msg && !color_msg->data.empty()
                && overlay_pub_->get_subscription_count() > 0) {
                cv::Mat passthrough(color_msg->height, color_msg->width, CV_8UC3,
                    const_cast<uint8_t*>(color_msg->data.data()));
                bool color_bgr = (color_msg->encoding == "bgr8" || color_msg->encoding == "8UC3");
                cv::Mat bgr;
                if (color_bgr) bgr = passthrough;
                else cv::cvtColor(passthrough, bgr, cv::COLOR_RGB2BGR);
                std::vector<uint8_t> jpeg_buf;
                cv::imencode(".jpg", bgr, jpeg_buf, {cv::IMWRITE_JPEG_QUALITY, 80});
                sensor_msgs::msg::CompressedImage comp_msg;
                comp_msg.header = mask_msg->header;
                comp_msg.format = "jpeg";
                comp_msg.data = std::move(jpeg_buf);
                overlay_pub_->publish(comp_msg);
            }
            return;
        }

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

        // ---- Phase 1: gather candidates ----
        struct Candidate {
            int32_t det_id;
            fv_msgs::msg::Detection2D det;
            CloudPtr sub;
            fluent_cloud::OBBMetrics obb;
            float distance{0.0f};
            int x_min{0}, y_min{0}, x_max{0}, y_max{0};
        };
        std::vector<Candidate> cands;
        cands.reserve(det_map.size());

        for (const auto &[det_id, det] : det_map) {
            int x_min = std::max(0, static_cast<int>(det.bbox_min.x));
            int y_min = std::max(0, static_cast<int>(det.bbox_min.y));
            int x_max = std::min(static_cast<int>(mask_w) - 1, static_cast<int>(det.bbox_max.x));
            int y_max = std::min(static_cast<int>(mask_h) - 1, static_cast<int>(det.bbox_max.y));
            if (x_min >= x_max || y_min >= y_max) continue;
            const uint32_t mask_instance_id = det.mask_instance_id;
            if (mask_instance_id > 255) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000,
                    "Detection2D.mask_instance_id=%u cannot match MONO8 mask topic %s; producer must publish a per-frame 1..255 mask slot, with stable track id in Detection2D.id",
                    mask_instance_id, mask_topic_.c_str());
                continue;
            }

            CloudPtr sub(new Cloud);
            sub->points.reserve(static_cast<size_t>((x_max - x_min) * (y_max - y_min) / 4));
            auto maskMatches = [&](int px, int py) -> bool {
                auto pixelMatches = [&](int qx, int qy) -> bool {
                    if (qx < 0 || qy < 0 || static_cast<uint32_t>(qx) >= mask_w || static_cast<uint32_t>(qy) >= mask_h) {
                        return false;
                    }
                    const uint8_t mv = mask_data[qy * mask_msg->step + qx];
                    if (mask_instance_id > 0) {
                        return static_cast<uint32_t>(mv) == mask_instance_id;
                    }
                    return mv != 0;
                };
                if (!pixelMatches(px, py)) return false;
                for (int dy = -mask_erode_px_; dy <= mask_erode_px_; ++dy) {
                    for (int dx = -mask_erode_px_; dx <= mask_erode_px_; ++dx) {
                        if (!pixelMatches(px + dx, py + dy)) return false;
                    }
                }
                return true;
            };

            for (int py = y_min; py <= y_max; py += point_stride_) {
                for (int px = x_min; px <= x_max; px += point_stride_) {
                    if (!maskMatches(px, py)) continue;
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
            const size_t raw_pts = sub->points.size();
            if (static_cast<int>(raw_pts) < min_points_) {
                noteExtractFail(det, "raw_pts", raw_pts, 0, 0, 0, 0);
                continue;
            }
            sub = filterByMedianDepth(sub);
            sub = filterStatisticalOutliers(sub);
            sub = keepLargestCluster(sub);
            if (static_cast<int>(sub->points.size()) < min_points_) {
                noteExtractFail(det, "filtered_pts", raw_pts, sub->points.size(), 0, 0, 0);
                continue;
            }

            auto obb = fluent_cloud::compute_obb_metrics(sub);
            if (obb.num_points < static_cast<uint32_t>(min_points_)) {
                noteExtractFail(det, "obb_pts", raw_pts, sub->points.size(), 0, 0, 0);
                continue;
            }

            float distance = obb.center.norm();
            float ex = 2.0f * obb.extents[0];
            float ey = 2.0f * obb.extents[1];
            float ez = 2.0f * obb.extents[2];
            float max_extent = std::max({ex, ey, ez});
            float min_extent = std::min({ex, ey, ez});
            float mid_extent = ex + ey + ez - max_extent - min_extent;

            if (!passFilters(distance, max_extent, mid_extent)) {
                noteExtractFail(det, "size_filters", raw_pts, sub->points.size(),
                                ex, ey, ez);
                continue;
            }

            Candidate c;
            c.det_id = det_id;
            c.det = det;
            c.sub = sub;
            c.obb = obb;
            c.distance = distance;
            c.x_min = x_min; c.y_min = y_min; c.x_max = x_max; c.y_max = y_max;
            cands.push_back(std::move(c));
        }

        // ---- Phase 1.5: depth-dropout hold ----
        // Refresh the per-track OBB cache from this frame's successes,
        // then re-emit the cached OBB for tracks whose 2D detection is
        // still live but whose depth extraction failed this frame.
        if (depth_hold_sec_ > 0.0) {
            const rclcpp::Time now = this->get_clock()->now();
            std::unordered_set<int32_t> produced;
            produced.reserve(cands.size());
            for (const auto &c : cands) {
                produced.insert(c.det_id);
                auto &h = hold_cache_[c.det_id];
                h.sub = c.sub;
                h.obb = c.obb;
                h.distance = c.distance;
                h.x_min = c.x_min; h.y_min = c.y_min;
                h.x_max = c.x_max; h.y_max = c.y_max;
                h.last_ok = now;
            }
            for (auto it = hold_cache_.begin(); it != hold_cache_.end();) {
                if ((now - it->second.last_ok).seconds() > depth_hold_sec_) {
                    it = hold_cache_.erase(it);
                    continue;
                }
                const int32_t det_id = it->first;
                auto dit = det_map.find(det_id);
                if (dit != det_map.end() && produced.count(det_id) == 0) {
                    const auto &h = it->second;
                    Candidate c;
                    c.det_id = det_id;
                    c.det = dit->second;  // fresh label/conf from the live 2D track
                    c.sub = h.sub;
                    c.obb = h.obb;
                    c.distance = h.distance;
                    c.x_min = h.x_min; c.y_min = h.y_min;
                    c.x_max = h.x_max; c.y_max = h.y_max;
                    cands.push_back(std::move(c));
                    RCLCPP_INFO_THROTTLE(
                        get_logger(), *get_clock(), 5000,
                        "depth hold: track %d (%s) kept from OBB %.2fs old (depth extraction failed this frame)",
                        det_id, dit->second.label.c_str(),
                        (now - h.last_ok).seconds());
                }
                ++it;
            }
        }

        // ---- Phase 2: 3D NMS with class-id rescue ----
        // Two heads can fire for the same physical object in one
        // frame (cube model: "blue" + "unknown_color"). We cluster
        // by 3D IoU on the axis-aligned bounding box around each
        // detection's OBB, keep the highest-confidence detection
        // per cluster, and *promote* its label/colour from a
        // suppressed sibling whenever the winner is "unknown" but
        // the cluster contains a more specific class. That way a
        // confident-but-vague "unknown_color" hit doesn't erase a
        // less-confident-but-specific "blue" hit at the same spot.
        // Build the world-AABB that encloses an OBB by walking the
        // 8 corners (centre ± half_x*ax0 ± half_y*ax1 ± half_z*ax2)
        // and taking componentwise min/max. This is a proper IoU
        // upper bound on the OBB-vs-OBB volume, and for nearly
        // axis-aligned cubes (our case) it's near-tight. Falls back
        // gracefully when the boxes are tilted differently — still
        // monotonic in true OBB overlap, just slightly looser.
        auto worldAabb = [](const fluent_cloud::OBBMetrics &m,
                            Eigen::Vector3f &lo, Eigen::Vector3f &hi) {
            lo.setConstant(std::numeric_limits<float>::max());
            hi.setConstant(-std::numeric_limits<float>::max());
            const float hx = m.extents[0], hy = m.extents[1], hz = m.extents[2];
            for (int sx = -1; sx <= 1; sx += 2) {
                for (int sy = -1; sy <= 1; sy += 2) {
                    for (int sz = -1; sz <= 1; sz += 2) {
                        Eigen::Vector3f corner = m.center
                            + sx * hx * m.axes[0]
                            + sy * hy * m.axes[1]
                            + sz * hz * m.axes[2];
                        lo = lo.cwiseMin(corner);
                        hi = hi.cwiseMax(corner);
                    }
                }
            }
        };
        auto bboxIoU = [&worldAabb](const Candidate &a, const Candidate &b) -> float {
            Eigen::Vector3f a_lo, a_hi, b_lo, b_hi;
            worldAabb(a.obb, a_lo, a_hi);
            worldAabb(b.obb, b_lo, b_hi);
            Eigen::Vector3f inter_lo = a_lo.cwiseMax(b_lo);
            Eigen::Vector3f inter_hi = a_hi.cwiseMin(b_hi);
            Eigen::Vector3f d = (inter_hi - inter_lo).cwiseMax(Eigen::Vector3f::Zero());
            float vi = d.x() * d.y() * d.z();
            Eigen::Vector3f ea = a_hi - a_lo;
            Eigen::Vector3f eb = b_hi - b_lo;
            float va = ea.x() * ea.y() * ea.z();
            float vb = eb.x() * eb.y() * eb.z();
            float vu = va + vb - vi;
            return (vu > 1e-9f) ? (vi / vu) : 0.0f;
        };
        auto isUnknown = [](const std::string &label) {
            return label.empty() || label.find("unknown") != std::string::npos;
        };
        if (cands.size() > 1) {
            std::sort(cands.begin(), cands.end(),
                [](const Candidate &a, const Candidate &b) {
                    return a.det.conf_fused > b.det.conf_fused;
                });
            std::vector<Candidate> kept;
            kept.reserve(cands.size());
            const float iou_threshold = 0.15f;     // 15% volume overlap —
                                                   // same physical cube
                                                   // can drift across
                                                   // frames; keep this
                                                   // permissive
            // Distance threshold acts as a fallback when both boxes
            // are tiny and their AABBs barely meet — operator can
            // tune via filter.nms_dist_m.
            const float dist_threshold = filter_nms_dist_;
            for (auto &c : cands) {
                int merge_idx = -1;
                for (size_t i = 0; i < kept.size(); ++i) {
                    const uint32_t c_inst = c.det.mask_instance_id;
                    const uint32_t k_inst = kept[i].det.mask_instance_id;
                    if (c_inst > 0 && k_inst > 0 && c_inst != k_inst) {
                        continue;
                    }
                    bool same_object = bboxIoU(c, kept[i]) >= iou_threshold;
                    if (!same_object && dist_threshold > 0.0f) {
                        Eigen::Vector3f d = c.obb.center - kept[i].obb.center;
                        same_object = d.norm() < dist_threshold;
                    }
                    if (same_object) { merge_idx = static_cast<int>(i); break; }
                }
                if (merge_idx < 0) {
                    kept.push_back(std::move(c));
                    continue;
                }
                // Merge: rescue a specific class from this candidate
                // into a kept "unknown" winner.
                Candidate &k = kept[merge_idx];
                if (isUnknown(k.det.label) && !isUnknown(c.det.label)) {
                    k.det.label    = c.det.label;
                    k.det.class_id = c.det.class_id;
                    // Recolour OBB stats too so the marker shows
                    // the rescued class's mean RGB.
                    k.obb.mean_r = c.obb.mean_r;
                    k.obb.mean_g = c.obb.mean_g;
                    k.obb.mean_b = c.obb.mean_b;
                }
            }
            cands = std::move(kept);
        }

        // ---- Phase 3: emit objects / cloud / overlay / markers ----
        for (const auto &c : cands) {
            const auto &det = c.det;
            const auto &obb = c.obb;
            const auto &sub = c.sub;
            const float distance = c.distance;
            const int x_min = c.x_min, y_min = c.y_min, x_max = c.x_max, y_max = c.y_max;

            fv_msgs::msg::Object3D obj;
            obj.header = out.header;
            obj.id = c.det_id;
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

                int cx_px = static_cast<int>(intr.fx * obb.center.x() / obb.center.z() + intr.cx);
                int cy_px = static_cast<int>(intr.fy * obb.center.y() / obb.center.z() + intr.cy);
                if (cx_px >= 0 && cx_px < static_cast<int>(mask_w) &&
                    cy_px >= 0 && cy_px < static_cast<int>(mask_h)) {
                    cv::circle(overlay, cv::Point(cx_px, cy_px), 5, cv::Scalar(0, 255, 255), cv::FILLED);
                    cv::circle(overlay, cv::Point(cx_px, cy_px), 5, cv::Scalar(0, 0, 0), 1);
                }

                char line1[80];
                std::snprintf(line1, sizeof(line1), "%s  %.0fmm",
                    det.label.c_str(),
                    static_cast<double>(distance) * 1000.0);
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
        }

        // Optional: re-express centroids + orientation in target_frame
        // (typically base_link). Done LAST so all the source-frame
        // detection / OBB math stays in camera frame where the depth
        // comes from. If the stamped lookup fails, keep the output in
        // the source frame; applying a stale "latest" TF corrupts object
        // and marker coordinates when the camera/arm is moving.
        geometry_msgs::msg::TransformStamped output_tf;
        bool got_output_tf = false;
        if (!target_frame_.empty() && target_frame_ != out.header.frame_id) {
            geometry_msgs::msg::TransformStamped tf;
            bool got_tf = false;
            try {
                tf = tf_buffer_->lookupTransform(
                    target_frame_, out.header.frame_id, out.header.stamp,
                    rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_));
                got_tf = true;
            } catch (const std::exception &e) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000,
                    "tf lookup %s -> %s failed: %s; publishing in source frame",
                    out.header.frame_id.c_str(), target_frame_.c_str(), e.what());
            }
            if (got_tf) {
                output_tf = tf;
                got_output_tf = true;
                for (auto &obj : out.objects) {
                    geometry_msgs::msg::PoseStamped src_pose, dst_pose;
                    src_pose.header = out.header;
                    src_pose.pose.position = obj.centroid;
                    src_pose.pose.orientation = obj.orientation;
                    tf2::doTransform(src_pose, dst_pose, tf);
                    obj.centroid = dst_pose.pose.position;
                    obj.orientation = dst_pose.pose.orientation;
                    // Per-object header.frame_id (if set) should match
                    // the array header so downstream tools don't see
                    // mixed frames.
                    obj.header.frame_id = target_frame_;
                }
                // Also re-frame markers so RViz / scene_viewer render
                // them in the new frame without per-consumer fixup.
                for (auto &m : markers.markers) {
                    m.header.frame_id = target_frame_;
                    if (m.action == visualization_msgs::msg::Marker::DELETE) continue;
                    geometry_msgs::msg::PoseStamped src_pose, dst_pose;
                    src_pose.header = out.header;
                    src_pose.pose = m.pose;
                    tf2::doTransform(src_pose, dst_pose, tf);
                    m.pose = dst_pose.pose;
                }
                out.header.frame_id = target_frame_;
            }
        }

        if (publish_markers_) {
            marker_pub_->publish(markers);
        }

        obj_pub_->publish(out);

        // Publish aggregated cloud
        if (!agg_cloud->points.empty()) {
            if (got_output_tf) {
                for (auto &pt : agg_cloud->points) {
                    transform_cloud_point(output_tf, pt);
                }
            }
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

    static void transform_cloud_point(
        const geometry_msgs::msg::TransformStamped &tf,
        pcl::PointXYZRGB &pt)
    {
        const auto &q = tf.transform.rotation;
        const auto &t = tf.transform.translation;

        const double x = pt.x;
        const double y = pt.y;
        const double z = pt.z;
        const double qx = q.x;
        const double qy = q.y;
        const double qz = q.z;
        const double qw = q.w;

        const double ix =  qw * x + qy * z - qz * y;
        const double iy =  qw * y + qz * x - qx * z;
        const double iz =  qw * z + qx * y - qy * x;
        const double iw = -qx * x - qy * y - qz * z;

        pt.x = static_cast<float>(ix * qw + iw * -qx + iy * -qz - iz * -qy + t.x);
        pt.y = static_cast<float>(iy * qw + iw * -qy + iz * -qx - ix * -qz + t.y);
        pt.z = static_cast<float>(iz * qw + iw * -qz + ix * -qy - iy * -qx + t.z);
    }

    CloudPtr filterByMedianDepth(const CloudPtr &in) const
    {
        if (!in || filter_depth_median_abs_ <= 0.0f ||
            static_cast<int>(in->points.size()) < min_points_) {
            return in;
        }
        std::vector<float> zs;
        zs.reserve(in->points.size());
        for (const auto &pt : in->points) {
            if (std::isfinite(pt.z) && pt.z > 0.0f) zs.push_back(pt.z);
        }
        if (static_cast<int>(zs.size()) < min_points_) return in;
        const size_t mid = zs.size() / 2;
        std::nth_element(zs.begin(), zs.begin() + mid, zs.end());
        const float median_z = zs[mid];

        CloudPtr out(new Cloud);
        out->points.reserve(in->points.size());
        for (const auto &pt : in->points) {
            if (!std::isfinite(pt.z) || pt.z <= 0.0f) continue;
            if (std::fabs(pt.z - median_z) > filter_depth_median_abs_) continue;
            out->points.push_back(pt);
        }
        out->width = out->points.size();
        out->height = 1;
        out->is_dense = false;
        return out;
    }

    CloudPtr filterStatisticalOutliers(const CloudPtr &in) const
    {
        if (!in || filter_sor_mean_k_ <= 0 ||
            static_cast<int>(in->points.size()) <= filter_sor_mean_k_) {
            return in;
        }
        return fluent_cloud::filters::StatisticalOutlierRemoval<pcl::PointXYZRGB>()
            .setMeanK(filter_sor_mean_k_)
            .setStddevMulThresh(filter_sor_stddev_mul_)
            .filter(in);
    }

    CloudPtr keepLargestCluster(const CloudPtr &in) const
    {
        if (!in || filter_cluster_tolerance_ <= 0.0f ||
            static_cast<int>(in->points.size()) < min_points_) {
            return in;
        }
        auto tree = pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>);
        tree->setInputCloud(in);

        std::vector<pcl::PointIndices> clusters;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance(filter_cluster_tolerance_);
        ec.setMinClusterSize(std::max(min_points_, filter_cluster_min_points_));
        ec.setMaxClusterSize(static_cast<int>(in->points.size()));
        ec.setSearchMethod(tree);
        ec.setInputCloud(in);
        ec.extract(clusters);
        if (clusters.empty()) return in;

        auto largest = std::max_element(
            clusters.begin(), clusters.end(),
            [](const pcl::PointIndices &a, const pcl::PointIndices &b) {
                return a.indices.size() < b.indices.size();
            });
        CloudPtr out(new Cloud);
        out->points.reserve(largest->indices.size());
        for (const int idx : largest->indices) {
            if (idx >= 0 && static_cast<size_t>(idx) < in->points.size()) {
                out->points.push_back(in->points[static_cast<size_t>(idx)]);
            }
        }
        out->width = out->points.size();
        out->height = 1;
        out->is_dense = false;
        return out;
    }

    std::string mask_topic_, det_topic_, depth_topic_, color_topic_;
    std::string alt_depth_topic_, depth_source_topic_;
    std::atomic<bool> use_alt_depth_{false};
    std::string camera_info_topic_, output_topic_, marker_topic_;
    bool publish_markers_{true}, publish_overlay_{true};
    int min_points_{10};

    // Per-track extraction-failure diagnostics: one INFO line per track
    // every 3s at most, naming the stage that rejected it and the numbers
    // the filters saw. Perception drops must stay observable - a silent
    // reject here read as "camera is broken" from the dashboard.
    void noteExtractFail(const fv_msgs::msg::Detection2D &det,
                         const char *stage, size_t raw_pts, size_t kept_pts,
                         float ex, float ey, float ez) {
        const auto now = this->get_clock()->now();
        auto &last = fail_log_last_[det.id];
        if (last.nanoseconds() != 0 && (now - last).seconds() < 3.0) {
            return;
        }
        last = now;
        RCLCPP_INFO(get_logger(),
            "3d extract fail: id=%d label=%s stage=%s raw_pts=%zu kept_pts=%zu "
            "extent=%.3f/%.3f/%.3f",
            det.id, det.label.c_str(), stage, raw_pts, kept_pts, ex, ey, ez);
    }
    std::unordered_map<int32_t, rclcpp::Time> fail_log_last_;

    // Depth-dropout hold (see depth_hold_sec param). Keyed by the stable
    // Detection2D.id; entries expire depth_hold_sec_ after their last
    // successful extraction.
    struct HeldObb {
        CloudPtr sub;
        fluent_cloud::OBBMetrics obb;
        float distance{0.0f};
        int x_min{0}, y_min{0}, x_max{0}, y_max{0};
        rclcpp::Time last_ok;
    };
    double depth_hold_sec_{0.0};
    std::unordered_map<int32_t, HeldObb> hold_cache_;
    int mask_erode_px_{0};
    double depth_scale_{0.001};
    float min_depth_m_{0.05f}, max_depth_m_{3.0f};
    float filter_min_size_{0.0f}, filter_max_size_{0.0f};
    float filter_max_aspect_{0.0f};
    float filter_nms_dist_{0.0f};
    float filter_min_dist_{0.0f}, filter_max_dist_{0.0f};
    float filter_depth_median_abs_{0.0f};
    int filter_sor_mean_k_{0};
    float filter_sor_stddev_mul_{1.0f};
    float filter_cluster_tolerance_{0.0f};
    int filter_cluster_min_points_{0};
    double max_fps_{10.0};
    int point_stride_{2};
    int64_t min_interval_ns_{0};
    int64_t last_process_ns_{0};
    int prev_marker_count_{0};

    // Optional output frame for re-publishing detections.
    std::string target_frame_;
    double tf_lookup_timeout_sec_{0.2};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::mutex mutex_;
    Intrinsics intr_;
    std::string camera_frame_;
    fv_msgs::msg::DetectionArray::SharedPtr last_det_;
    sensor_msgs::msg::Image::SharedPtr last_depth_, last_color_;

    rclcpp::Subscription<fv_msgs::msg::DetectionArray>::SharedPtr det_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_, color_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_alt_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr depth_source_sub_;

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
