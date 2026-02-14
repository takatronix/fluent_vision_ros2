#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <deque>
#include <unordered_map>
#include <string>

#include "fv_msgs/msg/detection_array.hpp"

class AsparaPointsNode : public rclcpp::Node {
public:
  explicit AsparaPointsNode(const rclcpp::NodeOptions &options)
      : rclcpp::Node("fv_aspara_points_node", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
    declareParams();
    readParams();

    points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(points_topic_, rclcpp::QoS(10));
    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(markers_topic_, rclcpp::QoS(10));

    if (!detections_topic_.empty()) {
      det_sub_ = this->create_subscription<fv_msgs::msg::DetectionArray>(
          detections_topic_, rclcpp::QoS(10).best_effort(), std::bind(&AsparaPointsNode::onDetections, this, std::placeholders::_1));
    }
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        depth_topic_, rclcpp::QoS(5).best_effort(), std::bind(&AsparaPointsNode::onDepth, this, std::placeholders::_1));
    info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, rclcpp::QoS(5).best_effort(), std::bind(&AsparaPointsNode::onInfo, this, std::placeholders::_1));

    if (detections_topic_.empty()) {
      // Detection-less mode: periodic grid sampling from depth
      timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_period_ms_),
        std::bind(&AsparaPointsNode::onTimerDepthOnly, this));
    }

    RCLCPP_INFO(get_logger(), "fv_aspara_points_node ready (det=%s depth=%s info=%s out=%s mode=%s)",
                detections_topic_.c_str(), depth_topic_.c_str(), camera_info_topic_.c_str(), points_topic_.c_str(),
                detections_topic_.empty()?"depth-only":"detections");
  }

private:
  struct Intrinsics {
    double fx{0.0}, fy{0.0}, cx{0.0}, cy{0.0};
    std::string frame_id;
    bool valid() const { return fx > 0.0 && fy > 0.0; }
  };

  struct TimedPoint {
    rclcpp::Time stamp;
    geometry_msgs::msg::Point p;
  };

  void declareParams() {
    this->declare_parameter<std::string>("detections_topic", "~/detections");
    this->declare_parameter<std::string>("depth_topic", "~/depth/image_rect_raw");
    this->declare_parameter<std::string>("camera_info_topic", "~/camera_info");
    this->declare_parameter<std::string>("points_topic", "~/points_map");
    this->declare_parameter<std::string>("markers_topic", "~/markers_map");
    this->declare_parameter<std::string>("target_frame", "map");
    this->declare_parameter<double>("min_confidence", 0.3);
    this->declare_parameter<int>("sample_stride_px", 3);
    this->declare_parameter<double>("depth_scale_m", 0.001);
    this->declare_parameter<double>("min_depth_m", 0.05);
    this->declare_parameter<double>("max_depth_m", 4.0);
    this->declare_parameter<int>("accumulate.max_points", 50000);
    this->declare_parameter<double>("accumulate.decay_sec", 600.0); // 10 minutes
    this->declare_parameter<double>("roi_bottom_ratio", 0.3); // use bottom 30% of ROI for depth
    this->declare_parameter<int>("timer_period_ms", 200);
  }

  void readParams() {
    detections_topic_ = this->get_parameter("detections_topic").as_string();
    depth_topic_ = this->get_parameter("depth_topic").as_string();
    camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
    points_topic_ = this->get_parameter("points_topic").as_string();
    markers_topic_ = this->get_parameter("markers_topic").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();
    min_confidence_ = this->get_parameter("min_confidence").as_double();
    sample_stride_px_ = std::max(1, static_cast<int>(this->get_parameter("sample_stride_px").as_int()));
    depth_scale_m_ = this->get_parameter("depth_scale_m").as_double();
    min_depth_m_ = this->get_parameter("min_depth_m").as_double();
    max_depth_m_ = this->get_parameter("max_depth_m").as_double();
    acc_max_points_ = std::max(1000, static_cast<int>(this->get_parameter("accumulate.max_points").as_int()));
    acc_decay_sec_ = this->get_parameter("accumulate.decay_sec").as_double();
    roi_bottom_ratio_ = std::clamp(this->get_parameter("roi_bottom_ratio").as_double(), 0.05, 1.0);
    timer_period_ms_ = static_cast<int>(std::max<int64_t>(50, this->get_parameter("timer_period_ms").as_int()));
  }

  void onDepth(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    last_depth_ = msg;
  }
  void onInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    last_info_ = msg;
    intr_.fx = msg->k[0];
    intr_.fy = msg->k[4];
    intr_.cx = msg->k[2];
    intr_.cy = msg->k[5];
    intr_.frame_id = msg->header.frame_id;
  }

  void onDetections(const fv_msgs::msg::DetectionArray::SharedPtr msg) {
    sensor_msgs::msg::Image::SharedPtr depth;
    sensor_msgs::msg::CameraInfo::SharedPtr info;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      depth = last_depth_;
      info = last_info_;
    }
    if (!depth || !info || !intr_.valid()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for depth/camera_info");
      return;
    }

    std::vector<geometry_msgs::msg::Point> points_cam;
    for (const auto &det : msg->detections) {
      if (det.conf_fused < min_confidence_) continue;
      // ROI bottom 30% median depth
      const int img_w = static_cast<int>(depth->width);
      const int img_h = static_cast<int>(depth->height);
      const int x_min = std::clamp(static_cast<int>(std::floor(det.bbox_min.x)), 0, img_w);
      const int y_min = std::clamp(static_cast<int>(std::floor(det.bbox_min.y)), 0, img_h);
      const int x_max = std::clamp(static_cast<int>(std::ceil(det.bbox_max.x)), 0, img_w);
      const int y_max = std::clamp(static_cast<int>(std::ceil(det.bbox_max.y)), 0, img_h);
      const int roi_h = std::max(1, y_max - y_min);
      const int bottom_y0 = y_min + static_cast<int>(std::round((1.0 - roi_bottom_ratio_) * roi_h));
      const int bottom_y1 = y_max;

      double depth_m = estimateMedianDepth(*depth, x_min, bottom_y0, x_max, bottom_y1);
      if (!std::isfinite(depth_m) || depth_m <= 0.0) {
        // fallback to hint
        depth_m = det.depth_hint_m;
      }
      if (!std::isfinite(depth_m) || depth_m <= 0.0) {
        continue;
      }

      const double u = 0.5 * (static_cast<double>(x_min + x_max));
      const double v = static_cast<double>(y_max);
      geometry_msgs::msg::Point p;
      p.z = depth_m;
      p.x = static_cast<double>((u - intr_.cx) * depth_m / intr_.fx);
      p.y = static_cast<double>((v - intr_.cy) * depth_m / intr_.fy);
      points_cam.emplace_back(p);
    }

    if (points_cam.empty()) return;

    // Transform to target frame if configured
    const rclcpp::Time stamp = rclcpp::Time(msg->header.stamp);
    std::vector<geometry_msgs::msg::Point> points_out;
    std::string frame_id = intr_.frame_id;
    if (!target_frame_.empty()) {
      for (const auto &pc : points_cam) {
        geometry_msgs::msg::PointStamped ps_cam, ps_map;
        ps_cam.header.stamp = msg->header.stamp;
        ps_cam.header.frame_id = intr_.frame_id;
        ps_cam.point = pc;
        try {
          ps_map = tf_buffer_.transform(ps_cam, target_frame_, tf2::durationFromSec(0.1));
          points_out.emplace_back(ps_map.point);
        } catch (const tf2::TransformException &ex) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF transform failed: %s", ex.what());
        }
      }
      frame_id = target_frame_;
    } else {
      points_out = points_cam;
    }

    // Accumulate with decay
    const rclcpp::Time now = this->now();
    for (const auto &p : points_out) {
      accumulated_.push_back({now, p});
    }
    pruneAccumulated(now);
    while (static_cast<int>(accumulated_.size()) > acc_max_points_) {
      accumulated_.pop_front();
    }

    publishCloud(frame_id, stamp);
    publishMarkers(frame_id, stamp);
  }

  double depthAt(const sensor_msgs::msg::Image &depth, int r, int c) const {
    const int height = static_cast<int>(depth.height);
    const int width = static_cast<int>(depth.width);
    r = std::clamp(r, 0, height - 1);
    c = std::clamp(c, 0, width - 1);
    const std::string &enc = depth.encoding;
    const uint8_t *row = depth.data.data() + static_cast<size_t>(r) * depth.step;
    if (enc == "16UC1") {
      const uint16_t *px = reinterpret_cast<const uint16_t*>(row) + c;
      uint16_t raw = *px;
      if (raw == 0) return std::numeric_limits<double>::quiet_NaN();
      return static_cast<double>(raw) * depth_scale_m_;
    } else if (enc == "32FC1") {
      const float *px = reinterpret_cast<const float*>(row) + c;
      float m = *px;
      if (!std::isfinite(m) || m <= 0.f) return std::numeric_limits<double>::quiet_NaN();
      return static_cast<double>(m);
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  double estimateMedianDepth(const sensor_msgs::msg::Image &depth, int x0, int y0, int x1, int y1) const {
    std::vector<double> samples;
    for (int y = y0; y < y1; y += sample_stride_px_) {
      for (int x = x0; x < x1; x += sample_stride_px_) {
        double d = depthAt(depth, y, x);
        if (std::isfinite(d) && d > 0.0) samples.push_back(d);
      }
    }
    if (samples.size() < 6) return std::numeric_limits<double>::quiet_NaN();
    size_t mid = samples.size() / 2;
    std::nth_element(samples.begin(), samples.begin() + mid, samples.end());
    return samples[mid];
  }

  void pruneAccumulated(const rclcpp::Time &now) {
    const rclcpp::Duration dur = rclcpp::Duration::from_seconds(acc_decay_sec_);
    while (!accumulated_.empty()) {
      if ((now - accumulated_.front().stamp) > dur) {
        accumulated_.pop_front();
      } else {
        break;
      }
    }
  }

  void onTimerDepthOnly() {
    sensor_msgs::msg::Image::SharedPtr depth;
    sensor_msgs::msg::CameraInfo::SharedPtr info;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      depth = last_depth_;
      info = last_info_;
    }
    if (!depth || !info || !intr_.valid()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 4000, "[depth-only] Waiting for depth/camera_info");
      return;
    }
    const int img_w = static_cast<int>(depth->width);
    const int img_h = static_cast<int>(depth->height);
    const int y_min = static_cast<int>(std::round((1.0 - roi_bottom_ratio_) * img_h));
    const int y_max = img_h;

    std::vector<geometry_msgs::msg::Point> points_cam;
    for (int y = y_min; y < y_max; y += sample_stride_px_) {
      for (int x = 0; x < img_w; x += sample_stride_px_) {
        double d = depthAt(*depth, y, x);
        if (!std::isfinite(d) || d <= 0.0) continue;
        if (d < min_depth_m_ || d > max_depth_m_) continue;
        geometry_msgs::msg::Point p;
        p.z = d;
        p.x = static_cast<double>((static_cast<double>(x) - intr_.cx) * d / intr_.fx);
        p.y = static_cast<double>((static_cast<double>(y) - intr_.cy) * d / intr_.fy);
        points_cam.emplace_back(p);
      }
    }
    if (points_cam.empty()) return;

    // Transform and accumulate
    std::vector<geometry_msgs::msg::Point> points_out;
    std::string frame_id = intr_.frame_id;
    const rclcpp::Time stamp = depth->header.stamp;
    if (!target_frame_.empty()) {
      for (const auto &pc : points_cam) {
        geometry_msgs::msg::PointStamped ps_cam, ps_map;
        ps_cam.header.stamp = stamp;
        ps_cam.header.frame_id = intr_.frame_id;
        ps_cam.point = pc;
        try {
          ps_map = tf_buffer_.transform(ps_cam, target_frame_, tf2::durationFromSec(0.1));
          points_out.emplace_back(ps_map.point);
        } catch (const tf2::TransformException &ex) {
          // Skip transform errors
        }
      }
      frame_id = target_frame_;
    } else {
      points_out = points_cam;
    }
    const rclcpp::Time now = this->now();
    for (const auto &p : points_out) {
      accumulated_.push_back({now, p});
    }
    pruneAccumulated(now);
    while (static_cast<int>(accumulated_.size()) > acc_max_points_) {
      accumulated_.pop_front();
    }
    publishCloud(frame_id, stamp);
    publishMarkers(frame_id, stamp);
  }

  void publishCloud(const std::string &frame_id, const rclcpp::Time &stamp) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->is_dense = false;
    cloud->points.reserve(accumulated_.size());
    for (const auto &tp : accumulated_) {
      cloud->points.emplace_back(pcl::PointXYZ{static_cast<float>(tp.p.x), static_cast<float>(tp.p.y), static_cast<float>(tp.p.z)});
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;
    points_pub_->publish(msg);
  }

  void publishMarkers(const std::string &frame_id, const rclcpp::Time &stamp) {
    if (!markers_pub_) return;
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.ns = "aspara_points";
    m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.03; // 3cm sphere
    m.scale.y = 0.03;
    m.scale.z = 0.03;
    m.color.a = 1.0f; m.color.r = 0.1f; m.color.g = 0.9f; m.color.b = 0.2f;
    m.id = 0;
    m.points.reserve(accumulated_.size());
    for (const auto &tp : accumulated_) {
      geometry_msgs::msg::Point p; p.x = tp.p.x; p.y = tp.p.y; p.z = tp.p.z;
      m.points.push_back(p);
    }
    arr.markers.push_back(m);
    markers_pub_->publish(arr);
  }

  // Params
  std::string detections_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string points_topic_;
  std::string markers_topic_;
  std::string target_frame_;
  double min_confidence_{0.3};
  int sample_stride_px_{3};
  double depth_scale_m_{0.001};
  double min_depth_m_{0.05};
  double max_depth_m_{4.0};
  int acc_max_points_{50000};
  double acc_decay_sec_{600.0};
  double roi_bottom_ratio_{0.3};
  int timer_period_ms_{200};

  // State
  std::mutex mutex_;
  sensor_msgs::msg::Image::SharedPtr last_depth_;
  sensor_msgs::msg::CameraInfo::SharedPtr last_info_;
  Intrinsics intr_;
  std::deque<TimedPoint> accumulated_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // IO
  rclcpp::Subscription<fv_msgs::msg::DetectionArray>::SharedPtr det_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AsparaPointsNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
