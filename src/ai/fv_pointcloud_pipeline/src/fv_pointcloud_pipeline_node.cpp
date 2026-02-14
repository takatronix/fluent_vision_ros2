#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <fluent_lib/cv_bridge_compat.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <mutex>
#include <system_error>
#include <optional>
#include <string>
#include <unordered_map>
#include <deque>
#include <utility>
#include <vector>

#include "fluent_lib/fluent_cloud/pipeline_builder.hpp"
#include "fv_msgs/msg/detection_array.hpp"
#include "fv_msgs/msg/detection_cloud_indices.hpp"

namespace fv_pointcloud_pipeline {

using PointT = pcl::PointXYZRGB;
using Cloud = pcl::PointCloud<PointT>;
using CloudPtr = Cloud::Ptr;

class PointcloudPipelineNode : public rclcpp::Node {
public:
  explicit PointcloudPipelineNode(const rclcpp::NodeOptions &options)
      : rclcpp::Node("fv_pointcloud_pipeline_node", options) {
    declareParameters();
    readParameters();
    loadPipelineConfig();

    // ポイントクラウドは巨大データなのでBEST_EFFORT+queue=1
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::QoS(1).best_effort());
    if (counts_topic_.empty()) {
      counts_topic_ = output_topic_ + "/indices";
    }
    counts_pub_ = this->create_publisher<fv_msgs::msg::DetectionCloudIndices>(counts_topic_, rclcpp::QoS(1).best_effort());

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        depth_topic_, makeQoS(depth_qos_depth_, depth_qos_reliability_),
        std::bind(&PointcloudPipelineNode::handleDepthImage, this, std::placeholders::_1));

    if (!color_topic_.empty()) {
      color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
          color_topic_, makeQoS(color_qos_depth_, color_qos_reliability_),
          std::bind(&PointcloudPipelineNode::handleColorImage, this, std::placeholders::_1));
    }

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, makeQoS(depth_qos_depth_, depth_qos_reliability_),
        std::bind(&PointcloudPipelineNode::handleCameraInfo, this, std::placeholders::_1));

    detections_sub_ = this->create_subscription<fv_msgs::msg::DetectionArray>(
        detections_topic_, rclcpp::QoS(10).best_effort(),
        std::bind(&PointcloudPipelineNode::handleDetections, this, std::placeholders::_1));

    // Optional ROI mask subscriber (best effort)
    if (!mask_topic_.empty()) {
      mask_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
          mask_topic_, rclcpp::QoS(1).best_effort(),
          std::bind(&PointcloudPipelineNode::handleMask, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "Subscribed roi mask: %s", mask_topic_.c_str());
    }

    parameter_callback_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
          return onParametersSet(params);
        });

    RCLCPP_INFO(get_logger(), "fv_pointcloud_pipeline_node ready (depth=%s detections=%s)",
                depth_topic_.c_str(), detections_topic_.c_str());
  }

private:
  struct CameraIntrinsics {
    double fx{0.0};
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};
    std::string frame_id;
    bool valid() const { return fx > 0.0 && fy > 0.0; }
  };

  struct StepDebugInfo {
    std::string topic;
  };

  void declareParameters() {
    this->declare_parameter<std::string>("color_topic", "");
    this->declare_parameter<std::string>("depth_topic", "~/depth/image_rect_raw");
    this->declare_parameter<std::string>("camera_info_topic", "~/camera_info");
    this->declare_parameter<std::string>("detections_topic", "~/detections");
    // Optional fused ROI mask (from detection_fusion)
    this->declare_parameter<std::string>("mask_topic", "");
    this->declare_parameter<int>("mask_threshold", 128);
    this->declare_parameter<std::string>("output_topic", "~/filtered");
    this->declare_parameter<std::string>("counts_topic", "");
    this->declare_parameter<std::string>("debug_namespace", "~/debug");
    this->declare_parameter<std::string>("pipeline_config", "/config/pipelines/fv_pointcloud_default.yaml");
    this->declare_parameter<double>("depth_scale_m", 0.001);
    this->declare_parameter<int>("sample_stride_px", 2);
    this->declare_parameter<double>("min_depth_m", 0.05);
    this->declare_parameter<double>("max_depth_m", 2.5);
    this->declare_parameter<int>("mask_dilate_px", 0);
    this->declare_parameter<int>("depth_qos_depth", 5);
    this->declare_parameter<std::string>("depth_qos_reliability", "reliable");
    this->declare_parameter<int>("color_qos_depth", 5);
    this->declare_parameter<std::string>("color_qos_reliability", "reliable");
    this->declare_parameter<bool>("publish_debug_clouds", true);
    this->declare_parameter<std::string>("output_frame", "");
    // Depth residual gate (optional): remove points far from root depth
    this->declare_parameter<bool>("depth_residual.enable", false);
    this->declare_parameter<double>("depth_residual.max_m", 0.025);
    // Temporal accumulation (optional)
    this->declare_parameter<bool>("accumulate.enable", false);
    this->declare_parameter<int>("accumulate.frames", 3);
    this->declare_parameter<double>("accumulate.min_iou", 0.5);
    this->declare_parameter<int>("accumulate.max_points", 50000);
  }

  void readParameters() {
    color_topic_ = this->get_parameter("color_topic").as_string();
    depth_topic_ = this->get_parameter("depth_topic").as_string();
    camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
    detections_topic_ = this->get_parameter("detections_topic").as_string();
    mask_topic_ = this->get_parameter("mask_topic").as_string();
    mask_threshold_ = this->get_parameter("mask_threshold").as_int();
    output_topic_ = this->get_parameter("output_topic").as_string();
    counts_topic_ = this->get_parameter("counts_topic").as_string();
    debug_namespace_ = this->get_parameter("debug_namespace").as_string();
    pipeline_config_path_ = this->get_parameter("pipeline_config").as_string();
    depth_scale_m_ = this->get_parameter("depth_scale_m").as_double();
    sample_stride_px_ = std::max(1, static_cast<int>(this->get_parameter("sample_stride_px").as_int()));
    min_depth_m_ = this->get_parameter("min_depth_m").as_double();
    max_depth_m_ = this->get_parameter("max_depth_m").as_double();
    mask_dilate_px_ = std::max(0, static_cast<int>(this->get_parameter("mask_dilate_px").as_int()));
    depth_qos_depth_ = this->get_parameter("depth_qos_depth").as_int();
    depth_qos_reliability_ = this->get_parameter("depth_qos_reliability").as_string();
    color_qos_depth_ = this->get_parameter("color_qos_depth").as_int();
    color_qos_reliability_ = this->get_parameter("color_qos_reliability").as_string();
    publish_debug_clouds_ = this->get_parameter("publish_debug_clouds").as_bool();
    output_frame_override_ = this->get_parameter("output_frame").as_string();
    depth_residual_enable_ = this->get_parameter("depth_residual.enable").as_bool();
    depth_residual_max_m_ = this->get_parameter("depth_residual.max_m").as_double();
    accumulate_enable_ = this->get_parameter("accumulate.enable").as_bool();
    accumulate_frames_ = std::max<int>(1, static_cast<int>(this->get_parameter("accumulate.frames").as_int()));
    accumulate_min_iou_ = this->get_parameter("accumulate.min_iou").as_double();
    accumulate_max_points_ = std::max<int>(1000, static_cast<int>(this->get_parameter("accumulate.max_points").as_int()));
  }

  rclcpp::QoS makeQoS(int depth, const std::string &reliability) const {
    const int valid_depth = depth > 0 ? depth : 1;
    rclcpp::QoS qos(valid_depth);

    std::string lowered = reliability;
    std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });

    if (lowered == "best_effort" || lowered == "best-effort") {
      qos.best_effort();
    } else if (lowered == "reliable" || lowered.empty()) {
      qos.reliable();
    } else if (lowered == "system_default" || lowered == "system-default") {
      // Keep default reliability (reliable) without modification.
    } else if (lowered == "sensor_data" || lowered == "sensor-data") {
      qos.best_effort();
    } else {
      RCLCPP_WARN(get_logger(), "Unknown QoS reliability '%s', defaulting to reliable", reliability.c_str());
      qos.reliable();
    }

    return qos;
  }

  rcl_interfaces::msg::SetParametersResult onParametersSet(const std::vector<rclcpp::Parameter> &params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    bool reload_pipeline = false;
    for (const auto &param : params) {
      const std::string &name = param.get_name();
      if (name == "pipeline_config") {
        pipeline_config_path_ = param.as_string();
        reload_pipeline = true;
      } else if (name == "depth_scale_m") {
        depth_scale_m_ = param.as_double();
      } else if (name == "sample_stride_px") {
        sample_stride_px_ = std::max(1, static_cast<int>(param.as_int()));
      } else if (name == "min_depth_m") {
        min_depth_m_ = param.as_double();
      } else if (name == "max_depth_m") {
        max_depth_m_ = param.as_double();
      } else if (name == "publish_debug_clouds") {
        publish_debug_clouds_ = param.as_bool();
      }
    }
    if (reload_pipeline) {
      try {
        loadPipelineConfig();
      } catch (const std::exception &ex) {
        result.successful = false;
        result.reason = ex.what();
        RCLCPP_ERROR(get_logger(), "Failed to reload pipeline config: %s", ex.what());
      }
    }
    return result;
  }

  void loadPipelineConfig() {
    const std::string resolved = resolveConfigPath(pipeline_config_path_);
    YAML::Node root = YAML::LoadFile(resolved);
    pipeline_config_ = fluent_cloud::pipeline::loadPipelineConfig(root);
    rebuildDebugPublishers();
    RCLCPP_INFO(get_logger(), "Loaded pipeline config: %s (%zu steps)",
                resolved.c_str(), pipeline_config_.steps.size());
  }

  std::string resolveConfigPath(const std::string &path) const {
    namespace fs = std::filesystem;
    if (path.empty()) {
      throw std::runtime_error("pipeline_config parameter is empty");
    }
    std::vector<fs::path> candidates;
    candidates.emplace_back(path);
    try {
      const auto share = ament_index_cpp::get_package_share_directory("fv_pointcloud_pipeline");
      fs::path share_root(share);
      candidates.emplace_back(share_root / path);
      candidates.emplace_back(share_root / "config" / fs::path(path).filename());
    } catch (const std::exception &ex) {
      (void)ex;
    }
    for (const auto &candidate : candidates) {
      if (candidate.empty()) {
        continue;
      }
      std::error_code ec;
      if (fs::exists(candidate, ec)) {
        return fs::weakly_canonical(candidate, ec).string();
      }
    }
    throw std::runtime_error("Failed to locate pipeline config file: " + path);
  }

  void rebuildDebugPublishers() {
    debug_publishers_.clear();
    step_debug_info_.clear();
    for (std::size_t i = 0; i < pipeline_config_.steps.size(); ++i) {
      const auto &step = pipeline_config_.steps[i];
      if (step.debug_topic.empty()) {
        step_debug_info_.push_back({""});
        continue;
      }
      const std::string topic = buildDebugTopic(step.debug_topic);
      auto pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic, rclcpp::QoS(1));
      debug_publishers_.emplace(topic, pub);
      step_debug_info_.push_back({topic});
    }
  }

  std::string buildDebugTopic(const std::string &suffix) const {
    if (suffix.empty()) {
      return "";
    }
    if (suffix.front() == '/') {
      return suffix;
    }
    std::string base = debug_namespace_;
    if (base.empty()) {
      base = output_topic_ + "/debug";
    }
    if (base.back() != '/') {
      base.push_back('/');
    }
    return base + suffix;
  }

  void handleDepthImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_depth_ = msg;
  }

  void handleColorImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_color_ = msg;
  }

  void handleCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_camera_info_ = msg;
    intrinsics_.fx = msg->k[0];
    intrinsics_.fy = msg->k[4];
    intrinsics_.cx = msg->k[2];
    intrinsics_.cy = msg->k[5];
    intrinsics_.frame_id = msg->header.frame_id;
  }

  void handleDetections(const fv_msgs::msg::DetectionArray::SharedPtr msg) {
    sensor_msgs::msg::Image::SharedPtr depth_msg;
    sensor_msgs::msg::Image::SharedPtr color_msg;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info;
    sensor_msgs::msg::Image::SharedPtr mask_msg;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      depth_msg = latest_depth_;
      color_msg = latest_color_;
      camera_info = latest_camera_info_;
      mask_msg = latest_mask_;
    }

    if (!depth_msg || !camera_info) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                           "Waiting for depth image and camera info before processing detections");
      return;
    }
    if (!intrinsics_.valid()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                           "Camera intrinsics invalid (fx,fy <= 0)");
      return;
    }

    const rclcpp::Time stamp = rclcpp::Time(msg->header.stamp);

    cv_bridge::CvImageConstPtr depth_bridge;
    try {
      depth_bridge = cv_bridge::toCvShare(depth_msg, depth_msg->encoding);
    } catch (const cv_bridge::Exception &ex) {
      RCLCPP_ERROR(get_logger(), "cv_bridge depth conversion failed: %s", ex.what());
      return;
    }

    cv_bridge::CvImageConstPtr color_bridge;
    if (color_msg) {
      try {
        color_bridge = cv_bridge::toCvShare(color_msg, sensor_msgs::image_encodings::BGR8);
      } catch (const cv_bridge::Exception &ex) {
        RCLCPP_WARN(get_logger(), "cv_bridge color conversion failed: %s", ex.what());
      }
    }

    CloudPtr aggregated(new Cloud);
    aggregated->is_dense = false;

    std::vector<int32_t> detection_ids;
    std::vector<uint32_t> detection_counts;
    detection_ids.reserve(msg->detections.size());
    detection_counts.reserve(msg->detections.size());

    // Prepare mask image (MONO8) resized to depth resolution if provided
    cv::Mat mask_for_depth;
    if (mask_msg) {
      try {
        auto mbridge = cv_bridge::toCvShare(mask_msg);
        if (mbridge->image.channels() == 1) {
          mask_for_depth = mbridge->image;
        } else {
          cv::cvtColor(mbridge->image, mask_for_depth, cv::COLOR_BGR2GRAY);
        }
        if (mask_for_depth.size() != depth_bridge->image.size()) {
          cv::resize(mask_for_depth, mask_for_depth, depth_bridge->image.size(), 0, 0, cv::INTER_NEAREST);
        }
        if (mask_dilate_px_ > 0) {
          int k = std::min(31, std::max(1, mask_dilate_px_));
          cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2*k+1, 2*k+1));
          cv::dilate(mask_for_depth, mask_for_depth, kernel, cv::Point(-1,-1), 1);
        }
      } catch (const std::exception &ex) {
        RCLCPP_WARN(get_logger(), "cv_bridge mask conversion failed: %s", ex.what());
        mask_for_depth.release();
      }
    }

    for (const auto &det : msg->detections) {
      detection_ids.push_back(det.id);
      uint32_t appended_points = 0;

      const auto roi = detectionToRoi(det, depth_bridge->image.cols, depth_bridge->image.rows);
      if (roi.width > 0 && roi.height > 0) {
        auto roi_cloud = extractCloud(depth_bridge->image,
                                      color_bridge ? color_bridge->image : cv::Mat(),
                                      mask_for_depth,
                                      roi, det);
        // Temporal accumulation across frames (per detection id)
        if (accumulate_enable_ && roi_cloud && !roi_cloud->empty()) {
          auto merged = accumulateCloud(det.id, roi, roi_cloud);
          if (merged && !merged->empty()) roi_cloud = merged;
        }
        if (roi_cloud && !roi_cloud->empty()) {
          fluent_cloud::pipeline::FilterContext ctx;
          // root_depth: 検出ヒントが無い場合はROI内深度の中央値でフォールバック
          double root_depth = std::numeric_limits<double>::quiet_NaN();
          if (std::isfinite(det.depth_hint_m) && det.depth_hint_m > 0.0f) {
            root_depth = static_cast<double>(det.depth_hint_m);
          } else {
            root_depth = estimateRoiMedianDepth(depth_bridge->image, mask_for_depth, roi);
          }
          if (std::isfinite(root_depth) && root_depth > 0.0) {
            ctx.scalars["root_depth"] = root_depth;
          }
          ctx.scalars["roi_width_px"] = static_cast<double>(roi.width);
          ctx.scalars["roi_height_px"] = static_cast<double>(roi.height);

          fluent_cloud::pipeline::PipelineOptions options;
          if (publish_debug_clouds_) {
            options.on_step = [this, &stamp](std::size_t step_index, const std::string &, const CloudPtr &cloud) {
              publishDebugCloud(step_index, cloud, stamp);
            };
          }

          try {
            roi_cloud = fluent_cloud::pipeline::apply(pipeline_config_, roi_cloud, ctx, options);
          } catch (const std::exception &ex) {
            RCLCPP_WARN(get_logger(), "Pipeline step failed for detection %d: %s", det.id, ex.what());
            roi_cloud.reset();
          }

          if (roi_cloud && !roi_cloud->empty()) {
            appended_points = static_cast<uint32_t>(roi_cloud->points.size());
            aggregated->points.insert(aggregated->points.end(), roi_cloud->points.begin(), roi_cloud->points.end());
          }
        }
      }

      detection_counts.push_back(appended_points);
    }

    aggregated->width = aggregated->points.size();
    aggregated->height = 1;

    publishPointCloud(aggregated, stamp, camera_info->header.frame_id);

    if (counts_pub_) {
      fv_msgs::msg::DetectionCloudIndices indices_msg;
      indices_msg.header = msg->header;
      indices_msg.ids = detection_ids;
      indices_msg.counts = detection_counts;
      counts_pub_->publish(indices_msg);
    }
  }

  cv::Rect detectionToRoi(const fv_msgs::msg::Detection2D &det, int width, int height) const {
    int x_min = static_cast<int>(std::floor(det.bbox_min.x));
    int y_min = static_cast<int>(std::floor(det.bbox_min.y));
    int x_max = static_cast<int>(std::ceil(det.bbox_max.x));
    int y_max = static_cast<int>(std::ceil(det.bbox_max.y));
    x_min = std::clamp(x_min, 0, width);
    y_min = std::clamp(y_min, 0, height);
    x_max = std::clamp(x_max, 0, width);
    y_max = std::clamp(y_max, 0, height);
    return cv::Rect(cv::Point(x_min, y_min), cv::Point(x_max, y_max));
  }

  CloudPtr extractCloud(const cv::Mat &depth, const cv::Mat &color,
                        const cv::Mat &mask,
                        const cv::Rect &roi, const fv_msgs::msg::Detection2D &det) const {
    CloudPtr cloud(new Cloud);
    cloud->is_dense = false;
    const bool has_color = !color.empty();
    const bool use_mask = !mask.empty();

    // Optional depth residual gate baseline
    double gate_root_m = std::numeric_limits<double>::quiet_NaN();
    if (depth_residual_enable_) {
      if (std::isfinite(det.depth_hint_m) && det.depth_hint_m > 0.0f) {
        gate_root_m = static_cast<double>(det.depth_hint_m);
      } else {
        gate_root_m = estimateRoiMedianDepth(depth, mask, roi);
      }
    }

    for (int v = roi.y; v < roi.y + roi.height; v += sample_stride_px_) {
      for (int u = roi.x; u < roi.x + roi.width; u += sample_stride_px_) {
        if (use_mask) {
          uint8_t m = mask.at<uint8_t>(std::clamp(v, 0, mask.rows - 1),
                                       std::clamp(u, 0, mask.cols - 1));
          if (m < mask_threshold_) continue; // outside ROI mask
        }
        double depth_m = depthAt(depth, v, u);
        if (!std::isfinite(depth_m) || depth_m <= 0.0) {
          continue;
        }
        if (depth_residual_enable_ && std::isfinite(gate_root_m)) {
          if (std::abs(depth_m - gate_root_m) > depth_residual_max_m_) {
            continue;
          }
        }
        if (depth_m < min_depth_m_ || depth_m > max_depth_m_) {
          continue;
        }

        PointT pt;
        pt.z = static_cast<float>(depth_m);
        pt.x = static_cast<float>((static_cast<double>(u) - intrinsics_.cx) * depth_m / intrinsics_.fx);
        pt.y = static_cast<float>((static_cast<double>(v) - intrinsics_.cy) * depth_m / intrinsics_.fy);

        if (has_color) {
          const cv::Vec3b &c = color.at<cv::Vec3b>(std::clamp(v, 0, color.rows - 1),
                                                   std::clamp(u, 0, color.cols - 1));
          pt.r = c[2];
          pt.g = c[1];
          pt.b = c[0];
        } else {
          pt.r = pt.g = pt.b = static_cast<uint8_t>(std::min(255.0, depth_m * 255.0));
        }

        cloud->points.emplace_back(pt);
      }
    }

    return cloud;
  }

  // IoU between two rects
  static double rectIoU(const cv::Rect &a, const cv::Rect &b) {
    int x1 = std::max(a.x, b.x);
    int y1 = std::max(a.y, b.y);
    int x2 = std::min(a.x + a.width, b.x + b.width);
    int y2 = std::min(a.y + a.height, b.y + b.height);
    int iw = std::max(0, x2 - x1);
    int ih = std::max(0, y2 - y1);
    double inter = static_cast<double>(iw) * static_cast<double>(ih);
    double uni = static_cast<double>(a.area() + b.area()) - inter + 1e-9;
    return inter / uni;
  }

  // Accumulate recent ROI clouds per detection id
  CloudPtr accumulateCloud(int32_t id, const cv::Rect &roi, const CloudPtr &current) {
    auto &acc = accum_[id];
    // Reset if ROI moves too much (low IoU)
    if (acc.has_roi) {
      double iou = rectIoU(acc.last_roi, roi);
      if (iou < accumulate_min_iou_) {
        acc.buffers.clear();
      }
    }
    acc.last_roi = roi;
    acc.has_roi = true;
    acc.buffers.push_back(current);
    while (static_cast<int>(acc.buffers.size()) > accumulate_frames_) {
      acc.buffers.pop_front();
    }
    // Merge with cap
    CloudPtr merged(new Cloud);
    merged->is_dense = false;
    for (const auto &c : acc.buffers) {
      if (!c) continue;
      for (const auto &p : c->points) {
        merged->points.push_back(p);
        if (static_cast<int>(merged->points.size()) >= accumulate_max_points_) break;
      }
      if (static_cast<int>(merged->points.size()) >= accumulate_max_points_) break;
    }
    merged->width = merged->points.size();
    merged->height = 1;
    return merged;
  }

  double depthAt(const cv::Mat &depth, int row, int col) const {
    row = std::clamp(row, 0, depth.rows - 1);
    col = std::clamp(col, 0, depth.cols - 1);
    if (depth.type() == CV_16UC1) {
      uint16_t raw = depth.at<uint16_t>(row, col);
      if (raw == 0) {
        return std::numeric_limits<double>::quiet_NaN();
      }
      return static_cast<double>(raw) * depth_scale_m_;
    }
    if (depth.type() == CV_32FC1) {
      float m = depth.at<float>(row, col);
      if (!std::isfinite(m) || m <= 0.f) {
        return std::numeric_limits<double>::quiet_NaN();
      }
      return static_cast<double>(m);
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  void publishPointCloud(const CloudPtr &cloud, const rclcpp::Time &stamp, const std::string &camera_frame) {
    if (!cloud) {
      return;
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = toMsg(stamp);
    if (!output_frame_override_.empty()) {
      msg.header.frame_id = output_frame_override_;
    } else if (!camera_frame.empty()) {
      msg.header.frame_id = camera_frame;
    } else {
      msg.header.frame_id = intrinsics_.frame_id;
    }
    pointcloud_pub_->publish(msg);
  }

  void publishDebugCloud(std::size_t step_index, const CloudPtr &cloud, const rclcpp::Time &stamp) {
    if (step_index >= step_debug_info_.size()) {
      return;
    }
    const auto &info = step_debug_info_[step_index];
    if (info.topic.empty() || !cloud) {
      return;
    }
    auto it = debug_publishers_.find(info.topic);
    if (it == debug_publishers_.end()) {
      return;
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = toMsg(stamp);
    msg.header.frame_id = !output_frame_override_.empty() ? output_frame_override_ : intrinsics_.frame_id;
    it->second->publish(msg);
  }

  static builtin_interfaces::msg::Time toMsg(const rclcpp::Time &time) {
    const int64_t nanoseconds = time.nanoseconds();
    builtin_interfaces::msg::Time msg;
    msg.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
    int64_t remainder = nanoseconds % 1000000000LL;
    if (remainder < 0) {
      remainder += 1000000000LL;
      msg.sec -= 1;
    }
    msg.nanosec = static_cast<uint32_t>(remainder);
    return msg;
  }

  // ROI内の深度中央値を推定（mask>=threshold の画素のみ使用）
  double estimateRoiMedianDepth(const cv::Mat &depth,
                                const cv::Mat &mask,
                                const cv::Rect &roi) const {
    std::vector<double> samples;
    samples.reserve(static_cast<size_t>((roi.width / std::max(1, sample_stride_px_)) *
                                        (roi.height / std::max(1, sample_stride_px_))));
    const bool use_mask = !mask.empty();
    for (int v = roi.y; v < roi.y + roi.height; v += std::max(1, sample_stride_px_)) {
      for (int u = roi.x; u < roi.x + roi.width; u += std::max(1, sample_stride_px_)) {
        if (use_mask) {
          uint8_t m = mask.at<uint8_t>(std::clamp(v, 0, mask.rows - 1),
                                       std::clamp(u, 0, mask.cols - 1));
          if (m < mask_threshold_) continue;
        }
        double d = depthAt(depth, v, u);
        if (std::isfinite(d) && d > 0.0 && d >= min_depth_m_ && d <= max_depth_m_) {
          samples.push_back(d);
        }
      }
    }
    if (samples.size() < 8) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    // メディアン
    size_t mid = samples.size() / 2;
    std::nth_element(samples.begin(), samples.begin() + mid, samples.end());
    double median = samples[mid];
    return median;
  }

  // Parameters
  std::string color_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string detections_topic_;
  std::string mask_topic_;
  std::string output_topic_;
  std::string counts_topic_;
  std::string debug_namespace_;
  std::string pipeline_config_path_;
  std::string output_frame_override_;
  double depth_scale_m_{0.001};
  double min_depth_m_{0.05};
  double max_depth_m_{2.5};
  int sample_stride_px_{2};
  int depth_qos_depth_{5};
  std::string depth_qos_reliability_{"reliable"};
  int color_qos_depth_{5};
  std::string color_qos_reliability_{"reliable"};
  bool publish_debug_clouds_{true};
  int mask_threshold_{128};
  int mask_dilate_px_{0};
  bool depth_residual_enable_{false};
  double depth_residual_max_m_{0.025};
  // Accumulation params
  bool accumulate_enable_{false};
  int accumulate_frames_{3};
  double accumulate_min_iou_{0.5};
  int accumulate_max_points_{50000};

  // Subscriptions / publishers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<fv_msgs::msg::DetectionArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<fv_msgs::msg::DetectionCloudIndices>::SharedPtr counts_pub_;
  std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> debug_publishers_;

  // Cached data
  std::mutex data_mutex_;
  sensor_msgs::msg::Image::SharedPtr latest_depth_;
  sensor_msgs::msg::Image::SharedPtr latest_color_;
  sensor_msgs::msg::CameraInfo::SharedPtr latest_camera_info_;
  sensor_msgs::msg::Image::SharedPtr latest_mask_;
  CameraIntrinsics intrinsics_;

  fluent_cloud::pipeline::PipelineConfig pipeline_config_;
  std::vector<StepDebugInfo> step_debug_info_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  void handleMask(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_mask_ = msg;
  }

  struct AccumState {
    std::deque<CloudPtr> buffers;
    cv::Rect last_roi;
    bool has_roi{false};
  };
  std::unordered_map<int32_t, AccumState> accum_;
};

}  // namespace fv_pointcloud_pipeline

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fv_pointcloud_pipeline::PointcloudPipelineNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
