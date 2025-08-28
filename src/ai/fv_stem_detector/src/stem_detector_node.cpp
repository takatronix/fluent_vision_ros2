#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include "fluent_lib/fluent.hpp"
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>
#include <limits>
#include <algorithm>
#include <atomic>
#include <mutex>
#include <fstream>
#include <sstream>
#include <functional>
#include <regex>
#include <sensor_msgs/image_encodings.hpp>
#include "fluent_lib/ui/info_window.hpp"

#include "fv_stem_detector/stem_detector.hpp"
#include "fv_stem_detector/aspara_3d_pipeline.hpp"
#include "fv_stem_detector/msg/stem_detection.hpp"
#include "fv_stem_detector/msg/stem_detection_array.hpp"
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <fv_stem_detector/srv/set_source_mode.hpp>
#include "fv_stem_detector/msg/region_cloud.hpp"
#include "fv_stem_detector/msg/region_cloud_array.hpp"

namespace fv_stem_detector_node {

using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud2;
using fv_stem_detector::StemDetector;
using fv_stem_detector::StemRegion;
using fv_stem_detector::StemDetectionResult;
using fv_stem_detector::msg::StemDetection;
using fv_stem_detector::msg::StemDetectionArray;
using vision_msgs::msg::Detection2DArray;

class StemDetectorNode : public rclcpp::Node {
public:
  explicit StemDetectorNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("fv_stem_detector_node", options) {
    declareParameters();
    getParameters();
    setupPublishers();
    setupSubscribers();
    setupServices();
    // 動的パラメータ反映
    param_cb_handle_ = this->add_on_set_parameters_callback(
        std::bind(&StemDetectorNode::onParamSet, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "fv_stem_detector_node started");
  }

private:
  void declareParameters() {
    this->declare_parameter<std::string>("camera_topic", "/fv/d405/color/image_raw");
    this->declare_parameter<std::string>("depth_topic", "/fv/d405/depth/image_rect_raw");
    // pointcloud_topic は未使用のため宣言を廃止（深度から生成）
    this->declare_parameter<std::string>("camera_info_topic", "/fv/d405/color/camera_info");
    this->declare_parameter<std::string>("depth_camera_info_topic", "/fv/d405/depth/camera_info");
    // 同期を使わず、最新のDepthで即時処理する運用をデフォルトにする
    this->declare_parameter<bool>("use_sync", false);

    this->declare_parameter<std::string>("output_annotated_image_topic", "/fv/d405/stem_detector/result");
    this->declare_parameter<std::string>("output_detection_topic", "/fv/d405/stem_detector/detections");
    this->declare_parameter<std::string>("detection_topic", "/fv/d405/object_detection/detections");
    // debug
    this->declare_parameter<bool>("debug.enable_roi_log", false);
    // 画像は購読なしでも常に出すか
    this->declare_parameter<bool>("output.always_publish_image", true);
    // ソースモード（object | fixed）
    this->declare_parameter<std::string>("source.mode", "object");
    // 固定情報の送信を抑止するフラグ（既定: false = 送信しない）
    this->declare_parameter<bool>("fixed.enable", false);

    // overlay
    // 色: 固定=青, アスパラ(YOLO)=緑
    this->declare_parameter<std::vector<int64_t>>("overlay.fixed_rect_color_bgr", {255, 0, 0});
    this->declare_parameter<std::vector<int64_t>>("overlay.yolo_rect_color_bgr", {0, 255, 0});
    this->declare_parameter<int>("overlay.rect_thickness", 2);
    this->declare_parameter<std::vector<int64_t>>("overlay.cut_point_color_bgr", {0, 0, 255});
    this->declare_parameter<int>("overlay.cut_point_radius", 6);
    this->declare_parameter<int>("overlay.cut_point_thickness", 2);
    this->declare_parameter<bool>("overlay.show_region_labels", true);
    this->declare_parameter<double>("overlay.text_scale", 0.5);
    this->declare_parameter<int>("overlay.text_thickness", 2);
    this->declare_parameter<int>("overlay.yolo_hold_ms", 80);
    this->declare_parameter<std::vector<int64_t>>("overlay.label_text_color_bgr", {255, 255, 255});
    // approach overlay
    this->declare_parameter<std::vector<int64_t>>("overlay.approach_point_color_bgr", {255, 0, 255});
    this->declare_parameter<int>("overlay.approach_point_radius", 5);
    this->declare_parameter<int>("overlay.approach_point_thickness", 2);

    this->declare_parameter<std::string>("publish_frame_id", "fv_d405_color_optical_frame");
    this->declare_parameter<double>("publish_rate", 10.0);
    this->declare_parameter<double>("depth_scale_m", 0.001); // 16U depth unit -> meters

    // detection.method は3D固定のため宣言を廃止
    // event mode (yolo trigger vs legacy sync)
    this->declare_parameter<std::string>("event.mode", "yolo_trigger");
    this->declare_parameter<bool>("event.skip_if_busy", true);
    // ID 割当設定（動的IDの開始番号）
    this->declare_parameter<int>("id.dynamic_start", 1);

    // approach depth sampling
    this->declare_parameter<int>("approach.depth.window_px", 5);
    this->declare_parameter<int>("approach.depth.min_valid", 5);
    // approach wide search（固定モードでセンター以外も探索）
    this->declare_parameter<bool>("approach.search.enable", false);
    this->declare_parameter<int>("approach.search.grid_step_px", 8);
    this->declare_parameter<double>("approach.search.exclude_bottom_ratio", 0.0);

    // keepalive / diagnostics
    this->declare_parameter<int>("keepalive.grace_ms", 100);
    this->declare_parameter<int>("max_depth_age_ms", 200);
    this->declare_parameter<bool>("overlay.show_wait_banner", false);

    // YOLO filtersと固定領域（固定の有効/無効は source.mode で制御）
    this->declare_parameter<double>("yolo.min_confidence", 0.20);
    this->declare_parameter<int>("yolo.min_bbox_w", 1);
    this->declare_parameter<int>("yolo.min_bbox_h", 1);
    this->declare_parameter<bool>("fixed.publish", true);
    this->declare_parameter<int>("pc.sample_stride_px", 2);
    // 3D信頼度（PCA）トップレベルもしきい値を追加（profiles.*があればそちら優先）
    this->declare_parameter<double>("pca.min_confidence", pca_min_confidence_);

    // QoS
    this->declare_parameter<int>("qos.input_queue_size", 1);
    this->declare_parameter<std::string>("qos.input_reliability", "best_effort");
    this->declare_parameter<int>("qos.output_queue_size", 1);
    this->declare_parameter<std::string>("qos.output_reliability", "best_effort");

    // check regions as a flat array of int (id,x,y,w,h)*N
    this->declare_parameter<std::vector<int64_t>>("check_regions_flat", std::vector<int64_t>{});
    // 単一固定矩形（優先適用）。形式: [x,y,w,h]
    this->declare_parameter<std::vector<int64_t>>("fixed.rect", std::vector<int64_t>{});
    this->declare_parameter<int>("fixed.rect_id", 0);
    // YAML保存先（空なら保存不可）
    this->declare_parameter<std::string>("fixed.save_yaml_path", "");

    // service namespace (optional). If empty, derive from camera_topic like "/fv/<camera>"
    this->declare_parameter<std::string>("service.ns", "");
    // ROI点群配信（比較用）
    this->declare_parameter<bool>("cloud.publish_roi_clouds", false);
    this->declare_parameter<double>("cloud.throttle_hz", 2.0);
    // トピック名（未指定=自動）
    this->declare_parameter<std::string>("cloud.topics.raw_zband", "");
    this->declare_parameter<std::string>("cloud.topics.post_voxel", "");
    this->declare_parameter<std::string>("cloud.topics.post_sor", "");
    // Foxglove表示向け: 各ステージのPointCloud2(マージ)出力
    this->declare_parameter<bool>("cloud.publish_raw_zband", false);
    this->declare_parameter<bool>("cloud.publish_post_voxel", false);
    this->declare_parameter<bool>("cloud.publish_post_sor", false);
    this->declare_parameter<bool>("cloud.publish_failed_rois", false);
    this->declare_parameter<std::string>("cloud.topics.merged_raw_zband", "");
    this->declare_parameter<std::string>("cloud.topics.merged_post_voxel", "");
    this->declare_parameter<std::string>("cloud.topics.merged_post_sor", "");

    // profiles: object overrides
    this->declare_parameter<double>("profiles.object.depth_band.minus_m", depth_band_minus_m_);
    this->declare_parameter<double>("profiles.object.depth_band.plus_m", depth_band_plus_m_);
    this->declare_parameter<double>("profiles.object.ground_cut.auto_max_p90_offset_m", ground_cut_auto_max_p90_offset_m_);
    this->declare_parameter<double>("profiles.object.voxel.leaf_m", voxel_leaf_m_);
    this->declare_parameter<int>("profiles.object.sor.mean_k", sor_mean_k_);
    this->declare_parameter<double>("profiles.object.sor.stddev_mul", sor_stddev_mul_);
    this->declare_parameter<double>("profiles.object.length_gate.min_length_m", min_length_m_);
    this->declare_parameter<double>("profiles.object.length_gate.max_length_m", max_length_m_);
    this->declare_parameter<double>("profiles.object.pca.min_confidence", pca_min_confidence_);
    this->declare_parameter<int>("profiles.object.pc.sample_stride_px", pc_sample_stride_px_);
    this->declare_parameter<int>("profiles.object.approach.depth.window_px", approach_window_px_);
    this->declare_parameter<int>("profiles.object.approach.depth.min_valid", approach_min_valid_);
    this->declare_parameter<double>("profiles.object.depth_scale_m", depth_scale_m_);
    this->declare_parameter<bool>("profiles.object.approach.search.enable", false);
    this->declare_parameter<int>("profiles.object.approach.search.grid_step_px", 8);
    this->declare_parameter<double>("profiles.object.approach.search.exclude_bottom_ratio", 0.0);

    // profiles: fixed overrides
    this->declare_parameter<double>("profiles.fixed.depth_band.minus_m", depth_band_minus_m_);
    this->declare_parameter<double>("profiles.fixed.depth_band.plus_m", depth_band_plus_m_);
    this->declare_parameter<double>("profiles.fixed.ground_cut.auto_max_p90_offset_m", ground_cut_auto_max_p90_offset_m_);
    this->declare_parameter<double>("profiles.fixed.voxel.leaf_m", voxel_leaf_m_);
    this->declare_parameter<int>("profiles.fixed.sor.mean_k", sor_mean_k_);
    this->declare_parameter<double>("profiles.fixed.sor.stddev_mul", sor_stddev_mul_);
    this->declare_parameter<double>("profiles.fixed.length_gate.min_length_m", min_length_m_);
    this->declare_parameter<double>("profiles.fixed.length_gate.max_length_m", max_length_m_);
    this->declare_parameter<double>("profiles.fixed.pca.min_confidence", pca_min_confidence_);
    this->declare_parameter<int>("profiles.fixed.pc.sample_stride_px", pc_sample_stride_px_);
    this->declare_parameter<int>("profiles.fixed.approach.depth.window_px", approach_window_px_);
    this->declare_parameter<int>("profiles.fixed.approach.depth.min_valid", approach_min_valid_);
    this->declare_parameter<double>("profiles.fixed.depth_scale_m", depth_scale_m_);
    this->declare_parameter<bool>("profiles.fixed.approach.search.enable", false);
    this->declare_parameter<int>("profiles.fixed.approach.search.grid_step_px", 8);
    this->declare_parameter<double>("profiles.fixed.approach.search.exclude_bottom_ratio", 0.0);

    // フォールバック（センター合わせ3D）の有効/無効（既定: OFF）
    this->declare_parameter<bool>("fallback.center3d.enable", false);
  }

  void getParameters() {
    this->get_parameter("camera_topic", camera_topic_);
    this->get_parameter("depth_topic", depth_topic_);
    // pointcloud_topic は未使用
    this->get_parameter("camera_info_topic", camera_info_topic_);
    this->get_parameter("depth_camera_info_topic", depth_camera_info_topic_);
    this->get_parameter("output_annotated_image_topic", output_image_topic_);
    this->get_parameter("output_detection_topic", output_detection_topic_);
    this->get_parameter("detection_topic", detection_topic_);
    this->get_parameter("use_sync", use_sync_);
    this->get_parameter("debug.enable_roi_log", debug_roi_log_);
    this->get_parameter("output.always_publish_image", always_publish_image_);
    // source mode (object | fixed)
    try { this->get_parameter("source.mode", source_mode_); } catch (...) { source_mode_ = "object"; }
    // 固定配信フラグ
    try { this->get_parameter("fixed.enable", fixed_enable_); } catch (...) { fixed_enable_ = false; }

    std::vector<int64_t> fixed_rect_color_vec{0,0,255};
    std::vector<int64_t> yolo_rect_color_vec{255,0,0};
    std::vector<int64_t> cut_color_vec{0,0,255};
    this->get_parameter("overlay.fixed_rect_color_bgr", fixed_rect_color_vec);
    this->get_parameter("overlay.yolo_rect_color_bgr", yolo_rect_color_vec);
    this->get_parameter("overlay.cut_point_color_bgr", cut_color_vec);
    fixed_rect_color_ = cv::Scalar(static_cast<int>(fixed_rect_color_vec[0]), static_cast<int>(fixed_rect_color_vec[1]), static_cast<int>(fixed_rect_color_vec[2]));
    yolo_rect_color_ = cv::Scalar(static_cast<int>(yolo_rect_color_vec[0]), static_cast<int>(yolo_rect_color_vec[1]), static_cast<int>(yolo_rect_color_vec[2]));
    cut_point_color_ = cv::Scalar(static_cast<int>(cut_color_vec[0]), static_cast<int>(cut_color_vec[1]), static_cast<int>(cut_color_vec[2]));
    this->get_parameter("overlay.rect_thickness", rect_thickness_);
    this->get_parameter("overlay.cut_point_radius", cut_point_radius_);
    this->get_parameter("overlay.cut_point_thickness", cut_point_thickness_);
    this->get_parameter("overlay.show_region_labels", show_region_labels_);
    this->get_parameter("overlay.text_scale", overlay_text_scale_);
    this->get_parameter("overlay.text_thickness", overlay_text_thickness_);
    this->get_parameter("overlay.yolo_hold_ms", yolo_hold_ms_);
    std::vector<int64_t> label_text_color_vec{255,255,255};
    this->get_parameter("overlay.label_text_color_bgr", label_text_color_vec);
    label_text_color_ = cv::Scalar(static_cast<int>(label_text_color_vec[0]), static_cast<int>(label_text_color_vec[1]), static_cast<int>(label_text_color_vec[2]));
    std::vector<int64_t> approach_color_vec{255,0,255};
    this->get_parameter("overlay.approach_point_color_bgr", approach_color_vec);
    approach_point_color_ = cv::Scalar(static_cast<int>(approach_color_vec[0]), static_cast<int>(approach_color_vec[1]), static_cast<int>(approach_color_vec[2]));
    this->get_parameter("overlay.approach_point_radius", approach_point_radius_);
    this->get_parameter("overlay.approach_point_thickness", approach_point_thickness_);

    this->get_parameter("publish_frame_id", publish_frame_id_);
    this->get_parameter("publish_rate", publish_rate_);
    this->get_parameter("depth_scale_m", depth_scale_m_);

    // detection.method は固定（3D）。取得ロジックは廃止
    this->get_parameter("event.mode", event_mode_);
    this->get_parameter("event.skip_if_busy", event_skip_if_busy_);
    this->get_parameter("id.dynamic_start", id_dynamic_start_);

    // pointcloud pipeline params
    this->get_parameter_or("depth_band.minus_m", depth_band_minus_m_, depth_band_minus_m_);
    this->get_parameter_or("depth_band.plus_m", depth_band_plus_m_, depth_band_plus_m_);
    this->get_parameter_or("ground_cut.auto_max_p90_offset_m", ground_cut_auto_max_p90_offset_m_, ground_cut_auto_max_p90_offset_m_);
    this->get_parameter_or("voxel.leaf_m", voxel_leaf_m_, voxel_leaf_m_);
    int mean_k = sor_mean_k_; this->get_parameter_or("sor.mean_k", mean_k, mean_k); sor_mean_k_ = mean_k;
    this->get_parameter_or("sor.stddev_mul", sor_stddev_mul_, sor_stddev_mul_);
    this->get_parameter_or("length_gate.min_length_m", min_length_m_, min_length_m_);
    this->get_parameter_or("length_gate.max_length_m", max_length_m_, max_length_m_);
    this->get_parameter_or("pca.min_confidence", pca_min_confidence_, pca_min_confidence_);

    // approach window params
    this->get_parameter_or("approach.depth.window_px", approach_window_px_, approach_window_px_);
    this->get_parameter_or("approach.depth.min_valid", approach_min_valid_, approach_min_valid_);

    // keepalive / diagnostics
    this->get_parameter("keepalive.grace_ms", keepalive_grace_ms_);
    this->get_parameter("max_depth_age_ms", max_depth_age_ms_);
    this->get_parameter("overlay.show_wait_banner", show_wait_banner_);
    // cloud outputs
    this->get_parameter("cloud.publish_roi_clouds", debug_publish_roi_clouds_);
    this->get_parameter("cloud.throttle_hz", debug_throttle_hz_);
    this->get_parameter("cloud.topics.raw_zband", cloud_topic_raw_);
    this->get_parameter("cloud.topics.post_voxel", cloud_topic_voxel_);
    this->get_parameter("cloud.topics.post_sor", cloud_topic_sor_);
    this->get_parameter("cloud.publish_raw_zband", cloud_publish_raw_zband_);
    this->get_parameter("cloud.publish_post_voxel", cloud_publish_post_voxel_);
    this->get_parameter("cloud.publish_post_sor", cloud_publish_post_sor_);
    this->get_parameter("cloud.publish_failed_rois", cloud_publish_failed_rois_);
    this->get_parameter("cloud.topics.merged_raw_zband", merged_topic_raw_);
    this->get_parameter("cloud.topics.merged_post_voxel", merged_topic_voxel_);
    this->get_parameter("cloud.topics.merged_post_sor", merged_topic_sor_);
    // YOLO filters & fixed
    this->get_parameter("yolo.min_confidence", yolo_min_confidence_);
    this->get_parameter("yolo.min_bbox_w", yolo_min_bbox_w_);
    this->get_parameter("yolo.min_bbox_h", yolo_min_bbox_h_);
    // fixed.enable は廃止
    // publish は廃止。存在する場合は警告して無視（enable がマスター）
    if (this->has_parameter("fixed.publish")) {
      bool tmp=false; this->get_parameter("fixed.publish", tmp);
      RCLCPP_WARN(this->get_logger(), "[DEPRECATED] fixed.publish is ignored. Use fixed.enable only.");
    }

    // check regions / fixed.rect（単一優先）
    std::vector<int64_t> flat;
    this->get_parameter("check_regions_flat", flat);
    // 単一固定矩形
    std::vector<int64_t> single;
    this->get_parameter("fixed.rect", single);
    int fixed_rect_id_param = 0; this->get_parameter("fixed.rect_id", fixed_rect_id_param);
    regions_.clear();
    if (fixed_enable_) {
      if (single.size() == 4) {
        StemRegion r; r.id = fixed_rect_id_param; r.x = static_cast<int>(single[0]); r.y = static_cast<int>(single[1]); r.w = static_cast<int>(single[2]); r.h = static_cast<int>(single[3]);
        regions_.push_back(r);
        RCLCPP_INFO(get_logger(), "Loaded single fixed.rect: id=%d rect=(%d,%d,%d,%d)", r.id, r.x, r.y, r.w, r.h);
      } else if (flat.size() % 5 == 0 && !flat.empty()) {
        StemRegion r; r.id = static_cast<int>(flat[0]); r.x = static_cast<int>(flat[1]); r.y = static_cast<int>(flat[2]); r.w = static_cast<int>(flat[3]); r.h = static_cast<int>(flat[4]);
        regions_.push_back(r);
        RCLCPP_INFO(get_logger(), "Loaded first region from check_regions_flat as single: id=%d rect=(%d,%d,%d,%d)", r.id, r.x, r.y, r.w, r.h);
      } else {
        RCLCPP_WARN(get_logger(), "No fixed region provided (fixed.rect or check_regions_flat)");
      }
    } else {
      RCLCPP_INFO(get_logger(), "fixed.enable=false -> fixed regions disabled (no fixed info will be sent)");
      regions_.clear();
    }

    // 動的ID管理はアナライザ側に集約。ここでは未使用。

    int iq = 3, oq = 3; std::string ir = "best_effort", orr = "best_effort";
    this->get_parameter("qos.input_queue_size", iq);
    this->get_parameter("qos.input_reliability", ir);
    this->get_parameter("qos.output_queue_size", oq);
    this->get_parameter("qos.output_reliability", orr);
    input_qos_ = rclcpp::QoS(rclcpp::KeepLast(iq)).best_effort().durability_volatile();
    output_qos_ = rclcpp::QoS(rclcpp::KeepLast(oq)).best_effort().durability_volatile();
    if (ir == "reliable") input_qos_.reliability(rclcpp::ReliabilityPolicy::Reliable);
    else input_qos_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    if (orr == "reliable") output_qos_.reliability(rclcpp::ReliabilityPolicy::Reliable);
    else output_qos_.reliability(rclcpp::ReliabilityPolicy::BestEffort);

    // 個別出力は廃止（単一detectionsに統一）

    // フォールバック（センター合わせ3D）フラグ
    try { this->get_parameter("fallback.center3d.enable", center3d_fallback_enable_); } catch (...) { center3d_fallback_enable_ = false; }
  }

  void setupPublishers() {
    annotated_pub_ = this->create_publisher<Image>(output_image_topic_, output_qos_);
    detections_pub_ = this->create_publisher<StemDetectionArray>(output_detection_topic_, output_qos_);
    // TODO: add stem vectors publisher in next stage
    // Compressed画像の出力（/compressed）
    std::string compressed_topic = output_image_topic_ + std::string("/compressed");
    annotated_compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(compressed_topic, output_qos_);
    // setup ROI cloud publishers (3 stages). If empty, derive from output_detection_topic_
    std::string base = output_detection_topic_;
    if (cloud_topic_raw_.empty())   cloud_topic_raw_   = base + std::string("/raw_zband");
    if (cloud_topic_voxel_.empty()) cloud_topic_voxel_ = base + std::string("/post_voxel");
    if (cloud_topic_sor_.empty())   cloud_topic_sor_   = base + std::string("/post_sor");
    cloud_pub_raw_   = this->create_publisher<fv_stem_detector::msg::RegionCloudArray>(cloud_topic_raw_,   output_qos_);
    cloud_pub_voxel_ = this->create_publisher<fv_stem_detector::msg::RegionCloudArray>(cloud_topic_voxel_, output_qos_);
    cloud_pub_sor_   = this->create_publisher<fv_stem_detector::msg::RegionCloudArray>(cloud_topic_sor_,   output_qos_);
    if (merged_topic_raw_.empty())   merged_topic_raw_   = base + std::string("/merged_raw_zband");
    if (merged_topic_voxel_.empty()) merged_topic_voxel_ = base + std::string("/merged_post_voxel");
    if (merged_topic_sor_.empty())   merged_topic_sor_   = base + std::string("/merged_post_sor");
    merged_pub_raw_   = this->create_publisher<sensor_msgs::msg::PointCloud2>(merged_topic_raw_,   output_qos_);
    merged_pub_voxel_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(merged_topic_voxel_, output_qos_);
    merged_pub_sor_   = this->create_publisher<sensor_msgs::msg::PointCloud2>(merged_topic_sor_,   output_qos_);
  }

  // 固定枠のON/OFFを動的に切り替えるサービス
  void setupServices() {
    // 汎用: サービス名をカメラ別の絶対パスで公開
    std::string service_ns;
    try { this->get_parameter("service.ns", service_ns); } catch (...) { service_ns.clear(); }
    if (service_ns.empty()) {
      // camera_topic から "/fv/<camera>" を抽出
      std::string cam_topic;
      try { this->get_parameter("camera_topic", cam_topic); } catch (...) { cam_topic.clear(); }
      if (!cam_topic.empty() && cam_topic.rfind("/fv/", 0) == 0) {
        // "/fv/" の次のスラッシュ位置→ "/fv/<camera>" を得る
        size_t p = cam_topic.find('/', 4);
        if (p != std::string::npos) {
          service_ns = cam_topic.substr(0, p);
        }
      }
    }
    std::string service_name;
    if (!service_ns.empty()) {
      service_name = service_ns + std::string("/stem_detector/set_source_mode");
    } else {
      // 最低限の衝突回避: ノード名を含めた絶対名
      service_name = std::string("/") + this->get_name() + std::string("/set_source_mode");
    }
    RCLCPP_INFO(this->get_logger(), "advertise service: %s", service_name.c_str());

    set_source_mode_srv_ = this->create_service<fv_stem_detector::srv::SetSourceMode>(
        service_name,
        [this](const std::shared_ptr<fv_stem_detector::srv::SetSourceMode::Request> req,
               std::shared_ptr<fv_stem_detector::srv::SetSourceMode::Response> res) {
          const std::string &m = req->mode;
          if (m != "object" && m != "fixed") {
            res->success = false;
            res->message = std::string("invalid mode: ") + m;
            return;
          }
          source_mode_ = m;
          this->set_parameter(rclcpp::Parameter("source.mode", source_mode_));
          res->success = true;
          res->message = std::string("source.mode set to ") + source_mode_;
        });

    // 固定矩形をYAMLに保存するサービス（std_srvs/SetBool）
    std::string save_srv_name;
    if (!service_ns.empty()) save_srv_name = service_ns + std::string("/stem_detector/save_fixed_rect_yaml");
    else save_srv_name = std::string("/") + this->get_name() + std::string("/save_fixed_rect_yaml");
    save_fixed_yaml_srv_ = this->create_service<std_srvs::srv::SetBool>(
        save_srv_name,
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
               std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
          (void)req;
          std::string path; this->get_parameter("fixed.save_yaml_path", path);
          if (path.empty()) {
            res->success = false; res->message = "fixed.save_yaml_path is empty"; return;
          }
          if (regions_.empty()) {
            res->success = false; res->message = "no fixed region to save"; return;
          }
          const auto &r = regions_.front();
          std::ofstream ofs(path, std::ios::out | std::ios::trunc);
          if (!ofs) { res->success = false; res->message = std::string("cannot open: ") + path; return; }
          // 最小限のオーバレイYAML: 単一要素のcheck_regions_flatを出力
          ofs << "check_regions_flat: [" << r.id << ", " << r.x << ", " << r.y << ", " << r.w << ", " << r.h << "]\n";
          ofs.close();
          res->success = true; res->message = std::string("saved: ") + path;
          RCLCPP_INFO(this->get_logger(), "saved fixed rect to %s", path.c_str());
        });
  }

  void setupSubscribers() {
    if (use_sync_ && event_mode_ == "legacy_sync") {
      using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;
      image_sub_.subscribe(this, camera_topic_, "raw", input_qos_.get_rmw_qos_profile());
      depth_sub_.subscribe(this, depth_topic_, "raw", input_qos_.get_rmw_qos_profile());
      sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), image_sub_, depth_sub_);
      sync_->registerCallback(std::bind(&StemDetectorNode::onImageDepth, this, std::placeholders::_1, std::placeholders::_2));
    } else {
      // 非同期: 個別購読して、カラー到着時に最新の深度（一定以内の古さOK）で処理
      image_async_sub_ = this->create_subscription<Image>(
          camera_topic_, input_qos_, std::bind(&StemDetectorNode::onColorAsync, this, std::placeholders::_1));
      depth_async_sub_ = this->create_subscription<Image>(
          depth_topic_, input_qos_, std::bind(&StemDetectorNode::onDepthAsync, this, std::placeholders::_1));
    }

    // YOLO detections (dynamic ROIs)
    detections_sub_ = this->create_subscription<Detection2DArray>(
        detection_topic_, input_qos_,
        std::bind(&StemDetectorNode::onDetections, this, std::placeholders::_1));

    // Camera info (intrinsics)
    // camera_info は RealSense 側 Publisher=best_effort に合わせて best_effort で購読
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, rclcpp::SensorDataQoS(),
        std::bind(&StemDetectorNode::onCameraInfo, this, std::placeholders::_1));
    camera_info_depth_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        depth_camera_info_topic_, rclcpp::SensorDataQoS(),
        std::bind(&StemDetectorNode::onCameraInfo, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "camera_info(color) 購読QoS: SensorDataQoS(best_effort), topic=%s", camera_info_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "camera_info(depth) 購読QoS: SensorDataQoS(best_effort), topic=%s", depth_camera_info_topic_.c_str());

    // Color-only preview when sync is missing（legacy_syncのみ）
    if (event_mode_ == "legacy_sync") {
      color_only_sub_ = this->create_subscription<Image>(
          camera_topic_, input_qos_,
          std::bind(&StemDetectorNode::onColorOnly, this, std::placeholders::_1));
    }
  }

  void drawOverlay(
      cv::Mat& bgr,
      const std::vector<StemRegion>& fixed_regions,
      const std::vector<StemDetectionResult>& fixed_dets,
      const std::vector<StemRegion>& yolo_regions,
      const std::vector<StemDetectionResult>& yolo_dets) {
    // draw_label は使用せず、必要箇所で fluent::text::draw を直接呼び出す方針
    for (const auto& r : fixed_regions) {
      cv::rectangle(bgr, cv::Rect(r.x, r.y, r.w, r.h), fixed_rect_color_, rect_thickness_);
      if (show_region_labels_) {
        int x = std::min(bgr.cols - 10, r.x + r.w + 6);
        int y = std::max(0, r.y + 24 - 6);
        fluent::text::draw(bgr, std::string("固定-") + std::to_string(r.id), cv::Point(x + 4, y), label_text_color_, overlay_text_scale_, overlay_text_thickness_, 0);
      }
    }
    for (const auto& r : yolo_regions) {
      cv::rectangle(bgr, cv::Rect(r.x, r.y, r.w, r.h), yolo_rect_color_, rect_thickness_);
      if (show_region_labels_) {
        int x = std::min(bgr.cols - 10, r.x + r.w + 6);
        int y = std::max(0, r.y + 24 - 6);
        fluent::text::draw(bgr, std::string("YOLO-") + std::to_string(r.id), cv::Point(x + 4, y), label_text_color_, overlay_text_scale_, overlay_text_thickness_, 0);
      }
    }
    // draw cut points (same style for both)
    auto draw_points = [&](const std::vector<StemDetectionResult>& dets){
      for (const auto& d : dets) {
        if (d.detected) {
          cv::circle(bgr, cv::Point(d.x, d.y), cut_point_radius_, cut_point_color_, cut_point_thickness_);
        }
      }
    };
    draw_points(fixed_dets);
    draw_points(yolo_dets);
  }

  // helper: compute text width (rough) for background box sizing if needed later
  int approxTextWidthPx(const std::string& s, double scale=0.6, int thickness=2) const {
    int baseline=0; cv::Size sz = cv::getTextSize(s, cv::FONT_HERSHEY_SIMPLEX, scale, thickness, &baseline);
    return sz.width;
  }

  void onDetections(const Detection2DArray::SharedPtr msg) {
    // YOLOの2D信頼度しきい値は動的反映（毎回取得）
    double eff_yolo_min_conf = yolo_min_confidence_;
    try { this->get_parameter("yolo.min_confidence", eff_yolo_min_conf); } catch(...) {}
    std::vector<StemRegion> tmp;
    tmp.reserve(msg->detections.size());
    int region_id = 0;
    for (const auto& det : msg->detections) {
      if (det.results.empty()) continue;
      const auto& bbox = det.bbox;
      int w = static_cast<int>(bbox.size_x);
      int h = static_cast<int>(bbox.size_y);
      if (w < yolo_min_bbox_w_ || h < yolo_min_bbox_h_) continue;
      float score = 0.0f;
      try { score = static_cast<float>(det.results[0].hypothesis.score); } catch (...) { score = 0.0f; }
      if (score < static_cast<float>(eff_yolo_min_conf)) continue;
      // クラスIDを数値化（非数や未設定は除外）。とりあえず 0=アスパラ のみ通す。
      int parsed_class_id = -1;
      try {
        std::string cid_str;
        try { cid_str = det.results[0].hypothesis.class_id; } catch (...) { cid_str = ""; }
        if (!cid_str.empty()) parsed_class_id = std::stoi(cid_str);
      } catch (...) { parsed_class_id = -1; }
      if (parsed_class_id != 0) {
        continue; // アスパラ以外は対象外
      }
      int x = static_cast<int>(bbox.center.position.x - bbox.size_x * 0.5);
      int y = static_cast<int>(bbox.center.position.y - bbox.size_y * 0.5);
      if (w <= 0 || h <= 0) continue;
      StemRegion r{region_id++, x, y, w, h};
      r.yolo_confidence = score;
      r.class_id = parsed_class_id;
      tmp.push_back(r);
    }
    {
      std::lock_guard<std::mutex> lock(yolo_regions_mutex_);
      if (!tmp.empty()) {
        yolo_regions_ = tmp;
        yolo_regions_hold_ = yolo_regions_;
        yolo_last_update_ = std::chrono::steady_clock::now();
      } else {
        // empty frame: keep last for a short hold period
        auto now = std::chrono::steady_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - yolo_last_update_).count();
        if (ms <= yolo_hold_ms_) {
          yolo_regions_ = yolo_regions_hold_;
        } else {
          yolo_regions_.clear();
        }
      }
    }
    // YOLOトリガ運用: 最新Color/Depthで即処理（Latest-1方式）。
    if (event_mode_ == "yolo_trigger") {
      if (event_skip_if_busy_ && processing_busy_) return;
      sensor_msgs::msg::Image::ConstSharedPtr color;
      sensor_msgs::msg::Image::ConstSharedPtr depth;
      {
        std::lock_guard<std::mutex> lk(latest_async_mutex_);
        color = latest_color_;
        depth = latest_depth_;
      }
      if (!color || !depth) return;
      processing_busy_ = true;
      onImageDepth(color, depth);
      processing_busy_ = false;
    }
  }

  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (msg->k.size() < 9) return;
    double fxn = msg->k[0];
    double fyn = msg->k[4];
    double cxn = msg->k[2];
    double cyn = msg->k[5];
    if (!(fxn > 0.0 && fyn > 0.0)) return;
    // 一度でも有効値を受けたら維持。未設定時のみ更新（color/depth両方を購読しているため）
    if (!intrinsics_ready_) {
      fx_ = fxn; fy_ = fyn; cx_ = cxn; cy_ = cyn; intrinsics_ready_ = true;
      RCLCPP_INFO(get_logger(), "camera_info 受信: fx=%.2f fy=%.2f cx=%.2f cy=%.2f", fx_, fy_, cx_, cy_);
    }
  }

  void onImageDepth(const Image::ConstSharedPtr& imgMsg, const Image::ConstSharedPtr& depthMsg) {
    // Process only if there is at least one subscriber to any output
    const bool need_image  = always_publish_image_ || (annotated_pub_ && annotated_pub_->get_subscription_count() > 0);
    const bool need_merged = detections_pub_ && detections_pub_->get_subscription_count() > 0;
    if (!need_image && !need_merged) {
      return;
    }

    cv_bridge::CvImageConstPtr imgCv;
    cv_bridge::CvImageConstPtr depthCv;
    try {
      imgCv = cv_bridge::toCvShare(imgMsg, "bgr8");
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "cv_bridge conversion error image: %s", e.what());
      return;
    }
    try {
      depthCv = cv_bridge::toCvShare(depthMsg);
    } catch (const std::exception& e) {
    // update last timestamps
    last_depth_tp_ = std::chrono::steady_clock::now();
    last_color_tp_ = last_depth_tp_;
      RCLCPP_WARN(get_logger(), "cv_bridge conversion error depth: %s", e.what());
      return;
    }

    if (detection_method_ == "pointcloud_linefit") {
      auto frame_t0 = std::chrono::high_resolution_clock::now();
      // Stage1: ROI→アプローチZ→Z±帯→yパススルー→Voxel→SOR→PCA主軸
      // 1) YOLO領域を取得
      std::vector<StemRegion> yolo_regions_copy;
      {
        std::lock_guard<std::mutex> lock(yolo_regions_mutex_);
        yolo_regions_copy = yolo_regions_;
      }
      // モードに応じた処理対象（フォールバック無し）
      const std::vector<StemRegion>& target_regions = (source_mode_ == std::string("object"))
          ? static_cast<const std::vector<StemRegion>&>(yolo_regions_copy)
          : static_cast<const std::vector<StemRegion>&>(regions_);

      // 2) 内参
      if (!intrinsics_ready_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 3000, "camera intrinsics not ready");
        return;
      }
      const double fx = fx_, fy = fy_, cx = cx_, cy = cy_;

      // ===== モード別プロファイル適用（未指定は共通値を採用） =====
      auto pref = (source_mode_ == std::string("object")) ? std::string("profiles.object.")
                                                          : std::string("profiles.fixed.");
      auto getD = [&](const std::string &key, double base){
        double v = base;
        // profiles.* 優先
        if (this->has_parameter(pref + key)) {
          try { v = this->get_parameter(pref + key).get_value<double>(); return v; } catch(...) {}
        }
        // トップレベルのフォールバック
        if (this->has_parameter(key)) {
          try { v = this->get_parameter(key).get_value<double>(); } catch(...) {}
        }
        return v;
      };
      auto getI = [&](const std::string &key, int base){
        int v = base;
        if (this->has_parameter(pref + key)) {
          try { v = this->get_parameter(pref + key).get_value<int>(); return v; } catch(...) {}
        }
        if (this->has_parameter(key)) {
          try { v = this->get_parameter(key).get_value<int>(); } catch(...) {}
        }
        return v;
      };
      auto getB = [&](const std::string &key, bool base){
        bool v = base;
        if (this->has_parameter(pref + key)) {
          try { v = this->get_parameter(pref + key).get_value<bool>(); return v; } catch(...) {}
        }
        if (this->has_parameter(key)) {
          try { v = this->get_parameter(key).get_value<bool>(); } catch(...) {}
        }
        return v;
      };

      const double eff_depth_band_minus_m = getD("depth_band.minus_m", depth_band_minus_m_);
      const double eff_depth_band_plus_m  = getD("depth_band.plus_m",  depth_band_plus_m_);
      const double eff_ground_cut_p90     = getD("ground_cut.auto_max_p90_offset_m", ground_cut_auto_max_p90_offset_m_);
      const double eff_voxel_leaf_m       = getD("voxel.leaf_m", voxel_leaf_m_);
      const int    eff_sor_mean_k         = getI("sor.mean_k", sor_mean_k_);
      const double eff_sor_stddev_mul     = getD("sor.stddev_mul", sor_stddev_mul_);
      const double eff_min_length_m       = getD("length_gate.min_length_m", min_length_m_);
      const double eff_max_length_m       = getD("length_gate.max_length_m", max_length_m_);
      const double eff_pca_min_conf       = getD("pca.min_confidence", pca_min_confidence_);
      const int    eff_pc_sample_stride   = getI("pc.sample_stride_px", pc_sample_stride_px_);
      const int    eff_approach_win_px    = getI("approach.depth.window_px", approach_window_px_);
      const int    eff_approach_min_valid = getI("approach.depth.min_valid", approach_min_valid_);
      const bool   eff_approach_search_enable = getB("approach.search.enable", false);
      const int    eff_approach_grid_step_px  = getI("approach.search.grid_step_px", 8);
      const double eff_approach_exclude_bottom_ratio = getD("approach.search.exclude_bottom_ratio", 0.0);
      // depth_scale_m はカメラ固有（D405=0.0001, D415=0.001）。
      // profiles.* 配下の宣言既定値(0.001)が D405 の 0.0001 を不正に上書きするため、
      // ここでは常にトップレベルの depth_scale_m_ を使用する。
      const double eff_depth_scale_m      = depth_scale_m_;

      auto compute_median_depth = [&](const cv::Rect& roi)->double{
        std::vector<uint16_t> vals; vals.reserve(64);
        int cxp = roi.x + roi.width/2; int cyp = roi.y + roi.height/2;
        int half = std::max(1, eff_approach_win_px/2);
        for (int dv = -half; dv <= half; ++dv) {
          for (int du = -half; du <= half; ++du) {
            int u = std::clamp(cxp + du, 0, depthCv->image.cols - 1);
            int v = std::clamp(cyp + dv, 0, depthCv->image.rows - 1);
            uint16_t raw = depthCv->image.at<uint16_t>(v, u);
            if (raw > 0) vals.push_back(raw);
          }
        }
        if (static_cast<int>(vals.size()) < eff_approach_min_valid) return std::numeric_limits<double>::quiet_NaN();
        std::nth_element(vals.begin(), vals.begin() + vals.size()/2, vals.end());
        uint16_t med = vals[vals.size()/2];
        return static_cast<double>(med) * eff_depth_scale_m;
      };

      auto make_cloud_from_roi = [&](const cv::Rect& roi, double z_min, double z_max)->pcl::PointCloud<pcl::PointXYZRGB>::Ptr{
        auto cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        int stride = std::max(1, eff_pc_sample_stride);
        for (int v = roi.y; v < roi.y + roi.height; v += stride) {
          for (int u = roi.x; u < roi.x + roi.width; u += stride) {
            uint16_t raw = depthCv->image.at<uint16_t>(v, u);
            if (raw == 0) continue;
            double Z = static_cast<double>(raw) * eff_depth_scale_m;
            if (Z < z_min || Z > z_max) continue;
            double X = (static_cast<double>(u) - cx) * Z / fx;
            double Y = (static_cast<double>(v) - cy) * Z / fy;
            pcl::PointXYZRGB pt; pt.x = static_cast<float>(X); pt.y = static_cast<float>(Y); pt.z = static_cast<float>(Z);
            const cv::Vec3b& c = imgCv->image.at<cv::Vec3b>(v, u);
            pt.r = c[2]; pt.g = c[1]; pt.b = c[0];
            cloud->points.push_back(pt);
          }
        }
        cloud->width = cloud->points.size(); cloud->height = 1; cloud->is_dense = false;
        return pcl::PointCloud<pcl::PointXYZRGB>::Ptr(cloud);
      };

      auto pass_through_y_auto = [&](pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)->pcl::PointCloud<pcl::PointXYZRGB>::Ptr{
        if (!cloud || cloud->empty()) return cloud;
        std::vector<float> ys; ys.reserve(cloud->points.size());
        for (auto &p : cloud->points) ys.push_back(p.y);
        size_t k = static_cast<size_t>(std::floor(0.9 * (ys.size()-1)));
        std::nth_element(ys.begin(), ys.begin()+k, ys.end());
        float y90 = ys[k];
        float y_max = y90 + static_cast<float>(eff_ground_cut_p90);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGB>);
        out->points.reserve(cloud->points.size());
        for (auto &p : cloud->points) if (p.y <= y_max) out->points.push_back(p);
        out->width = out->points.size(); out->height = 1; out->is_dense = false; return out;
      };

      auto voxel_sor = [&](pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)->pcl::PointCloud<pcl::PointXYZRGB>::Ptr{
        if (!cloud || cloud->empty()) return cloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr v(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::VoxelGrid<pcl::PointXYZRGB> vg; vg.setInputCloud(cloud);
        float ls = static_cast<float>(eff_voxel_leaf_m);
        vg.setLeafSize(ls, ls, ls); vg.filter(*v);
        if (v->size() < 10) return v;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr o(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor; sor.setInputCloud(v);
        sor.setMeanK(eff_sor_mean_k); sor.setStddevMulThresh(static_cast<float>(eff_sor_stddev_mul)); sor.filter(*o);
        return o;
      };

      auto pca_axis = [&](pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Eigen::Vector3f &center, Eigen::Vector3f &axis, float &conf)->bool{
        if (!cloud || cloud->size() < 10) return false;
        center = Eigen::Vector3f::Zero();
        for (auto &p : cloud->points) center += Eigen::Vector3f(p.x, p.y, p.z);
        center /= static_cast<float>(cloud->points.size());
        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        for (auto &p : cloud->points) {
          Eigen::Vector3f d(p.x, p.y, p.z); d -= center; cov += d * d.transpose();
        }
        cov /= static_cast<float>(cloud->points.size());
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
        if (es.info() != Eigen::Success) return false;
        Eigen::Vector3f evals = es.eigenvalues();
        Eigen::Matrix3f evecs = es.eigenvectors();
        // largest eigenvalue/eigenvector
        int idx_max; evals.maxCoeff(&idx_max);
        float sum = evals(0) + evals(1) + evals(2);
        conf = (sum > 0.0f) ? (evals(idx_max) / sum) : 0.0f;
        axis = evecs.col(idx_max).normalized();
        return true;
      };

      auto project_minmax = [&](pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const Eigen::Vector3f &center, const Eigen::Vector3f &axis, Eigen::Vector3f &pmin, Eigen::Vector3f &pmax)->bool{
        if (!cloud || cloud->empty()) return false;
        float smin = std::numeric_limits<float>::infinity();
        float smax = -std::numeric_limits<float>::infinity();
        for (auto &p : cloud->points) {
          Eigen::Vector3f d(p.x, p.y, p.z); d -= center; float s = d.dot(axis);
          if (s < smin) { smin = s; pmin = center + s * axis; }
          if (s > smax) { smax = s; pmax = center + s * axis; }
        }
        return std::isfinite(smin) && std::isfinite(smax);
      };

      // IDs for display を簡素化（IDトラッキング無効化のため領域ID/連番）
      std::vector<int> ids_for_display; ids_for_display.reserve(target_regions.size());
      for (size_t i = 0; i < target_regions.size(); ++i) {
        int idv = target_regions[i].id;
        if (idv < 0) idv = static_cast<int>(i);
        ids_for_display.push_back(idv);
      }
      // Processing per region
      cv::Mat annotated; if (need_image) imgCv->image.copyTo(annotated);
      // アクティブなソースモードの矩形のみ可視化
      if (need_image) {
        if (source_mode_ == std::string("fixed") && fixed_enable_) {
          for (const auto &fr : regions_) {
            cv::rectangle(annotated, cv::Rect(fr.x, fr.y, fr.w, fr.h), fixed_rect_color_, rect_thickness_);
            if (show_region_labels_) {
              std::string label = std::string("固定-") + std::to_string(fr.id);
              fluent::text::drawShadow(annotated, label, cv::Point(std::min(annotated.cols - 10, fr.x + fr.w + 6), fr.y + 24), label_text_color_, cv::Scalar(0,0,0), overlay_text_scale_, overlay_text_thickness_, 0);
            }
          }
        } else { // object or fixed disabled
          for (const auto &yr : yolo_regions_copy) {
            cv::rectangle(annotated, cv::Rect(yr.x, yr.y, yr.w, yr.h), yolo_rect_color_, rect_thickness_);
            if (show_region_labels_) {
              std::string label = std::string("YOLO-") + std::to_string(yr.id);
              fluent::text::drawShadow(annotated, label, cv::Point(std::min(annotated.cols - 10, yr.x + yr.w + 6), yr.y + 24), label_text_color_, cv::Scalar(0,0,0), overlay_text_scale_, overlay_text_thickness_, 0);
            }
          }
        }
      }
      double detect_total_ms3d = 0.0;
      // 出力メッセージ（merged）を準備
      StemDetectionArray merged3d;
      // 個別配列の常時出力は廃止（単一detectionsに統一）
      // ROI cloud arrays per stage
      fv_stem_detector::msg::RegionCloudArray arr_raw;   arr_raw.header = imgMsg->header;
      fv_stem_detector::msg::RegionCloudArray arr_voxel; arr_voxel.header = imgMsg->header;
      fv_stem_detector::msg::RegionCloudArray arr_sor;   arr_sor.header = imgMsg->header;
      bool allow_clouds = debug_publish_roi_clouds_ && (
        (cloud_pub_raw_ && cloud_pub_raw_->get_subscription_count()>0) ||
        (cloud_pub_voxel_ && cloud_pub_voxel_->get_subscription_count()>0) ||
        (cloud_pub_sor_ && cloud_pub_sor_->get_subscription_count()>0)
      );
      bool allow_merged = (cloud_publish_raw_zband_ || cloud_publish_post_voxel_ || cloud_publish_post_sor_) && (
        (merged_pub_raw_ && merged_pub_raw_->get_subscription_count()>0) ||
        (merged_pub_voxel_ && merged_pub_voxel_->get_subscription_count()>0) ||
        (merged_pub_sor_ && merged_pub_sor_->get_subscription_count()>0)
      );
      bool pass_rate = true;
      if (allow_clouds && debug_throttle_hz_ > 0.0) {
        auto now = this->get_clock()->now();
        double interval = 1.0 / debug_throttle_hz_;
        pass_rate = (now - last_debug_pub_time_).seconds() >= interval;
      }
      for (size_t ri = 0; ri < target_regions.size(); ++ri) {
        const auto &r = target_regions[ri];
        bool is_fixed = (&target_regions == &regions_);
        cv::Rect roi(std::clamp(r.x, 0, imgCv->image.cols-1), std::clamp(r.y, 0, imgCv->image.rows-1), r.w, r.h);
        roi &= cv::Rect(0,0,imgCv->image.cols,imgCv->image.rows);
        if (roi.width <=0 || roi.height <=0) continue;
        auto roi_t0 = std::chrono::high_resolution_clock::now();
        // call pipeline
        fv_stem_detector::Intrinsics K{fx, fy, cx, cy};
        fv_stem_detector::PipelineParams pp;
        pp.pc_sample_stride_px = eff_pc_sample_stride;
        pp.depth_band_minus_m = eff_depth_band_minus_m;
        pp.depth_band_plus_m  = eff_depth_band_plus_m;
        pp.ground_cut_auto_max_p90_offset_m = eff_ground_cut_p90;
        pp.voxel_leaf_m = eff_voxel_leaf_m;
        pp.sor_mean_k = eff_sor_mean_k;
        pp.sor_stddev_mul = eff_sor_stddev_mul;
        pp.pca_min_confidence = eff_pca_min_conf;
        pp.min_length_m = eff_min_length_m;
        pp.max_length_m = eff_max_length_m;
        pp.approach_window_px = eff_approach_win_px;
        pp.approach_min_valid = eff_approach_min_valid;
        pp.wide_approach_search = eff_approach_search_enable;
        pp.approach_grid_step_px = eff_approach_grid_step_px;
        pp.approach_exclude_bottom_ratio = eff_approach_exclude_bottom_ratio;
        pp.depth_scale_m = eff_depth_scale_m;
        // Foxgloveのmerged出力のみ購読があっても収集する
        pp.collect_debug_clouds = (allow_clouds || allow_merged) && pass_rate;
        fv_stem_detector::RegionInput rin; rin.roi = roi;
        auto rout = detector3d_.processRegion(imgCv->image, depthCv->image, K, pp, rin);
        bool pass = rout.ok;
        bool fallback_used = false;
        fv_stem_detector::RegionOutput rout_fb;
        if (!pass) {
          const char* reason = rout.fail_reason.empty() ? "unknown" : rout.fail_reason.c_str();
          RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                               "[3D fail] mode=%s id=%zu reason=%s pts=%d roi_ms=%.1f",
                               (is_fixed?"fixed":"object"), ri, reason, rout.point_count, rout.roi_ms);
          // パラメータ緩和でフォールバック再試行
          fv_stem_detector::PipelineParams pp2 = pp;
          pp2.pca_min_confidence = std::min(pp.pca_min_confidence, 0.40);
          pp2.sor_stddev_mul = std::max(pp.sor_stddev_mul, 2.0);
          pp2.ground_cut_auto_max_p90_offset_m = std::max(pp.ground_cut_auto_max_p90_offset_m, 0.010);
          pp2.depth_band_minus_m = std::max(pp.depth_band_minus_m, pp.depth_band_minus_m * 2.0);
          pp2.depth_band_plus_m  = std::max(pp.depth_band_plus_m,  pp.depth_band_plus_m  * 2.0);
          rout_fb = detector3d_.processRegion(imgCv->image, depthCv->image, K, pp2, rin);
          if (rout_fb.ok) {
            pass = true; fallback_used = true; rout = rout_fb;
            RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000,
                                 "[fallback ok] mode=%s id=%zu pca=%.2f len=%.3f z=%.3f pts=%d",
                                 (is_fixed?"fixed":"object"), ri, rout.pca_conf,
                                 static_cast<double>(rout.length_m), static_cast<double>(rout.approach_z_m), rout.point_count);
          } else {
            const char* fb_reason = rout_fb.fail_reason.empty() ? "unknown" : rout_fb.fail_reason.c_str();
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                                 "[fallback fail] mode=%s id=%zu reason=%s pts=%d roi_ms=%.1f",
                                 (is_fixed?"fixed":"object"), ri, fb_reason, rout_fb.point_count, rout_fb.roi_ms);
          }
        }
        // ===== ROI debug log (optional) =====
        if (debug_roi_log_) {
          double yolo_sc = std::numeric_limits<double>::quiet_NaN();
          int cls_id = -1;
          if (!is_fixed && ri < yolo_regions_copy.size()) {
            yolo_sc = static_cast<double>(yolo_regions_copy[ri].yolo_confidence);
            cls_id = yolo_regions_copy[ri].class_id;
          }
          RCLCPP_INFO(this->get_logger(),
            "[ROI] mode=%s id=%zu rect=(%d,%d,%d,%d) yolo=%.2f cls=%d pass=%s pca=%.2f len=%.3f z=%.3f pts=%d roi_ms=%.1f",
            (is_fixed?"fixed":"object"), ri, roi.x, roi.y, roi.width, roi.height,
            yolo_sc, cls_id, (pass?"OK":"NG"), static_cast<double>(rout.pca_conf),
            static_cast<double>(rout.length_m), static_cast<double>(rout.approach_z_m),
            rout.point_count, rout.roi_ms);
        }
        // 検出メッセージは合否に関わらず1件生成（pass=falseはスタブ）
        {
          detect_total_ms3d += rout.roi_ms;
          StemDetection m;
          m.region_id = static_cast<int>(ri);
          m.detected = pass;
          m.x = roi.x; m.y = roi.y; m.w = roi.width; m.h = roi.height;
          if (pass) {
            cv::Point approach_px = rout.approach_px;
            cv::Point pr = rout.root_px, pt = rout.tip_px;
            m.approach_px_x = approach_px.x; m.approach_px_y = approach_px.y;
            m.root_px_x = pr.x; m.root_px_y = pr.y;
            m.tip_px_x = pt.x; m.tip_px_y = pt.y;
            m.approach_camera.x = static_cast<double>((approach_px.x - cx) ) * rout.approach_z_m / fx;
            m.approach_camera.y = static_cast<double>((approach_px.y - cy) ) * rout.approach_z_m / fy;
            m.approach_camera.z = rout.approach_z_m;
            m.root_camera.x = static_cast<double>(rout.end_root.x()); m.root_camera.y = static_cast<double>(rout.end_root.y()); m.root_camera.z = static_cast<double>(rout.end_root.z());
            m.tip_camera.x  = static_cast<double>(rout.end_tip.x());  m.tip_camera.y  = static_cast<double>(rout.end_tip.y());  m.tip_camera.z  = static_cast<double>(rout.end_tip.z());
            m.approach_distance_m = static_cast<float>(rout.approach_z_m);
            m.root_distance_m = static_cast<float>(rout.end_root.z());
            m.tip_distance_m  = static_cast<float>(rout.end_tip.z());
            m.pca_score = rout.pca_conf;
            m.pca_eigenvalues[0] = std::numeric_limits<float>::quiet_NaN();
            m.pca_eigenvalues[1] = std::numeric_limits<float>::quiet_NaN();
            m.pca_eigenvalues[2] = std::numeric_limits<float>::quiet_NaN();
            m.axis_camera.x = rout.pca_axis.x(); m.axis_camera.y = rout.pca_axis.y(); m.axis_camera.z = rout.pca_axis.z();
            m.length_m = rout.length_m;
          } else {
            int cxp = roi.x + roi.width/2; int cyp = roi.y + roi.height/2;
            m.approach_px_x = cxp; m.approach_px_y = cyp;
            // フォールバック: 根本=底辺中央 / 先端=上辺中央
            m.root_px_x = cxp; m.root_px_y = roi.y + roi.height - 1;
            m.tip_px_x  = cxp; m.tip_px_y  = roi.y;
            // 3Dフォールバックはフラグで制御（既定OFF）
            if (center3d_fallback_enable_) {
              // 可能なら近傍深度の中央値で3Dを推定（失敗時はNaNのまま）
              auto sample_depth_m = [&](int px, int py) -> double {
                if (px < 0 || py < 0 || px >= depthCv->image.cols || py >= depthCv->image.rows) return std::numeric_limits<double>::quiet_NaN();
                int r = std::max(0, eff_approach_win_px);
                int min_valid = std::max(1, eff_approach_min_valid);
                std::vector<double> zs; zs.reserve((2*r+1)*(2*r+1));
                int x0 = std::clamp(px - r, 0, depthCv->image.cols-1);
                int x1 = std::clamp(px + r, 0, depthCv->image.cols-1);
                int y0 = std::clamp(py - r, 0, depthCv->image.rows-1);
                int y1 = std::clamp(py + r, 0, depthCv->image.rows-1);
                for (int yy=y0; yy<=y1; ++yy) for (int xx=x0; xx<=x1; ++xx) {
                  if (depthCv->image.type() == CV_16UC1) {
                    uint16_t v = depthCv->image.at<uint16_t>(yy, xx); if (v==0) continue; double mmm = v * eff_depth_scale_m; if (mmm>0.0) zs.push_back(mmm);
                  } else if (depthCv->image.type() == CV_32FC1) {
                    float v = depthCv->image.at<float>(yy, xx); if (!std::isfinite(v) || v<=0.0f) continue; zs.push_back(static_cast<double>(v));
                  }
                }
                if (static_cast<int>(zs.size()) < min_valid) return std::numeric_limits<double>::quiet_NaN();
                std::nth_element(zs.begin(), zs.begin()+zs.size()/2, zs.end());
                double z = zs[zs.size()/2];
                return (std::isfinite(z) && z>0.0) ? z : std::numeric_limits<double>::quiet_NaN();
              };
              auto backproject = [&](int px, int py, double z, geometry_msgs::msg::Point &out){
                if (std::isfinite(z) && fx>0.0 && fy>0.0) {
                  out.x = (static_cast<double>(px) - cx) * z / fx;
                  out.y = (static_cast<double>(py) - cy) * z / fy;
                  out.z = z;
                } else {
                  out.x = out.y = out.z = std::numeric_limits<double>::quiet_NaN();
                }
              };
              double z_app = sample_depth_m(m.approach_px_x, m.approach_px_y);
              double z_root = sample_depth_m(m.root_px_x, m.root_px_y);
              double z_tip  = sample_depth_m(m.tip_px_x,  m.tip_px_y);
              // 代替: ルートが取れて tipが取れない場合は同一Zを使う（近似）
              if (!std::isfinite(z_tip) && std::isfinite(z_root)) z_tip = z_root;
              if (!std::isfinite(z_root) && std::isfinite(z_app)) z_root = z_app;
              if (!std::isfinite(z_tip)  && std::isfinite(z_app)) z_tip  = z_app;
              backproject(m.approach_px_x, m.approach_px_y, z_app, m.approach_camera);
              backproject(m.root_px_x,     m.root_px_y,     z_root, m.root_camera);
              backproject(m.tip_px_x,      m.tip_px_y,      z_tip,  m.tip_camera);
              m.approach_distance_m = std::isfinite(z_app)  ? static_cast<float>(z_app)  : std::numeric_limits<float>::quiet_NaN();
              m.root_distance_m     = std::isfinite(z_root) ? static_cast<float>(z_root) : std::numeric_limits<float>::quiet_NaN();
              m.tip_distance_m      = std::isfinite(z_tip)  ? static_cast<float>(z_tip)  : std::numeric_limits<float>::quiet_NaN();
            } else {
              // 無効時は3DはNaNのまま
              m.approach_camera.x = std::numeric_limits<double>::quiet_NaN();
              m.approach_camera.y = std::numeric_limits<double>::quiet_NaN();
              m.approach_camera.z = std::numeric_limits<double>::quiet_NaN();
              m.root_camera.x = std::numeric_limits<double>::quiet_NaN();
              m.root_camera.y = std::numeric_limits<double>::quiet_NaN();
              m.root_camera.z = std::numeric_limits<double>::quiet_NaN();
              m.tip_camera.x = std::numeric_limits<double>::quiet_NaN();
              m.tip_camera.y = std::numeric_limits<double>::quiet_NaN();
              m.tip_camera.z = std::numeric_limits<double>::quiet_NaN();
              m.approach_distance_m = std::numeric_limits<float>::quiet_NaN();
              m.root_distance_m = std::numeric_limits<float>::quiet_NaN();
              m.tip_distance_m  = std::numeric_limits<float>::quiet_NaN();
            }
            m.pca_score = std::numeric_limits<float>::quiet_NaN();
            m.axis_camera.x = std::numeric_limits<double>::quiet_NaN();
            m.axis_camera.y = std::numeric_limits<double>::quiet_NaN();
            m.axis_camera.z = std::numeric_limits<double>::quiet_NaN();
            m.length_m = std::numeric_limits<float>::quiet_NaN();
          }
          if (is_fixed) {
            m.source = StemDetection::SOURCE_FIXED;
            m.class_id = -1; m.yolo_score = std::numeric_limits<float>::quiet_NaN();
          } else {
            m.source = StemDetection::SOURCE_YOLO;
            if (ri < yolo_regions_copy.size()) {
              m.yolo_score = yolo_regions_copy[ri].yolo_confidence;
              m.class_id = yolo_regions_copy[ri].class_id;
            }
          }
          m.point_count = rout.point_count;
          m.roi_ms = static_cast<float>(rout.roi_ms);
          m.frame_total_ms = static_cast<float>(detect_total_ms3d);
          m.observed_at = imgMsg->header.stamp;
          merged3d.detections.push_back(m);
          // 単一detectionsのみ配信する方針に統一
        }
        // ===== per-ROI overlay drawing using fluent_text (simple lines) =====
        if (need_image) {
          // draw points/lines first
          if (pass) {
            cv::circle(annotated, rout.root_px, cut_point_radius_, cut_point_color_, cut_point_thickness_);
            cv::circle(annotated, rout.tip_px,  cut_point_radius_, cv::Scalar(0,255,255), cut_point_thickness_);
            cv::line(annotated, rout.root_px, rout.tip_px, cv::Scalar(255,255,0), 1, cv::LINE_AA);
          }
          cv::circle(annotated, rout.approach_px, approach_point_radius_, approach_point_color_, approach_point_thickness_);

          // multiline text as vector
          int yolo_pct = -1; int cls = -1;
          if (!is_fixed && ri < yolo_regions_copy.size()) {
            yolo_pct = static_cast<int>(std::round(yolo_regions_copy[ri].yolo_confidence * 100.0f));
            cls = yolo_regions_copy[ri].class_id;
          }
          std::vector<std::string> lines;
          char buf[256];
          lines.emplace_back(std::string("mode: ") + (is_fixed?"fixed":"object"));
          lines.emplace_back("target: yes");
          snprintf(buf, sizeof(buf), "region_id: %zu", ri); lines.emplace_back(buf);
          snprintf(buf, sizeof(buf), "rect(x,y,w,h): (%d,%d,%d,%d)", roi.x, roi.y, roi.width, roi.height); lines.emplace_back(buf);
          int cxp = roi.x + roi.width/2; int cyp = roi.y + roi.height/2; snprintf(buf, sizeof(buf), "center(u,v): (%d,%d)", cxp, cyp); lines.emplace_back(buf);
          if (yolo_pct >= 0) { snprintf(buf, sizeof(buf), "yolo(cls,score%%): (%d,%d)", cls, yolo_pct); lines.emplace_back(buf); }
          if (pass) {
            lines.emplace_back("3d(status): OK");
            double approach_cm = rout.approach_z_m * 100.0;
            double length_cm   = static_cast<double>(rout.length_m) * 100.0;
            int pca_pct = static_cast<int>(std::round(rout.pca_conf * 100.0f));
            snprintf(buf, sizeof(buf), "3d(pca%%, len_cm, z_cm): (%d, %.1f, %.1f)", pca_pct, length_cm, approach_cm); lines.emplace_back(buf);
            snprintf(buf, sizeof(buf), "points: %d", rout.point_count); lines.emplace_back(buf);
          } else {
            lines.emplace_back("3d(status): NG");
          }
          snprintf(buf, sizeof(buf), "roi_ms: %.1f", rout.roi_ms); lines.emplace_back(buf);

          // place right by default; if exceeds, place left
          int est_width = 200; // conservative width
          int x_text = roi.x + roi.width + 6;
          if (x_text + est_width > annotated.cols - 6) {
            x_text = std::max(6, roi.x - est_width - 6);
          }
          int y_text = std::clamp(roi.y + 18, 0, annotated.rows - 10);
          int line_h = static_cast<int>(std::round(22 * std::clamp(overlay_text_scale_, 0.4, 1.2)));
          for (const auto &s : lines) {
            // 日本語対応のシンプル描画（影なし）に変更
            fluent::text::draw(annotated, s, cv::Point(x_text, std::clamp(y_text, 0, annotated.rows-10)), label_text_color_, overlay_text_scale_, overlay_text_thickness_, 0);
            y_text += line_h;
          }
        }
        // collect per stage
        if ((allow_clouds || allow_merged) && pass_rate) {
          fv_stem_detector::msg::RegionCloud rc;
          rc.header = imgMsg->header; rc.region_id = static_cast<int>(ri);
          if (allow_clouds) {
            if (cloud_pub_raw_   && cloud_pub_raw_->get_subscription_count()>0 && rout.cloud_raw_zband)   { auto r0 = rc; pcl::toROSMsg(*rout.cloud_raw_zband, r0.cloud);   arr_raw.items.push_back(r0); }
            if (cloud_pub_voxel_ && cloud_pub_voxel_->get_subscription_count()>0 && rout.cloud_post_voxel) { auto r1 = rc; pcl::toROSMsg(*rout.cloud_post_voxel, r1.cloud); arr_voxel.items.push_back(r1); }
            if (cloud_pub_sor_   && cloud_pub_sor_->get_subscription_count()>0 && rout.cloud_post_sor)   { auto r2 = rc; pcl::toROSMsg(*rout.cloud_post_sor, r2.cloud);   arr_sor.items.push_back(r2); }
          }
          if (allow_merged) {
            auto append = [&](const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc, sensor_msgs::msg::PointCloud2 &bag){
              if (!pc) return; sensor_msgs::msg::PointCloud2 tmp; pcl::toROSMsg(*pc, tmp); bag = tmp; };
            if (cloud_publish_raw_zband_)   append(rout.cloud_raw_zband, merged_raw_bag_);
            if (cloud_publish_post_voxel_)  append(rout.cloud_post_voxel, merged_voxel_bag_);
            if (cloud_publish_post_sor_)    append(rout.cloud_post_sor, merged_sor_bag_);
          }
        }
      }
      if ((allow_clouds || allow_merged) && pass_rate) {
        if (allow_clouds) {
          if (cloud_pub_raw_ && !arr_raw.items.empty())   cloud_pub_raw_->publish(arr_raw);
          if (cloud_pub_voxel_ && !arr_voxel.items.empty()) cloud_pub_voxel_->publish(arr_voxel);
          if (cloud_pub_sor_ && !arr_sor.items.empty())   cloud_pub_sor_->publish(arr_sor);
        }
        if (allow_merged) {
          if (cloud_publish_raw_zband_ && merged_pub_raw_ && merged_pub_raw_->get_subscription_count()>0 && !merged_raw_bag_.data.empty()) {
            merged_raw_bag_.header = imgMsg->header; merged_pub_raw_->publish(merged_raw_bag_);
          }
          if (cloud_publish_post_voxel_ && merged_pub_voxel_ && merged_pub_voxel_->get_subscription_count()>0 && !merged_voxel_bag_.data.empty()) {
            merged_voxel_bag_.header = imgMsg->header; merged_pub_voxel_->publish(merged_voxel_bag_);
          }
          if (cloud_publish_post_sor_ && merged_pub_sor_ && merged_pub_sor_->get_subscription_count()>0 && !merged_sor_bag_.data.empty()) {
            merged_sor_bag_.header = imgMsg->header; merged_pub_sor_->publish(merged_sor_bag_);
          }
          merged_raw_bag_.data.clear(); merged_voxel_bag_.data.clear(); merged_sor_bag_.data.clear();
        }
        last_debug_pub_time_ = this->get_clock()->now();
      }

      if (need_image) {
        auto frame_t1 = std::chrono::high_resolution_clock::now();
        double total_ms = std::chrono::duration_cast<std::chrono::microseconds>(frame_t1 - frame_t0).count() / 1000.0;
        // FPS update and draw
        fps_frame_count_ += 1;
        auto now = std::chrono::steady_clock::now();
        auto diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fps_time_).count();
        if (diff_ms >= 1000) {
          fps_ = static_cast<double>(fps_frame_count_) * 1000.0 / static_cast<double>(diff_ms);
          fps_frame_count_ = 0;
          last_fps_time_ = now;
        }
        char head[96]; snprintf(head, sizeof(head), "処理: %.1f ms | FPS: %.1f", total_ms, fps_);
        fluent::text::drawShadow(annotated, std::string(head), cv::Point(12, 26), cv::Scalar(0,255,0), cv::Scalar(0,0,0), overlay_text_scale_, overlay_text_thickness_, 0);
        char sumbuf[96]; snprintf(sumbuf, sizeof(sumbuf), "検出合計: %.1f ms", detect_total_ms3d);
        fluent::text::drawShadow(annotated, std::string(sumbuf), cv::Point(12, 46), cv::Scalar(255,255,0), cv::Scalar(0,0,0), overlay_text_scale_, overlay_text_thickness_, 0);
        {
          std::lock_guard<std::mutex> lk(last_annotated_mutex_);
          last_annotated_image_ = annotated.clone();
          has_last_annotated_image_ = true;
        }
        cv_bridge::CvImage outImg; outImg.header = imgMsg->header; outImg.header.frame_id = publish_frame_id_; outImg.encoding = sensor_msgs::image_encodings::BGR8; outImg.image = annotated;
        annotated_pub_->publish(*outImg.toImageMsg());
        sensor_msgs::msg::CompressedImage cim;
        cim.header = outImg.header; cim.format = "jpeg";
        std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, 85};
        cv::imencode(".jpg", annotated, cim.data, params);
        annotated_compressed_pub_->publish(cim);
      }

      // 常時1件以上を保証（空なら固定/YOLOの先頭枠でスタブを追加）
      if (merged3d.detections.empty()) {
        if (!yolo_regions_copy.empty()) {
          const auto &yr = yolo_regions_copy.front();
          StemDetection m; m.region_id = 0; m.detected = false; m.source = StemDetection::SOURCE_YOLO;
          m.x = yr.x; m.y = yr.y; m.w = yr.w; m.h = yr.h; m.class_id = yr.class_id; m.yolo_score = yr.yolo_confidence;
          m.observed_at = imgMsg->header.stamp; merged3d.detections.push_back(m);
          RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 5000, "[force stub] yolo region#0 published (detected=false)");
        } else if (fixed_enable_ && !regions_.empty()) {
          // fixed.enable が有効な場合のみ fixed スタブ
          const auto &fr = regions_.front();
          StemDetection m; m.region_id = 0; m.detected = false; m.source = StemDetection::SOURCE_FIXED;
          m.x = fr.x; m.y = fr.y; m.w = fr.w; m.h = fr.h; m.class_id = -1; m.yolo_score = std::numeric_limits<float>::quiet_NaN();
          m.observed_at = imgMsg->header.stamp; merged3d.detections.push_back(m);
          RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 5000, "[force stub] fixed region#0 published (detected=false)");
        } else {
          RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 3000, "no regions available to publish stub detection");
        }
      }
      // 単一detectionsのみ出力（購読有無に関わらず必ず配信）
      if (detections_pub_) {
        detections_pub_->publish(merged3d);
      }
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 3000, "Unknown detection.method=%s", detection_method_.c_str());
    }
  }

  void onColorOnly(const Image::ConstSharedPtr imgMsg) {
    if (!use_sync_) return; // 非同期運用では使用しない
    // If no recent sync, publish a status overlay so operators see why no image
    const auto now_tp = std::chrono::steady_clock::now();
    long since_sync_ms = ever_synced_ ? std::chrono::duration_cast<std::chrono::milliseconds>(now_tp - last_sync_time_).count()
                                      : static_cast<long>(-1);
    if (ever_synced_ && since_sync_ms < keepalive_grace_ms_) {
      return; // recent synced frame already published
    }
    if (!annotated_pub_ || annotated_pub_->get_subscription_count() == 0) {
      return;
    }
    cv_bridge::CvImagePtr imgCv;
    try {
      imgCv = cv_bridge::toCvCopy(imgMsg, "bgr8");
    } catch (...) { return; }
    last_color_tp_ = std::chrono::steady_clock::now();

    // Snapshot YOLO regions
    std::vector<StemRegion> yolo_regions_copy;
    {
      std::lock_guard<std::mutex> lock(yolo_regions_mutex_);
      yolo_regions_copy = yolo_regions_;
    }

    cv::Mat annotated; imgCv->image.copyTo(annotated);
    for (const auto& r : regions_) {
      cv::rectangle(annotated, cv::Rect(r.x, r.y, r.w, r.h), fixed_rect_color_, rect_thickness_);
    }
    for (const auto& r : yolo_regions_copy) {
      cv::rectangle(annotated, cv::Rect(r.x, r.y, r.w, r.h), yolo_rect_color_, rect_thickness_);
    }
    // Status text（黒背景は使わず影付きテキストのみ） + ages
    auto now = std::chrono::steady_clock::now();
    long color_age = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_color_tp_).count();
    long depth_age = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_depth_tp_).count();
    std::string jp;
    if (!show_wait_banner_) {
      jp = "";
    } else {
      if (!ever_synced_) jp = "同期待ち: 初回同期中"; else jp = std::string("同期待ち: 深度が未受信 (") + std::to_string(since_sync_ms) + " ms)";
    }
    if (!intrinsics_ready_) jp += " | カメラ内参: 受信待ち";
    char diag[96]; snprintf(diag, sizeof(diag), "C:%ldms D:%ldms", color_age, depth_age);
    bool used_keep = false;
    {
      std::lock_guard<std::mutex> lk(last_annotated_mutex_);
      if (has_last_annotated_image_) {
        cv::Mat keep = last_annotated_image_.clone();
        fluent::text::drawShadow(keep, jp, cv::Point(12, 28), cv::Scalar(255,255,255), cv::Scalar(0,0,0), 0.8, 2, 0);
        cv_bridge::CvImage outImg; outImg.header = imgMsg->header; outImg.header.frame_id = publish_frame_id_; outImg.encoding = sensor_msgs::image_encodings::BGR8; outImg.image = keep;
        annotated_pub_->publish(*outImg.toImageMsg());
        used_keep = true;
      }
    }
    if (!used_keep) {
      if (!jp.empty()) fluent::text::drawShadow(annotated, jp, cv::Point(12, 28), cv::Scalar(255,255,255), cv::Scalar(0,0,0), 0.8, 2, 0);
      fluent::text::drawShadow(annotated, std::string(diag), cv::Point(12, 50), cv::Scalar(255,255,0), cv::Scalar(0,0,0), 0.7, 2, 0);
      cv_bridge::CvImage outImg; outImg.header = imgMsg->header; outImg.header.frame_id = publish_frame_id_; outImg.encoding = sensor_msgs::image_encodings::BGR8; outImg.image = annotated;
      annotated_pub_->publish(*outImg.toImageMsg());
    }
  }

  // ===== 非同期運用: 最新Depthで即時処理 =====
  void onDepthAsync(const Image::ConstSharedPtr depthMsg) {
    std::lock_guard<std::mutex> lk(latest_async_mutex_);
    latest_depth_ = depthMsg;
    last_depth_tp_ = std::chrono::steady_clock::now();
  }
  void onColorAsync(const Image::ConstSharedPtr imgMsg) {
    // yolo_triggerではColorは保存のみ
    {
      std::lock_guard<std::mutex> lk(latest_async_mutex_);
      latest_color_ = imgMsg;
    }
    last_color_tp_ = std::chrono::steady_clock::now();
    if (event_mode_ == "legacy_sync") {
      if (!annotated_pub_ || annotated_pub_->get_subscription_count() == 0) return;
      sensor_msgs::msg::Image::ConstSharedPtr depthMsg;
      {
        std::lock_guard<std::mutex> lk(latest_async_mutex_);
        depthMsg = latest_depth_;
      }
      if (!depthMsg) return; // depthまだ
      long depth_age_ms = 0; // 旧挙動では未使用
      if (depth_age_ms > max_depth_age_ms_) return;
      onImageDepth(imgMsg, depthMsg);
    }
  }

  // 動的パラメータ更新（fixed.rect / fixed.rect_id / fixed.save_yaml_path）
  rcl_interfaces::msg::SetParametersResult onParamSet(const std::vector<rclcpp::Parameter> &params) {
    rcl_interfaces::msg::SetParametersResult r; r.successful = true; r.reason = "ok";
    bool rect_changed = false; int64_t nx=0, ny=0, nw=0, nh=0; int nid = 0; bool have_rect=false; bool id_changed=false;
    for (const auto &p : params) {
      if (p.get_name() == "fixed.rect" && p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
        auto v = p.as_integer_array(); if (v.size() == 4) { nx=v[0]; ny=v[1]; nw=v[2]; nh=v[3]; have_rect=true; rect_changed=true; }
      } else if (p.get_name() == "fixed.rect_id" && p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        nid = static_cast<int>(p.as_int()); id_changed = true;
      } else if (p.get_name() == "fixed.save_yaml_path" && p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        // 文字列はそのままrclcpp管理なので特に何もしない
      }
    }
    if (rect_changed || id_changed) {
      if (rect_changed && have_rect) {
        regions_.clear(); StemRegion r0; r0.id = id_changed ? nid : 0; this->get_parameter("fixed.rect_id", r0.id);
        r0.x = static_cast<int>(nx); r0.y = static_cast<int>(ny); r0.w = static_cast<int>(nw); r0.h = static_cast<int>(nh);
        regions_.push_back(r0);
        RCLCPP_INFO(this->get_logger(), "param update fixed.rect -> id=%d rect=(%d,%d,%d,%d)", r0.id, r0.x, r0.y, r0.w, r0.h);
        // 自動保存（保存先が指定済みなら）：check_regions_flat の行のみ置換して他は保持
        std::string path; this->get_parameter("fixed.save_yaml_path", path);
        if (!path.empty()) {
          std::ifstream ifs(path);
          if (!ifs) {
            RCLCPP_WARN(this->get_logger(), "auto-save failed: cannot open %s", path.c_str());
          } else {
            std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
            ifs.close();
            // 1) 既存の check_regions_flat 行（複数行含む）を置換
            std::regex re_block("^(\\s*)check_regions_flat:\\s*\\[[\\s\\S]*?\\]", std::regex_constants::multiline);
            std::ostringstream oss_line;
            oss_line << "$1" << "check_regions_flat: [" << r0.id << ", " << r0.x << ", " << r0.y << ", " << r0.w << ", " << r0.h << "]";
            std::string replaced = std::regex_replace(content, re_block, oss_line.str());
            if (replaced == content) {
              // 2) 見つからなければ、ros__parameters: の直後に1行挿入
              const std::string key = "ros__parameters:";
              std::size_t p = content.find(key);
              std::string ins;
              {
                std::ostringstream oss;
                oss << "    check_regions_flat: [" << r0.id << ", " << r0.x << ", " << r0.y << ", " << r0.w << ", " << r0.h << "]\n";
                ins = oss.str();
              }
              if (p != std::string::npos) {
                std::size_t line_end = content.find('\n', p);
                if (line_end == std::string::npos) line_end = content.size(); else line_end += 1;
                replaced = content.substr(0, line_end) + ins + content.substr(line_end);
              } else {
                if (!content.empty() && content.back() != '\n') content.push_back('\n');
                replaced = content + ins;
              }
            }
            std::ofstream ofs(path, std::ios::out | std::ios::trunc);
            if (!ofs) {
              RCLCPP_WARN(this->get_logger(), "auto-save failed: cannot write %s", path.c_str());
            } else {
              ofs << replaced;
              ofs.close();
              RCLCPP_INFO(this->get_logger(), "auto-saved fixed rect to %s (check_regions_flat updated in-place)", path.c_str());
            }
          }
        }
      } else if (id_changed && !regions_.empty()) {
        regions_.front().id = nid;
        RCLCPP_INFO(this->get_logger(), "param update fixed.rect_id -> %d", nid);
      }
    }
    return r;
  }

private:
  // params
  std::string camera_topic_;
  std::string depth_topic_;
  std::string pointcloud_topic_;
  std::string camera_info_topic_;
  std::string depth_camera_info_topic_;
  std::string output_image_topic_;
  std::string output_detection_fixed_topic_;
  std::string output_detection_yolo_topic_;
  std::string output_detection_topic_;
  std::string detection_topic_;
  std::string publish_frame_id_;
  double publish_rate_ {10.0};
  std::string detection_method_ {"pointcloud_linefit"};
  std::string source_mode_ {"object"};
  bool center3d_fallback_enable_ {false};
  std::string event_mode_ {"yolo_trigger"};
  bool event_skip_if_busy_ {true};
  bool debug_roi_log_ {false};
  int id_dynamic_start_ {1};
  // 固定情報配信の有効/無効
  bool fixed_enable_ {false};
  double depth_scale_m_ {0.001};
  double overlay_text_scale_ {0.5};
  int overlay_text_thickness_ {1};
  bool always_publish_image_ {true};

  cv::Scalar fixed_rect_color_ {0,0,255};
  cv::Scalar yolo_rect_color_ {255,0,0};
  int rect_thickness_ {2};
  cv::Scalar cut_point_color_ {0,0,255};
  int cut_point_radius_ {6};
  int cut_point_thickness_ {2};
  bool show_region_labels_ {true};
  cv::Scalar label_text_color_ {255,255,255};
  cv::Scalar approach_point_color_ {255,0,255};
  int approach_point_radius_ {5};
  int approach_point_thickness_ {2};

  // qos
  rclcpp::QoS input_qos_{rclcpp::KeepLast(10)};
  rclcpp::QoS output_qos_{rclcpp::KeepLast(10)};

  // subs/pubs
  image_transport::SubscriberFilter image_sub_;
  image_transport::SubscriberFilter depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<Image, Image>>> sync_;

  rclcpp::Publisher<Image>::SharedPtr annotated_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr annotated_compressed_pub_;
  rclcpp::Publisher<StemDetectionArray>::SharedPtr detections_pub_;
  // ROI cloud publishers (stages)
  rclcpp::Publisher<fv_stem_detector::msg::RegionCloudArray>::SharedPtr cloud_pub_raw_;
  rclcpp::Publisher<fv_stem_detector::msg::RegionCloudArray>::SharedPtr cloud_pub_voxel_;
  rclcpp::Publisher<fv_stem_detector::msg::RegionCloudArray>::SharedPtr cloud_pub_sor_;
  // Foxglove用: マージしたPointCloud2
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_pub_raw_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_pub_voxel_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_pub_sor_;
  rclcpp::Subscription<Detection2DArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_depth_sub_;
  rclcpp::Subscription<Image>::SharedPtr color_only_sub_;
  rclcpp::Service<fv_stem_detector::srv::SetSourceMode>::SharedPtr set_source_mode_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr save_fixed_yaml_srv_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  // async subscriptions
  rclcpp::Subscription<Image>::SharedPtr image_async_sub_;
  rclcpp::Subscription<Image>::SharedPtr depth_async_sub_;
  std::mutex latest_async_mutex_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_color_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_depth_;
  std::atomic<bool> processing_busy_{false};

  // logic
  fv_stem_detector::StemDetector detector_;
  fv_stem_detector::Aspara3DPipeline detector3d_;
  std::vector<StemRegion> regions_;
  std::vector<StemRegion> yolo_regions_;
  std::mutex yolo_regions_mutex_;
  // YOLO hold buffer
  std::vector<StemRegion> yolo_regions_hold_;
  std::chrono::steady_clock::time_point yolo_last_update_ { std::chrono::steady_clock::now() };
  int yolo_hold_ms_ {300};
  cv::Mat last_annotated_image_;
  bool has_last_annotated_image_ {false};
  std::mutex last_annotated_mutex_;

  // YOLO ID trackingは当面無効化（簡素化）

  // intrinsics
  double fx_ {0.0}, fy_ {0.0}, cx_ {0.0}, cy_ {0.0};
  bool intrinsics_ready_ {false};
  std::chrono::steady_clock::time_point last_sync_time_ { std::chrono::steady_clock::now() };
  std::chrono::steady_clock::time_point last_color_tp_ { std::chrono::steady_clock::now() };
  std::chrono::steady_clock::time_point last_depth_tp_ { std::chrono::steady_clock::now() };
  bool ever_synced_ {false};

  // params for pointcloud pipeline
  double depth_band_minus_m_ {0.05};
  double depth_band_plus_m_ {0.05};
  double ground_cut_auto_max_p90_offset_m_ {0.005};
  double voxel_leaf_m_ {0.004};
  int sor_mean_k_ {50};
  double sor_stddev_mul_ {1.2};
  double min_length_m_ {0.10};
  double max_length_m_ {0.40};
  double pca_min_confidence_ {0.60};
  // yolo filters and fixed regions
  double yolo_min_confidence_ {0.20};
  int yolo_min_bbox_w_ {1};
  int yolo_min_bbox_h_ {1};
  // fixed.enable は廃止（source.mode で制御）
  // deprecated: fixed 2D fallback is removed; always use 3D pipeline
  bool fixed_publish_ {true};
  int pc_sample_stride_px_ {2};

  // approach window params
  int approach_window_px_ {5};
  int approach_min_valid_ {5};
  int keepalive_grace_ms_ {100};
  int max_depth_age_ms_ {200};
  bool use_sync_ {false};
  bool show_wait_banner_ {false};
  // ROI cloud options
  bool debug_publish_roi_clouds_ {false};
  double debug_throttle_hz_ {2.0};
  std::string cloud_topic_raw_;
  std::string cloud_topic_voxel_;
  std::string cloud_topic_sor_;
  bool cloud_publish_raw_zband_ {false};
  bool cloud_publish_post_voxel_ {false};
  bool cloud_publish_post_sor_ {false};
  bool cloud_publish_failed_rois_ {false};
  std::string merged_topic_raw_;
  std::string merged_topic_voxel_;
  std::string merged_topic_sor_;
  sensor_msgs::msg::PointCloud2 merged_raw_bag_;
  sensor_msgs::msg::PointCloud2 merged_voxel_bag_;
  sensor_msgs::msg::PointCloud2 merged_sor_bag_;
  rclcpp::Time last_debug_pub_time_ {0,0, RCL_ROS_TIME};

  // FPS tracking
  int fps_frame_count_ {0};
  double fps_ {0.0};
  std::chrono::steady_clock::time_point last_fps_time_ { std::chrono::steady_clock::now() };

  // ===== ID assignment helpers =====
  std::vector<int> assignIdsForFixed(const std::vector<StemRegion> &regs) const {
    // 固定枠は仕様によりID=0を表示（複数枠でも同一ID）
    std::vector<int> ids; ids.reserve(regs.size());
    for (size_t i=0; i<regs.size(); ++i) ids.push_back(0);
    return ids;
  }
  std::vector<int> assignIdsForYolo(const std::vector<StemRegion> &regs) {
    // IDトラッキングは無効化。フレーム内連番のみ返す。
    std::vector<int> ids; ids.reserve(regs.size());
    for (size_t i=0;i<regs.size();++i) ids.push_back(static_cast<int>(i));
    return ids;
  }

  void fill3DIfPossible(const StemDetectionResult& d, const cv::Mat& depth16u, StemDetection& out) {
    // Fill camera-frame coordinates in meters if intrinsics and valid depth are available
    out.camera_x = std::numeric_limits<float>::quiet_NaN();
    out.camera_y = std::numeric_limits<float>::quiet_NaN();
    out.camera_z = std::numeric_limits<float>::quiet_NaN();
    if (!d.detected) return;
    if (!intrinsics_ready_) return;
    if (depth16u.empty() || depth16u.type() != CV_16UC1) return;
    int u = std::clamp(d.x, 0, depth16u.cols - 1);
    int v = std::clamp(d.y, 0, depth16u.rows - 1);
    uint16_t raw = depth16u.at<uint16_t>(v, u);
    if (raw == 0) return; // invalid depth
    float Z = static_cast<float>(raw) * static_cast<float>(depth_scale_m_);
    float X = static_cast<float>((static_cast<double>(u) - cx_) / fx_) * Z;
    float Y = static_cast<float>((static_cast<double>(v) - cy_) / fy_) * Z;
    out.camera_x = X; out.camera_y = Y; out.camera_z = Z;
  }
};

}  // namespace fv_stem_detector_node

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fv_stem_detector_node::StemDetectorNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


