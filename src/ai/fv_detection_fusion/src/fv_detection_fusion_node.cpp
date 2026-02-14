#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <fv_msgs/msg/detection_array.hpp>
#include <std_msgs/msg/header.hpp>
// 画像オーバーレイ用
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <fluent_lib/cv_bridge_compat.hpp>
// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
// 日本語対応の影付きテキスト描画
#include "fluent_text.hpp"
// Services
#include <std_srvs/srv/set_bool.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace fv_detection_fusion {

class DetectionFusionNode : public rclcpp::Node {
public:
  explicit DetectionFusionNode(const rclcpp::NodeOptions &options)
      : rclcpp::Node("fv_detection_fusion_node", options) {
    declareParameters();
    readParameters();

    output_pub_ = this->create_publisher<fv_msgs::msg::DetectionArray>(output_topic_, rclcpp::QoS(output_qos_depth_));

    // 画像オーバーレイ出力
    if (debug_enable_overlay_) {
      overlay_pub_ = this->create_publisher<sensor_msgs::msg::Image>(overlay_topic_, rclcpp::QoS(1));
    }

    for (const auto &source : sources_) {
      rclcpp::QoS qos(source.qos_depth);
      if (source.qos_reliability == "reliable") {
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
      } else {
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
      }
      auto sub = this->create_subscription<vision_msgs::msg::Detection2DArray>(
          source.topic, qos,
          [this, cfg = source](const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
            handleDetections(cfg, msg);
          });
      source_subs_.push_back(sub);
    }

    // 入力画像の購読（オーバーレイ用）
    if (debug_enable_overlay_ && !input_image_topic_.empty()) {
      image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
          input_image_topic_, rclcpp::QoS(1).best_effort(),
          std::bind(&DetectionFusionNode::handleImage, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "Subscribed image: %s", input_image_topic_.c_str());
    }

    // マスク購読
    if (!instance_mask_topic_.empty()) {
      inst_mask_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
          instance_mask_topic_, rclcpp::QoS(1).best_effort(),
          std::bind(&DetectionFusionNode::handleInstMask, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "Subscribed instance mask: %s", instance_mask_topic_.c_str());
    }
    if (!unet_mask_topic_.empty()) {
      unet_mask_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
          unet_mask_topic_, rclcpp::QoS(1).best_effort(),
          std::bind(&DetectionFusionNode::handleUnetMask, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "Subscribed UNet mask: %s", unet_mask_topic_.c_str());
    }

    // ROIマスク出力
    roi_mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>(roi_mask_topic_, rclcpp::QoS(1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(diagnostic_interval_ms_)),
        std::bind(&DetectionFusionNode::publishFusion, this));

    // Services: 無効領域 ON/OFF, 固定モード切替
    srv_invalid_area_ = this->create_service<std_srvs::srv::SetBool>(
        "set_invalid_area_enabled",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
               std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
          invalid_area_enabled_ = req->data;
          res->success = true;
          res->message = std::string("invalid_area.enabled=") + (invalid_area_enabled_?"true":"false");
        });
    srv_fixed_mode_ = this->create_service<std_srvs::srv::SetBool>(
        "set_fixed_mode",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
               std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
          fixed_mode_enabled_ = req->data;
          res->success = true;
          res->message = std::string("mode.fixed_enabled=") + (fixed_mode_enabled_?"true":"false");
        });

    RCLCPP_INFO(get_logger(), "fv_detection_fusion_node ready (sources=%zu)", sources_.size());
  }

private:
  struct SourceConfig {
    std::string label;
    std::string topic;
    float confidence_multiplier{1.0f};
    float min_confidence{0.0f};
    int qos_depth{5};
    std::string qos_reliability{"best_effort"}; // best_effort | reliable
  };

  struct DetRecord {
    fv_msgs::msg::Detection2D detection;
    rclcpp::Time last_update{0};
    int hold_frames{0};
    rclcpp::Time first_seen{0};
    int seen_frames{0};
  };

  struct Box {
    float x{0}, y{0}, w{0}, h{0};
  };

  static float y_iou(const Box &a, const Box &b) {
    const float ay1 = a.y;
    const float ay2 = a.y + a.h;
    const float by1 = b.y;
    const float by2 = b.y + b.h;
    const float iy1 = std::max(ay1, by1);
    const float iy2 = std::min(ay2, by2);
    const float ih = std::max(0.0f, iy2 - iy1);
    const float ua = std::max(0.0f, a.h);
    const float ub = std::max(0.0f, b.h);
    const float uni_h = ua + ub - ih + 1e-6f;
    return ih / uni_h;
  }

  static Box toBox(const vision_msgs::msg::Detection2D &det) {
    Box b;
    b.x = static_cast<float>(det.bbox.center.position.x - det.bbox.size_x * 0.5);
    b.y = static_cast<float>(det.bbox.center.position.y - det.bbox.size_y * 0.5);
    b.w = static_cast<float>(det.bbox.size_x);
    b.h = static_cast<float>(det.bbox.size_y);
    return b;
  }

  static float iou(const Box &a, const Box &b) {
    const float ax2 = a.x + a.w;
    const float ay2 = a.y + a.h;
    const float bx2 = b.x + b.w;
    const float by2 = b.y + b.h;
    const float ix1 = std::max(a.x, b.x);
    const float iy1 = std::max(a.y, b.y);
    const float ix2 = std::min(ax2, bx2);
    const float iy2 = std::min(ay2, by2);
    const float iw = std::max(0.0f, ix2 - ix1);
    const float ih = std::max(0.0f, iy2 - iy1);
    const float inter = iw * ih;
    const float area_a = std::max(0.0f, a.w) * std::max(0.0f, a.h);
    const float area_b = std::max(0.0f, b.w) * std::max(0.0f, b.h);
    const float uni = area_a + area_b - inter + 1e-6f;
    return inter / uni;
  }

  template <typename T>
  void declareIfMissing(const std::string &name, const T &default_value) {
    if (!this->has_parameter(name)) {
      this->declare_parameter<T>(name, default_value);
    }
  }

  void declareParameters() {
    declareIfMissing<std::string>("output_topic", "~/rois");
    declareIfMissing<int>("output_qos_depth", 5);
    declareIfMissing<double>("diagnostic_interval_ms", 100.0);
    declareIfMissing<int>("hold_frames", 5);
    declareIfMissing<std::string>("frame_id", "");

    // 画像/デバッグ
    declareIfMissing<std::string>("input_image_topic", std::string(""));
    declareIfMissing<bool>("debug.enable_overlay", true);
    declareIfMissing<std::string>("debug.overlay_topic", std::string("overlay"));
    declareIfMissing<bool>("debug.draw_history", false);
    declareIfMissing<std::string>("debug.roi_mask_topic", std::string("roi_mask"));
    declareIfMissing<bool>("debug.overlay_tint_mask", true);
    declareIfMissing<bool>("debug.overlay.instance_fullmask", true);
    declareIfMissing<bool>("debug.overlay.unet_fullmask", false);
    declareIfMissing<double>("debug.overlay.alpha_inst", 0.4);
    declareIfMissing<double>("debug.overlay.alpha_unet", 0.3);
    // 元検出（object/instance）の矩形に寸法等を描画するオプション
    declareIfMissing<bool>("debug.show_src_dims", false);
    declareIfMissing<bool>("debug.src_dims.show_filter_eval", true);
    declareIfMissing<double>("debug.src_dims.font_scale", 0.4);
    declareIfMissing<int>("debug.src_dims.thickness", 1);
    // 元検出の枠の描画制御と線の太さ
    declareIfMissing<bool>("debug.draw_src.object", true);
    declareIfMissing<bool>("debug.draw_src.instance", true);
    declareIfMissing<int>("debug.src_box.thickness", 1);

    // 融合パラメータ
    declareIfMissing<double>("fusion.iou_threshold", 0.30);
    declareIfMissing<double>("fusion.nms_iou_threshold", 0.50);
    declareIfMissing<double>("fusion.nms_contain_ratio", 0.85);
    declareIfMissing<bool>("fusion.nms_keep_contained_cross_source", true);
    // 比較軸の選択: iou | y_iou
    declareIfMissing<std::string>("fusion.match_metric", std::string("iou"));
    declareIfMissing<std::string>("fusion.nms_metric", std::string("iou"));
    declareIfMissing<int>("fusion.center_x_max_offset_px", 0); // 0で無効
    // 隣接個体の潰れ抑止: NMS時の横方向ゲート
    declareIfMissing<double>("fusion.nms_min_x_overlap_ratio", 0.15); // inter_w / min(w)
    declareIfMissing<int>("fusion.nms_center_x_max_offset_px", 0);    // 0で無効
    declareIfMissing<std::string>("fusion.nms_cross_contained_policy", std::string("prefer_instance"));
    declareIfMissing<double>("fusion.track_match_iou", 0.50);
    
    // マスク購読・後処理
    declareIfMissing<std::string>("masks.instance_mask_topic", std::string(""));
    declareIfMissing<std::string>("masks.unet_mask_topic", std::string(""));
    declareIfMissing<double>("masks.max_stamp_diff_ms", 120.0);
    declareIfMissing<std::string>("mask.prefer", std::string("instance_then_unet"));
    declareIfMissing<std::string>("mask.combine_mode", std::string("prefer"));
    declareIfMissing<int>("roi.pad_px", 6);
    declareIfMissing<int>("roi.min_area_px", 300);
    declareIfMissing<double>("roi.min_aspect_ratio", 0.1);
    declareIfMissing<double>("roi.max_aspect_ratio", 1.2);
    declareIfMissing<int>("mask.postprocess.binarize_threshold", 128);
    declareIfMissing<int>("mask.postprocess.morph.open_kernel", 3);
    declareIfMissing<int>("mask.postprocess.morph.open_iter", 1);
    declareIfMissing<int>("mask.postprocess.morph.close_kernel", 5);
    declareIfMissing<int>("mask.postprocess.morph.close_iter", 1);
    declareIfMissing<bool>("mask.postprocess.keep_largest", true);
    declareIfMissing<bool>("mask.postprocess.fill_holes", true);
    declareIfMissing<int>("mask.postprocess.max_hole_area_px", 2000);
    declareIfMissing<double>("fusion.weights.w_object", 0.6);
    declareIfMissing<double>("fusion.weights.w_instance", 0.4);
    declareIfMissing<double>("fusion.weights.bonus_overlap", 0.05);
    declareIfMissing<std::string>("fusion.prefer_bbox", std::string("yolov10"));

    // モード/無効領域/フィルタ
    declareIfMissing<bool>("mode.fixed_enabled", false);
    declareIfMissing<bool>("invalid_area.enabled", true);
    declareIfMissing<bool>("invalid_area.any_overlap", true);        // true: 少しでも重なれば無効
    declareIfMissing<bool>("invalid_area.touch_inclusive", true);    // true: 辺が接触(面積0)でも無効
    declareIfMissing<double>("invalid_area.box.center_x_ratio", 0.5);
    declareIfMissing<double>("invalid_area.box.y_top_ratio", 0.5);
    declareIfMissing<double>("invalid_area.box.width_ratio", 0.5);
    declareIfMissing<double>("invalid_area.box.height_ratio", 0.5);
    declareIfMissing<bool>("debug.draw_invalid_area", true);

    declareIfMissing<double>("fixed_roi.box.x_center_ratio", 0.5);
    declareIfMissing<int>("fixed_roi.box.y_top_px", 5);
    declareIfMissing<double>("fixed_roi.box.width_ratio", 0.20);
    declareIfMissing<double>("fixed_roi.box.height_ratio", 0.50);

    declareIfMissing<double>("detection.filter.min_aspect_ratio", 0.05);
    declareIfMissing<double>("detection.filter.max_aspect_ratio", 2.0);
    declareIfMissing<int>("detection.filter.min_w_px", 8);
    declareIfMissing<int>("detection.filter.min_h_px", 20);
    declareIfMissing<int>("detection.filter.max_w_px", 8192);
    declareIfMissing<int>("detection.filter.max_h_px", 8192);
  }

  void readParameters() {
    output_topic_ = this->get_parameter("output_topic").as_string();
    output_qos_depth_ = this->get_parameter("output_qos_depth").as_int();
    diagnostic_interval_ms_ = this->get_parameter("diagnostic_interval_ms").as_double();
    hold_frames_ = this->get_parameter("hold_frames").as_int();
    frame_id_override_ = this->get_parameter("frame_id").as_string();

    input_image_topic_ = this->get_parameter("input_image_topic").as_string();
    debug_enable_overlay_ = this->get_parameter("debug.enable_overlay").as_bool();
    overlay_topic_ = this->get_parameter("debug.overlay_topic").as_string();
    debug_draw_history_ = this->get_parameter("debug.draw_history").as_bool();
    roi_mask_topic_ = this->get_parameter("debug.roi_mask_topic").as_string();
    debug_overlay_tint_mask_ = this->get_parameter("debug.overlay_tint_mask").as_bool();
    overlay_inst_full_ = this->get_parameter("debug.overlay.instance_fullmask").as_bool();
    overlay_unet_full_ = this->get_parameter("debug.overlay.unet_fullmask").as_bool();
    overlay_alpha_inst_ = this->get_parameter("debug.overlay.alpha_inst").as_double();
    overlay_alpha_unet_ = this->get_parameter("debug.overlay.alpha_unet").as_double();
    // label params（既定はdeclareParametersで宣言済み、ここでは取得のみ）
    label_enable_ = this->get_parameter("debug.label.enable").as_bool();
    label_font_scale_ = this->get_parameter("debug.label.font_scale").as_double();
    label_thickness_ = this->get_parameter("debug.label.thickness").as_int();
    // 元検出の寸法描画
    debug_show_src_dims_ = this->get_parameter("debug.show_src_dims").as_bool();
    debug_src_show_filter_ = this->get_parameter("debug.src_dims.show_filter_eval").as_bool();
    src_dims_font_scale_ = this->get_parameter("debug.src_dims.font_scale").as_double();
    src_dims_thickness_ = this->get_parameter("debug.src_dims.thickness").as_int();
    // 元検出の枠描画
    draw_src_obj_boxes_ = this->get_parameter("debug.draw_src.object").as_bool();
    draw_src_inst_boxes_ = this->get_parameter("debug.draw_src.instance").as_bool();
    src_box_thickness_ = this->get_parameter("debug.src_box.thickness").as_int();

    iou_th_ = this->get_parameter("fusion.iou_threshold").as_double();
    nms_iou_th_ = this->get_parameter("fusion.nms_iou_threshold").as_double();
    nms_contain_ratio_ = this->get_parameter("fusion.nms_contain_ratio").as_double();
    nms_keep_contained_cross_source_ = this->get_parameter("fusion.nms_keep_contained_cross_source").as_bool();
    match_metric_ = this->get_parameter("fusion.match_metric").as_string();
    nms_metric_ = this->get_parameter("fusion.nms_metric").as_string();
    center_x_max_offset_px_ = this->get_parameter("fusion.center_x_max_offset_px").as_int();
    nms_min_x_overlap_ratio_ = this->get_parameter("fusion.nms_min_x_overlap_ratio").as_double();
    nms_center_x_max_offset_px_ = this->get_parameter("fusion.nms_center_x_max_offset_px").as_int();
    nms_cross_contained_policy_ = this->get_parameter("fusion.nms_cross_contained_policy").as_string();
    track_match_iou_th_ = this->get_parameter("fusion.track_match_iou").as_double();
    w_object_ = this->get_parameter("fusion.weights.w_object").as_double();
    w_instance_ = this->get_parameter("fusion.weights.w_instance").as_double();
    bonus_overlap_ = this->get_parameter("fusion.weights.bonus_overlap").as_double();
    prefer_bbox_ = this->get_parameter("fusion.prefer_bbox").as_string();

    // モード/無効領域/フィルタ
    fixed_mode_enabled_ = this->get_parameter("mode.fixed_enabled").as_bool();
    invalid_area_enabled_ = this->get_parameter("invalid_area.enabled").as_bool();
    invalid_any_overlap_ = this->get_parameter("invalid_area.any_overlap").as_bool();
    invalid_touch_inclusive_ = this->get_parameter("invalid_area.touch_inclusive").as_bool();
    draw_invalid_area_ = this->get_parameter("debug.draw_invalid_area").as_bool();
    inv_cx_r_ = this->get_parameter("invalid_area.box.center_x_ratio").as_double();
    inv_y_top_r_ = this->get_parameter("invalid_area.box.y_top_ratio").as_double();
    inv_w_r_ = this->get_parameter("invalid_area.box.width_ratio").as_double();
    inv_h_r_ = this->get_parameter("invalid_area.box.height_ratio").as_double();

    fixed_cx_r_ = this->get_parameter("fixed_roi.box.x_center_ratio").as_double();
    fixed_y_top_px_ = this->get_parameter("fixed_roi.box.y_top_px").as_int();
    fixed_w_r_ = this->get_parameter("fixed_roi.box.width_ratio").as_double();
    fixed_h_r_ = this->get_parameter("fixed_roi.box.height_ratio").as_double();

    filt_min_ar_ = this->get_parameter("detection.filter.min_aspect_ratio").as_double();
    filt_max_ar_ = this->get_parameter("detection.filter.max_aspect_ratio").as_double();
    filt_min_w_ = this->get_parameter("detection.filter.min_w_px").as_int();
    filt_min_h_ = this->get_parameter("detection.filter.min_h_px").as_int();
    filt_max_w_ = this->get_parameter("detection.filter.max_w_px").as_int();
    filt_max_h_ = this->get_parameter("detection.filter.max_h_px").as_int();

    instance_mask_topic_ = this->get_parameter("masks.instance_mask_topic").as_string();
    unet_mask_topic_ = this->get_parameter("masks.unet_mask_topic").as_string();
    max_stamp_diff_ms_ = this->get_parameter("masks.max_stamp_diff_ms").as_double();
    mask_prefer_ = this->get_parameter("mask.prefer").as_string();
    mask_combine_mode_ = this->get_parameter("mask.combine_mode").as_string();
    roi_pad_px_ = this->get_parameter("roi.pad_px").as_int();
    roi_min_area_px_ = this->get_parameter("roi.min_area_px").as_int();
    roi_min_aspect_ = this->get_parameter("roi.min_aspect_ratio").as_double();
    roi_max_aspect_ = this->get_parameter("roi.max_aspect_ratio").as_double();
    bin_thresh_ = this->get_parameter("mask.postprocess.binarize_threshold").as_int();
    open_k_ = this->get_parameter("mask.postprocess.morph.open_kernel").as_int();
    open_iter_ = this->get_parameter("mask.postprocess.morph.open_iter").as_int();
    close_k_ = this->get_parameter("mask.postprocess.morph.close_kernel").as_int();
    close_iter_ = this->get_parameter("mask.postprocess.morph.close_iter").as_int();
    keep_largest_ = this->get_parameter("mask.postprocess.keep_largest").as_bool();
    fill_holes_ = this->get_parameter("mask.postprocess.fill_holes").as_bool();
    max_hole_area_px_ = this->get_parameter("mask.postprocess.max_hole_area_px").as_int();

    auto param_list = this->list_parameters({"sources"}, 10);
    std::unordered_map<std::string, SourceConfig> configs;
    for (const auto &name : param_list.names) {
      if (name.rfind("sources.", 0) != 0) {
        continue;
      }
      const std::string without_prefix = name.substr(std::string("sources.").size());
      const auto dot = without_prefix.find('.');
      if (dot == std::string::npos) {
        continue;
      }
      const std::string key = without_prefix.substr(0, dot);
      const std::string param = without_prefix.substr(dot + 1);
      SourceConfig &cfg = configs[key];
      if (param == "label") {
        cfg.label = this->get_parameter(name).as_string();
      } else if (param == "topic") {
        cfg.topic = this->get_parameter(name).as_string();
      } else if (param == "confidence_multiplier") {
        cfg.confidence_multiplier = static_cast<float>(this->get_parameter(name).as_double());
      } else if (param == "min_confidence") {
        cfg.min_confidence = static_cast<float>(this->get_parameter(name).as_double());
      } else if (param == "qos_depth") {
        cfg.qos_depth = this->get_parameter(name).as_int();
      } else if (param == "qos_reliability" || param == "reliability") {
        cfg.qos_reliability = this->get_parameter(name).as_string();
      }
    }

    for (auto &entry : configs) {
      auto &cfg = entry.second;
      if (cfg.topic.empty()) {
        RCLCPP_WARN(get_logger(), "sources.%s.topic missing, skipping", entry.first.c_str());
        continue;
      }
      if (cfg.label.empty()) {
        cfg.label = entry.first;
      }
      sources_.push_back(cfg);
    }

    if (sources_.empty()) {
      RCLCPP_WARN(get_logger(), "No detection sources configured; fusion will output empty arrays");
    }
  }

  void handleDetections(const SourceConfig &cfg, const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_by_label_[cfg.label] = msg;
    // ここでは配列をキャッシュするだけ。融合はタイマーで実施。
  }

  void handleImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      std::lock_guard<std::mutex> lock(data_mutex_);
      last_image_ = cv_ptr->image.clone();
      last_image_header_ = msg->header;
      have_image_ = true;
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "cv_bridge failed: %s", e.what());
    }
  }

  void handleInstMask(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
      std::lock_guard<std::mutex> lock(data_mutex_);
      last_inst_mask_ = cv_ptr->image.clone();
      last_inst_mask_header_ = msg->header;
      have_inst_mask_ = true;
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "inst mask cv_bridge failed: %s", e.what());
    }
  }

  void handleUnetMask(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
      std::lock_guard<std::mutex> lock(data_mutex_);
      last_unet_mask_ = cv_ptr->image.clone();
      last_unet_mask_header_ = msg->header;
      have_unet_mask_ = true;
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "unet mask cv_bridge failed: %s", e.what());
    }
  }

  void publishFusion() {
    if (!output_pub_) {
      return;
    }

    fv_msgs::msg::DetectionArray out;
    out.header.stamp = this->now();
    out.header.frame_id = frame_id_override_;

    // 検出配列を取得
    std::shared_ptr<vision_msgs::msg::Detection2DArray> obj_msg;
    std::shared_ptr<vision_msgs::msg::Detection2DArray> inst_msg;
    sensor_msgs::msg::Image::SharedPtr dummy; // not used
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      auto it_obj = latest_by_label_.find(object_label_);
      if (it_obj != latest_by_label_.end()) obj_msg = it_obj->second;
      auto it_inst = latest_by_label_.find(instance_label_);
      if (it_inst != latest_by_label_.end()) inst_msg = it_inst->second;
    }

    // フレームごとの一時融合リスト
    struct TmpDet { vision_msgs::msg::Detection2D det; float score; };
    std::vector<TmpDet> objs, insts;
    // 画像サイズ（無効領域/固定枠算出用）
    int img_w = 0, img_h = 0;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!last_image_.empty()) { img_w = last_image_.cols; img_h = last_image_.rows; }
      else if (!last_unet_mask_.empty()) { img_w = last_unet_mask_.cols; img_h = last_unet_mask_.rows; }
      else if (!last_inst_mask_.empty()) { img_w = last_inst_mask_.cols; img_h = last_inst_mask_.rows; }
    }

    auto make_invalid_rect = [&](int W, int H){
      if (W<=0 || H<=0) return cv::Rect();
      int iw = std::max(1, static_cast<int>(std::round(inv_w_r_ * W)));
      int ih = std::max(1, static_cast<int>(std::round(inv_h_r_ * H)));
      int cx = static_cast<int>(std::round(inv_cx_r_ * W));
      int x = std::clamp(cx - iw/2, 0, std::max(0, W-1));
      int y = std::clamp(static_cast<int>(std::round(inv_y_top_r_ * H)), 0, std::max(0, H-1));
      if (x+iw>W) iw = W-x; if (y+ih>H) ih = H-y;
      return cv::Rect(x,y, std::max(0,iw), std::max(0,ih));
    };
    auto make_fixed_rect = [&](int W, int H){
      if (W<=0 || H<=0) return cv::Rect();
      int fw = std::max(1, static_cast<int>(std::round(fixed_w_r_ * W)));
      int fh = std::max(1, static_cast<int>(std::round(fixed_h_r_ * H)));
      int cx = static_cast<int>(std::round(fixed_cx_r_ * W));
      int x = std::clamp(cx - fw/2, 0, std::max(0, W-1));
      int y = std::clamp(fixed_y_top_px_, 0, std::max(0, H-1));
      if (x+fw>W) fw = W-x; if (y+fh>H) fh = H-y;
      return cv::Rect(x,y, std::max(0,fw), std::max(0,fh));
    };
    const cv::Rect invalid_rect = make_invalid_rect(img_w, img_h);
    auto rect_overlap = [&](const cv::Rect &a, const cv::Rect &b){
      int x1 = std::max(a.x, b.x);
      int y1 = std::max(a.y, b.y);
      int x2 = std::min(a.x + a.width,  b.x + b.width);
      int y2 = std::min(a.y + a.height, b.y + b.height);
      if (invalid_touch_inclusive_) return (x2 >= x1) && (y2 >= y1);
      return (x2 > x1) && (y2 > y1);
    };
    auto det_to_rect = [&](const vision_msgs::msg::Detection2D &d){
      int x = static_cast<int>(std::round(d.bbox.center.position.x - d.bbox.size_x * 0.5));
      int y = static_cast<int>(std::round(d.bbox.center.position.y - d.bbox.size_y * 0.5));
      int w = static_cast<int>(std::round(d.bbox.size_x));
      int h = static_cast<int>(std::round(d.bbox.size_y));
      x = std::max(0, std::min(x, std::max(0, img_w-1)));
      y = std::max(0, std::min(y, std::max(0, img_h-1)));
      if (x + w > img_w) w = img_w - x; if (y + h > img_h) h = img_h - y;
      return cv::Rect(x,y, std::max(0,w), std::max(0,h));
    };

    if (obj_msg) {
      for (auto &d : obj_msg->detections) {
        // サイズ/アスペクト比フィルタ
        const double w = d.bbox.size_x;
        const double h = d.bbox.size_y;
        if (!(w >= filt_min_w_ && h >= filt_min_h_ && w <= filt_max_w_ && h <= filt_max_h_)) continue;
        const double ar = (h <= 1e-6) ? 0.0 : (w / h);
        if (ar < filt_min_ar_ || ar > filt_max_ar_) continue;
        // 無効領域（検出モード時）
        if (!fixed_mode_enabled_ && invalid_area_enabled_ && img_w>0 && img_h>0) {
          bool hit = false;
          if (invalid_any_overlap_) hit = rect_overlap(det_to_rect(d), invalid_rect);
          else {
            int cx = static_cast<int>(std::round(d.bbox.center.position.x));
            int cy = static_cast<int>(std::round(d.bbox.center.position.y));
            hit = invalid_rect.contains(cv::Point(cx, cy));
          }
          if (hit) continue; // 反応しない
        }
        float s = d.results.empty() ? 1.0f : static_cast<float>(d.results.front().hypothesis.score);
        s *= findCfg(object_label_).confidence_multiplier;
        if (s >= findCfg(object_label_).min_confidence) objs.push_back({d, s});
      }
    }
    if (inst_msg) {
      for (auto &d : inst_msg->detections) {
        // YOLOv8-seg のbboxにも無効領域を適用（検出モード時）
        if (!fixed_mode_enabled_ && invalid_area_enabled_ && img_w>0 && img_h>0) {
          bool hit = false;
          if (invalid_any_overlap_) hit = rect_overlap(det_to_rect(d), invalid_rect);
          else {
            int cx = static_cast<int>(std::round(d.bbox.center.position.x));
            int cy = static_cast<int>(std::round(d.bbox.center.position.y));
            hit = invalid_rect.contains(cv::Point(cx, cy));
          }
          if (hit) continue;
        }
        float s = d.results.empty() ? 1.0f : static_cast<float>(d.results.front().hypothesis.score);
        s *= findCfg(instance_label_).confidence_multiplier;
        if (s >= findCfg(instance_label_).min_confidence) insts.push_back({d, s});
      }
    }

    // 固定モード or マッチング
    std::vector<int> inst_matched(insts.size(), -1);
    std::vector<fv_msgs::msg::Detection2D> candidates;
    if (!fixed_mode_enabled_) {
      // マッチング（metric切替: iou / y_iou）
      for (size_t i = 0; i < objs.size(); ++i) {
        const Box bo = toBox(objs[i].det);
        float best_s = 0.0f; int best_j = -1;
        for (size_t j = 0; j < insts.size(); ++j) {
          if (inst_matched[j] >= 0) continue;
          const Box bi = toBox(insts[j].det);
          // 中心Xのズレ制限（任意）
          if (center_x_max_offset_px_ > 0) {
            float cx_o = bo.x + 0.5f * bo.w;
            float cx_i = bi.x + 0.5f * bi.w;
            if (std::fabs(cx_o - cx_i) > static_cast<float>(center_x_max_offset_px_)) continue;
          }
          float s = (match_metric_ == std::string("y_iou")) ? y_iou(bo, bi) : iou(bo, bi);
          if (s >= static_cast<float>(iou_th_) && s > best_s) { best_s = s; best_j = static_cast<int>(j); }
        }
        if (best_j >= 0) inst_matched[best_j] = static_cast<int>(i);
      }
    }

    if (!fixed_mode_enabled_) {
      // 生成: マッチしたペア
      for (size_t j = 0; j < insts.size(); ++j) {
        int oi = inst_matched[j];
        if (oi < 0) continue;
        const auto &od = objs[oi];
        const auto &id = insts[j];
        fv_msgs::msg::Detection2D det;
        det.header = od.det.header; // 代表
        det.label = "fused";
        det.source_mask = det.SOURCE_OBJECT | det.SOURCE_INSTANCE;
        det.class_id = 0;
        // bbox選択
        const auto sel = (prefer_bbox_ == std::string("yolov8") ? id.det : od.det);
        det.bbox_min.x = sel.bbox.center.position.x - sel.bbox.size_x * 0.5;
        det.bbox_min.y = sel.bbox.center.position.y - sel.bbox.size_y * 0.5;
        det.bbox_max.x = sel.bbox.center.position.x + sel.bbox.size_x * 0.5;
        det.bbox_max.y = sel.bbox.center.position.y + sel.bbox.size_y * 0.5;
        det.conf_object = od.score;
        det.conf_instance = id.score;
        det.conf_semantic = 0.0f;
        det.conf_fused = static_cast<float>(w_object_ * od.score + w_instance_ * id.score + bonus_overlap_);
        // インスタンス由来のIDを8bitで保持（UIの色分けに利用）
        try {
          int mid = std::stoi(id.det.id);
          if (mid < 0) mid = -mid;
          det.mask_instance_id = static_cast<uint32_t>(mid & 0xFF);
        } catch (...) {
          det.mask_instance_id = 0;
        }
        det.observed_at = sel.header.stamp;
        candidates.emplace_back(det);
      }
    }

    if (!fixed_mode_enabled_) {
      // 生成: 片側のみ（object）
      for (size_t i = 0; i < objs.size(); ++i) {
        bool is_paired = false;
        for (int m : inst_matched) { if (m == static_cast<int>(i)) { is_paired = true; break; } }
        if (is_paired) continue;
        const auto &od = objs[i];
        fv_msgs::msg::Detection2D det;
        det.header = od.det.header;
        det.label = "fused";
        det.source_mask = det.SOURCE_OBJECT;
        det.class_id = 0;
        det.bbox_min.x = od.det.bbox.center.position.x - od.det.bbox.size_x * 0.5;
        det.bbox_min.y = od.det.bbox.center.position.y - od.det.bbox.size_y * 0.5;
        det.bbox_max.x = od.det.bbox.center.position.x + od.det.bbox.size_x * 0.5;
        det.bbox_max.y = od.det.bbox.center.position.y + od.det.bbox.size_y * 0.5;
        det.conf_object = od.score;
        det.conf_instance = 0.0f;
        det.conf_semantic = 0.0f;
        det.conf_fused = static_cast<float>(w_object_ * od.score);
        det.observed_at = od.det.header.stamp;
        candidates.emplace_back(det);
      }
    }

    if (!fixed_mode_enabled_) {
      // 生成: 片側のみ（instance）
      for (size_t j = 0; j < insts.size(); ++j) {
        if (inst_matched[j] >= 0) continue;
        const auto &id = insts[j];
        fv_msgs::msg::Detection2D det;
        det.header = id.det.header;
        det.label = "fused";
        det.source_mask = det.SOURCE_INSTANCE;
        det.class_id = 0;
        det.bbox_min.x = id.det.bbox.center.position.x - id.det.bbox.size_x * 0.5;
        det.bbox_min.y = id.det.bbox.center.position.y - id.det.bbox.size_y * 0.5;
        det.bbox_max.x = id.det.bbox.center.position.x + id.det.bbox.size_x * 0.5;
        det.bbox_max.y = id.det.bbox.center.position.y + id.det.bbox.size_y * 0.5;
        det.conf_object = 0.0f;
        det.conf_instance = id.score;
        det.conf_semantic = 0.0f;
        det.conf_fused = static_cast<float>(w_instance_ * id.score);
        // インスタンスIDを8bitにエンコード
        try {
          int mid = std::stoi(id.det.id);
          if (mid < 0) mid = -mid;
          det.mask_instance_id = static_cast<uint32_t>(mid & 0xFF);
        } catch (...) {
          det.mask_instance_id = 0;
        }
        det.observed_at = id.det.header.stamp;
        candidates.emplace_back(det);
      }
    }

    // 固定モードの強制枠（赤枠として扱う）
    if (fixed_mode_enabled_ && img_w>0 && img_h>0) {
      const cv::Rect fr = make_fixed_rect(img_w, img_h);
      if (fr.width>0 && fr.height>0) {
        fv_msgs::msg::Detection2D det;
        det.header.stamp = this->now();
        det.label = "fixed";
        det.source_mask = det.SOURCE_INSTANCE; // マスク優先
        det.class_id = 0;
        det.bbox_min.x = fr.x; det.bbox_min.y = fr.y;
        det.bbox_max.x = fr.x + fr.width; det.bbox_max.y = fr.y + fr.height;
        det.conf_object = 0.0f; det.conf_instance = 1.0f; det.conf_semantic = 0.0f; det.conf_fused = 1.0f;
        det.observed_at = out.header.stamp;
        candidates.emplace_back(det);
      }
    }

    // NMSで候補を間引き（包含対策: インスタンスとオブジェクトが強く内包関係のときは抑制を緩和）
    auto det_iou = [&](const fv_msgs::msg::Detection2D &a, const fv_msgs::msg::Detection2D &b){
      auto ax1 = a.bbox_min.x, ay1 = a.bbox_min.y, ax2 = a.bbox_max.x, ay2 = a.bbox_max.y;
      auto bx1 = b.bbox_min.x, by1 = b.bbox_min.y, bx2 = b.bbox_max.x, by2 = b.bbox_max.y;
      const float ix1 = static_cast<float>(std::max(ax1, bx1));
      const float iy1 = static_cast<float>(std::max(ay1, by1));
      const float ix2 = static_cast<float>(std::min(ax2, bx2));
      const float iy2 = static_cast<float>(std::min(ay2, by2));
      const float iw = std::max(0.0f, ix2 - ix1);
      const float ih = std::max(0.0f, iy2 - iy1);
      const float inter = iw * ih;
      const float area_a = std::max(0.0f, static_cast<float>((ax2-ax1)*(ay2-ay1)));
      const float area_b = std::max(0.0f, static_cast<float>((bx2-bx1)*(by2-by1)));
      const float uni = area_a + area_b - inter + 1e-6f;
      return inter / uni;
    };
    auto det_y_iou = [&](const fv_msgs::msg::Detection2D &a, const fv_msgs::msg::Detection2D &b){
      const float ay1 = static_cast<float>(a.bbox_min.y);
      const float ay2 = static_cast<float>(a.bbox_max.y);
      const float by1 = static_cast<float>(b.bbox_min.y);
      const float by2 = static_cast<float>(b.bbox_max.y);
      const float iy1 = std::max(ay1, by1);
      const float iy2 = std::min(ay2, by2);
      const float ih = std::max(0.0f, iy2 - iy1);
      const float ha = std::max(0.0f, ay2 - ay1);
      const float hb = std::max(0.0f, by2 - by1);
      const float uni_h = ha + hb - ih + 1e-6f;
      return ih / uni_h;
    };
    auto contain_ratio = [&](const fv_msgs::msg::Detection2D &a, const fv_msgs::msg::Detection2D &b){
      auto ax1 = a.bbox_min.x, ay1 = a.bbox_min.y, ax2 = a.bbox_max.x, ay2 = a.bbox_max.y;
      auto bx1 = b.bbox_min.x, by1 = b.bbox_min.y, bx2 = b.bbox_max.x, by2 = b.bbox_max.y;
      const float ix1 = static_cast<float>(std::max(ax1, bx1));
      const float iy1 = static_cast<float>(std::max(ay1, by1));
      const float ix2 = static_cast<float>(std::min(ax2, bx2));
      const float iy2 = static_cast<float>(std::min(ay2, by2));
      const float iw = std::max(0.0f, ix2 - ix1);
      const float ih = std::max(0.0f, iy2 - iy1);
      const float inter = iw * ih;
      const float area_a = std::max(0.0f, static_cast<float>((ax2-ax1)*(ay2-ay1)));
      const float area_b = std::max(0.0f, static_cast<float>((bx2-bx1)*(by2-by1)));
      const float small = std::max(1e-6f, std::min(area_a, area_b));
      return inter / small; // 小さい方に対する包含率
    };
    auto is_object = [&](const fv_msgs::msg::Detection2D &d){ return (d.source_mask & d.SOURCE_OBJECT) != 0; };
    auto is_instance = [&](const fv_msgs::msg::Detection2D &d){ return (d.source_mask & d.SOURCE_INSTANCE) != 0; };

    auto has_obj = [&](const fv_msgs::msg::Detection2D &d){ return (d.source_mask & d.SOURCE_OBJECT) != 0; };
    auto has_inst = [&](const fv_msgs::msg::Detection2D &d){ return (d.source_mask & d.SOURCE_INSTANCE) != 0; };
    auto is_fused_src = [&](const fv_msgs::msg::Detection2D &d){ return has_obj(d) && has_inst(d); };
    auto priority = [&](const fv_msgs::msg::Detection2D &d){
      if (is_fused_src(d)) return 2;
      if (has_inst(d)) return 1;
      return 0; // object-only 他
    };

    std::vector<int> order(candidates.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b){
      int pa = priority(candidates[a]);
      int pb = priority(candidates[b]);
      if (pa != pb) return pa > pb; // 高優先度から
      return candidates[a].conf_fused > candidates[b].conf_fused;
    });
    std::vector<char> suppressed(candidates.size(), 0);
    std::vector<fv_msgs::msg::Detection2D> selected;
    for (size_t oi = 0; oi < order.size(); ++oi) {
      int i = order[oi];
      if (suppressed[i]) continue;
      selected.emplace_back(candidates[i]);
      for (size_t oj = oi + 1; oj < order.size(); ++oj) {
        int j = order[oj];
        if (suppressed[j]) continue;
        float iouv = (nms_metric_ == std::string("y_iou")) ? det_y_iou(candidates[i], candidates[j]) : det_iou(candidates[i], candidates[j]);
        if (iouv >= static_cast<float>(nms_iou_th_)) {
          // 横方向ゲート: 横重なりが小さい/中心差が大きいペアはNMS抑制対象から外す
          auto ax1 = candidates[i].bbox_min.x; auto ax2 = candidates[i].bbox_max.x;
          auto bx1 = candidates[j].bbox_min.x; auto bx2 = candidates[j].bbox_max.x;
          const float iwx = std::max(0.0f, static_cast<float>(std::min(ax2, bx2) - std::max(ax1, bx1)));
          const float wa = std::max(0.0f, static_cast<float>(ax2 - ax1));
          const float wb = std::max(0.0f, static_cast<float>(bx2 - bx1));
          const float minw = std::max(1e-6f, std::min(wa, wb));
          const float x_overlap_ratio = iwx / minw;
          const float cxa = static_cast<float>(0.5 * (ax1 + ax2));
          const float cxb = static_cast<float>(0.5 * (bx1 + bx2));
          const float cx_diff = std::fabs(cxa - cxb);
          if (nms_center_x_max_offset_px_ > 0 && cx_diff > static_cast<float>(nms_center_x_max_offset_px_)) {
            continue; // 十分離れている → 別個体とみなす
          }
          if (nms_min_x_overlap_ratio_ > 0.0 && x_overlap_ratio < static_cast<float>(nms_min_x_overlap_ratio_)) {
            continue; // 横の重なりが小さい → 別個体とみなす
          }
          bool cross_source = (has_obj(candidates[i]) != has_obj(candidates[j])) || (has_inst(candidates[i]) != has_inst(candidates[j]));
          float cr = contain_ratio(candidates[i], candidates[j]);
          if (cross_source && cr >= static_cast<float>(nms_contain_ratio_)) {
            if (nms_keep_contained_cross_source_) {
              continue; // 両方残す
            }
            // ポリシーに従って低優先度側を抑制
            int pi = priority(candidates[i]);
            int pj = priority(candidates[j]);
            if (nms_cross_contained_policy_ == std::string("prefer_fused")) {
              // fused > instance > object
              if (pj < pi) suppressed[j] = 1; else suppressed[j] = 1; // 同等時はj抑制
            } else { // prefer_instance（デフォルト）: instance(含fused)を優先
              bool i_has_inst = has_inst(candidates[i]);
              bool j_has_inst = has_inst(candidates[j]);
              if (i_has_inst && !j_has_inst) { suppressed[j] = 1; }
              else if (!i_has_inst && j_has_inst) { continue; } // jを残す
              else { suppressed[j] = 1; }
            }
          } else {
            suppressed[j] = 1; // 通常のNMS
          }
        }
      }
    }

    // 融合アクティブを更新（TTL）＋トラックID安定化（IoUマッチ）
    std::vector<int32_t> updated_ids;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      for (auto &kv : fused_active_) kv.second.hold_frames -= 1;

      auto iou_track = [&](const fv_msgs::msg::Detection2D &a, const fv_msgs::msg::Detection2D &b){
        return det_iou(a,b);
      };

      for (auto &det : selected) {
        // 既存トラックにマッチ
        int32_t best_id = -1; float best_iou = 0.0f;
        for (auto &kv : fused_active_) {
          float iv = iou_track(det, kv.second.detection);
          if (iv > best_iou && iv >= track_match_iou_th_) { best_iou = iv; best_id = kv.first; }
        }
        if (best_id < 0) {
          best_id = next_track_id_++;
        }
        det.id = best_id;
        auto &rec = fused_active_[best_id];
        rec.detection = det;
        rec.last_update = this->now();
        rec.hold_frames = hold_frames_;
        if (rec.first_seen.nanoseconds() == 0) rec.first_seen = rec.last_update;
        rec.seen_frames += 1;
        updated_ids.push_back(best_id);
      }
    }

    // TTLで出力収集
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      for (auto it = fused_active_.begin(); it != fused_active_.end();) {
        if (it->second.hold_frames <= 0) { it = fused_active_.erase(it); continue; }
        out.detections.emplace_back(it->second.detection);
        ++it;
      }
    }

    // publish fused array（現フレーム選抜のみ）
    out.detections = selected;
    output_pub_->publish(out);

    // overlay + ROI mask
    if ((debug_enable_overlay_ && overlay_pub_) || roi_mask_pub_) {
      cv::Mat img;
      std_msgs::msg::Header hdr;
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (have_image_) { img = last_image_.clone(); hdr = last_image_header_; }
      }
      if (!img.empty()) {
        // 取得した最新マスク（必要に応じてサイズを画像に合わせる）
        cv::Mat inst_mask, unet_mask;
        std_msgs::msg::Header inst_hdr, unet_hdr;
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          if (have_inst_mask_) { inst_mask = last_inst_mask_.clone(); inst_hdr = last_inst_mask_header_; }
          if (have_unet_mask_) { unet_mask = last_unet_mask_.clone(); unet_hdr = last_unet_mask_header_; }
        }
        auto resize_to = [&](cv::Mat &m){ if (!m.empty() && (m.cols!=img.cols || m.rows!=img.rows)) cv::resize(m, m, img.size(), 0,0, cv::INTER_NEAREST); };
        resize_to(inst_mask); resize_to(unet_mask);

        // フルフレームのマスクオーバーレイ（背景的に塗布）
        if (debug_enable_overlay_ && debug_overlay_tint_mask_) {
          if (overlay_unet_full_ && !unet_mask.empty()) {
            cv::Mat color(img.size(), CV_8UC3, cv::Scalar(255,255,0)); // Cyan
            cv::Mat blended; cv::addWeighted(img, 1.0 - overlay_alpha_unet_, color, overlay_alpha_unet_, 0.0, blended);
            blended.copyTo(img, unet_mask);
          }
          if (overlay_inst_full_ && !inst_mask.empty()) {
            cv::Mat color(img.size(), CV_8UC3, cv::Scalar(255,0,255)); // Magenta
            cv::Mat blended; cv::addWeighted(img, 1.0 - overlay_alpha_inst_, color, overlay_alpha_inst_, 0.0, blended);
            blended.copyTo(img, inst_mask);
          }
        }

        cv::Mat roi_mask(img.size(), CV_8UC1, cv::Scalar(0));
        auto within_time = [&](const std_msgs::msg::Header &mh, const rclcpp::Time &det_t){
          if (mh.stamp.sec==0 && mh.stamp.nanosec==0) return false;
          rclcpp::Time mt(mh.stamp);
          double dt_ms = std::abs((det_t - mt).nanoseconds())/1e6;
          return dt_ms <= max_stamp_diff_ms_;
        };
        auto postprocess = [&](cv::Mat &m){
          if (m.empty()) return;
          if (bin_thresh_ >= 0) cv::threshold(m, m, bin_thresh_, 255, cv::THRESH_BINARY);
          if (open_k_>1 && open_iter_>0) { cv::Mat k=cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(open_k_,open_k_)); cv::morphologyEx(m,m,cv::MORPH_OPEN,k,cv::Point(-1,-1),open_iter_); }
          if (close_k_>1 && close_iter_>0) { cv::Mat k=cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(close_k_,close_k_)); cv::morphologyEx(m,m,cv::MORPH_CLOSE,k,cv::Point(-1,-1),close_iter_); }
          if (fill_holes_) {
            // 背景と接続していない穴だけを埋める
            cv::Mat bordered; cv::copyMakeBorder(m, bordered, 1,1,1,1, cv::BORDER_CONSTANT, cv::Scalar(0));
            cv::Mat flooded = bordered.clone();
            cv::floodFill(flooded, cv::Point(0,0), cv::Scalar(255));
            cv::Mat flood_crop = flooded(cv::Rect(1,1,m.cols, m.rows));
            cv::bitwise_not(flood_crop, flood_crop); // 穴領域（背景に接続していない0領域）
            if (max_hole_area_px_ > 0) {
              cv::Mat labels, stats, centroids; int n = cv::connectedComponentsWithStats(flood_crop, labels, stats, centroids, 8, CV_32S);
              cv::Mat small_holes = cv::Mat::zeros(m.size(), CV_8UC1);
              for (int i=1;i<n;++i) {
                int area = stats.at<int>(i, cv::CC_STAT_AREA);
                if (area <= max_hole_area_px_) small_holes.setTo(255, labels==i);
              }
              cv::bitwise_or(m, small_holes, m);
            } else {
              cv::bitwise_or(m, flood_crop, m);
            }
          }
        };
        auto keep_largest_cc = [&](cv::Mat &m){
          if (!keep_largest_ || m.empty()) return;
          cv::Mat labels, stats, centroids;
          int n = cv::connectedComponentsWithStats(m, labels, stats, centroids, 8, CV_32S);
          int best=-1; int best_area=0;
          for (int i=1;i<n;++i){ int area=stats.at<int>(i, cv::CC_STAT_AREA); if (area>best_area){best_area=area; best=i;} }
          if (best<=0) { m.setTo(0); return; }
          cv::Mat out = cv::Mat::zeros(m.size(), CV_8UC1);
          out.setTo(255, labels==best);
          m = out;
        };

        // ROIごとに処理
        const bool prefer_unet_only = fixed_mode_enabled_ || (mask_prefer_ == std::string("unet_only"));
        for (const auto &det : selected) {
          int x = std::max(0, static_cast<int>(det.bbox_min.x) - roi_pad_px_);
          int y = std::max(0, static_cast<int>(det.bbox_min.y) - roi_pad_px_);
          int w = std::min(img.cols - x, static_cast<int>(det.bbox_max.x - det.bbox_min.x) + 2*roi_pad_px_);
          int h = std::min(img.rows - y, static_cast<int>(det.bbox_max.y - det.bbox_min.y) + 2*roi_pad_px_);
          if (w<=0 || h<=0) continue;
          cv::Rect roi(x,y,w,h);

          cv::Mat base; // 最終ROIマスク
          const bool inst_ok = (!inst_mask.empty() && within_time(inst_hdr, rclcpp::Time(det.observed_at)));
          const bool unet_ok = (!unet_mask.empty()); // UNetはstampがnow()のため時刻ゲートを緩和

          if (mask_combine_mode_ == std::string("union") && (inst_ok || unet_ok)) {
            cv::Mat mi, mu;
            if (inst_ok) mi = inst_mask(roi).clone();
            if (unet_ok) mu = unet_mask(roi).clone();
            if (!mi.empty()) { postprocess(mi); keep_largest_cc(mi); }
            if (!mu.empty()) { postprocess(mu); keep_largest_cc(mu); }
            if (mi.empty()) base = mu;
            else if (mu.empty()) base = mi;
            else { base = mi | mu; }

            int area = base.empty()?0:cv::countNonZero(base);
            if (area < roi_min_area_px_) continue;

            // 合成マスク反映
            base.copyTo(roi_mask(roi), base);

            // オーバーレイ: UNETのみ領域=シアン、INST領域=マゼンタ
            if (debug_enable_overlay_ && debug_overlay_tint_mask_) {
              // 下地: UNet（シアン）→ 上書き: インスタンス（マゼンタ）
              if (!mu.empty()) {
                cv::Mat color_roi(roi.size(), CV_8UC3, cv::Scalar(255,255,0)); // Cyan
                cv::Mat blended; cv::addWeighted(img(roi), 0.6, color_roi, 0.4, 0.0, blended);
                blended.copyTo(img(roi), mu); // 重なり含め塗布
              }
              if (!mi.empty()) {
                cv::Mat color_roi(roi.size(), CV_8UC3, cv::Scalar(255,0,255)); // Magenta
                cv::Mat blended; cv::addWeighted(img(roi), 0.6, color_roi, 0.4, 0.0, blended);
                blended.copyTo(img(roi), mi); // 重なりはマゼンタで上描き
              }
            }
          } else {
            // prefer モード（inst優先→unet）; 固定モード時はUNetのみ
            bool used_inst=false;
            if (!prefer_unet_only && inst_ok) { base = inst_mask(roi).clone(); used_inst=true; }
            else if (unet_ok) { base = unet_mask(roi).clone(); }
            else { continue; }
            postprocess(base);
            keep_largest_cc(base);
            int area = cv::countNonZero(base);
            if (area < roi_min_area_px_) continue;

            // 合成マスクに反映
            base.copyTo(roi_mask(roi), base);

            // オーバーレイに半透明で塗布
            if (debug_enable_overlay_ && debug_overlay_tint_mask_) {
              cv::Mat color_roi(roi.size(), CV_8UC3, used_inst?cv::Scalar(255,0,255):cv::Scalar(255,255,0));
              cv::Mat blended;
              cv::addWeighted(img(roi), 0.6, color_roi, 0.4, 0.0, blended);
              blended.copyTo(img(roi), base);
            }
          }
        }

        // 合成マスクの配信
        if (roi_mask_pub_) {
          cv_bridge::CvImage mask_msg;
          mask_msg.header = hdr;
          mask_msg.encoding = sensor_msgs::image_encodings::MONO8;
          mask_msg.image = roi_mask;
          roi_mask_pub_->publish(*mask_msg.toImageMsg());
        }

        // 無効領域の描画（有効時のみ）
        if (debug_enable_overlay_ && draw_invalid_area_ && invalid_area_enabled_ && invalid_rect.width>0 && invalid_rect.height>0) {
          cv::rectangle(img, invalid_rect, cv::Scalar(96,96,96), 2);
        }

        // 元検出の描画（参照用）
        auto draw_box = [&](const vision_msgs::msg::Detection2D &d, const cv::Scalar &color){
          int x = static_cast<int>(d.bbox.center.position.x - d.bbox.size_x * 0.5);
          int y = static_cast<int>(d.bbox.center.position.y - d.bbox.size_y * 0.5);
          int w = static_cast<int>(d.bbox.size_x);
          int h = static_cast<int>(d.bbox.size_y);
          if (!fixed_mode_enabled_ && invalid_area_enabled_ && invalid_rect.width>0 && invalid_rect.height>0) {
            if (invalid_any_overlap_) {
              cv::Rect rr(x,y,std::max(0,w),std::max(0,h));
              if (rect_overlap(rr, invalid_rect)) return; // 描画しない
            } else {
              int cx = x + w/2, cy = y + h/2; if (invalid_rect.contains(cv::Point(cx,cy))) return; // 描画しない
            }
          }
          // 薄色（パステル）で描画するため、白とブレンドした色を用意
          auto pastel = [&](const cv::Scalar &c){ return cv::Scalar(
              std::min(255.0, c[0]*0.7 + 255*0.3),
              std::min(255.0, c[1]*0.7 + 255*0.3),
              std::min(255.0, c[2]*0.7 + 255*0.3)); };
          cv::Scalar col = pastel(color);
          cv::rectangle(img, cv::Rect(x, y, std::max(0,w), std::max(0,h)), col, std::max(1, src_box_thickness_));
          if (debug_show_src_dims_) {
            double ar = (h <= 1e-6) ? 0.0 : (static_cast<double>(w) / static_cast<double>(h));
            bool pass_size = (w >= filt_min_w_ && h >= filt_min_h_ && w <= filt_max_w_ && h <= filt_max_h_);
            bool pass_ar = (ar >= filt_min_ar_ && ar <= filt_max_ar_);
            bool pass = debug_src_show_filter_ ? (pass_size && pass_ar) : true;
            double conf = d.results.empty() ? 1.0 : d.results.front().hypothesis.score;
            // 表示テキスト
            char buf[128];
            if (debug_src_show_filter_) snprintf(buf, sizeof(buf), "S:%.2f W:%d H:%d AR:%.2f %s", conf, w, h, ar, pass?"OK":"NG");
            else snprintf(buf, sizeof(buf), "S:%.2f W:%d H:%d AR:%.2f", conf, w, h, ar);
            // テキスト背景ボックス
            int baseline = 0;
            cv::Size sz = cv::getTextSize(buf, cv::FONT_HERSHEY_SIMPLEX, std::max(0.3, src_dims_font_scale_), std::max(1, src_dims_thickness_), &baseline);
            int pad = 3;
            int bx = x; int by = std::max(0, y - (sz.height + pad*2) - 2);
            int bw = sz.width + pad*2; int bh = sz.height + pad*2;
            if (bx + bw > img.cols) bx = std::max(0, img.cols - bw - 2);
            cv::rectangle(img, cv::Rect(bx, by, bw, bh), cv::Scalar(32,32,32), cv::FILLED);
            // 影付き日本語描画ユーティリティで可読性を上げる（ASCII主体でも有効）
            fluent::text::drawShadow(img, std::string(buf), cv::Point(bx + pad, by + pad + sz.height),
                                     pass ? cv::Scalar(255,255,255) : cv::Scalar(80,80,255),
                                     cv::Scalar(0,0,0),
                                     std::max(0.3, src_dims_font_scale_), std::max(1, src_dims_thickness_), 0);
          }
        };
        // 固定モード時は元検出の描画を抑止（固定枠のみ表示）
        if (!fixed_mode_enabled_) {
          if (obj_msg && draw_src_obj_boxes_) for (auto &d : obj_msg->detections) draw_box(d, cv::Scalar(255,0,0)); // Blue YOLOv10 (pastelized)
          if (inst_msg && draw_src_inst_boxes_) for (auto &d : inst_msg->detections) draw_box(d, cv::Scalar(255,0,255)); // Magenta YOLOv8-seg (pastelized)
        }

        // 融合結果の描画（緑） + ラベル
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          auto draw_one = [&](const fv_msgs::msg::Detection2D &fd){
            // 固定モード時は固定枠（label=="fixed"）のみ描画する
            const bool is_fixed = (fd.label == std::string("fixed"));
            if (fixed_mode_enabled_ && !is_fixed) return;
            int x = static_cast<int>(fd.bbox_min.x);
            int y = static_cast<int>(fd.bbox_min.y);
            int w = static_cast<int>(fd.bbox_max.x - fd.bbox_min.x);
            int h = static_cast<int>(fd.bbox_max.y - fd.bbox_min.y);
            cv::Scalar col = is_fixed ? cv::Scalar(0,0,255) : cv::Scalar(0,255,0); // 固定=赤, 通常=緑
            cv::rectangle(img, cv::Rect(x, y, std::max(0,w), std::max(0,h)), col, 2);
            if (label_enable_) {
              // 取得情報
              double fused = fd.conf_fused;
              double obj = fd.conf_object;
              double inst = fd.conf_instance;
              // 遅延（画像ヘッダと観測時刻の差）。どちらか未設定なら非表示
              bool have_img_stamp = (hdr.stamp.sec != 0 || hdr.stamp.nanosec != 0);
              bool have_obs_stamp = (fd.observed_at.sec != 0 || fd.observed_at.nanosec != 0);
              std::string dt_str = "-";
              if (have_img_stamp && have_obs_stamp) {
                rclcpp::Time obs(fd.observed_at);
                rclcpp::Time ih(hdr.stamp);
                double dt_ms = std::abs((ih - obs).nanoseconds()) / 1e6; // ms
                if (dt_ms < 1000.0) {
                  char b[32]; snprintf(b, sizeof(b), "%dms", (int)std::round(dt_ms)); dt_str = b;
                } else {
                  char b[32]; snprintf(b, sizeof(b), "%.1fs", dt_ms/1000.0); dt_str = b;
                }
              }
              int ttl = 0, frames = 0; double age_s = 0.0;
              auto it = fused_active_.find(fd.id);
              if (it != fused_active_.end()) {
                ttl = it->second.hold_frames;
                frames = it->second.seen_frames;
                if (it->second.first_seen.nanoseconds() != 0) {
                  age_s = (it->second.last_update - it->second.first_seen).seconds();
                }
              }
              // ラベル文字列（2行、日本語）
              char line1[160]; snprintf(line1, sizeof(line1), "ID:%d  統合:%.2f  物体:%.2f  インスタ:%.2f", fd.id, fused, obj, inst);
              char line2[160]; snprintf(line2, sizeof(line2), "経過:%.1fs  枚数:%df  遅延:%s  残:%d", age_s, frames, dt_str.c_str(), ttl);
              std::string l1(line1), l2(line2);
              // 位置（bbox上）
              int baseline = 0;
              int pad = 4, gap = 2;
              // 日本語はgetTextSizeで幅が過大になることがあるため、ASCIIベースで測り+余白
              char m1[160]; snprintf(m1, sizeof(m1), "ID:%d  F:%.2f O:%.2f I:%.2f", fd.id, fused, obj, inst);
              char m2[160]; snprintf(m2, sizeof(m2), "%.1fs  %df  %s  %d", age_s, frames, dt_str.c_str(), ttl);
              cv::Size sz1 = cv::getTextSize(m1, cv::FONT_HERSHEY_SIMPLEX, label_font_scale_, label_thickness_, &baseline);
              cv::Size sz2 = cv::getTextSize(m2, cv::FONT_HERSHEY_SIMPLEX, label_font_scale_, label_thickness_, &baseline);
              int jp_extra = 12; // 日本語分の余白
              int bw = std::max(sz1.width, sz2.width) + pad*2 + jp_extra;
              int bh = (sz1.height + sz2.height + gap) + pad*2;
              int bx = x; int by = std::max(0, y - bh - 2);
              if (bx + bw > img.cols) bx = std::max(0, img.cols - bw - 2);
              cv::rectangle(img, cv::Rect(bx, by, bw, bh), cv::Scalar(32,32,32), cv::FILLED);
              // 日本語対応の描画（影付き）
              fluent::text::drawShadow(img, l1, cv::Point(bx + pad, by + pad + sz1.height),
                                       cv::Scalar(255,255,255), cv::Scalar(0,0,0),
                                       label_font_scale_, std::max(1, label_thickness_), 0);
              fluent::text::drawShadow(img, l2, cv::Point(bx + pad, by + pad + sz1.height + gap + sz2.height),
                                       cv::Scalar(255,255,255), cv::Scalar(0,0,0),
                                       label_font_scale_, std::max(1, label_thickness_), 0);
            }
          };
          if (debug_draw_history_) {
            for (const auto &kv : fused_active_) draw_one(kv.second.detection);
          } else {
            for (int32_t id : updated_ids) draw_one(fused_active_[id].detection);
          }
        }

        cv_bridge::CvImage out_cv;
        out_cv.header = hdr;
        out_cv.encoding = sensor_msgs::image_encodings::BGR8;
        out_cv.image = img;
        if (debug_enable_overlay_ && overlay_pub_) {
          overlay_pub_->publish(*out_cv.toImageMsg());
        }
      }
    }
  }

  int32_t makeKey(const std::string &label, const vision_msgs::msg::Detection2D &det) const {
    const double cx = det.bbox.center.position.x;
    const double cy = det.bbox.center.position.y;
    const double w = det.bbox.size_x;
    const double h = det.bbox.size_y;
    std::size_t hash = std::hash<std::string>{}(label);
    hash ^= std::hash<long long>{}(static_cast<long long>(cx * 1000.0)) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    hash ^= std::hash<long long>{}(static_cast<long long>(cy * 1000.0)) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    hash ^= std::hash<long long>{}(static_cast<long long>(w * 1000.0)) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    hash ^= std::hash<long long>{}(static_cast<long long>(h * 1000.0)) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    return static_cast<int32_t>(hash & 0x7FFFFFFF);
  }

  int32_t makeKeyFused(const std::string &label, const fv_msgs::msg::Detection2D &det) const {
    const double cx = (det.bbox_min.x + det.bbox_max.x) * 0.5;
    const double cy = (det.bbox_min.y + det.bbox_max.y) * 0.5;
    const double w = (det.bbox_max.x - det.bbox_min.x);
    const double h = (det.bbox_max.y - det.bbox_min.y);
    std::size_t hash = std::hash<std::string>{}(label);
    hash ^= std::hash<long long>{}(static_cast<long long>(cx * 1000.0)) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    hash ^= std::hash<long long>{}(static_cast<long long>(cy * 1000.0)) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    hash ^= std::hash<long long>{}(static_cast<long long>(w * 1000.0)) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    hash ^= std::hash<long long>{}(static_cast<long long>(h * 1000.0)) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    return static_cast<int32_t>(hash & 0x7FFFFFFF);
  }

  std::string output_topic_;
  int output_qos_depth_{5};
  double diagnostic_interval_ms_{100.0};
  int hold_frames_{5};
  std::string frame_id_override_;

  // 画像/デバッグ
  std::string input_image_topic_;
  bool debug_enable_overlay_{true};
  std::string overlay_topic_{"overlay"};
  bool debug_draw_history_{false};
  std::string roi_mask_topic_{"roi_mask"};
  bool debug_overlay_tint_mask_{true};
  bool overlay_inst_full_{true};
  bool overlay_unet_full_{false};
  double overlay_alpha_inst_{0.4};
  double overlay_alpha_unet_{0.3};
  bool label_enable_{true};
  double label_font_scale_{0.5};
  int label_thickness_{1};
  bool debug_show_src_dims_{false};
  bool debug_src_show_filter_{true};
  double src_dims_font_scale_{0.4};
  int src_dims_thickness_{1};
  bool draw_src_obj_boxes_{true};
  bool draw_src_inst_boxes_{true};
  int src_box_thickness_{1};

  std::vector<SourceConfig> sources_;
  std::vector<rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr> source_subs_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr inst_mask_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr unet_mask_sub_;

  std::mutex data_mutex_;
  std::unordered_map<std::string, vision_msgs::msg::Detection2DArray::SharedPtr> latest_by_label_;
  std::unordered_map<int32_t, DetRecord> fused_active_;
  cv::Mat last_image_;
  std_msgs::msg::Header last_image_header_;
  bool have_image_{false};
  cv::Mat last_inst_mask_;
  cv::Mat last_unet_mask_;
  std_msgs::msg::Header last_inst_mask_header_;
  std_msgs::msg::Header last_unet_mask_header_;
  bool have_inst_mask_{false};
  bool have_unet_mask_{false};

  rclcpp::Publisher<fv_msgs::msg::DetectionArray>::SharedPtr output_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr roi_mask_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_invalid_area_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_fixed_mode_;

  // 融合設定
  std::string object_label_{"object"};
  std::string instance_label_{"instance"};
  double iou_th_{0.30};
  double nms_iou_th_{0.50};
  double nms_contain_ratio_{0.85};
  bool nms_keep_contained_cross_source_{true};
  std::string nms_cross_contained_policy_{"prefer_instance"};
  std::string match_metric_{"iou"};
  std::string nms_metric_{"iou"};
  int center_x_max_offset_px_{0};
  double nms_min_x_overlap_ratio_{0.15};
  int nms_center_x_max_offset_px_{0};
  double track_match_iou_th_{0.50};
  double w_object_{0.6};
  double w_instance_{0.4};
  double bonus_overlap_{0.05};
  std::string prefer_bbox_{"yolov10"};
  int32_t next_track_id_{1};

  // マスク・ROI設定
  std::string instance_mask_topic_;
  std::string unet_mask_topic_;
  double max_stamp_diff_ms_{120.0};
  std::string mask_prefer_{"instance_then_unet"};
  std::string mask_combine_mode_{"prefer"}; // prefer | union
  int roi_pad_px_{6};
  int roi_min_area_px_{300};
  double roi_min_aspect_{0.1};
  double roi_max_aspect_{1.2};
  int bin_thresh_{128};
  int open_k_{3};
  int open_iter_{1};
  int close_k_{5};
  int close_iter_{1};
  bool keep_largest_{true};
  bool fill_holes_{true};
  int max_hole_area_px_{2000};

  // モード/無効領域/フィルタ
  bool fixed_mode_enabled_{false};
  bool invalid_area_enabled_{true};
  bool draw_invalid_area_{true};
  bool invalid_any_overlap_{true};
  bool invalid_touch_inclusive_{true};
  double inv_cx_r_{0.5}, inv_y_top_r_{0.5}, inv_w_r_{0.5}, inv_h_r_{0.5};
  double fixed_cx_r_{0.5};
  int fixed_y_top_px_{5};
  double fixed_w_r_{0.2}, fixed_h_r_{0.5};
  double filt_min_ar_{0.05}, filt_max_ar_{2.0};
  int filt_min_w_{8}, filt_min_h_{20}, filt_max_w_{8192}, filt_max_h_{8192};

  const SourceConfig &findCfg(const std::string &label) const {
    for (const auto &s : sources_) if (s.label == label) return s;
    static SourceConfig kDefault; return kDefault;
  }
};

}  // namespace fv_detection_fusion

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<fv_detection_fusion::DetectionFusionNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
