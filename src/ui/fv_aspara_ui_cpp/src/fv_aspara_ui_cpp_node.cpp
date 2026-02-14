// Minimal C++ Asparagus UI overlay node
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <fluent_lib/cv_bridge_compat.hpp>
#include <opencv2/imgproc.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "fluent_text.hpp"
#include <string>
#include <memory>
#include <unordered_map>
#include <unordered_set>

#include "fv_msgs/msg/detection_array.hpp"
#include "fv_msgs/msg/stalk_metrics_array.hpp"

namespace fv_aspara_ui_cpp {

class AsparaUiCppNode : public rclcpp::Node {
public:
  explicit AsparaUiCppNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("fv_aspara_ui_cpp", options)
  {
    declare_parameter<std::string>("image_topic", "~/color/image_raw");
    declare_parameter<std::string>("detections_topic", "~/detections");
    declare_parameter<std::string>("metrics_topic", "~/metrics");
    declare_parameter<std::string>("camera_info_topic", "~/camera_info");
    declare_parameter<std::string>("annotated_topic", "~/annotated_image");
    // Image QoS reliability: best_effort | reliable | system_default
    declare_parameter<std::string>("image_qos_reliability", "best_effort");
    // Optional mask overlay
    declare_parameter<std::string>("mask_topic", "");
    // mask_topic が instance id (0..255) を持つ /mono8 の場合 true にする（高速化）
    declare_parameter<bool>("mask_is_instance_id", false);
    declare_parameter<int>("mask_threshold", 128);
    declare_parameter<double>("mask_overlay_alpha", 0.30);
    declare_parameter<int>("mask_overlay_color_bgr[0]", 0);
    declare_parameter<int>("mask_overlay_color_bgr[1]", 255);
    declare_parameter<int>("mask_overlay_color_bgr[2]", 0);
    // IDごとの色（B,G,Rの順で3つ1組）
    declare_parameter<std::vector<int>>("id_color_palette_bgr",
      std::vector<int>{
        // vivid BGR palette (no grays)
        0,0,255,    0,255,0,    255,0,0,    0,255,255,  255,0,255,  255,255,0,
        0,165,255,  180,105,255,  0,215,255,  203,192,255,  77,255,255,  255,144,30
      });
    // bbox colors (avoid grays)
    declare_parameter<int>("selected_box_color_bgr[0]", 0);
    declare_parameter<int>("selected_box_color_bgr[1]", 255);
    declare_parameter<int>("selected_box_color_bgr[2]", 0);
    declare_parameter<int>("unselected_box_color_bgr[0]", 0);   // orange-ish
    declare_parameter<int>("unselected_box_color_bgr[1]", 165);
    declare_parameter<int>("unselected_box_color_bgr[2]", 255);
    declare_parameter<bool>("draw_labels", true);
    declare_parameter<bool>("draw_confidence", true);
    declare_parameter<bool>("draw_metrics", true);
    declare_parameter<bool>("draw_root_tip", true);
    declare_parameter<bool>("draw_root_xyz", false);
    // スケルトン描画（root→tip間を等間隔に分割して点列表示）
    declare_parameter<bool>("draw_skeleton", true);
    declare_parameter<int>("skeleton.num_points", 10);  // 分割数
    declare_parameter<int>("skeleton.point_radius", 3); // 点の半径
    // Metrics units
    declare_parameter<std::string>("metrics.length_unit", "cm");  // cm|mm|m
    declare_parameter<std::string>("metrics.diameter_unit", "mm"); // cm|mm|m
    declare_parameter<bool>("projection.transform_to_camera_info", true);
    // Selected info (top-left)
    declare_parameter<bool>("selected_info.enable", true);
    declare_parameter<double>("selected_info.font_scale", 0.5);
    declare_parameter<int>("selected_info.thickness", 1);
    declare_parameter<int>("selected_info.offset_x", 8);
    declare_parameter<int>("selected_info.offset_y", 14);
    // Selected TF publish
    declare_parameter<bool>("selected_tf.enable", true);
    declare_parameter<std::string>("selected_tf.child_frame", "aspara_selected");
    // 画面左上タイトルは使わず、各矩形にタイトル（アスパラ）を描画する方針
    // 後方互換のため draw_title パラメータは宣言のみ行い無視する
    declare_parameter<bool>("draw_title", false);
    // Invalid area overlay (same schema as detection_fusion)
    declare_parameter<bool>("draw_invalid_area", true);
    declare_parameter<bool>("invalid_area.enabled", false);
    declare_parameter<bool>("invalid_area.any_overlap", true);
    declare_parameter<bool>("invalid_area.touch_inclusive", true);
    declare_parameter<double>("invalid_area.box.center_x_ratio", 0.5);
    declare_parameter<double>("invalid_area.box.y_top_ratio", 0.5);
    declare_parameter<double>("invalid_area.box.width_ratio", 0.5);
    declare_parameter<double>("invalid_area.box.height_ratio", 0.5);
    // Selection drawing
    declare_parameter<bool>("draw_selection_box", true);
    declare_parameter<std::string>("selection.mode", "auto"); // auto|fixed
    declare_parameter<std::string>("selection.auto_strategy", "highest_conf"); // highest_conf|largest_height
    declare_parameter<double>("selection.fixed.box.x_center_ratio", 0.5);
    declare_parameter<int>("selection.fixed.box.y_top_px", 5);
    declare_parameter<double>("selection.fixed.box.width_ratio", 0.20);
    declare_parameter<double>("selection.fixed.box.height_ratio", 0.50);
    // Selection stability
    declare_parameter<int>("selection.lock_frames", 10);
    declare_parameter<int>("selection.hold_frames", 5);
    declare_parameter<double>("selection.switch_conf_margin", 0.05);
    // Display filter (UI only)
    declare_parameter<bool>("display_filter.enable", true);
    declare_parameter<bool>("display_filter.only_vertical", false);
    declare_parameter<double>("display_filter.min_aspect_ratio", 0.05);
    declare_parameter<double>("display_filter.max_aspect_ratio", 2.0);
    declare_parameter<int>("display_filter.min_w_px", 1);
    declare_parameter<int>("display_filter.min_h_px", 1);
    declare_parameter<bool>("display_filter.draw_filtered_boxes", true);
    // Fallbacks for far/small detections
    declare_parameter<bool>("selection.ignore_filter_fallback", true);
    declare_parameter<bool>("metrics.draw_even_if_filtered", true);
    // Overlay gating
    declare_parameter<bool>("overlay.show_only_selected", false);
    declare_parameter<double>("overlay.min_confidence", 0.0);
    // FPS制限（rqt表示負荷軽減）
    declare_parameter<double>("output.max_fps", 0.0);

    image_topic_ = get_parameter("image_topic").as_string();
    detections_topic_ = get_parameter("detections_topic").as_string();
    metrics_topic_ = get_parameter("metrics_topic").as_string();
    camera_info_topic_ = get_parameter("camera_info_topic").as_string();
    annotated_topic_ = get_parameter("annotated_topic").as_string();
    image_qos_reliability_ = get_parameter("image_qos_reliability").as_string();
    mask_topic_ = get_parameter("mask_topic").as_string();
    mask_is_instance_id_ = get_parameter("mask_is_instance_id").as_bool();
    mask_threshold_ = get_parameter("mask_threshold").as_int();
    mask_alpha_ = get_parameter("mask_overlay_alpha").as_double();
    mask_color_bgr_[0] = get_parameter("mask_overlay_color_bgr[0]").as_int();
    mask_color_bgr_[1] = get_parameter("mask_overlay_color_bgr[1]").as_int();
    mask_color_bgr_[2] = get_parameter("mask_overlay_color_bgr[2]").as_int();
    {
      auto pal = get_parameter("id_color_palette_bgr").as_integer_array();
      id_palette_bgr_.assign(pal.begin(), pal.end());
      rebuildPaletteColors();
    }
    draw_labels_ = get_parameter("draw_labels").as_bool();
    draw_confidence_ = get_parameter("draw_confidence").as_bool();
    draw_metrics_ = get_parameter("draw_metrics").as_bool();
    draw_root_tip_ = get_parameter("draw_root_tip").as_bool();
    draw_root_xyz_ = get_parameter("draw_root_xyz").as_bool();
    draw_skeleton_ = get_parameter("draw_skeleton").as_bool();
    skeleton_num_points_ = std::max(2, static_cast<int>(get_parameter("skeleton.num_points").as_int()));
    skeleton_point_radius_ = std::max(1, static_cast<int>(get_parameter("skeleton.point_radius").as_int()));
    length_unit_ = get_parameter("metrics.length_unit").as_string();
    diameter_unit_ = get_parameter("metrics.diameter_unit").as_string();
    proj_transform_to_cinfo_ = get_parameter("projection.transform_to_camera_info").as_bool();
    selinfo_enable_ = get_parameter("selected_info.enable").as_bool();
    selinfo_font_scale_ = get_parameter("selected_info.font_scale").as_double();
    selinfo_thickness_ = get_parameter("selected_info.thickness").as_int();
    selinfo_offx_ = get_parameter("selected_info.offset_x").as_int();
    selinfo_offy_ = get_parameter("selected_info.offset_y").as_int();
    seltf_enable_ = get_parameter("selected_tf.enable").as_bool();
    seltf_child_ = get_parameter("selected_tf.child_frame").as_string();
    rebuildUnitScales();
    
    draw_title_ = get_parameter("draw_title").as_bool();
    draw_invalid_area_ = get_parameter("draw_invalid_area").as_bool();
    invalid_enabled_ = get_parameter("invalid_area.enabled").as_bool();
    invalid_any_overlap_ = get_parameter("invalid_area.any_overlap").as_bool();
    invalid_touch_inclusive_ = get_parameter("invalid_area.touch_inclusive").as_bool();
    ia_cx_ = get_parameter("invalid_area.box.center_x_ratio").as_double();
    ia_yt_ = get_parameter("invalid_area.box.y_top_ratio").as_double();
    ia_wr_ = get_parameter("invalid_area.box.width_ratio").as_double();
    ia_hr_ = get_parameter("invalid_area.box.height_ratio").as_double();
    draw_selection_box_ = get_parameter("draw_selection_box").as_bool();
    sel_mode_ = get_parameter("selection.mode").as_string();
    sel_auto_strategy_ = get_parameter("selection.auto_strategy").as_string();
    sel_fix_xc_ = get_parameter("selection.fixed.box.x_center_ratio").as_double();
    sel_fix_yt_px_ = get_parameter("selection.fixed.box.y_top_px").as_int();
    sel_fix_wr_ = get_parameter("selection.fixed.box.width_ratio").as_double();
    sel_fix_hr_ = get_parameter("selection.fixed.box.height_ratio").as_double();
    sel_lock_frames_ = std::max<int>(0, static_cast<int>(get_parameter("selection.lock_frames").as_int()));
    sel_hold_frames_ = std::max<int>(0, static_cast<int>(get_parameter("selection.hold_frames").as_int()));
    sel_switch_margin_ = get_parameter("selection.switch_conf_margin").as_double();
    disp_filter_enable_ = get_parameter("display_filter.enable").as_bool();
    disp_only_vertical_ = get_parameter("display_filter.only_vertical").as_bool();
    disp_min_ar_ = get_parameter("display_filter.min_aspect_ratio").as_double();
    disp_max_ar_ = get_parameter("display_filter.max_aspect_ratio").as_double();
    disp_min_w_ = get_parameter("display_filter.min_w_px").as_int();
    disp_min_h_ = get_parameter("display_filter.min_h_px").as_int();
    disp_draw_filtered_boxes_ = get_parameter("display_filter.draw_filtered_boxes").as_bool();
    sel_ignore_filter_fallback_ = get_parameter("selection.ignore_filter_fallback").as_bool();
    metrics_draw_even_if_filtered_ = get_parameter("metrics.draw_even_if_filtered").as_bool();
    overlay_only_selected_ = get_parameter("overlay.show_only_selected").as_bool();
    overlay_min_conf_draw_ = get_parameter("overlay.min_confidence").as_double();
    output_max_fps_ = get_parameter("output.max_fps").as_double();

    // TF buffer/listener/broadcaster for alignment and selected TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // 全てBEST_EFFORT + queue=1（最新フレームのみ処理、遅延最小化）
    rclcpp::QoS qos_sensor(1);
    qos_sensor.best_effort();

    sub_img_ = create_subscription<sensor_msgs::msg::Image>(image_topic_, qos_sensor,
      std::bind(&AsparaUiCppNode::onImage, this, std::placeholders::_1));
    sub_det_ = create_subscription<fv_msgs::msg::DetectionArray>(detections_topic_, rclcpp::QoS(1).best_effort(),
      std::bind(&AsparaUiCppNode::onDetections, this, std::placeholders::_1));
    if (!metrics_topic_.empty()) {
      sub_metrics_ = create_subscription<fv_msgs::msg::StalkMetricsArray>(metrics_topic_, rclcpp::QoS(1).best_effort(),
        std::bind(&AsparaUiCppNode::onMetrics, this, std::placeholders::_1));
    }
    if (!mask_topic_.empty()) {
      sub_mask_ = create_subscription<sensor_msgs::msg::Image>(mask_topic_, rclcpp::QoS(1).best_effort(),
        std::bind(&AsparaUiCppNode::onMask, this, std::placeholders::_1));
    }
    if (!camera_info_topic_.empty()) {
      rclcpp::QoS qos_info(5);
      qos_info.best_effort();
      sub_cinfo_ = create_subscription<sensor_msgs::msg::CameraInfo>(camera_info_topic_, qos_info,
        std::bind(&AsparaUiCppNode::onCameraInfo, this, std::placeholders::_1));
    }

    pub_anno_ = create_publisher<sensor_msgs::msg::Image>(annotated_topic_, qos_sensor);

    RCLCPP_INFO(get_logger(), "Aspara UI CPP started: img=%s det=%s met=%s mask=%s (instance_id=%s) out=%s",
                image_topic_.c_str(), detections_topic_.c_str(), metrics_topic_.c_str(),
                mask_topic_.c_str(), mask_is_instance_id_ ? "true" : "false",
                annotated_topic_.c_str());

    srv_get_sel_ = create_service<std_srvs::srv::Trigger>(
      "/get_selected_asparagus",
      std::bind(&AsparaUiCppNode::onSrvGetSelected, this, std::placeholders::_1, std::placeholders::_2));
    srv_next_ = create_service<std_srvs::srv::Trigger>(
      "/next_asparagus",
      std::bind(&AsparaUiCppNode::onSrvNext, this, std::placeholders::_1, std::placeholders::_2));
    srv_prev_ = create_service<std_srvs::srv::Trigger>(
      "/prev_asparagus",
      std::bind(&AsparaUiCppNode::onSrvPrev, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void onDetections(const fv_msgs::msg::DetectionArray::SharedPtr msg) {
    last_dets_ = msg;
  }

  void onMetrics(const fv_msgs::msg::StalkMetricsArray::SharedPtr msg) {
    last_metrics_ = msg;
  }

  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    fx_ = msg->k[0]; fy_ = msg->k[4]; cx_ = msg->k[2]; cy_ = msg->k[5];
    cinfo_frame_ = msg->header.frame_id;
  }

  void onImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    auto callback_start = std::chrono::steady_clock::now();

    cv::Mat view;
    auto cvbridge_start = std::chrono::steady_clock::now();
    try {
      auto cvp = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      view = cvp->image;
    } catch (const cv_bridge::Exception &ex) {
      RCLCPP_WARN(get_logger(), "cv_bridge convert to BGR8 failed: %s", ex.what());
      return;
    }
    auto cvbridge_end = std::chrono::steady_clock::now();
    double cvbridge_ms = std::chrono::duration<double, std::milli>(cvbridge_end - cvbridge_start).count();
    if (cvbridge_ms > 10.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "⚠️ cv_bridge took %.1fms", cvbridge_ms);
    }

    // compute invalid area rect in pixels if enabled
    cv::Rect invalid_rect;
    if (invalid_enabled_) {
      invalid_rect = makeInvalidRect(view.cols, view.rows);
    }

    int selected_index = -1;
    int selected_id = -1;
    double selected_conf = 0.0;
    if (sel_mode_ == "auto" && last_dets_ && !last_dets_->detections.empty()) {
      int cand_index = pickSelectedIndex(*last_dets_, sel_auto_strategy_, view.cols, view.rows);
      if (cand_index < 0 && sel_ignore_filter_fallback_) {
        // Retry selection without display filter (to keep far small targets)
        cand_index = pickSelectedIndex(*last_dets_, sel_auto_strategy_, view.cols, view.rows, /*ignore_filter=*/true);
      }
      if (cand_index >= 0 && cand_index < static_cast<int>(last_dets_->detections.size())) {
        const auto &cand = last_dets_->detections[static_cast<size_t>(cand_index)];
        double cand_conf = static_cast<double>(cand.conf_fused);
        // Hard lock: keep previous id for sel_lock_frames_ if still present
        if (last_selected_id_ >= 0 && lock_left_ > 0) {
          int prev_idx = -1;
          for (size_t i = 0; i < last_dets_->detections.size(); ++i) {
            if (last_dets_->detections[i].id == last_selected_id_) { prev_idx = static_cast<int>(i); break; }
          }
          if (prev_idx >= 0) {
            selected_index = prev_idx;
            selected_id = last_selected_id_;
            selected_conf = static_cast<double>(last_dets_->detections[static_cast<size_t>(prev_idx)].conf_fused);
            lock_left_ = std::max(0, lock_left_ - 1);
            // skip further switching logic while locked
          } else {
            // lost previous target, fall through to normal logic and reset lock on selection
          }
        }
        if (selected_id < 0) {
        // Hysteresis: keep previous selection for a few frames unless strong improvement
        if (last_selected_id_ >= 0 && cand.id != last_selected_id_ && hold_count_ < sel_hold_frames_) {
          if (cand_conf <= last_selected_conf_ + sel_switch_margin_) {
            // Try to keep previous id if it still exists in current detections
            int prev_idx = -1;
            for (size_t i = 0; i < last_dets_->detections.size(); ++i) {
              if (last_dets_->detections[i].id == last_selected_id_) { prev_idx = static_cast<int>(i); break; }
            }
            if (prev_idx >= 0) {
              selected_index = prev_idx;
              selected_id = last_selected_id_;
              selected_conf = last_selected_conf_;
              hold_count_++;
            } else {
              selected_index = cand_index; selected_id = cand.id; selected_conf = cand_conf; hold_count_ = 0;
              if (sel_lock_frames_ > 0) lock_left_ = sel_lock_frames_;
            }
          } else {
            selected_index = cand_index; selected_id = cand.id; selected_conf = cand_conf; hold_count_ = 0;
            if (sel_lock_frames_ > 0) lock_left_ = sel_lock_frames_;
          }
        } else {
          selected_index = cand_index; selected_id = cand.id; selected_conf = cand_conf;
          if (selected_id == last_selected_id_) { hold_count_++; } else { hold_count_ = 0; if (sel_lock_frames_ > 0) lock_left_ = sel_lock_frames_; }
        }
      }
    }

    // Manual override (via services)
    if (manual_active_ && last_dets_ && !last_dets_->detections.empty()) {
      int idx = -1;
      for (size_t i = 0; i < last_dets_->detections.size(); ++i) {
        if (last_dets_->detections[i].id == manual_selected_id_) { idx = static_cast<int>(i); break; }
      }
      if (idx >= 0) {
        selected_index = idx;
        selected_id = manual_selected_id_;
        selected_conf = static_cast<double>(last_dets_->detections[static_cast<size_t>(idx)].conf_fused);
      } else {
        manual_active_ = false;
      }
    }

    if (selected_id < 0 && last_dets_ && selected_index >= 0 && selected_index < static_cast<int>(last_dets_->detections.size())) {
      selected_id = last_dets_->detections[static_cast<size_t>(selected_index)].id;
      selected_conf = static_cast<double>(last_dets_->detections[static_cast<size_t>(selected_index)].conf_fused);
    }
    if (selected_id >= 0) { last_selected_id_ = selected_id; last_selected_conf_ = selected_conf; }

    // 検出枠は draw_title_ に依存させない（常に描画）
    if (last_dets_) {
      for (size_t idx = 0; idx < last_dets_->detections.size(); ++idx) {
        const auto &d = last_dets_->detections[idx];
        int x1 = static_cast<int>(std::floor(d.bbox_min.x));
        int y1 = static_cast<int>(std::floor(d.bbox_min.y));
        int x2 = static_cast<int>(std::ceil(d.bbox_max.x));
        int y2 = static_cast<int>(std::ceil(d.bbox_max.y));
        x1 = std::clamp(x1, 0, std::max(0, view.cols-1));
        y1 = std::clamp(y1, 0, std::max(0, view.rows-1));
        x2 = std::clamp(x2, 0, std::max(0, view.cols-1));
        y2 = std::clamp(y2, 0, std::max(0, view.rows-1));
        cv::Rect bb(cv::Point(x1,y1), cv::Point(x2,y2));
        // draw bbox: vivid colors (no grays)
        bool is_selected = (static_cast<int>(idx) == selected_index);
        bool filtered_out = !passDisplayFilter(bb);
        double det_conf = static_cast<double>(d.conf_fused);
        if (overlay_only_selected_ && !is_selected) {
          continue;
        }
        if (!is_selected && det_conf < overlay_min_conf_draw_) {
          continue;
        }
        if (filtered_out && !(is_selected || disp_draw_filtered_boxes_)) {
          continue;
        }
        cv::Scalar color = is_selected ? sel_box_color_ : unsel_box_color_;
        int thickness = is_selected && draw_selection_box_ ? 2 : 1; // thinner selection box
        cv::rectangle(view, bb, color, thickness);
        if (is_selected) {
          // Draw selected panel: ID, conf, size, distance if available
          //char line1[128]; std::snprintf(line1, sizeof(line1), "アスパラ#%d conf=%.2f", d.id, std::max(0.0, std::min(1.0, (double)d.conf_fused)));
          //char line2[128]; std::snprintf(line2, sizeof(line2), "サイズ %dx%d px", bb.width, bb.height);
          std::string line3;
          // find distance from metrics (root z)
          if (last_metrics_) {
            for (const auto &m : last_metrics_->stalks) {
              if (m.id == d.id && m.root_camera.z > 0.0) {
                char buf[128]; std::snprintf(buf, sizeof(buf), "距離 %.1f cm", (double)m.root_camera.z*100.0);
                line3 = buf; break;
              }
            }
          }
          int base_y = std::max(0, y1 - 28);
          //fluent::text::drawShadow(view, std::string(line1), cv::Point(x1, base_y), cv::Scalar(255,255,255), cv::Scalar(0,0,0), 0.6, 2, 0);
          //fluent::text::drawShadow(view, std::string(line2), cv::Point(x1, base_y+16), cv::Scalar(230,230,230), cv::Scalar(0,0,0), 0.5, 1, 0);
          if (!line3.empty()) {
            fluent::text::drawShadow(view, line3, cv::Point(x1, base_y+32), cv::Scalar(200,255,200), cv::Scalar(0,0,0), 0.5, 1, 0);
          }
        }
        if (!is_selected && (draw_labels_ || draw_confidence_)) {
          std::string label;
          if (draw_labels_) label = "ID " + std::to_string(d.id);
          if (draw_confidence_) {
            double conf = static_cast<double>(d.conf_fused);
            char buf[64]; std::snprintf(buf, sizeof(buf), " (%.2f)", conf);
            label += buf;
          }
          if (!label.empty()) {
            cv::putText(view, label, cv::Point(x1, std::max(0, y1-5)), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
          }
        }
      }
    }

    // mask overlay per asparagus ID (instance id -> detection id -> palette color)
    if (last_mask_ && !mask_topic_.empty()) {
      auto sp = last_mask_;
      if (sp) {
        try {
          cv_bridge::CvImageConstPtr mbridge = cv_bridge::toCvShare(sp);
          cv::Mat mask;
          if (mbridge->image.channels() == 1) {
            mask = mbridge->image;
          } else {
            cv::cvtColor(mbridge->image, mask, cv::COLOR_BGR2GRAY);
          }
          if (mask.size() != view.size()) {
            cv::resize(mask, mask, view.size(), 0, 0, cv::INTER_NEAREST);
          }
          // build LUT for instance id -> BGR based on current detections
          cv::Vec3b lut[256];
          for (int i = 0; i < 256; ++i) {
            lut[i] = cv::Vec3b(
                (uchar)mask_color_bgr_[0], (uchar)mask_color_bgr_[1], (uchar)mask_color_bgr_[2]);
          }
          if (last_dets_) {
            for (const auto &d : last_dets_->detections) {
              uint32_t inst = d.mask_instance_id;
              if (inst == 0) continue;
              cv::Vec3b c = colorForDetId(d.id);
              lut[inst & 0xFF] = c;
            }
          }

          cv::Mat overlay = view.clone();
          if (mask_is_instance_id_ && mask.type() == CV_8UC1) {
            // 高速: 1ch mask (instance id) を LUT で 3ch に変換して合成
            cv::Mat lut_mat(1, 256, CV_8UC3);
            for (int i = 0; i < 256; ++i) {
              lut_mat.at<cv::Vec3b>(0, i) = lut[i];
            }
            cv::Mat colored;
            cv::LUT(mask, lut_mat, colored);
            cv::Mat nonzero = (mask != 0);
            colored.copyTo(overlay, nonzero);
          } else {
            // 互換: 二値/その他のマスクは従来ロジック（重い）
            if (mask.type() == CV_8UC1) {
              for (int y = 0; y < mask.rows; ++y) {
                const uint8_t* mr = mask.ptr<uint8_t>(y);
                cv::Vec3b* ov = overlay.ptr<cv::Vec3b>(y);
                for (int x = 0; x < mask.cols; ++x) {
                  uint8_t m = mr[x];
                  if (m != 0) { ov[x] = lut[m]; }
                }
              }
              if (last_dets_) {
                for (const auto &d : last_dets_->detections) {
                  int x1 = std::max(0, (int)std::floor(d.bbox_min.x));
                  int y1 = std::max(0, (int)std::floor(d.bbox_min.y));
                  int x2 = std::min(view.cols, (int)std::ceil(d.bbox_max.x));
                  int y2 = std::min(view.rows, (int)std::ceil(d.bbox_max.y));
                  if (x2 <= x1 || y2 <= y1) continue;
                  cv::Rect roi(x1,y1,x2-x1,y2-y1);
                  cv::Vec3b ic = colorForDetId(d.id);
                  for (int yy = roi.y; yy < roi.y + roi.height; ++yy) {
                    const uint8_t* mr = mask.ptr<uint8_t>(yy);
                    cv::Vec3b* ov = overlay.ptr<cv::Vec3b>(yy);
                    for (int xx = roi.x; xx < roi.x + roi.width; ++xx) {
                      if (mr[xx] != 0) ov[xx] = ic;
                    }
                  }
                }
              }
            } else {
              for (int y = 0; y < mask.rows; ++y) {
                const uint16_t* mr = mask.ptr<uint16_t>(y);
                cv::Vec3b* ov = overlay.ptr<cv::Vec3b>(y);
                for (int x = 0; x < mask.cols; ++x) {
                  uint16_t m = mr[x];
                  if ((m & 0xFF) != 0) { ov[x] = lut[(m & 0xFF)]; }
                }
              }
            }
          }

          double a = std::clamp(mask_alpha_, 0.0, 1.0);
          cv::addWeighted(overlay, a, view, 1.0 - a, 0.0, view);
        } catch (...) {}
      }
    }

    // Draw per-bbox JP title only when label/conf requested
    if ((draw_labels_ || draw_confidence_) && last_dets_) {
      for (size_t idx = 0; idx < last_dets_->detections.size(); ++idx) {
        const auto &d = last_dets_->detections[idx];
        int x1 = static_cast<int>(std::floor(d.bbox_min.x));
        int y1 = static_cast<int>(std::floor(d.bbox_min.y));
        int x2 = static_cast<int>(std::ceil(d.bbox_max.x));
        int y2 = static_cast<int>(std::ceil(d.bbox_max.y));
        x1 = std::clamp(x1, 0, std::max(0, view.cols-1));
        y1 = std::clamp(y1, 0, std::max(0, view.rows-1));
        x2 = std::clamp(x2, 0, std::max(0, view.cols-1));
        y2 = std::clamp(y2, 0, std::max(0, view.rows-1));
        cv::Rect bb(cv::Point(x1,y1), cv::Point(x2,y2));
        if (!passDisplayFilter(bb)) continue;
        int title_y = std::max(0, y1 - 6);
        int perc = 0;
        if (idx < last_dets_->detections.size()) {
          const auto &dd = last_dets_->detections[idx];
          double p = std::max(0.0, std::min(1.0, static_cast<double>(dd.conf_fused)));
          perc = static_cast<int>(std::round(p * 100.0));
        }
        char title[64];
        std::snprintf(title, sizeof(title), "アスパラ#%d(%d%%)", d.id, perc);
        fluent::text::drawShadow(view, std::string(title), cv::Point(x1 + 2, title_y),
                                 cv::Scalar(255,255,255), cv::Scalar(0,0,0), 0.6, 2, 0);
      }
    }

    // draw fixed selection box if requested
    if (draw_selection_box_ && sel_mode_ == "fixed") {
      cv::Rect fixed = makeFixedRect(view.cols, view.rows);
      cv::rectangle(view, fixed, cv::Scalar(0, 255, 255), 2);
    }

    // draw invalid area overlay last
    if (draw_invalid_area_ && invalid_enabled_) {
      cv::Mat overlay = view.clone();
      cv::rectangle(overlay, invalid_rect, cv::Scalar(0, 0, 255), cv::FILLED);
      double alpha = 0.25; // 25% red tint
      cv::addWeighted(overlay, alpha, view, 1.0 - alpha, 0.0, view);
      // border
      cv::rectangle(view, invalid_rect, cv::Scalar(0, 0, 255), 2);
    }

    // metrics: 表示対象の検出枠(id)が存在するものだけ描画
    if (draw_metrics_ && last_metrics_ && last_dets_ && !last_dets_->detections.empty()) {
      std::unordered_set<int> valid_ids;
      for (const auto &d : last_dets_->detections) {
        int x1 = static_cast<int>(std::floor(d.bbox_min.x));
        int y1 = static_cast<int>(std::floor(d.bbox_min.y));
        int x2 = static_cast<int>(std::ceil(d.bbox_max.x));
        int y2 = static_cast<int>(std::ceil(d.bbox_max.y));
        x1 = std::clamp(x1, 0, std::max(0, view.cols-1));
        y1 = std::clamp(y1, 0, std::max(0, view.rows-1));
        x2 = std::clamp(x2, 0, std::max(0, view.cols-1));
        y2 = std::clamp(y2, 0, std::max(0, view.rows-1));
        cv::Rect bb(cv::Point(x1,y1), cv::Point(x2,y2));
        if (passDisplayFilter(bb)) valid_ids.insert(d.id);
        else if (metrics_draw_even_if_filtered_) valid_ids.insert(d.id);
      }
      if (!valid_ids.empty()) {
        for (const auto &m : last_metrics_->stalks) {
          if (valid_ids.find(m.id) == valid_ids.end()) continue;

          cv::Point mid_pt(10, 20);
          if (validIntrinsics()) {
            geometry_msgs::msg::Point root_p = m.root_camera;
            geometry_msgs::msg::Point tip_p = m.tip_camera;
            if (proj_transform_to_cinfo_) {
              root_p = transformToCinfo(root_p, m.header.frame_id, msg->header.stamp);
              tip_p  = transformToCinfo(tip_p,  m.header.frame_id, msg->header.stamp);
            }
            if (root_p.z > 0.0 && tip_p.z > 0.0) {
              auto uv0 = project(root_p.x, root_p.y, root_p.z);
              auto uv1 = project(tip_p.x, tip_p.y, tip_p.z);
              cv::Point p0((int)uv0.first,(int)uv0.second);
              cv::Point p1((int)uv1.first,(int)uv1.second);
              mid_pt = cv::Point((p0.x + p1.x)/2, (p0.y + p1.y)/2);
            } else if (tip_p.z > 0.0) {
              auto uv = project(tip_p.x, tip_p.y, tip_p.z);
              mid_pt = cv::Point((int)uv.first,(int)uv.second);
            }
          }

          char lenjp[64];
          std::snprintf(lenjp, sizeof(lenjp), "長さ %.1f %s", m.length_m * length_scale_, length_unit_.c_str());
          fluent::text::drawShadow(view, std::string(lenjp), mid_pt + cv::Point(8,-8),
                                   cv::Scalar(255,255,255), cv::Scalar(0,0,0), 0.7, 2, 0);
          char diajp[64];
          std::snprintf(diajp, sizeof(diajp), "直径 %.1f %s", m.thickness_m * diameter_scale_, diameter_unit_.c_str());
          fluent::text::drawShadow(view, std::string(diajp), mid_pt + cv::Point(8,14),
                                   cv::Scalar(200,255,200), cv::Scalar(0,0,0), 0.6, 1, 0);
          // 曲げ角度を表示
          double bend_angle = computeBendAngle(m.root_camera, m.tip_camera);
          char anglejp[64];
          std::snprintf(anglejp, sizeof(anglejp), "角度 %.1f°", bend_angle);
          fluent::text::drawShadow(view, std::string(anglejp), mid_pt + cv::Point(8,36),
                                   cv::Scalar(255,200,200), cv::Scalar(0,0,0), 0.6, 1, 0);

          if (draw_root_tip_ && validIntrinsics()) {
            geometry_msgs::msg::Point root_p = m.root_camera;
            geometry_msgs::msg::Point tip_p = m.tip_camera;
            if (proj_transform_to_cinfo_) {
              root_p = transformToCinfo(root_p, m.header.frame_id, msg->header.stamp);
              tip_p  = transformToCinfo(tip_p,  m.header.frame_id, msg->header.stamp);
            }
            if (root_p.z > 0.0) {
              auto uv0 = project(root_p.x, root_p.y, root_p.z);
              cv::Point p0((int)uv0.first,(int)uv0.second);
              cv::circle(view, p0, 4, cv::Scalar(0,0,255), -1);
              if (selected_id == m.id) cv::circle(view, p0, 7, cv::Scalar(0,255,255), 2);
              if (draw_root_xyz_) {
                char xyz[128];
                std::snprintf(xyz, sizeof(xyz), "x=%.1f y=%.1f z=%.1f cm",
                              (double)root_p.x*100.0, (double)root_p.y*100.0, (double)root_p.z*100.0);
                cv::Point torg = p0 + cv::Point(8, -6);
                fluent::text::drawShadow(view, std::string(xyz), torg,
                                         cv::Scalar(255,255,255), cv::Scalar(0,0,0), 0.5, 1, 0);
              }
            }
            if (tip_p.z > 0.0) {
              auto uv1 = project(tip_p.x, tip_p.y, tip_p.z);
              cv::circle(view, cv::Point((int)uv1.first,(int)uv1.second), 4, cv::Scalar(255,0,0), -1);
              if (root_p.z > 0.0) {
                // オレンジ色のroot-tip直線は削除（骨格点で表現）

                // スケルトン描画: skeleton_pointsがあれば実際の形状、なければ直線補間
                if (draw_skeleton_) {
                  if (!m.skeleton_points.empty()) {
                    // 半径（直径の半分）を渡して円柱輪郭も描画
                    double radius_m = m.thickness_m * 0.5;
                    drawSkeletonFromPoints(view, m.skeleton_points, radius_m);
                  } else {
                    drawSkeleton(view, root_p, tip_p);
                  }
                }
              }
            }

            // Selected top-left info and TF broadcast
            if (m.id == selected_id) {
              // Compose text block (small font)
              if (selinfo_enable_) {
                int x = selinfo_offx_, y = selinfo_offy_;
                char ln1[128]; std::snprintf(ln1, sizeof(ln1), "ID:%d conf=%.2f", m.id, std::max(0.0, std::min(1.0, (double)selected_conf)));
                fluent::text::drawShadow(view, ln1, cv::Point(x,y), cv::Scalar(255,255,255), cv::Scalar(0,0,0), selinfo_font_scale_, selinfo_thickness_, 0);
                y += 14;
                auto uv0 = project(root_p.x, root_p.y, root_p.z);
                auto uv1 = project(tip_p.x, tip_p.y, tip_p.z);
                char ln2[128]; std::snprintf(ln2, sizeof(ln2), "2D root(%.0f,%.0f) tip(%.0f,%.0f)", uv0.first, uv0.second, uv1.first, uv1.second);
                fluent::text::drawShadow(view, ln2, cv::Point(x,y), cv::Scalar(230,230,230), cv::Scalar(0,0,0), selinfo_font_scale_, selinfo_thickness_, 0);
                y += 14;
                char ln3[128]; std::snprintf(ln3, sizeof(ln3), "3D root(%.1f,%.1f,%.1f)cm", root_p.x*100.0, root_p.y*100.0, root_p.z*100.0);
                fluent::text::drawShadow(view, ln3, cv::Point(x,y), cv::Scalar(200,255,200), cv::Scalar(0,0,0), selinfo_font_scale_, selinfo_thickness_, 0);
                y += 14;
                char ln4[128];
                std::snprintf(ln4, sizeof(ln4), "長さ %.1f %s 直径 %.1f %s",
                              m.length_m * length_scale_, length_unit_.c_str(),
                              m.thickness_m * diameter_scale_, diameter_unit_.c_str());
                fluent::text::drawShadow(view, ln4, cv::Point(x,y),
                                         cv::Scalar(255,255,255), cv::Scalar(0,0,0),
                                         selinfo_font_scale_, selinfo_thickness_, 0);
                y += 14;
                double bend_angle = computeBendAngle(root_p, tip_p);
                char ln5[128];
                std::snprintf(ln5, sizeof(ln5), "曲げ角度 %.1f°", bend_angle);
                fluent::text::drawShadow(view, ln5, cv::Point(x,y),
                                         cv::Scalar(255,200,200), cv::Scalar(0,0,0),
                                         selinfo_font_scale_, selinfo_thickness_, 0);
              }

              if (seltf_enable_ && tf_broadcaster_) {
                geometry_msgs::msg::TransformStamped ts;
                ts.header.stamp = msg->header.stamp;
                ts.header.frame_id = cinfo_frame_.empty() ? m.header.frame_id : cinfo_frame_;
                ts.child_frame_id = seltf_child_;
                ts.transform.translation.x = root_p.x;
                ts.transform.translation.y = root_p.y;
                ts.transform.translation.z = root_p.z;
                // Orientation: 簡易に恒等（必要あれば後で厳密対応）
                ts.transform.rotation.w = 1.0;
                ts.transform.rotation.x = 0.0;
                ts.transform.rotation.y = 0.0;
                ts.transform.rotation.z = 0.0;
                tf_broadcaster_->sendTransform(ts);
              }
            }
          }
        }
      }
    }

    // ここまでの入れ子ブロックで括弧が1つ閉じ忘れられていたため補完
    // （onImage関数のスコープを正しく閉じるための修正）
    }

    // no global title; each bbox has its own JP title

    auto publish_start = std::chrono::steady_clock::now();
    auto out = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, view).toImageMsg();
    pub_anno_->publish(*out);
    auto publish_end = std::chrono::steady_clock::now();

    auto callback_end = std::chrono::steady_clock::now();
    double publish_ms = std::chrono::duration<double, std::milli>(publish_end - publish_start).count();
    double total_ms = std::chrono::duration<double, std::milli>(callback_end - callback_start).count();

    if (publish_ms > 10.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "⚠️ UI publish took %.1fms", publish_ms);
    }
    if (total_ms > 50.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "⚠️ UI total processing took %.1fms", total_ms);
    }

    static int frame_count = 0;
    if (++frame_count % 50 == 0) {
      RCLCPP_INFO(get_logger(), "🎨 UI Performance: Total=%.1fms (%.1f fps)", total_ms, 1000.0 / total_ms);
    }
  }

  bool validIntrinsics() const { return fx_ > 0.0 && fy_ > 0.0; }

  // 曲げ角度を計算: X方向（左右）の傾きのみ（Z方向の奥行きは除外）
  static double computeBendAngle(const geometry_msgs::msg::Point &root, const geometry_msgs::msg::Point &tip) {
    double dx = tip.x - root.x;
    double dy = tip.y - root.y;
    double dz = tip.z - root.z;
    if (std::abs(dy) < 1e-6) return 0.0;
    // カメラ座標系: X軸が左右、Y軸が上下、Z軸が奥行き
    // 左右方向（X）の曲がりのみを計算（Z方向の遠近は曲がりではないので除外）
    double horizontal_x = std::abs(dx);  // X方向（左右）の距離のみ
    double vertical = std::abs(dy);      // 垂直方向の距離
    double angle_rad = std::atan2(horizontal_x, vertical);
    return angle_rad * 180.0 / M_PI;
  }
  geometry_msgs::msg::Point transformToCinfo(const geometry_msgs::msg::Point &p, const std::string &from_frame, const rclcpp::Time &stamp) {
    geometry_msgs::msg::Point out = p;
    if (!proj_transform_to_cinfo_) return out;
    if (cinfo_frame_.empty() || from_frame.empty() || from_frame == cinfo_frame_) return out;
    try {
      geometry_msgs::msg::PointStamped ps, tr;
      ps.header.stamp = stamp;
      ps.header.frame_id = from_frame;
      ps.point = p;
      auto tf = tf_buffer_->lookupTransform(cinfo_frame_, from_frame, stamp, rclcpp::Duration::from_seconds(0.05));
      tf2::doTransform(ps, tr, tf);
      return tr.point;
    } catch (...) {
      return out;
    }
  }
  std::pair<double,double> project(double x, double y, double z) const {
    if (z <= 0.0 || !validIntrinsics()) return {0.0, 0.0};
    double u = cx_ + fx_ * (x / z);
    double v = cy_ + fy_ * (y / z);
    return {u, v};
  }

  // スケルトン描画: root→tip間を等間隔にサンプリングして色付き点列を描画（直線補間版）
  // 根本=赤(255,0,0)、先端=緑(0,255,0)のグラデーション
  void drawSkeleton(cv::Mat &img, const geometry_msgs::msg::Point &root, const geometry_msgs::msg::Point &tip) {
    if (!validIntrinsics()) return;
    int n = skeleton_num_points_;
    if (n < 2) n = 2;
    for (int i = 0; i < n; ++i) {
      double t = static_cast<double>(i) / static_cast<double>(n - 1);  // 0.0 〜 1.0
      // 3D点を補間
      double px = root.x + t * (tip.x - root.x);
      double py = root.y + t * (tip.y - root.y);
      double pz = root.z + t * (tip.z - root.z);
      if (pz <= 0.0) continue;
      // 2Dに投影
      auto uv = project(px, py, pz);
      cv::Point pt(static_cast<int>(uv.first), static_cast<int>(uv.second));
      // 画面外チェック
      if (pt.x < 0 || pt.x >= img.cols || pt.y < 0 || pt.y >= img.rows) continue;
      // グラデーション色: BGR形式 root=赤(0,0,255) → tip=緑(0,255,0)
      int b = 0;
      int g = static_cast<int>(255.0 * t);
      int r = static_cast<int>(255.0 * (1.0 - t));
      cv::Scalar color(b, g, r);
      cv::circle(img, pt, skeleton_point_radius_, color, -1);
      // 外枠（視認性向上）
      cv::circle(img, pt, skeleton_point_radius_, cv::Scalar(0, 0, 0), 1);
    }
  }

  // 実際の骨格点列から描画（曲がりを反映）
  void drawSkeletonFromPoints(cv::Mat &img, const std::vector<geometry_msgs::msg::Point> &points, double radius_m = 0.0) {
    if (!validIntrinsics() || points.empty()) return;
    size_t n = points.size();
    std::vector<cv::Point> pts2d;
    std::vector<geometry_msgs::msg::Point> pts3d_valid;
    pts2d.reserve(n);
    pts3d_valid.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      const auto &p = points[i];
      if (p.z <= 0.0) continue;
      auto uv = project(p.x, p.y, p.z);
      cv::Point pt(static_cast<int>(uv.first), static_cast<int>(uv.second));
      if (pt.x < 0 || pt.x >= img.cols || pt.y < 0 || pt.y >= img.rows) continue;
      pts2d.push_back(pt);
      pts3d_valid.push_back(p);
    }
    if (pts2d.empty()) return;

    // 円柱の輪郭線を描画（半径情報がある場合）
    if (radius_m > 0.0 && pts2d.size() >= 2) {
      std::vector<cv::Point> left_contour, right_contour;
      left_contour.reserve(pts2d.size());
      right_contour.reserve(pts2d.size());

      for (size_t i = 0; i < pts2d.size(); ++i) {
        // セグメントの方向ベクトルを計算
        cv::Point2f dir;
        if (i == 0) {
          // 最初の点: 次の点との方向
          dir = cv::Point2f(pts2d[1].x - pts2d[0].x, pts2d[1].y - pts2d[0].y);
        } else if (i == pts2d.size() - 1) {
          // 最後の点: 前の点との方向
          dir = cv::Point2f(pts2d[i].x - pts2d[i-1].x, pts2d[i].y - pts2d[i-1].y);
        } else {
          // 中間点: 前後の平均方向
          cv::Point2f dir1(pts2d[i].x - pts2d[i-1].x, pts2d[i].y - pts2d[i-1].y);
          cv::Point2f dir2(pts2d[i+1].x - pts2d[i].x, pts2d[i+1].y - pts2d[i+1].y);
          dir = cv::Point2f((dir1.x + dir2.x) * 0.5f, (dir1.y + dir2.y) * 0.5f);
        }

        // 正規化
        float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
        if (len > 1e-6f) {
          dir.x /= len;
          dir.y /= len;
        }

        // 垂直ベクトル（右回り90度回転）
        cv::Point2f perp(-dir.y, dir.x);

        // 3D半径を2D画像上のピクセル半径に変換
        const auto &p3d = pts3d_valid[i];
        // 半径をカメラ座標系のX方向オフセットとして扱い、投影
        auto uv_center = project(p3d.x, p3d.y, p3d.z);
        auto uv_right = project(p3d.x + radius_m, p3d.y, p3d.z);
        float pixel_radius = static_cast<float>(std::abs(uv_right.first - uv_center.first));

        // 左右の輪郭点
        cv::Point left_pt(
          static_cast<int>(pts2d[i].x + perp.x * pixel_radius),
          static_cast<int>(pts2d[i].y + perp.y * pixel_radius)
        );
        cv::Point right_pt(
          static_cast<int>(pts2d[i].x - perp.x * pixel_radius),
          static_cast<int>(pts2d[i].y - perp.y * pixel_radius)
        );
        left_contour.push_back(left_pt);
        right_contour.push_back(right_pt);
      }

      // 円柱の輪郭線を白い1ピクセルラインで描画
      for (size_t i = 1; i < left_contour.size(); ++i) {
        cv::line(img, left_contour[i-1], left_contour[i], cv::Scalar(255, 255, 255), 1);
        cv::line(img, right_contour[i-1], right_contour[i], cv::Scalar(255, 255, 255), 1);
      }
    }

    // 中心線を描画（骨格線）
    for (size_t i = 1; i < pts2d.size(); ++i) {
      double t = static_cast<double>(i) / static_cast<double>(pts2d.size() - 1);
      int b = 0;
      int g = static_cast<int>(255.0 * t);
      int r = static_cast<int>(255.0 * (1.0 - t));
      cv::line(img, pts2d[i-1], pts2d[i], cv::Scalar(b, g, r), 2);
    }
    // 点を描画
    for (size_t i = 0; i < pts2d.size(); ++i) {
      double t = static_cast<double>(i) / static_cast<double>(pts2d.size() > 1 ? pts2d.size() - 1 : 1);
      int b = 0;
      int g = static_cast<int>(255.0 * t);
      int r = static_cast<int>(255.0 * (1.0 - t));
      cv::Scalar color(b, g, r);
      cv::circle(img, pts2d[i], skeleton_point_radius_, color, -1);
      cv::circle(img, pts2d[i], skeleton_point_radius_, cv::Scalar(0, 0, 0), 1);
    }
  }

  cv::Rect makeInvalidRect(int width, int height) const {
    int w = std::max(1, static_cast<int>(std::round(width * ia_wr_)));
    int h = std::max(1, static_cast<int>(std::round(height * ia_hr_)));
    int cx = static_cast<int>(std::round(width * ia_cx_));
    int x0 = std::clamp(cx - w/2, 0, std::max(0, width - 1));
    int y0 = std::clamp(static_cast<int>(std::round(height * ia_yt_)), 0, std::max(0, height - 1));
    int x1 = std::clamp(x0 + w, 0, width);
    int y1 = std::clamp(y0 + h, 0, height);
    return cv::Rect(cv::Point(x0, y0), cv::Point(x1, y1));
  }

  cv::Rect makeFixedRect(int width, int height) const {
    int w = std::max(1, static_cast<int>(std::round(width * sel_fix_wr_)));
    int h = std::max(1, static_cast<int>(std::round(height * sel_fix_hr_)));
    int cx = static_cast<int>(std::round(width * sel_fix_xc_));
    int x0 = std::clamp(cx - w/2, 0, std::max(0, width - 1));
    int y0 = std::clamp(sel_fix_yt_px_, 0, std::max(0, height - 1));
    int x1 = std::clamp(x0 + w, 0, width);
    int y1 = std::clamp(y0 + h, 0, height);
    return cv::Rect(cv::Point(x0, y0), cv::Point(x1, y1));
  }

  static double aspectRatio(const cv::Rect &r) {
    if (r.height <= 0) return 0.0;
    return static_cast<double>(r.width) / static_cast<double>(r.height);
  }

  bool passDisplayFilter(const cv::Rect &bb) const {
    if (!disp_filter_enable_) return true;
    if (bb.width < disp_min_w_ || bb.height < disp_min_h_) return false;
    double ar = aspectRatio(bb);
    if (ar < disp_min_ar_ || ar > disp_max_ar_) return false;
    if (disp_only_vertical_ && bb.width > bb.height) return false;
    return true;
  }

  int pickSelectedIndex(const fv_msgs::msg::DetectionArray &arr, const std::string &strategy, int width, int height, bool ignore_filter=false) const {
    if (arr.detections.empty()) return -1;
    int best = 0; double best_val = -1e9;
    for (size_t i = 0; i < arr.detections.size(); ++i) {
      const auto &d = arr.detections[i];
      int x1 = static_cast<int>(std::floor(d.bbox_min.x));
      int y1 = static_cast<int>(std::floor(d.bbox_min.y));
      int x2 = static_cast<int>(std::ceil(d.bbox_max.x));
      int y2 = static_cast<int>(std::ceil(d.bbox_max.y));
      x1 = std::clamp(x1, 0, std::max(0, width-1));
      y1 = std::clamp(y1, 0, std::max(0, height-1));
      x2 = std::clamp(x2, 0, std::max(0, width-1));
      y2 = std::clamp(y2, 0, std::max(0, height-1));
      cv::Rect bb(cv::Point(x1,y1), cv::Point(x2,y2));
      if (!ignore_filter && !passDisplayFilter(bb)) continue;
      double val = 0.0;
      if (strategy == "largest_height") {
        val = static_cast<double>(bb.height);
      } else { // highest_conf (default)
        val = static_cast<double>(d.conf_fused);
      }
      if (val > best_val) { best_val = val; best = static_cast<int>(i); }
    }
    return best;
  }

  // params
  std::string image_topic_;
  std::string detections_topic_;
  std::string metrics_topic_;
  std::string camera_info_topic_;
  std::string annotated_topic_;
  std::string image_qos_reliability_;
  bool draw_labels_{true};
  bool draw_confidence_{true};
  bool draw_metrics_{true};
  bool draw_root_tip_{true};
  bool draw_root_xyz_{false};
  bool draw_skeleton_{true};
  int skeleton_num_points_{10};
  int skeleton_point_radius_{3};
  bool draw_invalid_area_{true};
  bool invalid_enabled_{false};
  bool invalid_any_overlap_{true};
  bool invalid_touch_inclusive_{true};
  double ia_cx_{0.5}, ia_yt_{0.5}, ia_wr_{0.5}, ia_hr_{0.5};
  bool draw_selection_box_{true};
  std::string sel_mode_{"auto"};
  std::string sel_auto_strategy_{"highest_conf"};
  double sel_fix_xc_{0.5};
  int sel_fix_yt_px_{5};
  double sel_fix_wr_{0.20}, sel_fix_hr_{0.50};
  int sel_hold_frames_{5};
  double sel_switch_margin_{0.05};
  int sel_lock_frames_{10};
  int lock_left_{0};
  int last_selected_id_{-1};
  int hold_count_{0};
  double last_selected_conf_{0.0};
  bool disp_filter_enable_{true};
  bool disp_only_vertical_{false};
  double disp_min_ar_{0.05}, disp_max_ar_{2.0};
  int disp_min_w_{1}, disp_min_h_{1};
  bool sel_ignore_filter_fallback_{true};
  bool metrics_draw_even_if_filtered_{true};
  bool disp_draw_filtered_boxes_{true};
  bool overlay_only_selected_{false};
  double overlay_min_conf_draw_{0.0};
  double output_max_fps_{0.0};

  // subs/pubs
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
  rclcpp::Subscription<fv_msgs::msg::DetectionArray>::SharedPtr sub_det_;
  rclcpp::Subscription<fv_msgs::msg::StalkMetricsArray>::SharedPtr sub_metrics_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_mask_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_cinfo_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_anno_;

  // cache
  fv_msgs::msg::DetectionArray::SharedPtr last_dets_;
  fv_msgs::msg::StalkMetricsArray::SharedPtr last_metrics_;
  sensor_msgs::msg::Image::SharedPtr last_mask_;
  double fx_{0.0}, fy_{0.0}, cx_{0.0}, cy_{0.0};
  std::string cinfo_frame_;
  bool proj_transform_to_cinfo_{true};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // mask params
  std::string mask_topic_;
  bool mask_is_instance_id_{false};
  int mask_threshold_{128};
  double mask_alpha_{0.3};
  int mask_color_bgr_[3]{0,255,0};
  std::vector<int> id_palette_bgr_;
  std::vector<cv::Vec3b> id_palette_colors_;

  bool draw_title_{true};
  cv::Scalar sel_box_color_{0,255,0};
  cv::Scalar unsel_box_color_{0,165,255};
  // units
  std::string length_unit_{"cm"};
  std::string diameter_unit_{"mm"};
  double length_scale_{100.0};   // m -> cm
  double diameter_scale_{1000.0}; // m -> mm
  // selected info/TF
  bool selinfo_enable_{true};
  double selinfo_font_scale_{0.5};
  int selinfo_thickness_{1};
  int selinfo_offx_{8}, selinfo_offy_{14};
  bool seltf_enable_{true};
  std::string seltf_child_{"aspara_selected"};

  // services / manual selection
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_get_sel_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_next_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_prev_;
  bool manual_active_{false};
  int manual_selected_id_{-1};

  void onMask(const sensor_msgs::msg::Image::SharedPtr msg) {
    // 強参照で保持し、onImage側で確実に利用できるようにする
    last_mask_ = msg;
  }

  void rebuildPaletteColors() {
    id_palette_colors_.clear();
    for (size_t i = 0; i + 2 < id_palette_bgr_.size(); i += 3) {
      int b = std::clamp(id_palette_bgr_[i+0], 0, 255);
      int g = std::clamp(id_palette_bgr_[i+1], 0, 255);
      int r = std::clamp(id_palette_bgr_[i+2], 0, 255);
      id_palette_colors_.emplace_back((uchar)b,(uchar)g,(uchar)r);
    }
    if (id_palette_colors_.empty()) {
      id_palette_colors_.push_back(cv::Vec3b(0,255,0));
    }
  }

  void rebuildUnitScales() {
    auto to_scale = [](const std::string &u)->double{
      std::string s=u; for (auto &c: s) c=static_cast<char>(::tolower(c));
      if (s=="m") return 1.0; if (s=="cm") return 100.0; if (s=="mm") return 1000.0; return 100.0;
    };
    length_scale_ = to_scale(length_unit_);
    diameter_scale_ = to_scale(diameter_unit_);
  }

  cv::Vec3b colorForDetId(int id) const {
    if (id_palette_colors_.empty()) return cv::Vec3b(0,255,0);
    size_t idx = static_cast<size_t>(std::abs(id)) % id_palette_colors_.size();
    return id_palette_colors_[idx];
  }

  void onSrvGetSelected(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    int id = manual_active_ ? manual_selected_id_ : last_selected_id_;
    if (id < 0 || !last_dets_) {
      res->success = false;
      res->message = std::string("no selection");
      return;
    }
    const fv_msgs::msg::Detection2D* det_ptr = nullptr;
    for (const auto &d : last_dets_->detections) { if (d.id == id) { det_ptr = &d; break; } }
    if (!det_ptr) {
      res->success = false;
      res->message = std::string("selection not in current detections");
      return;
    }
    const fv_msgs::msg::Detection2D &d = *det_ptr;
    double x1 = d.bbox_min.x, y1 = d.bbox_min.y, x2 = d.bbox_max.x, y2 = d.bbox_max.y;
    double conf = static_cast<double>(d.conf_fused);
    double length_m = -1.0, thickness_m = -1.0;
    geometry_msgs::msg::Point root, tip;
    std::string frame = cinfo_frame_;
    if (last_metrics_) {
      for (const auto &m : last_metrics_->stalks) {
        if (m.id == id) {
          length_m = m.length_m; thickness_m = m.thickness_m;
          root = m.root_camera; tip = m.tip_camera;
          if (frame.empty()) frame = m.header.frame_id;
          break;
        }
      }
    }
    char buf[1024];
    std::snprintf(buf, sizeof(buf),
      "{\"id\":%d,\"conf\":%.3f,\"bbox\":{\"x1\":%.1f,\"y1\":%.1f,\"x2\":%.1f,\"y2\":%.1f},\"length_m\":%.4f,\"thickness_m\":%.4f,\"root_camera\":{\"x\":%.4f,\"y\":%.4f,\"z\":%.4f},\"tip_camera\":{\"x\":%.4f,\"y\":%.4f,\"z\":%.4f},\"frame_id\":\"%s\"}",
      id, conf, x1,y1,x2,y2, length_m, thickness_m, root.x,root.y,root.z, tip.x,tip.y,tip.z, frame.c_str());
    res->success = true;
    res->message = std::string(buf);
  }

  static std::vector<int> listCurrentIds(const fv_msgs::msg::DetectionArray::SharedPtr &arr) {
    std::vector<int> ids;
    if (!arr) return ids;
    ids.reserve(arr->detections.size());
    for (const auto &d : arr->detections) ids.push_back(d.id);
    return ids;
  }

  void onSrvNext(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    if (!last_dets_ || last_dets_->detections.empty()) {
      res->success = false; res->message = "no detections"; return;
    }
    auto ids = listCurrentIds(last_dets_);
    if (ids.empty()) { res->success = false; res->message = "no detections"; return; }
    int base = manual_active_ ? manual_selected_id_ : last_selected_id_;
    int pos = -1;
    for (size_t i = 0; i < ids.size(); ++i) if (ids[i] == base) { pos = static_cast<int>(i); break; }
    int next_pos = (pos < 0) ? 0 : (pos + 1) % static_cast<int>(ids.size());
    manual_selected_id_ = ids[static_cast<size_t>(next_pos)];
    manual_active_ = true;
    res->success = true;
    res->message = std::string("selected_id=") + std::to_string(manual_selected_id_);
  }

  void onSrvPrev(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    if (!last_dets_ || last_dets_->detections.empty()) {
      res->success = false; res->message = "no detections"; return;
    }
    auto ids = listCurrentIds(last_dets_);
    if (ids.empty()) { res->success = false; res->message = "no detections"; return; }
    int base = manual_active_ ? manual_selected_id_ : last_selected_id_;
    int pos = -1;
    for (size_t i = 0; i < ids.size(); ++i) if (ids[i] == base) { pos = static_cast<int>(i); break; }
    int prev_pos;
    if (pos < 0) prev_pos = static_cast<int>(ids.size()) - 1;
    else prev_pos = (pos - 1 + static_cast<int>(ids.size())) % static_cast<int>(ids.size());
    manual_selected_id_ = ids[static_cast<size_t>(prev_pos)];
    manual_active_ = true;
    res->success = true;
    res->message = std::string("selected_id=") + std::to_string(manual_selected_id_);
  }
};

// 保守: まれな括弧不整合対策として名前空間終端直前に閉じ括弧を明示
// （クラス/関数の括弧は上で閉じられている）

} // namespace fv_aspara_ui_cpp

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fv_aspara_ui_cpp::AsparaUiCppNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
