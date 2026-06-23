#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <fv_msgs/msg/detection_array.hpp>
#include <opencv2/core.hpp>
#include <memory>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <cstdint>

#include "fv_instance_seg/backends/inferencer.hpp"

namespace fv_instance_seg {

class InstanceSegNode : public rclcpp::Node {
 public:
  explicit InstanceSegNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~InstanceSegNode();

 private:
  struct TrackState {
    int id = 0;
    cv::Rect bbox;
    cv::Mat mask;
    float score = 0.f;
    int cls = 0;
    cv::Point2f center;
    int misses = 0;
    bool active = false;
    cv::Scalar color{0, 255, 0};
    rclcpp::Time first_seen;
    rclcpp::Time last_seen;
    int age_frames = 0;
  };

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void publishOverlay(const cv::Mat& bgr, const std_msgs::msg::Header& header);
  void publishMask(const cv::Mat& mask_mono, const std_msgs::msg::Header& header);
  void publishDetections(const std::vector<TrackState>& tracks, const std_msgs::msg::Header& header);
  void updateTracking(const InferResult& res, const rclcpp::Time& stamp);
  bool loadModel(const std::string& backend,
                 const std::string& model_path,
                 const std::string& device,
                 const std::string& fallback_device,
                 bool nms_class_agnostic,
                 int max_detections,
                 bool debug_shapes,
                 int infer_timeout_ms);
  rcl_interfaces::msg::SetParametersResult onParametersSet(
      const std::vector<rclcpp::Parameter>& parameters);

  cv::Scalar nextColor();
  void updateStats(double inference_ms, double total_ms, std::size_t detection_count);
  void drawStats(cv::Mat& image);

  // 非同期描画スレッド
  void overlayWorker();
  void enqueueOverlay(const sensor_msgs::msg::Image::ConstSharedPtr& msg,
                      const std::vector<TrackState>& tracks,
                      const std_msgs::msg::Header& header);


  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr id_mask_pub_;
  rclcpp::Publisher<fv_msgs::msg::DetectionArray>::SharedPtr fv_dets_pub_;

  std::unique_ptr<Inferencer> inferencer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  std::mutex inferencer_mutex_;

  std::string backend_;
  std::string model_path_;
  std::string device_;
  std::string fallback_device_;
  std::string input_image_topic_;
  // Class index -> name. The TRT .engine carries no names metadata (the .onnx
  // does, but trtexec drops it), so the operator supplies them via the
  // `class_names` param (profile / settings UI). Empty -> label left blank.
  std::vector<std::string> class_names_;
  std::string labelForClass(int cls) const {
    return (cls >= 0 && cls < static_cast<int>(class_names_.size()))
               ? class_names_[cls]
               : std::string();
  }
  // Read `<model_path stem>.names` (one class per line) next to the model.
  // Empty vector when the sidecar is absent/empty (caller keeps its fallback).
  std::vector<std::string> loadNamesSidecar(const std::string& model_path) const;
  // Post-inference false-detection filter. All thresholds default to 0
  // (disabled) so the filter is opt-in via params / the settings UI.
  double min_box_area_px_ = 0.0;
  double max_box_area_px_ = 0.0;
  double min_aspect_ = 0.0;     // w/h lower bound
  double max_aspect_ = 0.0;     // w/h upper bound
  double min_mask_fill_ = 0.0;  // countNonZero(mask) / mask.total()
  void filterDetections(InferResult& res) const;
  double conf_thres_;
  double iou_thres_;
  bool publish_detections_;
  bool publish_overlay_;
  bool nms_class_agnostic_ = true;
  int max_detections_ = 100;
  bool debug_shapes_ = false;
  int infer_timeout_ms_ = 0;

  std::vector<TrackState> tracks_;
  int next_track_id_ = 1;
  int hold_frames_ = 3;
  int drop_frames_ = 10;
  double match_distance_px_ = 80.0;
  std::vector<cv::Scalar> palette_;
  std::size_t palette_index_ = 0;

  double stats_fps_ = 0.0;
  double stats_inference_ms_ = 0.0;
  double stats_total_ms_ = 0.0;
  std::size_t stats_detection_count_ = 0;

  // FPS制限
  double max_fps_ = 0.0;  // 0 = 無制限
  std::chrono::steady_clock::time_point last_publish_time_;

  // 非同期描画用
  struct OverlayTask {
    sensor_msgs::msg::Image::ConstSharedPtr image_msg;
    std::vector<TrackState> tracks;
    std_msgs::msg::Header header;
  };
  std::mutex overlay_mutex_;
  std::condition_variable overlay_cv_;
  OverlayTask pending_overlay_;
  bool has_pending_overlay_ = false;
  std::thread overlay_thread_;
  std::atomic<bool> overlay_thread_running_{false};

  // callbackとoverlay双方の詰まり検知用（watchdogスレッドで監視）
  std::atomic<bool> overlay_in_progress_{false};
  std::atomic<int64_t> overlay_last_progress_ns_{0};
  std::atomic<int> overlay_stage_{0};

  // マスク用の再利用バッファ（imageCallback内のみで使用）
  cv::Mat reusable_empty_mask_;
  cv::Mat reusable_empty_id_mask_;
  cv::Mat reusable_combined_mask_;
  cv::Mat reusable_id_mask_;

  // watchdog (detect long blocking in imageCallback/inference)
  std::thread watchdog_thread_;
  std::atomic<bool> watchdog_running_{false};
  std::atomic<bool> in_callback_{false};
  std::atomic<int64_t> last_progress_ns_{0};
  int watchdog_stall_ms_ = 0;
  int watchdog_warn_ms_ = 0;
  std::atomic<int> stage_{0};
};

} // namespace fv_instance_seg
