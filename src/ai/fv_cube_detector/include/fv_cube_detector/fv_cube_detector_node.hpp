#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <fv_msgs/msg/detection_array.hpp>
#include <opencv2/core.hpp>
#include <memory>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include "fv_cube_detector/ort_yolo_seg.hpp"
#ifdef FV_HAS_TENSORRT
#include "fv_cube_detector/trt_yolo_seg.hpp"
#endif

namespace fv_cube_detector {

class FvCubeDetectorNode : public rclcpp::Node {
 public:
  explicit FvCubeDetectorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~FvCubeDetectorNode();

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
  void updateTracking(const SegResult& res, const rclcpp::Time& stamp);

  cv::Scalar colorForClass(int cls) const;
  void updateStats(double inference_ms, double total_ms, std::size_t detection_count);
  void drawStats(cv::Mat& image);

  // Async overlay thread
  void overlayWorker();
  void enqueueOverlay(const sensor_msgs::msg::Image::ConstSharedPtr& msg,
                      const std::vector<TrackState>& tracks,
                      const std_msgs::msg::Header& header);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_;
  rclcpp::Publisher<fv_msgs::msg::DetectionArray>::SharedPtr fv_dets_pub_;

  std::unique_ptr<OrtYoloSeg> inferencer_;
#ifdef FV_HAS_TENSORRT
  std::unique_ptr<TrtYoloSeg> trt_inferencer_;
#endif
  bool use_trt_ = false;

  std::string model_path_;
  std::string input_image_topic_;
  double conf_thres_;
  double iou_thres_;
  bool use_gpu_;
  bool publish_overlay_;

  static constexpr int NUM_CLASSES = 6;
  static constexpr const char* CLASS_NAMES[NUM_CLASSES] = {
      "blue", "green", "red", "white", "yellow", "unknown_cube"};

  static const cv::Scalar CLASS_COLORS[NUM_CLASSES];

  std::vector<TrackState> tracks_;
  int next_track_id_ = 1;
  int hold_frames_ = 3;
  int drop_frames_ = 10;
  double match_distance_px_ = 80.0;

  double stats_fps_ = 0.0;
  double stats_inference_ms_ = 0.0;
  double stats_total_ms_ = 0.0;
  std::size_t stats_detection_count_ = 0;

  double max_fps_ = 0.0;
  std::chrono::steady_clock::time_point last_publish_time_;

  // Async overlay
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

  // Reusable buffers
  cv::Mat reusable_empty_mask_;
  cv::Mat reusable_combined_mask_;
};

}  // namespace fv_cube_detector
