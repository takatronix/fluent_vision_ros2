#include "fv_cube_detector/fv_cube_detector_node.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <sstream>
#include <iomanip>

#include <fluent_lib/cv_bridge_compat.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/imgproc.hpp>
#include "fluent_text.hpp"

namespace fv_cube_detector {

using sensor_msgs::msg::Image;
using fv_msgs::msg::DetectionArray;
using FvDetection2D = fv_msgs::msg::Detection2D;

static rclcpp::QoS make_qos(const std::string& reliability, int depth) {
  rclcpp::QoS qos(depth);
  if (reliability == "reliable")
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  else
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  return qos;
}

constexpr const char* FvCubeDetectorNode::CLASS_NAMES[];

const cv::Scalar FvCubeDetectorNode::CLASS_COLORS[NUM_CLASSES] = {
    cv::Scalar(255, 0, 0),     // blue
    cv::Scalar(0, 200, 0),     // green
    cv::Scalar(0, 0, 255),     // red
    cv::Scalar(220, 220, 220), // white
    cv::Scalar(0, 255, 255),   // yellow
    cv::Scalar(128, 128, 128)  // unknown_cube
};

FvCubeDetectorNode::FvCubeDetectorNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("cube_detector_node", options) {
  model_path_ = this->declare_parameter<std::string>("model_path", "");
  auto trt_engine_path = this->declare_parameter<std::string>("trt_engine_path", "");
  input_image_topic_ = this->declare_parameter<std::string>(
      "input_image_topic", "/fv/d415/color/image_raw");
  conf_thres_ = this->declare_parameter<double>("conf_thres", 0.25);
  iou_thres_ = this->declare_parameter<double>("iou_thres", 0.5);
  use_gpu_ = this->declare_parameter<bool>("use_gpu", true);
  publish_overlay_ = this->declare_parameter<bool>("publish_overlay", true);
  max_fps_ = this->declare_parameter<double>("max_fps", 0.0);
  match_distance_px_ = this->declare_parameter<double>(
      "tracking.match_max_distance_px", match_distance_px_);
  hold_frames_ = this->declare_parameter<int>("tracking.hold_frames", hold_frames_);
  drop_frames_ = this->declare_parameter<int>("tracking.drop_frames", drop_frames_);

  if (drop_frames_ < hold_frames_) drop_frames_ = hold_frames_;
  if (match_distance_px_ <= 0.0) match_distance_px_ = 80.0;

  std::string qos_rel = this->declare_parameter<std::string>("qos.reliability", "best_effort");
  int qos_depth = this->declare_parameter<int>("qos.queue_size", 10);
  auto qos = make_qos(qos_rel, qos_depth);

  // Auto-detect TRT engine path: replace .onnx with .engine
  if (trt_engine_path.empty() && !model_path_.empty()) {
    auto pos = model_path_.rfind(".onnx");
    if (pos != std::string::npos) {
      trt_engine_path = model_path_.substr(0, pos) + ".engine";
      // Check models/tensorrt/ path too
      auto slash = trt_engine_path.rfind('/');
      if (slash != std::string::npos) {
        std::string dir = trt_engine_path.substr(0, slash);
        // Try replacing /onnx/ with /tensorrt/
        auto onnx_pos = dir.rfind("/onnx");
        if (onnx_pos != std::string::npos) {
          std::string trt_dir = dir.substr(0, onnx_pos) + "/tensorrt";
          std::string trt_alt = trt_dir + trt_engine_path.substr(slash);
          std::ifstream test(trt_alt);
          if (test.good()) trt_engine_path = trt_alt;
        }
      }
    }
  }

  overlay_pub_ = this->create_publisher<Image>("overlay", qos);
  mask_pub_ = this->create_publisher<Image>("mask", qos);
  fv_dets_pub_ = this->create_publisher<DetectionArray>("detections", qos);

  // Load model: TensorRT engine → ONNX Runtime (GPU) → ONNX Runtime (CPU)
#ifdef FV_HAS_TENSORRT
  if (use_gpu_ && !trt_engine_path.empty()) {
    std::ifstream test(trt_engine_path);
    if (test.good()) {
      test.close();
      trt_inferencer_ = std::make_unique<TrtYoloSeg>();
      if (trt_inferencer_->load(trt_engine_path)) {
        use_trt_ = true;
        RCLCPP_INFO(this->get_logger(), "Model loaded: %s (device=TensorRT)",
                    trt_engine_path.c_str());
      } else {
        trt_inferencer_.reset();
        RCLCPP_WARN(this->get_logger(), "TRT engine load failed, falling back to ORT");
      }
    }
  }
#endif

  if (!use_trt_) {
    inferencer_ = std::make_unique<OrtYoloSeg>();
    if (!model_path_.empty()) {
      bool loaded = inferencer_->load(model_path_, use_gpu_);
      if (!loaded) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load model: %s", model_path_.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "Model loaded: %s (device=%s)",
                    model_path_.c_str(), inferencer_->device_name().c_str());
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "No model_path specified");
    }
  }

  image_sub_ = this->create_subscription<Image>(
      input_image_topic_, qos,
      std::bind(&FvCubeDetectorNode::imageCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to %s", input_image_topic_.c_str());

  if (publish_overlay_) {
    overlay_thread_running_ = true;
    overlay_thread_ = std::thread(&FvCubeDetectorNode::overlayWorker, this);
    RCLCPP_INFO(this->get_logger(), "Overlay async rendering thread started");
  }
}

FvCubeDetectorNode::~FvCubeDetectorNode() {
  overlay_thread_running_ = false;
  overlay_cv_.notify_all();
  if (overlay_thread_.joinable()) overlay_thread_.join();
}

void FvCubeDetectorNode::imageCallback(const Image::SharedPtr msg) {
  auto callback_start = std::chrono::steady_clock::now();

  // FPS limiting
  if (max_fps_ > 0.0) {
    auto now = std::chrono::steady_clock::now();
    double elapsed_ms =
        std::chrono::duration<double, std::milli>(now - last_publish_time_).count();
    if (elapsed_ms < 1000.0 / max_fps_) return;
    last_publish_time_ = now;
  }

  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const std::exception& e) {
    RCLCPP_WARN(this->get_logger(), "cv_bridge failed: %s", e.what());
    return;
  }

  auto infer_start = std::chrono::steady_clock::now();
  SegResult res;
  bool ok = false;
#ifdef FV_HAS_TENSORRT
  if (use_trt_ && trt_inferencer_) {
    ok = trt_inferencer_->infer(cv_ptr->image, static_cast<float>(conf_thres_),
                                 static_cast<float>(iou_thres_), &res);
  } else
#endif
  {
    ok = inferencer_ && inferencer_->infer(
        cv_ptr->image, static_cast<float>(conf_thres_),
        static_cast<float>(iou_thres_), &res);
  }
  auto infer_end = std::chrono::steady_clock::now();

  if (!ok) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "infer() failed");
    res = SegResult();
  }

  rclcpp::Time stamp(msg->header.stamp);
  if (stamp.nanoseconds() == 0) stamp = this->get_clock()->now();

  updateTracking(res, stamp);

  // Collect active tracks
  std::vector<TrackState> publish_tracks;
  publish_tracks.reserve(tracks_.size());
  for (const auto& track : tracks_) {
    if ((track.active || track.misses <= hold_frames_) && !track.mask.empty())
      publish_tracks.push_back(track);
  }

  const int rows = cv_ptr->image.rows;
  const int cols = cv_ptr->image.cols;

  if (publish_tracks.empty()) {
    // Publish empty mask and detections
    if (reusable_empty_mask_.empty() || reusable_empty_mask_.rows != rows ||
        reusable_empty_mask_.cols != cols) {
      reusable_empty_mask_.create(rows, cols, CV_8UC1);
    }
    reusable_empty_mask_.setTo(0);
    publishMask(reusable_empty_mask_, msg->header);

    DetectionArray empty_arr;
    empty_arr.header = msg->header;
    fv_dets_pub_->publish(empty_arr);

    if (publish_overlay_) enqueueOverlay(msg, publish_tracks, msg->header);

    auto callback_end = std::chrono::steady_clock::now();
    double inference_ms =
        std::chrono::duration<double, std::milli>(infer_end - infer_start).count();
    double total_ms =
        std::chrono::duration<double, std::milli>(callback_end - callback_start).count();
    updateStats(inference_ms, total_ms, 0);
    return;
  }

  // Combine masks
  if (reusable_combined_mask_.empty() || reusable_combined_mask_.rows != rows ||
      reusable_combined_mask_.cols != cols) {
    reusable_combined_mask_.create(rows, cols, CV_8UC1);
  }
  reusable_combined_mask_.setTo(0);

  for (auto& track : publish_tracks) {
    if (track.mask.empty()) continue;
    if (track.mask.size() != reusable_combined_mask_.size()) {
      cv::Mat resized;
      cv::resize(track.mask, resized, reusable_combined_mask_.size(), 0, 0,
                 cv::INTER_NEAREST);
      track.mask = resized;
    }
    reusable_combined_mask_ |= track.mask;
  }

  publishMask(reusable_combined_mask_, msg->header);

  if (publish_overlay_) enqueueOverlay(msg, publish_tracks, msg->header);

  publishDetections(publish_tracks, msg->header);

  auto callback_end = std::chrono::steady_clock::now();
  double inference_ms =
      std::chrono::duration<double, std::milli>(infer_end - infer_start).count();
  double total_ms =
      std::chrono::duration<double, std::milli>(callback_end - callback_start).count();
  updateStats(inference_ms, total_ms, publish_tracks.size());
}

void FvCubeDetectorNode::publishOverlay(const cv::Mat& bgr,
                                         const std_msgs::msg::Header& header) {
  cv_bridge::CvImage out;
  out.header = header;
  out.encoding = sensor_msgs::image_encodings::BGR8;
  out.image = bgr;
  overlay_pub_->publish(*out.toImageMsg());
}

void FvCubeDetectorNode::publishMask(const cv::Mat& mask,
                                      const std_msgs::msg::Header& header) {
  cv::Mat mono;
  if (mask.type() != CV_8UC1)
    mask.convertTo(mono, CV_8UC1, 255.0);
  else
    mono = mask;
  cv_bridge::CvImage out;
  out.header = header;
  out.encoding = sensor_msgs::image_encodings::MONO8;
  out.image = mono;
  mask_pub_->publish(*out.toImageMsg());
}

void FvCubeDetectorNode::publishDetections(const std::vector<TrackState>& tracks,
                                            const std_msgs::msg::Header& header) {
  DetectionArray fv_arr;
  fv_arr.header = header;
  fv_arr.detections.reserve(tracks.size());

  for (const auto& track : tracks) {
    FvDetection2D fv_det;
    fv_det.header = header;
    fv_det.id = track.id;
    fv_det.source_mask = FvDetection2D::SOURCE_INSTANCE;
    fv_det.class_id = track.cls;
    fv_det.label = (track.cls >= 0 && track.cls < NUM_CLASSES)
                       ? CLASS_NAMES[track.cls]
                       : "unknown";
    fv_det.conf_fused = track.score;
    fv_det.conf_object = 0.0f;
    fv_det.conf_instance = track.score;
    fv_det.conf_semantic = 0.0f;

    fv_det.bbox_min.x = static_cast<float>(track.bbox.x);
    fv_det.bbox_min.y = static_cast<float>(track.bbox.y);
    fv_det.bbox_min.z = 0.0f;
    fv_det.bbox_max.x = static_cast<float>(track.bbox.x + track.bbox.width);
    fv_det.bbox_max.y = static_cast<float>(track.bbox.y + track.bbox.height);
    fv_det.bbox_max.z = 0.0f;

    fv_det.mask_instance_id = static_cast<uint32_t>(track.id);
    fv_det.mask_semantic_id = 0;
    fv_det.depth_hint_m = 0.0f;
    fv_det.observed_at = header.stamp;

    fv_arr.detections.push_back(fv_det);
  }

  fv_dets_pub_->publish(fv_arr);
}

void FvCubeDetectorNode::updateTracking(const SegResult& res,
                                         const rclcpp::Time& stamp) {
  for (auto& track : tracks_) track.active = false;

  for (std::size_t i = 0; i < res.boxes.size(); ++i) {
    cv::Rect rect = res.boxes[i];
    if (rect.width < 0) rect.width = 0;
    if (rect.height < 0) rect.height = 0;
    cv::Point2f center(rect.x + rect.width * 0.5f, rect.y + rect.height * 0.5f);
    cv::Mat mask = res.masks.size() > i ? res.masks[i] : cv::Mat();
    if (!mask.empty() && mask.type() != CV_8UC1) {
      cv::Mat tmp;
      mask.convertTo(tmp, CV_8UC1, 255.0);
      mask = tmp;
    }

    TrackState* best = nullptr;
    double best_dist = match_distance_px_;
    for (auto& track : tracks_) {
      double dist = cv::norm(track.center - center);
      if (dist <= best_dist) {
        best_dist = dist;
        best = &track;
      }
    }

    if (best) {
      best->bbox = rect;
      best->mask = mask.clone();
      best->score = res.scores.size() > i ? res.scores[i] : 0.f;
      best->cls = res.classes.size() > i ? res.classes[i] : 0;
      best->center = center;
      best->misses = 0;
      best->active = true;
      best->last_seen = stamp;
      best->age_frames += 1;
    } else {
      TrackState track;
      track.id = next_track_id_++;
      track.bbox = rect;
      track.mask = mask.clone();
      track.score = res.scores.size() > i ? res.scores[i] : 0.f;
      track.cls = res.classes.size() > i ? res.classes[i] : 0;
      track.center = center;
      track.misses = 0;
      track.active = true;
      track.color = colorForClass(track.cls);
      track.first_seen = stamp;
      track.last_seen = stamp;
      track.age_frames = 1;
      tracks_.push_back(std::move(track));
    }
  }

  for (auto& track : tracks_) {
    if (!track.active) track.misses += 1;
    if (track.active) {
      track.last_seen = stamp;
      if (track.first_seen.nanoseconds() == 0) track.first_seen = stamp;
    }
  }

  tracks_.erase(
      std::remove_if(tracks_.begin(), tracks_.end(),
                     [&](const TrackState& t) { return t.misses > drop_frames_; }),
      tracks_.end());
}

cv::Scalar FvCubeDetectorNode::colorForClass(int cls) const {
  if (cls >= 0 && cls < NUM_CLASSES) return CLASS_COLORS[cls];
  return cv::Scalar(128, 128, 128);
}

void FvCubeDetectorNode::updateStats(double inference_ms, double total_ms,
                                      std::size_t detection_count) {
  stats_inference_ms_ = inference_ms;
  stats_total_ms_ = total_ms;
  stats_detection_count_ = detection_count;

  if (total_ms > 0.0) {
    double inst_fps = 1000.0 / total_ms;
    if (stats_fps_ <= 0.0)
      stats_fps_ = inst_fps;
    else
      stats_fps_ = 0.8 * stats_fps_ + 0.2 * inst_fps;
  }

  static int frame_count = 0;
  if (++frame_count % 50 == 0) {
    RCLCPP_INFO(get_logger(),
                "Performance: FPS=%.1f, Inference=%.1fms, Total=%.1fms, Cubes=%zu",
                stats_fps_, stats_inference_ms_, stats_total_ms_, stats_detection_count_);
  }
}

void FvCubeDetectorNode::drawStats(cv::Mat& image) {
  int line = 0;
  auto put = [&](const std::string& s) {
    fluent::text::drawShadow(image, s, cv::Point(10, 30 + line * 22),
                             cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 0), 0.6, 2, 0);
    ++line;
  };

  std::string dev_name;
#ifdef FV_HAS_TENSORRT
  if (use_trt_ && trt_inferencer_) dev_name = trt_inferencer_->device_name();
  else
#endif
  dev_name = inferencer_ ? ("ONNX Runtime " + inferencer_->device_name()) : std::string("N/A");
  put("CubeDetector (" + dev_name + ")");
  put("FPS: " + std::to_string(static_cast<int>(stats_fps_)));
  put("Inference: " + std::to_string(static_cast<int>(stats_inference_ms_)) + "ms");
  put("Cubes: " + std::to_string(stats_detection_count_));
}

void FvCubeDetectorNode::enqueueOverlay(
    const Image::ConstSharedPtr& msg, const std::vector<TrackState>& tracks,
    const std_msgs::msg::Header& header) {
  std::lock_guard<std::mutex> lock(overlay_mutex_);
  pending_overlay_.image_msg = msg;
  pending_overlay_.tracks = tracks;
  pending_overlay_.header = header;
  has_pending_overlay_ = true;
  overlay_cv_.notify_one();
}

void FvCubeDetectorNode::overlayWorker() {
  cv::Mat color_layer;
  cv::Mat overlay;
  cv::Mat resized_mask;

  while (overlay_thread_running_) {
    OverlayTask task;
    {
      std::unique_lock<std::mutex> lock(overlay_mutex_);
      overlay_cv_.wait(lock, [this]() {
        return !overlay_thread_running_ || has_pending_overlay_;
      });
      if (!overlay_thread_running_) break;
      task = std::move(pending_overlay_);
      has_pending_overlay_ = false;
    }

    if (!task.image_msg) continue;

    try {
      cv_bridge::CvImageConstPtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvShare(task.image_msg, sensor_msgs::image_encodings::BGR8);
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "overlay cv_bridge failed: %s", e.what());
        continue;
      }
      const cv::Mat& image = cv_ptr->image;
      overlay.create(image.size(), CV_8UC3);

      if (task.tracks.empty()) {
        image.copyTo(overlay);
      } else {
        color_layer.create(image.size(), CV_8UC3);
        color_layer.setTo(cv::Scalar(0, 0, 0));

        for (const auto& track : task.tracks) {
          if (track.mask.empty()) continue;
          const cv::Mat* mask_ptr = &track.mask;
          if (track.mask.size() != color_layer.size()) {
            cv::resize(track.mask, resized_mask, color_layer.size(), 0, 0,
                       cv::INTER_NEAREST);
            mask_ptr = &resized_mask;
          }
          color_layer.setTo(track.color, *mask_ptr);
        }

        cv::addWeighted(image, 0.6, color_layer, 0.4, 0.0, overlay);
      }

      // Draw bounding boxes and labels
      for (const auto& track : task.tracks) {
        cv::rectangle(overlay, track.bbox, track.color, 2);

        int conf_pct = static_cast<int>(track.score * 100.0f + 0.5f);
        conf_pct = std::clamp(conf_pct, 0, 100);

        std::string label_name = (track.cls >= 0 && track.cls < NUM_CLASSES)
                                     ? CLASS_NAMES[track.cls]
                                     : "?";

        std::ostringstream label;
        label << label_name << " " << conf_pct << "% ID" << track.id;

        int tx = track.bbox.x;
        int ty = std::max(15, track.bbox.y - 5);
        fluent::text::drawShadow(overlay, label.str(), cv::Point(tx, ty),
                                 cv::Scalar(255, 255, 255), cv::Scalar(0, 0, 0),
                                 0.6, 2, 0);
      }

      drawStats(overlay);
      publishOverlay(overlay, task.header);

    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Overlay worker exception: %s", e.what());
    }
  }
}

}  // namespace fv_cube_detector

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fv_cube_detector::FvCubeDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
