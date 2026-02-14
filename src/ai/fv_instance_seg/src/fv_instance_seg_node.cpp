#include "fv_instance_seg/fv_instance_seg_node.hpp"

#include <algorithm>
#include <cstdint>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <sstream>
#include <iomanip>
#include <condition_variable>
#include <fluent_lib/cv_bridge_compat.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/imgproc.hpp>
#include "fluent_text.hpp"

namespace fv_instance_seg {

using sensor_msgs::msg::Image;
using fv_msgs::msg::DetectionArray;
using FvDetection2D = fv_msgs::msg::Detection2D;

static rclcpp::QoS make_qos(const std::string& reliability, int depth) {
  rclcpp::QoS qos(depth);
  if (reliability == "reliable") qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  else qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  return qos;
}

InstanceSegNode::InstanceSegNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("instance_seg_node", options) {
  backend_ = this->declare_parameter<std::string>("backend", "openvino");
  model_path_ = this->declare_parameter<std::string>("model_path", "");
  device_ = this->declare_parameter<std::string>("device", "CPU");
  fallback_device_ = this->declare_parameter<std::string>("fallback_device", "");
  input_image_topic_ = this->declare_parameter<std::string>("input_image_topic", "~/input/image_raw");
  conf_thres_ = this->declare_parameter<double>("conf_thres", 0.25);
  iou_thres_ = this->declare_parameter<double>("iou_thres", 0.5);
  publish_detections_ = this->declare_parameter<bool>("publish_detections", true);
  publish_overlay_ = this->declare_parameter<bool>("publish_overlay", true);
  nms_class_agnostic_ = this->declare_parameter<bool>("nms_class_agnostic", true);
  max_detections_ = this->declare_parameter<int>("max_detections", 100);
  debug_shapes_ = this->declare_parameter<bool>("debug_shapes", false);

  match_distance_px_ = this->declare_parameter<double>("tracking.match_max_distance_px", match_distance_px_);
  hold_frames_ = this->declare_parameter<int>("tracking.hold_frames", hold_frames_);
  drop_frames_ = this->declare_parameter<int>("tracking.drop_frames", drop_frames_);
  max_fps_ = this->declare_parameter<double>("max_fps", 0.0);
  int infer_timeout_ms = this->declare_parameter<int>("infer.timeout_ms", 0);
  watchdog_stall_ms_ = this->declare_parameter<int>("watchdog.stall_ms", 0);
  watchdog_warn_ms_ = this->declare_parameter<int>("watchdog.warn_ms", watchdog_stall_ms_ > 0 ? std::min(500, watchdog_stall_ms_ / 2) : 0);

  const std::vector<int64_t> default_palette_ints;
  std::vector<int64_t> palette_vals = this->declare_parameter<std::vector<int64_t>>(
      "tracking.color_palette_bgr", default_palette_ints);
  if (!palette_vals.empty()) {
    if (palette_vals.size() % 3 != 0) {
      RCLCPP_WARN(this->get_logger(),
                  "tracking.color_palette_bgr size (%zu) is not divisible by 3",
                  palette_vals.size());
    }
    for (std::size_t i = 0; i + 2 < palette_vals.size(); i += 3) {
      palette_.emplace_back(static_cast<double>(palette_vals[i]),
                           static_cast<double>(palette_vals[i + 1]),
                           static_cast<double>(palette_vals[i + 2]));
    }
  }
  if (palette_.empty()) {
    palette_ = {
        cv::Scalar(255, 0, 0),   cv::Scalar(0, 255, 0),   cv::Scalar(0, 0, 255),
        cv::Scalar(255, 255, 0), cv::Scalar(255, 0, 255), cv::Scalar(0, 255, 255)};
  }
  if (drop_frames_ < hold_frames_) {
    drop_frames_ = hold_frames_;
  }
  if (match_distance_px_ <= 0.0) {
    match_distance_px_ = 80.0;
  }

  std::string qos_rel = this->declare_parameter<std::string>("qos.reliability", "best_effort");
  int qos_depth = this->declare_parameter<int>("qos.queue_size", 10);
  auto qos = make_qos(qos_rel, qos_depth);

  overlay_pub_ = this->create_publisher<Image>("~/overlay", qos);
  mask_pub_ = this->create_publisher<Image>("~/mask", qos);
  id_mask_pub_ = this->create_publisher<Image>("~/mask_id", qos);
  if (publish_detections_) {
    fv_dets_pub_ = this->create_publisher<DetectionArray>("~/detections", qos);
  }

  inferencer_ = CreateInferencer(backend_);
  if (inferencer_) {
    inferencer_->set_timeout_ms(infer_timeout_ms);
  }
  if (inferencer_) inferencer_->configure(nms_class_agnostic_, max_detections_, debug_shapes_);
  if (!model_path_.empty() && inferencer_) {
    bool loaded = inferencer_->load(model_path_, device_);
    if (!loaded && !fallback_device_.empty() && fallback_device_ != device_) {
      RCLCPP_WARN(this->get_logger(), "Failed to load model on %s, retrying on %s", device_.c_str(), fallback_device_.c_str());
      loaded = inferencer_->load(model_path_, fallback_device_);
      if (loaded) {
        device_ = fallback_device_;
      }
    }
    if (!loaded) {
      RCLCPP_WARN(this->get_logger(), "Failed to load model: %s on %s", model_path_.c_str(), device_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Loaded model: %s on %s", model_path_.c_str(), device_.c_str());
    }
  }

  image_sub_ = this->create_subscription<Image>(
      input_image_topic_, qos, std::bind(&InstanceSegNode::imageCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to %s", input_image_topic_.c_str());

  if (publish_overlay_) {
    overlay_thread_running_ = true;
    overlay_thread_ = std::thread(&InstanceSegNode::overlayWorker, this);
    RCLCPP_INFO(this->get_logger(), "🎨 Overlay async rendering thread started");
  } else {
    RCLCPP_INFO(this->get_logger(), "🛈 Overlay publishing disabled by parameter");
  }

  if (watchdog_stall_ms_ > 0) {
    watchdog_running_ = true;
    last_progress_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                std::chrono::steady_clock::now().time_since_epoch())
                                .count(),
                            std::memory_order_relaxed);
    overlay_last_progress_ns_.store(last_progress_ns_.load(std::memory_order_relaxed), std::memory_order_relaxed);
    watchdog_thread_ = std::thread([this]() {
      const auto poll = std::chrono::milliseconds(50);
      int last_warn_stage = -1;
      int64_t last_warn_at_ns = 0;
      int last_warn_overlay_stage = -1;
      int64_t last_warn_overlay_at_ns = 0;
      while (watchdog_running_.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(poll);
        const int64_t now_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count();

        // --- imageCallback (inference/tracking/publish) ---
        if (in_callback_.load(std::memory_order_relaxed)) {
          const int64_t last_ns = last_progress_ns_.load(std::memory_order_relaxed);
          const int64_t stall_ns = now_ns - last_ns;
          if (stall_ns > 0) {
            const int64_t stall_ms = stall_ns / 1000000;
            const int stg = stage_.load(std::memory_order_relaxed);
            if (watchdog_warn_ms_ > 0 && stall_ms >= watchdog_warn_ms_) {
              // avoid spamming: print at most once per 1000ms per stage
              if (stg != last_warn_stage || (now_ns - last_warn_at_ns) > 1000000000LL) {
                std::fprintf(stderr,
                             "[fv_instance_seg] WATCHDOG warn: stalled=%ld ms stage=%d (warn=%d ms, kill=%d ms)\n",
                             (long)stall_ms, stg, watchdog_warn_ms_, watchdog_stall_ms_);
                std::fflush(stderr);
                last_warn_stage = stg;
                last_warn_at_ns = now_ns;
              }
            }
            if (stall_ms > watchdog_stall_ms_) {
              // NOTE: rclcpp logger may deadlock if the hang is in logging internals;
              // keep this minimal.
              std::fprintf(stderr,
                           "[fv_instance_seg] WATCHDOG KILL: stalled=%ld ms stage=%d (threshold=%d ms). Exiting...\n",
                           (long)stall_ms, stg, watchdog_stall_ms_);
              std::fflush(stderr);
              std::quick_exit(2);
            }
          }
        }

        // --- overlayWorker (render/publish) ---
        if (overlay_in_progress_.load(std::memory_order_relaxed)) {
          const int64_t last_ns = overlay_last_progress_ns_.load(std::memory_order_relaxed);
          const int64_t stall_ns = now_ns - last_ns;
          if (stall_ns > 0) {
            const int64_t stall_ms = stall_ns / 1000000;
            const int stg = overlay_stage_.load(std::memory_order_relaxed);
            if (watchdog_warn_ms_ > 0 && stall_ms >= watchdog_warn_ms_) {
              if (stg != last_warn_overlay_stage || (now_ns - last_warn_overlay_at_ns) > 1000000000LL) {
                std::fprintf(stderr,
                             "[fv_instance_seg] WATCHDOG warn: overlay stalled=%ld ms stage=%d (warn=%d ms, kill=%d ms)\n",
                             (long)stall_ms, stg, watchdog_warn_ms_, watchdog_stall_ms_);
                std::fflush(stderr);
                last_warn_overlay_stage = stg;
                last_warn_overlay_at_ns = now_ns;
              }
            }
            if (stall_ms > watchdog_stall_ms_) {
              std::fprintf(stderr,
                           "[fv_instance_seg] WATCHDOG KILL: overlay stalled=%ld ms stage=%d (threshold=%d ms). Exiting...\n",
                           (long)stall_ms, stg, watchdog_stall_ms_);
              std::fflush(stderr);
              std::quick_exit(2);
            }
          }
        }
      }
    });
    RCLCPP_INFO(this->get_logger(), "🛡️ Watchdog enabled: warn_ms=%d stall_ms=%d", watchdog_warn_ms_, watchdog_stall_ms_);
  }
}

InstanceSegNode::~InstanceSegNode() {
  watchdog_running_ = false;
  if (watchdog_thread_.joinable()) {
    watchdog_thread_.join();
  }
  overlay_thread_running_ = false;
  overlay_cv_.notify_all();
  if (overlay_thread_.joinable()) {
    overlay_thread_.join();
  }
  RCLCPP_INFO(this->get_logger(), "Overlay worker thread stopped");
}

void InstanceSegNode::imageCallback(const Image::SharedPtr msg) {
  in_callback_.store(true, std::memory_order_relaxed);
  struct CallbackGuard {
    std::atomic<bool>* in_callback;
    std::atomic<int>* stage;
    ~CallbackGuard() {
      if (in_callback) {
        in_callback->store(false, std::memory_order_relaxed);
      }
      if (stage) {
        stage->store(0, std::memory_order_relaxed);
      }
    }
  } guard{&in_callback_, &stage_};

  stage_.store(1, std::memory_order_relaxed);  // enter
  last_progress_ns_.store(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count(),
      std::memory_order_relaxed);
  auto callback_start = std::chrono::steady_clock::now();

  // FPS制限（フレームスキップ方式）
  if (max_fps_ > 0.0) {
    auto now = std::chrono::steady_clock::now();
    double target_interval_ms = 1000.0 / max_fps_;
    double elapsed_ms = std::chrono::duration<double, std::milli>(now - last_publish_time_).count();
    if (elapsed_ms < target_interval_ms) {
      return;  // フレームスキップ（同期を保つため）
    }
    last_publish_time_ = now;
  }

  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    stage_.store(2, std::memory_order_relaxed);  // cv_bridge
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const std::exception& e) {
    RCLCPP_WARN(this->get_logger(), "cv_bridge failed: %s", e.what());
    return;
  }
  last_progress_ns_.store(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count(),
      std::memory_order_relaxed);

  auto infer_start = std::chrono::steady_clock::now();
  InferResult res;
  stage_.store(3, std::memory_order_relaxed);  // infer
  bool ok = inferencer_ && inferencer_->infer(cv_ptr->image, static_cast<float>(conf_thres_), static_cast<float>(iou_thres_), &res);
  auto infer_end = std::chrono::steady_clock::now();

  if (!ok) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "⚠️ infer() failed/timeout (backend=%s device=%s)",
                         backend_.c_str(), device_.c_str());
    res = InferResult();
  }
  last_progress_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
                              std::chrono::steady_clock::now().time_since_epoch())
                              .count(),
                          std::memory_order_relaxed);

  rclcpp::Time stamp(msg->header.stamp);
  if (stamp.nanoseconds() == 0) {
    stamp = this->get_clock()->now();
  }

  auto tracking_start = std::chrono::steady_clock::now();
  stage_.store(4, std::memory_order_relaxed);  // tracking
  updateTracking(res, stamp);
  auto tracking_end = std::chrono::steady_clock::now();
  double tracking_ms = std::chrono::duration<double, std::milli>(tracking_end - tracking_start).count();
  if (tracking_ms > 10.0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "⚠️ updateTracking took %.1fms", tracking_ms);
  }

  // アクティブなトラックを先に抽出
  std::vector<TrackState> publish_tracks;
  publish_tracks.reserve(tracks_.size());
  for (const auto& track : tracks_) {
    bool keep = (track.active || track.misses <= hold_frames_) && !track.mask.empty();
    if (keep) {
      publish_tracks.push_back(track);
    }
  }

  // 検出が0個の場合は空マスクをパブリッシュ（オーバーレイは後で描画）
  if (publish_tracks.empty()) {
    stage_.store(5, std::memory_order_relaxed);  // publish(empty)
    const int rows = cv_ptr->image.rows;
    const int cols = cv_ptr->image.cols;
    if (reusable_empty_mask_.empty() || reusable_empty_mask_.rows != rows || reusable_empty_mask_.cols != cols) {
      reusable_empty_mask_.create(rows, cols, CV_8UC1);
      reusable_empty_id_mask_.create(rows, cols, CV_8UC1);
    }
    reusable_empty_mask_.setTo(0);
    reusable_empty_id_mask_.setTo(0);

    publishMask(reusable_empty_mask_, msg->header);
    {
      cv_bridge::CvImage out;
      out.header = msg->header;
      out.encoding = sensor_msgs::image_encodings::MONO8;
      out.image = reusable_empty_id_mask_;
      id_mask_pub_->publish(*out.toImageMsg());
    }

    if (publish_detections_) {
      DetectionArray empty_arr;
      empty_arr.header = msg->header;
      fv_dets_pub_->publish(empty_arr);
    }

    if (publish_overlay_) {
      enqueueOverlay(msg, publish_tracks, msg->header);
    }

    auto callback_end = std::chrono::steady_clock::now();
    double inference_ms = std::chrono::duration<double, std::milli>(infer_end - infer_start).count();
    double total_ms = std::chrono::duration<double, std::milli>(callback_end - callback_start).count();
    updateStats(inference_ms, total_ms, 0);
    return;
  }

  // 検出があるときのみマスク処理を実行
  stage_.store(6, std::memory_order_relaxed);  // publish(mask/dets)
  const int rows = cv_ptr->image.rows;
  const int cols = cv_ptr->image.cols;
  if (reusable_combined_mask_.empty() || reusable_combined_mask_.rows != rows || reusable_combined_mask_.cols != cols) {
    reusable_combined_mask_.create(rows, cols, CV_8UC1);
    reusable_id_mask_.create(rows, cols, CV_8UC1);
  }
  reusable_combined_mask_.setTo(0);
  reusable_id_mask_.setTo(0);

  // publish_tracksのmaskを「フル解像度」に揃えて再利用（overlay側の二重resizeを削減）
  for (auto& track : publish_tracks) {
    if (track.mask.empty()) {
      continue;
    }
    if (track.mask.size() != reusable_combined_mask_.size()) {
      cv::Mat resized;
      cv::resize(track.mask, resized, reusable_combined_mask_.size(), 0, 0, cv::INTER_NEAREST);
      track.mask = resized;
    }
    reusable_combined_mask_ |= track.mask;
    unsigned char vid = static_cast<unsigned char>(track.id & 0xFF);
    if (vid == 0) vid = 255; // 0は背景に予約
    reusable_id_mask_.setTo(cv::Scalar(vid), track.mask);
  }

  publishMask(reusable_combined_mask_, msg->header);
  // IDマスクの配信（常時）
  {
    cv_bridge::CvImage out;
    out.header = msg->header;
    out.encoding = sensor_msgs::image_encodings::MONO8;
    out.image = reusable_id_mask_;
    id_mask_pub_->publish(*out.toImageMsg());
  }

  if (publish_overlay_) {
    enqueueOverlay(msg, publish_tracks, msg->header);
  }

  auto det_start = std::chrono::steady_clock::now();
  if (publish_detections_) {
    publishDetections(publish_tracks, msg->header);
  }
  auto det_end = std::chrono::steady_clock::now();
  double det_ms = std::chrono::duration<double, std::milli>(det_end - det_start).count();
  if (det_ms > 10.0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "⚠️ publishDetections took %.1fms", det_ms);
  }

  auto callback_end = std::chrono::steady_clock::now();
  double inference_ms = std::chrono::duration<double, std::milli>(infer_end - infer_start).count();
  double total_ms = std::chrono::duration<double, std::milli>(callback_end - callback_start).count();

  // トータルと推論の差が大きければ警告
  double overhead_ms = total_ms - inference_ms;
  if (overhead_ms > 20.0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "⚠️ Overhead %.1fms (total=%.1fms - inference=%.1fms)", overhead_ms, total_ms, inference_ms);
  }

  updateStats(inference_ms, total_ms, publish_tracks.size());
}

void InstanceSegNode::publishOverlay(const cv::Mat& bgr, const std_msgs::msg::Header& header){ cv_bridge::CvImage out; out.header=header; out.encoding=sensor_msgs::image_encodings::BGR8; out.image=bgr; overlay_pub_->publish(*out.toImageMsg()); }
void InstanceSegNode::publishMask(const cv::Mat& mask, const std_msgs::msg::Header& header){ cv::Mat mono; if(mask.type()!=CV_8UC1) mask.convertTo(mono, CV_8UC1, 255.0); else mono=mask; cv_bridge::CvImage out; out.header=header; out.encoding=sensor_msgs::image_encodings::MONO8; out.image=mono; mask_pub_->publish(*out.toImageMsg()); }

void InstanceSegNode::publishDetections(const std::vector<TrackState>& tracks, const std_msgs::msg::Header& header) {
  DetectionArray fv_arr;
  fv_arr.header = header;
  fv_arr.detections.reserve(tracks.size());

  for (const auto& track : tracks) {
    FvDetection2D fv_det;
    fv_det.header = header;
    fv_det.id = track.id;
    fv_det.source_mask = FvDetection2D::SOURCE_INSTANCE;
    fv_det.class_id = track.cls;
    fv_det.label = "";
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

void InstanceSegNode::updateTracking(const InferResult& res, const rclcpp::Time& stamp) {
  for (auto& track : tracks_) {
    track.active = false;
  }

  auto toCvRect = [](const cv::Rect& r) {
    cv::Rect clipped = r;
    if (clipped.width < 0) clipped.width = 0;
    if (clipped.height < 0) clipped.height = 0;
    return clipped;
  };

  for (std::size_t i = 0; i < res.boxes.size(); ++i) {
    cv::Rect rect = toCvRect(res.boxes[i]);
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
      track.color = nextColor();
      track.first_seen = stamp;
      track.last_seen = stamp;
      track.age_frames = 1;
      tracks_.push_back(std::move(track));
    }
  }

  for (auto& track : tracks_) {
    if (!track.active) {
      track.misses += 1;
    }
    if (track.active) {
      track.last_seen = stamp;
      if (track.age_frames <= 0) {
        track.age_frames = 1;
      }
      if (track.first_seen.nanoseconds() == 0) {
        track.first_seen = stamp;
      }
    }
  }

  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const TrackState& t) {
                     return t.misses > drop_frames_;
                   }),
                 tracks_.end());
}

cv::Scalar InstanceSegNode::nextColor() {
  auto isColorInUse = [&](const cv::Scalar& candidate) {
    for (const auto& track : tracks_) {
      if ((track.active || track.misses <= hold_frames_) && track.color == candidate) {
        return true;
      }
    }
    return false;
  };

  if (!palette_.empty()) {
    const std::size_t palette_size = palette_.size();
    for (std::size_t offset = 0; offset < palette_size; ++offset) {
      std::size_t idx = (palette_index_ + offset) % palette_size;
      const cv::Scalar& candidate = palette_[idx];
      if (!isColorInUse(candidate)) {
        palette_index_ = (idx + 1) % palette_size;
        return candidate;
      }
    }
  }

  for (int attempt = 0; attempt < 360; ++attempt) {
    int seed = next_track_id_ + attempt;
    cv::Scalar candidate(
        static_cast<double>((37 * seed) % 256),
        static_cast<double>((67 * seed) % 256),
        static_cast<double>((97 * seed) % 256));
    if (!isColorInUse(candidate)) {
      return candidate;
    }
  }

  return cv::Scalar(0, 255, 255);
}

void InstanceSegNode::updateStats(double inference_ms, double total_ms, std::size_t detection_count) {
  stats_inference_ms_ = inference_ms;
  stats_total_ms_ = total_ms;
  stats_detection_count_ = detection_count;

  if (total_ms > 0.0) {
    double inst_fps = 1000.0 / total_ms;
    if (stats_fps_ <= 0.0) {
      stats_fps_ = inst_fps;
    } else {
      const double alpha = 0.2;
      stats_fps_ = (1.0 - alpha) * stats_fps_ + alpha * inst_fps;
    }
  }

  // 5秒ごとにパフォーマンス統計をログ出力
  static int frame_count = 0;
  if (++frame_count % 50 == 0) {  // 約5秒ごと（10fps想定）
    RCLCPP_INFO(get_logger(), "📊 Performance: FPS=%.1f, Inference=%.1fms, Total=%.1fms, Detections=%zu",
                stats_fps_, stats_inference_ms_, stats_total_ms_, stats_detection_count_);
  }
}

void InstanceSegNode::drawStats(cv::Mat& image) {
  int line = 0;
  auto put = [&](const std::string& s) {
    fluent::text::drawShadow(image, s, cv::Point(10, 30 + line * 22), cv::Scalar(0, 255, 0), cv::Scalar(0,0,0), 0.6, 2, 0);
    ++line;
  };

  put(std::string("デバイス: ") + device_);
  put(std::string("バックエンド: ") + backend_);
  put(std::string("FPS: ") + std::to_string(static_cast<int>(stats_fps_)));
  put(std::string("推論: ") + std::to_string(static_cast<int>(stats_inference_ms_)) + "ms");
  put(std::string("総処理: ") + std::to_string(static_cast<int>(stats_total_ms_)) + "ms");
  put(std::string("セグメント: ") + std::to_string(stats_detection_count_));
  if (!model_path_.empty()) {
    put(std::string("モデル: ") + model_path_);
  }
}

void InstanceSegNode::enqueueOverlay(const Image::ConstSharedPtr& msg,
                                     const std::vector<TrackState>& tracks,
                                     const std_msgs::msg::Header& header) {
  std::lock_guard<std::mutex> lock(overlay_mutex_);
  // 最新フレームのみ保持（古いフレームは捨てる）
  pending_overlay_.image_msg = msg;
  pending_overlay_.tracks = tracks;
  pending_overlay_.header = header;
  has_pending_overlay_ = true;
  overlay_cv_.notify_one();
}

void InstanceSegNode::overlayWorker() {
  RCLCPP_INFO(get_logger(), "🎨 Overlay worker thread loop started");
  cv::Mat color_layer;
  cv::Mat overlay;
  cv::Mat resized_mask;
  while (overlay_thread_running_) {
    OverlayTask task;
    {
      std::unique_lock<std::mutex> lock(overlay_mutex_);
      overlay_cv_.wait(lock, [this]() { return !overlay_thread_running_ || has_pending_overlay_; });
      if (!overlay_thread_running_) {
        break;
      }
      task = std::move(pending_overlay_);
      has_pending_overlay_ = false;
    }

    if (!task.image_msg) {
      continue;
    }

    overlay_in_progress_.store(true, std::memory_order_relaxed);
    overlay_stage_.store(1, std::memory_order_relaxed);  // cv_bridge
    overlay_last_progress_ns_.store(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count(),
        std::memory_order_relaxed);

    struct OverlayGuard {
      std::atomic<bool>* in_progress;
      std::atomic<int>* stage;
      ~OverlayGuard() {
        if (in_progress) in_progress->store(false, std::memory_order_relaxed);
        if (stage) stage->store(0, std::memory_order_relaxed);
      }
    } overlay_guard{&overlay_in_progress_, &overlay_stage_};

    {
      try {
        auto overlay_start = std::chrono::steady_clock::now();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "🎨 Rendering overlay for %zu tracks", task.tracks.size());

        cv_bridge::CvImageConstPtr cv_ptr;
        try {
          cv_ptr = cv_bridge::toCvShare(task.image_msg, sensor_msgs::image_encodings::BGR8);
        } catch (const std::exception& e) {
          RCLCPP_WARN(get_logger(), "overlay cv_bridge failed: %s", e.what());
          continue;
        }
        const cv::Mat& image = cv_ptr->image;

        overlay_stage_.store(2, std::memory_order_relaxed);  // compose
        overlay_last_progress_ns_.store(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count(),
            std::memory_order_relaxed);

        // 出力バッファ確保
        overlay.create(image.size(), CV_8UC3);

        if (task.tracks.empty()) {
          // 検出ゼロのときは余計なブレンドを避け、素直にコピーする
          image.copyTo(overlay);
        } else {
          color_layer.create(image.size(), CV_8UC3);
          color_layer.setTo(cv::Scalar(0, 0, 0));

          for (const auto& track : task.tracks) {
            if (track.mask.empty()) {
              continue;
            }
            const cv::Mat* mask_ptr = &track.mask;
            if (track.mask.size() != color_layer.size()) {
              cv::resize(track.mask, resized_mask, color_layer.size(), 0, 0, cv::INTER_NEAREST);
              mask_ptr = &resized_mask;
            }
            // full-frame Mat生成を避ける（setTo + mask）
            color_layer.setTo(track.color, *mask_ptr);
          }

          cv::addWeighted(image, 0.6, color_layer, 0.4, 0.0, overlay);
        }

        overlay_stage_.store(3, std::memory_order_relaxed);  // draw
        overlay_last_progress_ns_.store(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count(),
            std::memory_order_relaxed);

        // バウンディングボックスとラベル描画
        for (const auto& track : task.tracks) {
          cv::rectangle(overlay, track.bbox, track.color, 2);

          int conf_pct = static_cast<int>(track.score * 100.0f + 0.5f);
          if (conf_pct < 0) conf_pct = 0;
          if (conf_pct > 100) conf_pct = 100;
          double duration_sec = (track.last_seen - track.first_seen).seconds();
          if (duration_sec < 0.0) duration_sec = 0.0;

          std::ostringstream label;
          label << "ID " << track.id
                << " " << conf_pct << "%"
                << " " << std::fixed << std::setprecision(1) << duration_sec << "s"
                << " " << track.age_frames << "f";

          int tx = track.bbox.x;
          int ty = std::max(15, track.bbox.y - 5);
          fluent::text::drawShadow(overlay, label.str(), cv::Point(tx, ty), cv::Scalar(255, 255, 255), cv::Scalar(0,0,0), 0.6, 2, 0);
        }

        drawStats(overlay);

        overlay_stage_.store(4, std::memory_order_relaxed);  // publish
        overlay_last_progress_ns_.store(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count(),
            std::memory_order_relaxed);

        publishOverlay(overlay, task.header);

        auto overlay_end = std::chrono::steady_clock::now();
        double overlay_ms = std::chrono::duration<double, std::milli>(overlay_end - overlay_start).count();
        if (overlay_ms > 20.0) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "⚠️ Overlay rendering took %.1fms (async)", overlay_ms);
        }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "✅ Overlay published successfully");
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "❌ Overlay worker exception: %s", e.what());
      } catch (...) {
        RCLCPP_ERROR(get_logger(), "❌ Overlay worker unknown exception");
      }
    }
  }
}

} // namespace fv_instance_seg

int main(int argc, char** argv){ rclcpp::init(argc, argv); auto node= std::make_shared<fv_instance_seg::InstanceSegNode>(); rclcpp::spin(node); rclcpp::shutdown(); return 0; }
