#include "fv_instance_seg/fv_instance_seg_node.hpp"

#include <algorithm>
#include <cstdint>
#include <cctype>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <utility>
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

static std::string lower_path(const std::string& path) {
  std::string lower = path;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return lower;
}

static bool ends_with_any(const std::string& text, const std::vector<std::string>& suffixes) {
  for (const auto& suffix : suffixes) {
    if (text.size() >= suffix.size() &&
        text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0) {
      return true;
    }
  }
  return false;
}

static bool is_tensorrt_model_path(const std::string& path) {
  return ends_with_any(lower_path(path), {".engine", ".plan"});
}

static std::string infer_backend_for_model_path(const std::string& path,
                                                const std::string& fallback) {
  const auto lower = lower_path(path);
  if (ends_with_any(lower, {".engine", ".plan"})) {
    return "tensorrt";
  }
  if (ends_with_any(lower, {".onnx", ".xml"})) {
    return "openvino";
  }
  return fallback;
}

// A 0-byte engine left behind by a killed/failed trtexec build must NOT be
// treated as usable (it deserializes to nothing and blinds perception).
static bool file_is_nonempty(const std::string& path) {
  std::ifstream f(path, std::ios::binary | std::ios::ate);
  return f.is_open() && static_cast<std::streamoff>(f.tellg()) > 0;
}

// 「基本 TensorRT・動かない環境では ONNX」解決。 backend が TRT 寄り
// (auto/tensorrt/trt/空) のとき、 portable な .onnx/.xml を host-local の
// TensorRT engine (/ros2_ws/models/tensorrt/<stem>.engine) に差し替える ——
// ただし engine が存在し かつ 非0バイトのときだけ。 engine が無い / 0バイト
// (= TRT 非対応ホスト or ビルド失敗) なら .onnx のまま返して OpenVINO/ORT で
// 走らせる。 明示 "openvino" は常に尊重 (operator が ONNX を強制)。 dashboard
// の _resolve_runtime_model_path と挙動を一致させ launch 時と runtime 選択を揃える。
static std::pair<std::string, std::string> resolve_trt_preferred(
    const std::string& model_path, const std::string& backend) {
  const std::string bk = lower_path(backend);
  const bool prefer_trt =
      bk.empty() || bk == "auto" || bk == "tensorrt" || bk == "trt";
  if (!prefer_trt || model_path.empty()) {
    return {model_path, backend};
  }
  const std::string lower = lower_path(model_path);
  if (ends_with_any(lower, {".engine", ".plan"})) {
    return {model_path, "tensorrt"};
  }
  if (ends_with_any(lower, {".onnx", ".xml"})) {
    const auto slash = model_path.find_last_of('/');
    const std::string base =
        (slash == std::string::npos) ? model_path : model_path.substr(slash + 1);
    const auto dot = base.find_last_of('.');
    const std::string stem = (dot == std::string::npos) ? base : base.substr(0, dot);
    const std::string engine = "/ros2_ws/models/tensorrt/" + stem + ".engine";
    if (file_is_nonempty(engine)) {
      return {engine, "tensorrt"};
    }
    return {model_path, "openvino"};
  }
  return {model_path, backend};
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
  class_names_ = this->declare_parameter<std::vector<std::string>>(
      "class_names", std::vector<std::string>{});
  min_box_area_px_ = this->declare_parameter<double>("min_box_area_px", 0.0);
  max_box_area_px_ = this->declare_parameter<double>("max_box_area_px", 0.0);
  min_aspect_ = this->declare_parameter<double>("min_aspect", 0.0);
  max_aspect_ = this->declare_parameter<double>("max_aspect", 0.0);
  min_mask_fill_ = this->declare_parameter<double>("min_mask_fill", 0.0);
  debug_shapes_ = this->declare_parameter<bool>("debug_shapes", false);

  match_distance_px_ = this->declare_parameter<double>("tracking.match_max_distance_px", match_distance_px_);
  hold_frames_ = this->declare_parameter<int>("tracking.hold_frames", hold_frames_);
  drop_frames_ = this->declare_parameter<int>("tracking.drop_frames", drop_frames_);
  track_enter_conf_ = this->declare_parameter<double>("tracking.enter_conf", track_enter_conf_);
  max_fps_ = this->declare_parameter<double>("max_fps", 0.0);
  infer_timeout_ms_ = this->declare_parameter<int>("infer.timeout_ms", 0);
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

  (void)loadModel(backend_, model_path_, device_, fallback_device_,
                  nms_class_agnostic_, max_detections_, debug_shapes_,
                  infer_timeout_ms_);

  parameter_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&InstanceSegNode::onParametersSet, this, std::placeholders::_1));

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

bool InstanceSegNode::loadModel(const std::string& backend,
                                const std::string& model_path,
                                const std::string& device,
                                const std::string& fallback_device,
                                bool nms_class_agnostic,
                                int max_detections,
                                bool debug_shapes,
                                int infer_timeout_ms) {
  // Snapshot the current good model so we can restore it if the new load fails.
  std::string prev_model, prev_backend;
  {
    std::lock_guard<std::mutex> lock(inferencer_mutex_);
    prev_model = model_path_;
    prev_backend = backend_;
  }

  // Build a fully-loaded inferencer for (mp, bk) WITHOUT touching inferencer_.
  // Returns null on any failure. out_device receives the device actually used.
  auto build = [&](const std::string& mp, const std::string& bk,
                   std::string& out_device) {
    auto inf = CreateInferencer(bk);
    if (!inf) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create inferencer for backend: %s",
                   bk.c_str());
      return inf;  // null
    }
    inf->set_timeout_ms(infer_timeout_ms);
    inf->configure(nms_class_agnostic, max_detections, debug_shapes);
    out_device = device;
    if (!mp.empty()) {
      bool loaded = inf->load(mp, device);
      if (!loaded && !fallback_device.empty() && fallback_device != device) {
        RCLCPP_WARN(this->get_logger(), "Failed to load model on %s, retrying on %s",
                    device.c_str(), fallback_device.c_str());
        loaded = inf->load(mp, fallback_device);
        if (loaded) out_device = fallback_device;
      }
      if (!loaded) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load model: %s on %s",
                     mp.c_str(), device.c_str());
        inf.reset();  // null -> caller treats as failure
      }
    }
    return inf;
  };

  // Commit a freshly-built inferencer as the live one (under the lock).
  auto commit = [&](auto inf, const std::string& mp, const std::string& bk,
                    const std::string& used_device) {
    std::lock_guard<std::mutex> lock(inferencer_mutex_);
    inferencer_ = std::move(inf);
    backend_ = bk;
    model_path_ = mp;
    // Class labels travel with the model: load the `<model>.names` sidecar
    // (one class per line) that sits next to the model file, so switching
    // models re-loads the matching class names. The TRT .engine carries no
    // names metadata. Falls back to the class_names param when no sidecar.
    auto sidecar_names = loadNamesSidecar(mp);
    if (!sidecar_names.empty()) {
      class_names_ = std::move(sidecar_names);
    }
    device_ = used_device;
    fallback_device_ = fallback_device;
    nms_class_agnostic_ = nms_class_agnostic;
    max_detections_ = max_detections;
    debug_shapes_ = debug_shapes;
    infer_timeout_ms_ = infer_timeout_ms;
    tracks_.clear();
    next_track_id_ = 1;
    palette_index_ = 0;
  };

  // Tear down the previous inferencer FIRST so the new TensorRT context/stream
  // is built on a clean CUDA state. Two coexisting TRT contexts plus an
  // unsynchronized teardown corrupted the reloaded context (infer() then failed
  // every frame after a runtime model switch). inferencer_ is briefly null here;
  // imageCallback guards with `inferencer_ &&`. Because inferencer_ stays null
  // throughout every build() below, two contexts never coexist.
  {
    std::lock_guard<std::mutex> lock(inferencer_mutex_);
    inferencer_.reset();
  }

  std::string used_device;
  // 基本 TensorRT・動かない環境では ONNX: redirect a portable .onnx to its
  // host-local engine when one is usable, else keep the .onnx for OpenVINO/ORT.
  auto resolved = resolve_trt_preferred(model_path, backend);
  std::string eff_path = resolved.first;
  std::string eff_backend = resolved.second;
  auto next_inferencer = build(eff_path, eff_backend, used_device);

  // Runtime fallback: an engine that exists but fails to deserialize (corrupt,
  // built for another GPU SM / TRT version) must not blind perception — retry
  // the portable ONNX via OpenVINO. Operator-agreed degradation, logged loud.
  if (!next_inferencer && eff_backend == "tensorrt" && eff_path != model_path &&
      !model_path.empty()) {
    RCLCPP_WARN(this->get_logger(),
                "TensorRT engine load failed (%s); falling back to ONNX/OpenVINO (%s)",
                eff_path.c_str(), model_path.c_str());
    eff_path = model_path;
    eff_backend = "openvino";
    next_inferencer = build(eff_path, eff_backend, used_device);
  }

  if (next_inferencer) {
    commit(std::move(next_inferencer), eff_path, eff_backend, used_device);
    if (eff_path.empty()) {
      RCLCPP_WARN(this->get_logger(), "No model_path specified");
    } else {
      RCLCPP_INFO(this->get_logger(), "Loaded model: %s on %s (backend=%s, %zu classes)",
                  eff_path.c_str(), used_device.c_str(), eff_backend.c_str(),
                  class_names_.size());
    }
    return true;
  }

  // The requested model failed to load and the old inferencer is already gone,
  // so perception is down. Try to restore the previous known-good model rather
  // than leaving the node permanently blind (fail-closed recovery).
  if (!prev_model.empty() && (prev_model != model_path || prev_backend != backend)) {
    RCLCPP_ERROR(this->get_logger(),
                 "Model load failed for %s; restoring previous model %s",
                 model_path.c_str(), prev_model.c_str());
    std::string rec_device;
    auto recovered = build(prev_model, prev_backend, rec_device);
    if (recovered) {
      commit(std::move(recovered), prev_model, prev_backend, rec_device);
      RCLCPP_WARN(this->get_logger(),
                  "Restored previous model %s after failed switch to %s",
                  prev_model.c_str(), model_path.c_str());
      return false;  // requested switch failed; perception alive on previous model
    }
  }

  RCLCPP_ERROR(this->get_logger(),
               "Model load failed and no inferencer is active — perception is DOWN");
  return false;
}

std::vector<std::string> InstanceSegNode::loadNamesSidecar(
    const std::string& model_path) const {
  if (model_path.empty()) {
    return {};
  }
  // Replace the model file extension with ".names" (keep dots in dir names).
  std::string sidecar = model_path;
  const auto slash = sidecar.find_last_of('/');
  const auto dot = sidecar.find_last_of('.');
  if (dot != std::string::npos && (slash == std::string::npos || dot > slash)) {
    sidecar.resize(dot);
  }
  sidecar += ".names";

  std::ifstream f(sidecar);
  if (!f.is_open()) {
    return {};
  }
  std::vector<std::string> names;
  std::string line;
  while (std::getline(f, line)) {
    while (!line.empty() && (line.back() == '\r' || line.back() == ' ' ||
                             line.back() == '\t')) {
      line.pop_back();
    }
    if (!line.empty()) {
      names.push_back(line);
    }
  }
  if (!names.empty()) {
    RCLCPP_INFO(this->get_logger(), "Loaded %zu class names from %s",
                names.size(), sidecar.c_str());
  }
  return names;
}

void InstanceSegNode::filterDetections(InferResult& res) const {
  if (min_box_area_px_ <= 0.0 && max_box_area_px_ <= 0.0 &&
      min_aspect_ <= 0.0 && max_aspect_ <= 0.0 && min_mask_fill_ <= 0.0) {
    return;  // no filter configured
  }
  const std::size_t n = res.boxes.size();
  std::vector<std::size_t> keep;
  keep.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    const auto& b = res.boxes[i];
    const double area = static_cast<double>(b.width) * static_cast<double>(b.height);
    const double aspect =
        (b.height > 0) ? (static_cast<double>(b.width) / static_cast<double>(b.height)) : 0.0;
    if (min_box_area_px_ > 0.0 && area < min_box_area_px_) continue;
    if (max_box_area_px_ > 0.0 && area > max_box_area_px_) continue;
    if (min_aspect_ > 0.0 && aspect < min_aspect_) continue;
    if (max_aspect_ > 0.0 && aspect > max_aspect_) continue;
    if (min_mask_fill_ > 0.0 && i < res.masks.size() && !res.masks[i].empty()) {
      // The mask is full-frame; measure how much of the BOUNDING BOX it fills
      // (compactness), not the whole image — otherwise the ratio scales with
      // object-size-vs-frame and resolution, making the threshold meaningless.
      const cv::Mat& m = res.masks[i];
      const cv::Rect roi = b & cv::Rect(0, 0, m.cols, m.rows);  // clamp box to image
      const double box_area = static_cast<double>(roi.area());
      const double fill = (box_area > 0.0)
          ? (static_cast<double>(cv::countNonZero(m(roi) > 0)) / box_area)
          : 0.0;
      if (fill < min_mask_fill_) continue;
    }
    keep.push_back(i);
  }
  if (keep.size() == n) return;  // nothing filtered

  InferResult out;
  out.mask_proto_size = res.mask_proto_size;
  out.boxes.reserve(keep.size());
  out.classes.reserve(keep.size());
  out.scores.reserve(keep.size());
  out.masks.reserve(keep.size());
  for (std::size_t idx : keep) {
    out.boxes.push_back(res.boxes[idx]);
    out.classes.push_back(res.classes[idx]);
    out.scores.push_back(res.scores[idx]);
    if (idx < res.masks.size()) out.masks.push_back(res.masks[idx]);
  }
  res = std::move(out);
}

rcl_interfaces::msg::SetParametersResult InstanceSegNode::onParametersSet(
    const std::vector<rclcpp::Parameter>& parameters) {
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;

  auto as_double = [](const rclcpp::Parameter& param) -> double {
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      return static_cast<double>(param.as_int());
    }
    return param.as_double();
  };
  auto as_int = [](const rclcpp::Parameter& param) -> int {
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      return static_cast<int>(param.as_double());
    }
    return static_cast<int>(param.as_int());
  };

  std::string current_backend;
  std::string current_model_path;
  std::string current_device;
  std::string current_fallback_device;
  bool current_nms_class_agnostic = true;
  int current_max_detections = 100;
  bool current_debug_shapes = false;
  int current_infer_timeout_ms = 0;

  std::string next_backend;
  std::string next_model_path;
  std::string next_device;
  std::string next_fallback_device;
  double next_conf = 0.25;
  double next_iou = 0.5;
  double next_max_fps = 0.0;
  double next_match_distance = 80.0;
  int next_hold_frames = 3;
  int next_drop_frames = 10;
  double next_enter_conf = 0.0;
  bool next_nms_class_agnostic = true;
  int next_max_detections = 100;
  bool next_debug_shapes = false;
  int next_infer_timeout_ms = 0;
  double next_min_box_area = 0.0;
  double next_max_box_area = 0.0;
  double next_min_aspect = 0.0;
  double next_max_aspect = 0.0;
  double next_min_mask_fill = 0.0;
  bool model_path_changed = false;
  bool backend_changed = false;
  bool device_changed = false;

  {
    std::lock_guard<std::mutex> lock(inferencer_mutex_);
    current_backend = backend_;
    current_model_path = model_path_;
    current_device = device_;
    current_fallback_device = fallback_device_;
    current_nms_class_agnostic = nms_class_agnostic_;
    current_max_detections = max_detections_;
    current_debug_shapes = debug_shapes_;
    current_infer_timeout_ms = infer_timeout_ms_;

    next_backend = backend_;
    next_model_path = model_path_;
    next_device = device_;
    next_fallback_device = fallback_device_;
    next_conf = conf_thres_;
    next_iou = iou_thres_;
    next_max_fps = max_fps_;
    next_match_distance = match_distance_px_;
    next_hold_frames = hold_frames_;
    next_drop_frames = drop_frames_;
    next_enter_conf = track_enter_conf_;
    next_nms_class_agnostic = nms_class_agnostic_;
    next_max_detections = max_detections_;
    next_debug_shapes = debug_shapes_;
    next_infer_timeout_ms = infer_timeout_ms_;
    next_min_box_area = min_box_area_px_;
    next_max_box_area = max_box_area_px_;
    next_min_aspect = min_aspect_;
    next_max_aspect = max_aspect_;
    next_min_mask_fill = min_mask_fill_;
  }

  try {
    for (const auto& param : parameters) {
      const auto& name = param.get_name();
      if (name == "backend") {
        next_backend = param.as_string();
        backend_changed = true;
      } else if (name == "model_path") {
        next_model_path = param.as_string();
        model_path_changed = true;
      } else if (name == "device") {
        next_device = param.as_string();
        device_changed = true;
      } else if (name == "fallback_device") {
        next_fallback_device = param.as_string();
      } else if (name == "conf_thres") {
        next_conf = as_double(param);
      } else if (name == "iou_thres") {
        next_iou = as_double(param);
      } else if (name == "max_fps") {
        next_max_fps = as_double(param);
      } else if (name == "tracking.match_max_distance_px") {
        next_match_distance = as_double(param);
      } else if (name == "tracking.hold_frames") {
        next_hold_frames = as_int(param);
      } else if (name == "tracking.drop_frames") {
        next_drop_frames = as_int(param);
      } else if (name == "tracking.enter_conf") {
        next_enter_conf = as_double(param);
      } else if (name == "nms_class_agnostic") {
        next_nms_class_agnostic = param.as_bool();
      } else if (name == "max_detections") {
        next_max_detections = as_int(param);
      } else if (name == "debug_shapes") {
        next_debug_shapes = param.as_bool();
      } else if (name == "infer.timeout_ms") {
        next_infer_timeout_ms = as_int(param);
      } else if (name == "min_box_area_px") {
        next_min_box_area = as_double(param);
      } else if (name == "max_box_area_px") {
        next_max_box_area = as_double(param);
      } else if (name == "min_aspect") {
        next_min_aspect = as_double(param);
      } else if (name == "max_aspect") {
        next_max_aspect = as_double(param);
      } else if (name == "min_mask_fill") {
        next_min_mask_fill = as_double(param);
      }
    }
  } catch (const std::exception& e) {
    result.successful = false;
    result.reason = e.what();
    return result;
  }

  if (model_path_changed && !backend_changed) {
    next_backend = infer_backend_for_model_path(next_model_path, next_backend);
  }
  if (model_path_changed && !device_changed && is_tensorrt_model_path(next_model_path)) {
    next_device = "GPU";
  }

  if (next_conf < 0.0 || next_conf > 1.0) {
    result.successful = false;
    result.reason = "conf_thres must be within [0.0, 1.0]";
    return result;
  }
  if (next_iou < 0.0 || next_iou > 1.0) {
    result.successful = false;
    result.reason = "iou_thres must be within [0.0, 1.0]";
    return result;
  }
  if (next_max_fps < 0.0) {
    result.successful = false;
    result.reason = "max_fps must be >= 0.0";
    return result;
  }
  if (next_match_distance <= 0.0) {
    result.successful = false;
    result.reason = "tracking.match_max_distance_px must be > 0.0";
    return result;
  }
  if (next_hold_frames < 0 || next_drop_frames < 0) {
    result.successful = false;
    result.reason = "tracking hold/drop frames must be >= 0";
    return result;
  }
  if (next_enter_conf < 0.0 || next_enter_conf > 1.0) {
    result.successful = false;
    result.reason = "tracking.enter_conf must be within [0.0, 1.0]";
    return result;
  }
  if (next_max_detections <= 0) {
    result.successful = false;
    result.reason = "max_detections must be > 0";
    return result;
  }
  if (next_infer_timeout_ms < 0) {
    result.successful = false;
    result.reason = "infer.timeout_ms must be >= 0";
    return result;
  }
  if (next_drop_frames < next_hold_frames) {
    next_drop_frames = next_hold_frames;
  }

  const bool reload_needed =
      next_backend != current_backend ||
      next_model_path != current_model_path ||
      next_device != current_device ||
      next_fallback_device != current_fallback_device;
  const bool configure_changed =
      next_nms_class_agnostic != current_nms_class_agnostic ||
      next_max_detections != current_max_detections ||
      next_debug_shapes != current_debug_shapes ||
      next_infer_timeout_ms != current_infer_timeout_ms;

  if (reload_needed &&
      !loadModel(next_backend, next_model_path, next_device, next_fallback_device,
                 next_nms_class_agnostic, next_max_detections, next_debug_shapes,
                 next_infer_timeout_ms)) {
    result.successful = false;
    result.reason = "model reload failed";
    return result;
  }

  {
    std::lock_guard<std::mutex> lock(inferencer_mutex_);
    conf_thres_ = next_conf;
    iou_thres_ = next_iou;
    max_fps_ = next_max_fps;
    match_distance_px_ = next_match_distance;
    hold_frames_ = next_hold_frames;
    drop_frames_ = next_drop_frames;
    track_enter_conf_ = next_enter_conf;
    min_box_area_px_ = next_min_box_area;
    max_box_area_px_ = next_max_box_area;
    min_aspect_ = next_min_aspect;
    max_aspect_ = next_max_aspect;
    min_mask_fill_ = next_min_mask_fill;
    if (!reload_needed) {
      backend_ = next_backend;
      model_path_ = next_model_path;
      device_ = next_device;
      fallback_device_ = next_fallback_device;
      if (configure_changed && inferencer_) {
        inferencer_->set_timeout_ms(next_infer_timeout_ms);
        inferencer_->configure(next_nms_class_agnostic, next_max_detections,
                               next_debug_shapes);
      }
      nms_class_agnostic_ = next_nms_class_agnostic;
      max_detections_ = next_max_detections;
      debug_shapes_ = next_debug_shapes;
      infer_timeout_ms_ = next_infer_timeout_ms;
    }
  }

  return result;
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
  double max_fps = 0.0;
  {
    std::lock_guard<std::mutex> lock(inferencer_mutex_);
    max_fps = max_fps_;
  }
  if (max_fps > 0.0) {
    auto now = std::chrono::steady_clock::now();
    double target_interval_ms = 1000.0 / max_fps;
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
  bool ok = false;
  std::string backend_snapshot;
  std::string device_snapshot;
  {
    std::lock_guard<std::mutex> lock(inferencer_mutex_);
    backend_snapshot = backend_;
    device_snapshot = device_;
    ok = inferencer_ && inferencer_->infer(
        cv_ptr->image, static_cast<float>(conf_thres_),
        static_cast<float>(iou_thres_), &res);
  }
  auto infer_end = std::chrono::steady_clock::now();

  // Drop false detections by box size / aspect / mask fill (opt-in via params).
  if (ok) {
    filterDetections(res);
  }

  if (!ok) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "⚠️ infer() failed/timeout (backend=%s device=%s)",
                         backend_snapshot.c_str(), device_snapshot.c_str());
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
  std::vector<TrackState> publish_tracks;
  {
    std::lock_guard<std::mutex> lock(inferencer_mutex_);
    updateTracking(res, stamp);
    publish_tracks.reserve(tracks_.size());
    for (const auto& track : tracks_) {
      bool keep = (track.active || track.misses <= hold_frames_) && !track.mask.empty();
      if (keep) {
        publish_tracks.push_back(track);
      }
    }
  }
  auto tracking_end = std::chrono::steady_clock::now();
  double tracking_ms = std::chrono::duration<double, std::milli>(tracking_end - tracking_start).count();
  if (tracking_ms > 10.0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "⚠️ updateTracking took %.1fms", tracking_ms);
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

  // publish_tracksのmaskを「フル解像度」に揃えて再利用（overlay側の二重resizeを削減）。
  // MONO8 ~/mask_id には per-frame slot (1..255, 0は背景に予約) を割り当て、
  // 同じ値を Detection2D.mask_instance_id に載せる（安定 track ID は Detection2D.id 側）。
  uint8_t next_slot = 0;
  for (auto& track : publish_tracks) {
    track.publish_slot_id = 0;
    if (track.mask.empty()) {
      continue;
    }
    if (track.mask.size() != reusable_combined_mask_.size()) {
      cv::Mat resized;
      cv::resize(track.mask, resized, reusable_combined_mask_.size(), 0, 0, cv::INTER_NEAREST);
      track.mask = resized;
    }
    reusable_combined_mask_ |= track.mask;
    if (next_slot == 255) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "more than 255 active tracks in one frame; extra tracks will not be reachable from MONO8 mask_id consumers");
      continue;
    }
    next_slot += 1;
    track.publish_slot_id = next_slot;
    reusable_id_mask_.setTo(cv::Scalar(next_slot), track.mask);
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
    fv_det.label = labelForClass(track.cls);
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

    // 安定IDは Detection2D.id（上の fv_det.id）。mask_instance_id は MONO8
    // ~/mask_id への per-frame key なので publish_slot_id を載せる。
    fv_det.mask_instance_id = static_cast<uint32_t>(track.publish_slot_id);
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
      if (track.active) {
        continue;  // one detection must not overwrite another detection's track in the same frame
      }
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
      // Hysteresis entry gate: only a confident detection may start a
      // new track. Existing tracks were refreshed above at any score
      // >= conf_thres, so borderline objects stay once acquired but
      // sub-threshold noise can no longer strobe one-frame boxes.
      const float score = res.scores.size() > i ? res.scores[i] : 0.f;
      if (score < static_cast<float>(track_enter_conf_)) {
        continue;
      }
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

  std::string device;
  std::string backend;
  std::string model_path;
  {
    std::lock_guard<std::mutex> lock(inferencer_mutex_);
    device = device_;
    backend = backend_;
    model_path = model_path_;
  }

  // ASCII only: the overlay text is drawn with cv::putText (Hershey font),
  // which has no glyphs for multibyte/Japanese and renders them as "????".
  put(std::string("Device: ") + device);
  put(std::string("Backend: ") + backend);
  {
    std::ostringstream cs;
    cs << "Conf: " << std::fixed << std::setprecision(2) << conf_thres_
       << "  IoU: " << std::fixed << std::setprecision(2) << iou_thres_;
    put(cs.str());
  }
  put(std::string("FPS: ") + std::to_string(static_cast<int>(stats_fps_)));
  put(std::string("Infer: ") + std::to_string(static_cast<int>(stats_inference_ms_)) + "ms");
  put(std::string("Total: ") + std::to_string(static_cast<int>(stats_total_ms_)) + "ms");
  put(std::string("Detections: ") + std::to_string(stats_detection_count_));
  if (!model_path.empty()) {
    put(std::string("Model: ") + model_path);
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
          const std::string cls_name = labelForClass(track.cls);
          if (!cls_name.empty()) label << cls_name << " ";
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
