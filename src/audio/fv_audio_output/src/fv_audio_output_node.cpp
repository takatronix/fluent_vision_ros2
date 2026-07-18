#include "fv_audio_output/fv_audio_output_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <fstream>
#include <utility>

namespace fv_audio_output
{

namespace
{
constexpr const char * kEncodingPcm16 = "PCM16LE";
}

FvAudioOutputNode::FvAudioOutputNode()
: rclcpp::Node("fv_audio_output")
{
  RCLCPP_INFO(this->get_logger(), "Starting FV Audio Output node");
  loadParameters();

  bytes_per_frame_ = config_.channels * (config_.bit_depth / 8);
  if (bytes_per_frame_ == 0) {
    throw std::runtime_error("Invalid audio configuration: bytes_per_frame is zero");
  }

  if (!openDevice()) {
    throw std::runtime_error("Failed to open ALSA playback device");
  }

  // QoS は fv_tts 側と合わせ、reliable/best_effort をパラメータで切り替える。
  rclcpp::QoS audio_qos(config_.qos_depth);
  if (config_.qos_reliable) {
    audio_qos.reliable();
  } else {
    audio_qos.best_effort();
  }
  audio_sub_ = this->create_subscription<fv_audio::msg::AudioFrame>(
    "audio/output/frame", audio_qos,
    std::bind(&FvAudioOutputNode::handleFrame, this, std::placeholders::_1));

  // UI volume is latched so a restarted output node immediately receives the
  // last setting from the scene viewer.
  rclcpp::QoS volume_qos(1);
  volume_qos.reliable().transient_local();
  volume_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    config_.volume_topic, volume_qos,
    std::bind(&FvAudioOutputNode::handleVolume, this, std::placeholders::_1));

  play_file_srv_ = this->create_service<fv_audio_output::srv::PlayFile>(
    "audio/output/play_file",
    std::bind(&FvAudioOutputNode::handlePlayFile, this,
      std::placeholders::_1, std::placeholders::_2));

  // 停止リクエスト subscription
  stop_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "audio/output/stop", 10,
    std::bind(&FvAudioOutputNode::handleStop, this, std::placeholders::_1));

  // 一時停止リクエスト subscription
  pause_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "audio/output/pause", 10,
    std::bind(&FvAudioOutputNode::handlePause, this, std::placeholders::_1));

  // 再開リクエスト subscription
  resume_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "audio/output/resume", 10,
    std::bind(&FvAudioOutputNode::handleResume, this, std::placeholders::_1));

  // 再生完了通知 publisher
  playback_done_pub_ = this->create_publisher<fv_audio::msg::PlaybackDone>(
    "audio/output/playback_done", 10);

  // 一時停止完了通知 publisher
  paused_pub_ = this->create_publisher<std_msgs::msg::Empty>(
    "audio/output/paused", 10);

  running_.store(true);
  playback_thread_ = std::thread(&FvAudioOutputNode::playbackThread, this);
}

FvAudioOutputNode::~FvAudioOutputNode()
{
  running_.store(false);
  queue_cv_.notify_all();
  if (playback_thread_.joinable()) {
    playback_thread_.join();
  }
  closeDevice();
}

void FvAudioOutputNode::loadParameters()
{
  const uint32_t default_chunk_duration_ms = config_.chunk_duration_ms;
  const size_t default_qos_depth = config_.qos_depth;
  const bool default_qos_reliable = config_.qos_reliable;
  const uint32_t default_start_threshold_frames = config_.start_threshold_frames;
  const bool default_drain_after_frame = config_.drain_after_frame;

  this->declare_parameter("audio.device_id", config_.device_id);
  this->declare_parameter("audio.volume_topic", config_.volume_topic);
  this->declare_parameter<double>("audio.volume", static_cast<double>(config_.volume));
  this->declare_parameter<int>("audio.sample_rate", static_cast<int>(config_.sample_rate));
  this->declare_parameter<int>("audio.channels", static_cast<int>(config_.channels));
  this->declare_parameter<int>("audio.bit_depth", static_cast<int>(config_.bit_depth));
  this->declare_parameter<int>("queue.max_frames", static_cast<int>(config_.max_queue_frames));
  this->declare_parameter<int>("audio.chunk_duration_ms", static_cast<int>(default_chunk_duration_ms));
  this->declare_parameter<int>(
    "audio.start_threshold_frames", static_cast<int>(default_start_threshold_frames));
  this->declare_parameter<bool>("audio.drain_after_frame", default_drain_after_frame);
  this->declare_parameter<int>("audio.qos.depth", static_cast<int>(default_qos_depth));
  this->declare_parameter<bool>("audio.qos.reliable", default_qos_reliable);

  config_.device_id = this->get_parameter("audio.device_id").as_string();
  config_.volume_topic = this->get_parameter("audio.volume_topic").as_string();
  config_.volume = static_cast<float>(std::clamp(
    this->get_parameter("audio.volume").as_double(), 0.0, 1.0));
  volume_.store(config_.volume);
  config_.sample_rate = this->get_parameter("audio.sample_rate").as_int();
  config_.channels = this->get_parameter("audio.channels").as_int();
  config_.bit_depth = this->get_parameter("audio.bit_depth").as_int();
  const int64_t max_frames_param = this->get_parameter("queue.max_frames").as_int();
  config_.max_queue_frames = static_cast<size_t>(std::max<int64_t>(1, max_frames_param));
  const int64_t chunk_duration_ms_param = this->get_parameter("audio.chunk_duration_ms").as_int();
  if (chunk_duration_ms_param <= 0) {
    RCLCPP_WARN(this->get_logger(),
      "audio.chunk_duration_ms must be > 0, fallback to default: %u",
      default_chunk_duration_ms);
    config_.chunk_duration_ms = default_chunk_duration_ms;
  } else if (chunk_duration_ms_param > 1000) {
    RCLCPP_WARN(this->get_logger(),
      "audio.chunk_duration_ms too large (%ld), clamp to 1000",
      static_cast<long>(chunk_duration_ms_param));
    config_.chunk_duration_ms = 1000;
  } else {
    config_.chunk_duration_ms = static_cast<uint32_t>(chunk_duration_ms_param);
  }
  const int64_t start_threshold_param =
    this->get_parameter("audio.start_threshold_frames").as_int();
  config_.start_threshold_frames = static_cast<uint32_t>(
    std::max<int64_t>(0, start_threshold_param));
  config_.drain_after_frame = this->get_parameter("audio.drain_after_frame").as_bool();

  const int64_t qos_depth_param = this->get_parameter("audio.qos.depth").as_int();
  if (qos_depth_param <= 0) {
    RCLCPP_WARN(this->get_logger(),
      "audio.qos.depth must be > 0, fallback to default: %zu",
      default_qos_depth);
    config_.qos_depth = default_qos_depth;
  } else {
    config_.qos_depth = static_cast<size_t>(qos_depth_param);
  }
  config_.qos_reliable = this->get_parameter("audio.qos.reliable").as_bool();

  RCLCPP_INFO(this->get_logger(),
    "Output config: device=%s volume=%.2f rate=%uHz channels=%u bits=%u queue=%zu "
    "chunk=%ums start_threshold=%u drain_after_frame=%s qos_depth=%zu reliable=%s",
    config_.device_id.c_str(), config_.volume, config_.sample_rate,
    config_.channels, config_.bit_depth,
    config_.max_queue_frames, config_.chunk_duration_ms, config_.start_threshold_frames,
    config_.drain_after_frame ? "true" : "false", config_.qos_depth,
    config_.qos_reliable ? "true" : "false");
}

bool FvAudioOutputNode::openDevice()
{
  closeDevice();

  snd_pcm_t * handle = nullptr;
  int err = snd_pcm_open(&handle, config_.device_id.c_str(), SND_PCM_STREAM_PLAYBACK, 0);
  if (err < 0) {
    RCLCPP_ERROR(this->get_logger(), "snd_pcm_open failed: %s", snd_strerror(err));
    return false;
  }

  snd_pcm_format_t format = SND_PCM_FORMAT_UNKNOWN;
  if (config_.bit_depth == 16) {
    format = SND_PCM_FORMAT_S16_LE;
  } else if (config_.bit_depth == 32) {
    format = SND_PCM_FORMAT_S32_LE;
  }

  if (format == SND_PCM_FORMAT_UNKNOWN) {
    RCLCPP_ERROR(this->get_logger(), "Unsupported bit depth: %u", config_.bit_depth);
    snd_pcm_close(handle);
    return false;
  }

  // Use hardware parameters for more control over buffer settings
  snd_pcm_hw_params_t *hw_params;
  snd_pcm_hw_params_alloca(&hw_params);

  err = snd_pcm_hw_params_any(handle, hw_params);
  if (err < 0) {
    RCLCPP_ERROR(this->get_logger(), "snd_pcm_hw_params_any failed: %s", snd_strerror(err));
    snd_pcm_close(handle);
    return false;
  }

  // Enable software resampling for better stability
  err = snd_pcm_hw_params_set_rate_resample(handle, hw_params, 1);
  if (err < 0) {
    RCLCPP_WARN(this->get_logger(), "snd_pcm_hw_params_set_rate_resample failed: %s", snd_strerror(err));
  }

  err = snd_pcm_hw_params_set_access(handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
  if (err < 0) {
    RCLCPP_ERROR(this->get_logger(), "snd_pcm_hw_params_set_access failed: %s", snd_strerror(err));
    snd_pcm_close(handle);
    return false;
  }

  err = snd_pcm_hw_params_set_format(handle, hw_params, format);
  if (err < 0) {
    RCLCPP_ERROR(this->get_logger(), "snd_pcm_hw_params_set_format failed: %s", snd_strerror(err));
    snd_pcm_close(handle);
    return false;
  }

  err = snd_pcm_hw_params_set_channels(handle, hw_params, config_.channels);
  if (err < 0) {
    RCLCPP_ERROR(this->get_logger(), "snd_pcm_hw_params_set_channels failed: %s", snd_strerror(err));
    snd_pcm_close(handle);
    return false;
  }

  unsigned int rate = config_.sample_rate;
  err = snd_pcm_hw_params_set_rate_near(handle, hw_params, &rate, 0);
  if (err < 0) {
    RCLCPP_ERROR(this->get_logger(), "snd_pcm_hw_params_set_rate_near failed: %s", snd_strerror(err));
    snd_pcm_close(handle);
    return false;
  }

  // Set larger buffer size to prevent XRUNs: 16384 frames = ~341ms at 48kHz
  snd_pcm_uframes_t buffer_size = 16384;
  err = snd_pcm_hw_params_set_buffer_size_near(handle, hw_params, &buffer_size);
  if (err < 0) {
    RCLCPP_ERROR(this->get_logger(), "snd_pcm_hw_params_set_buffer_size_near failed: %s", snd_strerror(err));
    snd_pcm_close(handle);
    return false;
  }

  // Set period size: buffer_size / 4
  snd_pcm_uframes_t period_size = buffer_size / 4;
  err = snd_pcm_hw_params_set_period_size_near(handle, hw_params, &period_size, 0);
  if (err < 0) {
    RCLCPP_ERROR(this->get_logger(), "snd_pcm_hw_params_set_period_size_near failed: %s", snd_strerror(err));
    snd_pcm_close(handle);
    return false;
  }

  err = snd_pcm_hw_params(handle, hw_params);
  if (err < 0) {
    RCLCPP_ERROR(this->get_logger(), "snd_pcm_hw_params failed: %s", snd_strerror(err));
    snd_pcm_close(handle);
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "ALSA buffer: %lu frames, period: %lu frames",
              buffer_size, period_size);

  // Configure software parameters for better playback behavior
  snd_pcm_sw_params_t *sw_params;
  snd_pcm_sw_params_alloca(&sw_params);

  err = snd_pcm_sw_params_current(handle, sw_params);
  if (err < 0) {
    RCLCPP_WARN(this->get_logger(), "snd_pcm_sw_params_current failed: %s", snd_strerror(err));
  } else {
    const snd_pcm_uframes_t start_threshold =
      config_.start_threshold_frames == 0 ?
      std::max<snd_pcm_uframes_t>(1, buffer_size / 2) :
      std::min<snd_pcm_uframes_t>(
        buffer_size,
        std::max<snd_pcm_uframes_t>(1, config_.start_threshold_frames));

    err = snd_pcm_sw_params_set_start_threshold(handle, sw_params, start_threshold);
    if (err < 0) {
      RCLCPP_WARN(this->get_logger(), "snd_pcm_sw_params_set_start_threshold failed: %s", snd_strerror(err));
    }

    // Wake up the writer when at least one period of space is available
    err = snd_pcm_sw_params_set_avail_min(handle, sw_params, period_size);
    if (err < 0) {
      RCLCPP_WARN(this->get_logger(), "snd_pcm_sw_params_set_avail_min failed: %s", snd_strerror(err));
    }

    err = snd_pcm_sw_params(handle, sw_params);
    if (err < 0) {
      RCLCPP_WARN(this->get_logger(), "snd_pcm_sw_params failed: %s", snd_strerror(err));
    } else {
      RCLCPP_INFO(this->get_logger(), "ALSA software params: start_threshold=%lu frames", start_threshold);
    }
  }

  pcm_handle_ = handle;
  return true;
}

void FvAudioOutputNode::closeDevice()
{
  if (pcm_handle_) {
    snd_pcm_drop(pcm_handle_);
    snd_pcm_close(pcm_handle_);
    pcm_handle_ = nullptr;
  }
}

void FvAudioOutputNode::handleFrame(const fv_audio::msg::AudioFrame::SharedPtr msg)
{
  RCLCPP_DEBUG_THROTTLE(
    this->get_logger(), *this->get_clock(), 2000,
    "Frame received: %zu bytes, %uHz, %u ch, encoding=%s",
    msg->data.size(), msg->sample_rate, msg->channels, msg->encoding.c_str());

  if (!validateFrame(*msg)) {
    return;
  }

  // Barge-in 後の古いフレームをフィルタリング
  {
    std::lock_guard<std::mutex> lock(stop_time_mutex_);
    rclcpp::Time frame_time(msg->header.stamp);
    if (last_stop_time_.nanoseconds() > 0 && frame_time < last_stop_time_) {
      RCLCPP_WARN(this->get_logger(),
        "Dropping stale frame (frame_time < last_stop_time): %.3f < %.3f",
        frame_time.seconds(), last_stop_time_.seconds());
      return;
    }
  }

  // stop 後に遅れて到着した「旧リクエスト由来のフレーム」を破棄する。
  // publish 時刻 stamp では判別できないため epoch を使う。
  const uint32_t frame_epoch = msg->epoch;
  const uint32_t current_epoch = current_epoch_.load();
  if (frame_epoch != 0 && frame_epoch < current_epoch) {
    RCLCPP_WARN(this->get_logger(),
      "Dropping stale frame (epoch < current_epoch): %u < %u",
      static_cast<unsigned int>(frame_epoch),
      static_cast<unsigned int>(current_epoch));
    return;
  }

  QueuedFrame queued_frame;
  queued_frame.data = std::vector<uint8_t>(msg->data.begin(), msg->data.end());
  queued_frame.header = msg->header;
  queued_frame.epoch = msg->epoch;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (frame_queue_.size() >= config_.max_queue_frames) {
      frame_queue_.pop_front();
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Playback queue overflow, dropping oldest frame");
    }
    frame_queue_.emplace_back(std::move(queued_frame));
  }
  queue_cv_.notify_one();
  RCLCPP_DEBUG_THROTTLE(
    this->get_logger(), *this->get_clock(), 2000,
    "Frame queued for playback");
}

void FvAudioOutputNode::handleVolume(const std_msgs::msg::Float32::SharedPtr msg)
{
  const float next = std::clamp(msg->data, 0.0F, 1.0F);
  const float previous = volume_.exchange(next);
  if (previous != next) {
    RCLCPP_INFO(this->get_logger(), "Master volume: %.0f%%", next * 100.0F);
  }
}

void FvAudioOutputNode::handleStop(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
  RCLCPP_INFO(this->get_logger(), "Stop requested");

  const auto new_epoch = current_epoch_.fetch_add(1) + 1;
  RCLCPP_INFO(this->get_logger(), "Playback epoch advanced: %u", static_cast<unsigned int>(new_epoch));

  // 停止時刻を記録（この時刻より前のフレームは無視される）
  {
    std::lock_guard<std::mutex> lock(stop_time_mutex_);
    last_stop_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Recording stop time: %.3f", last_stop_time_.seconds());
  }

  // 停止フラグを立てる
  stop_requested_.store(true);

  // キューをクリア
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    frame_queue_.clear();
  }

  // ALSA バッファをドロップ（即座に停止）
  if (pcm_handle_) {
    snd_pcm_drop(pcm_handle_);
    snd_pcm_prepare(pcm_handle_);
  }

  // 停止フラグをリセット
  stop_requested_.store(false);

  // 一時停止も解除
  is_paused_.store(false);
  pause_requested_.store(false);

  RCLCPP_INFO(this->get_logger(), "Playback stopped");
}

void FvAudioOutputNode::handlePause(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
  if (is_paused_.load()) {
    RCLCPP_INFO(this->get_logger(), "Already paused, ignoring pause request");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Pause requested");
  pause_requested_.store(true);
  queue_cv_.notify_one();  // playbackThreadを起こす
}

void FvAudioOutputNode::handleResume(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
  if (!is_paused_.load()) {
    RCLCPP_DEBUG(this->get_logger(), "Not paused, ignoring resume request");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Resume requested");
  is_paused_.store(false);
  queue_cv_.notify_all();  // playbackThreadを起こす
}

bool FvAudioOutputNode::validateFrame(const fv_audio::msg::AudioFrame & msg)
{
  if (msg.encoding != kEncodingPcm16) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 3000,
      "Unsupported encoding %s, expected %s", msg.encoding.c_str(), kEncodingPcm16);
    return false;
  }
  if (msg.sample_rate != config_.sample_rate || msg.channels != config_.channels ||
    msg.bit_depth != config_.bit_depth) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 3000,
      "Frame format mismatch: frame=%uHz/%u/%u config=%uHz/%u/%u",
      msg.sample_rate, msg.channels, msg.bit_depth,
      config_.sample_rate, config_.channels, config_.bit_depth);
    return false;
  }
  if (msg.data.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 3000,
      "Empty audio frame data, dropping");
    return false;
  }
  if (bytes_per_frame_ > 0 && (msg.data.size() % bytes_per_frame_) != 0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 3000,
      "Invalid audio frame size: %zu (bytes_per_frame=%zu)",
      msg.data.size(), bytes_per_frame_);
    return false;
  }
  return true;
}

void FvAudioOutputNode::playbackThread()
{
  // チャンクのサンプル数を計算（停止/一時停止の応答性向上用）
  const uint64_t chunk_samples_u64 =
    (static_cast<uint64_t>(config_.sample_rate) * config_.chunk_duration_ms) / 1000;
  const size_t chunk_samples = std::max<size_t>(1, static_cast<size_t>(chunk_samples_u64));
  const size_t chunk_bytes = chunk_samples * bytes_per_frame_;

  rclcpp::Rate idle_rate(50);
  while (running_.load()) {
    // 一時停止中なら待機
    if (is_paused_.load()) {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait_for(lock, std::chrono::milliseconds(100), [this]() {
        return !is_paused_.load() || !running_.load() || stop_requested_.load();
      });
      continue;
    }

    // 一時停止リクエストがあれば即座に停止
    if (pause_requested_.load()) {
      // ALSAバッファをドロップして即座に停止
      if (pcm_handle_) {
        snd_pcm_drop(pcm_handle_);
        snd_pcm_prepare(pcm_handle_);
      }
      is_paused_.store(true);
      pause_requested_.store(false);
      RCLCPP_INFO(this->get_logger(), "Playback paused immediately");
      paused_pub_->publish(std_msgs::msg::Empty());
      continue;
    }

    QueuedFrame queued_frame;
    bool has_frame = false;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait_for(lock, std::chrono::milliseconds(100), [this]() {
        return !frame_queue_.empty() || !running_.load() || pause_requested_.load();
      });
      if (!frame_queue_.empty() && !pause_requested_.load()) {
        queued_frame = std::move(frame_queue_.front());
        frame_queue_.pop_front();
        has_frame = true;
      }
    }

    if (has_frame && !queued_frame.data.empty()) {
      if (!pcm_handle_) {
        if (!openDevice()) {
          RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "ALSA device unavailable, dropping frame");
          continue;
        }
      }

      const float master_volume = volume_.load();
      if (master_volume < 1.0F) {
        applyVolumeScale(queued_frame.data, master_volume);
      }
      size_t total_bytes = queued_frame.data.size();
      size_t bytes_written = 0;
      uint8_t * data_ptr = queued_frame.data.data();
      bool interrupted = false;

      // チャンク単位で書き込み（停止/一時停止の応答性向上）
      while (bytes_written < total_bytes && running_.load()) {
        // 停止/一時停止チェック
        if (stop_requested_.load() || pause_requested_.load()) {
          interrupted = true;
          break;
        }

        // 今回書き込むバイト数（最大chunk_bytes）
        size_t write_bytes = std::min(chunk_bytes, total_bytes - bytes_written);
        size_t write_frames = write_bytes / bytes_per_frame_;

        snd_pcm_sframes_t result = snd_pcm_writei(
          pcm_handle_,
          data_ptr + bytes_written,
          write_frames);

        if (result == -EPIPE) {
          RCLCPP_WARN(this->get_logger(), "XRUN detected, preparing device");
          snd_pcm_prepare(pcm_handle_);
          continue;
        } else if (result == -EAGAIN) {
          // バッファがいっぱい、少し待つ
          std::this_thread::sleep_for(std::chrono::microseconds(500));
          continue;
        } else if (result < 0) {
          RCLCPP_ERROR(this->get_logger(), "snd_pcm_writei failed: %s", snd_strerror(result));
          snd_pcm_prepare(pcm_handle_);
          break;
        }

        bytes_written += static_cast<size_t>(result) * bytes_per_frame_;
      }

      if (interrupted) {
        // 中断時: ALSAバッファをドロップして即停止
        if (pcm_handle_) {
          snd_pcm_drop(pcm_handle_);
          snd_pcm_prepare(pcm_handle_);
        }
        RCLCPP_INFO(this->get_logger(), "Playback interrupted, dropped remaining audio");
      } else if (bytes_written > 0) {
        // Discrete cues can be shorter than the ALSA buffer. Optionally wait
        // until the hardware has consumed the complete frame before reporting
        // completion or accepting an interrupting cue.
        if (config_.drain_after_frame && pcm_handle_) {
          const int drain_result = snd_pcm_drain(pcm_handle_);
          if (drain_result < 0) {
            RCLCPP_WARN(
              this->get_logger(), "snd_pcm_drain failed: %s", snd_strerror(drain_result));
          }
          const int prepare_result = snd_pcm_prepare(pcm_handle_);
          if (prepare_result < 0) {
            RCLCPP_ERROR(
              this->get_logger(), "snd_pcm_prepare after drain failed: %s",
              snd_strerror(prepare_result));
          }
        }
        RCLCPP_INFO(this->get_logger(), "Playback done, publishing notification");
        fv_audio::msg::PlaybackDone done_msg;
        done_msg.header = queued_frame.header;
        done_msg.request_id = queued_frame.header.frame_id;
        done_msg.epoch = queued_frame.epoch;
        playback_done_pub_->publish(done_msg);
      }
    } else {
      // キューが空の場合のみ、残りのALSAバッファを再生完了待ち
      if (pcm_handle_) {
        snd_pcm_state_t state = snd_pcm_state(pcm_handle_);
        if (state == SND_PCM_STATE_RUNNING) {
          // まだ再生中なら少し待つ
          idle_rate.sleep();
          continue;
        }
      }
      idle_rate.sleep();
    }
  }
}

void FvAudioOutputNode::handlePlayFile(
  const std::shared_ptr<fv_audio_output::srv::PlayFile::Request> request,
  std::shared_ptr<fv_audio_output::srv::PlayFile::Response> response)
{
  std::vector<uint8_t> wav_data;
  uint32_t sample_rate, channels, bit_depth;

  if (!loadWavFile(request->file_path, wav_data, sample_rate, channels, bit_depth)) {
    response->success = false;
    response->message = "Failed to load WAV file: " + request->file_path;
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  if (sample_rate != config_.sample_rate || channels != config_.channels ||
    bit_depth != config_.bit_depth)
  {
    response->success = false;
    response->message = "WAV format mismatch: file=" +
      std::to_string(sample_rate) + "Hz/" + std::to_string(channels) + "ch/" +
      std::to_string(bit_depth) + "bit, expected=" +
      std::to_string(config_.sample_rate) + "Hz/" + std::to_string(config_.channels) + "ch/" +
      std::to_string(config_.bit_depth) + "bit";
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  if (request->volume_scale > 0.0f && request->volume_scale != 1.0f) {
    applyVolumeScale(wav_data, request->volume_scale);
  }

  QueuedFrame queued_frame;
  queued_frame.data = std::move(wav_data);
  queued_frame.header.stamp = this->now();
  queued_frame.header.frame_id = "file:" + request->file_path;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (request->interrupt) {
      frame_queue_.clear();
    }
    if (frame_queue_.size() >= config_.max_queue_frames) {
      frame_queue_.pop_front();
      RCLCPP_WARN(this->get_logger(), "Queue overflow, dropping oldest frame for file playback");
    }
    frame_queue_.emplace_back(std::move(queued_frame));
  }
  queue_cv_.notify_one();

  response->success = true;
  response->message = "File queued for playback";
  RCLCPP_INFO(this->get_logger(), "Queued audio file: %s", request->file_path.c_str());
}

bool FvAudioOutputNode::loadWavFile(const std::string & file_path, std::vector<uint8_t> & out_data,
  uint32_t & out_sample_rate, uint32_t & out_channels, uint32_t & out_bit_depth)
{
  std::ifstream file(file_path, std::ios::binary);
  if (!file) {
    RCLCPP_ERROR(this->get_logger(), "Cannot open file: %s", file_path.c_str());
    return false;
  }

  // Read RIFF header
  char riff[4];
  file.read(riff, 4);
  if (std::strncmp(riff, "RIFF", 4) != 0) {
    RCLCPP_ERROR(this->get_logger(), "Not a valid RIFF file");
    return false;
  }

  uint32_t file_size;
  file.read(reinterpret_cast<char*>(&file_size), 4);

  char wave[4];
  file.read(wave, 4);
  if (std::strncmp(wave, "WAVE", 4) != 0) {
    RCLCPP_ERROR(this->get_logger(), "Not a valid WAVE file");
    return false;
  }

  // Find fmt chunk
  bool found_fmt = false;
  uint16_t audio_format = 0;
  uint32_t byte_rate = 0;
  uint16_t block_align = 0;
  uint16_t channels16 = 0;
  uint16_t bit_depth16 = 0;

  while (file) {
    char chunk_id[4];
    file.read(chunk_id, 4);
    if (!file) break;

    uint32_t chunk_size;
    file.read(reinterpret_cast<char*>(&chunk_size), 4);

    if (std::strncmp(chunk_id, "fmt ", 4) == 0) {
      file.read(reinterpret_cast<char*>(&audio_format), 2);
      file.read(reinterpret_cast<char*>(&channels16), 2);
      file.read(reinterpret_cast<char*>(&out_sample_rate), 4);
      file.read(reinterpret_cast<char*>(&byte_rate), 4);
      file.read(reinterpret_cast<char*>(&block_align), 2);
      file.read(reinterpret_cast<char*>(&bit_depth16), 2);

      out_channels = static_cast<uint32_t>(channels16);
      out_bit_depth = static_cast<uint32_t>(bit_depth16);

      // Skip remaining fmt chunk
      file.seekg(chunk_size - 16, std::ios::cur);
      found_fmt = true;
    } else if (std::strncmp(chunk_id, "data", 4) == 0) {
      if (!found_fmt) {
        RCLCPP_ERROR(this->get_logger(), "data chunk before fmt chunk");
        return false;
      }

      out_data.resize(chunk_size);
      file.read(reinterpret_cast<char*>(out_data.data()), chunk_size);

      if (audio_format != 1) {  // PCM
        RCLCPP_ERROR(this->get_logger(), "Unsupported audio format: %u (expected PCM=1)", audio_format);
        return false;
      }

      return true;
    } else {
      // Skip unknown chunk
      file.seekg(chunk_size, std::ios::cur);
    }
  }

  RCLCPP_ERROR(this->get_logger(), "No data chunk found in WAV file");
  return false;
}

void FvAudioOutputNode::applyVolumeScale(std::vector<uint8_t> & data, float volume_scale)
{
  if (config_.bit_depth == 16) {
    int16_t * samples = reinterpret_cast<int16_t*>(data.data());
    size_t num_samples = data.size() / 2;
    for (size_t i = 0; i < num_samples; ++i) {
      int32_t scaled = static_cast<int32_t>(samples[i] * volume_scale);
      samples[i] = static_cast<int16_t>(std::clamp(scaled, -32768, 32767));
    }
  }
}

}  // namespace fv_audio_output

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fv_audio_output::FvAudioOutputNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
