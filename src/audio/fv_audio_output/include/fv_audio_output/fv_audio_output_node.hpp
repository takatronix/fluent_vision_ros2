#pragma once

#include <alsa/asoundlib.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "fv_audio/msg/audio_frame.hpp"
#include "fv_audio/msg/playback_done.hpp"
#include "fv_audio_output/srv/play_file.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/header.hpp"

namespace fv_audio_output
{

struct OutputConfig
{
  std::string device_id = "default";
  std::string volume_topic = "/audio/output/volume";
  float volume = 1.0F;
  uint32_t sample_rate = 48000;
  uint32_t channels = 1;
  uint32_t bit_depth = 16;
  size_t max_queue_frames = 8;
  uint32_t chunk_duration_ms = 30;  // 内部チャンク分割サイズ（停止応答性向上用）
  uint32_t start_threshold_frames = 0;  // 0 = ALSA buffer の半分
  size_t qos_depth = 10;
  bool qos_reliable = true;
  bool drain_after_frame = false;
  bool reconnect_enabled = true;
  uint32_t reconnect_interval_ms = 1000;
  std::string mixer_card;
  std::string mixer_control = "PCM";
  int32_t mixer_volume_percent = -1;  // -1 = leave the hardware mixer unchanged
};

// 再生キュー用の構造体（フレームデータとヘッダーをペアで保持）
struct QueuedFrame
{
  std::vector<uint8_t> data;
  std_msgs::msg::Header header;
  uint32_t epoch{0};
};

class FvAudioOutputNode : public rclcpp::Node
{
public:
  FvAudioOutputNode();
  ~FvAudioOutputNode() override;

private:
  void loadParameters();
  bool openDevice();
  void closeDevice();
  void closeDeviceLocked();
  bool applyMixerSettings();
  bool deviceIsOpen();
  bool waitForReconnect();
  void playbackThread();
  void handleFrame(const fv_audio::msg::AudioFrame::SharedPtr msg);
  void handleVolume(const std_msgs::msg::Float32::SharedPtr msg);
  void handleStop(const std_msgs::msg::Empty::SharedPtr msg);
  void handlePause(const std_msgs::msg::Empty::SharedPtr msg);
  void handleResume(const std_msgs::msg::Empty::SharedPtr msg);
  bool validateFrame(const fv_audio::msg::AudioFrame & msg);
  void handlePlayFile(
    const std::shared_ptr<fv_audio_output::srv::PlayFile::Request> request,
    std::shared_ptr<fv_audio_output::srv::PlayFile::Response> response);
  bool loadWavFile(const std::string & file_path, std::vector<uint8_t> & out_data,
    uint32_t & out_sample_rate, uint32_t & out_channels, uint32_t & out_bit_depth);
  void applyVolumeScale(std::vector<uint8_t> & data, float volume_scale);

  OutputConfig config_;
  snd_pcm_t * pcm_handle_ = nullptr;
  std::mutex pcm_mutex_;
  size_t bytes_per_frame_ = 0;

  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<QueuedFrame> frame_queue_;
  std::thread playback_thread_;
  std::atomic<bool> running_{false};

  rclcpp::Subscription<fv_audio::msg::AudioFrame>::SharedPtr audio_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr volume_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr pause_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr resume_sub_;
  rclcpp::Service<fv_audio_output::srv::PlayFile>::SharedPtr play_file_srv_;
  rclcpp::Publisher<fv_audio::msg::PlaybackDone>::SharedPtr playback_done_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr paused_pub_;  // 一時停止完了通知

  std::atomic<float> volume_{1.0F};
  std::atomic<bool> stop_requested_{false};  // 停止リクエストフラグ
  std::atomic<bool> pause_requested_{false};  // 一時停止リクエストフラグ
  std::atomic<bool> is_paused_{false};  // 一時停止中フラグ

  // Barge-in 後の古いフレームをフィルタリングするためのタイムスタンプ
  std::mutex stop_time_mutex_;
  rclcpp::Time last_stop_time_{0, 0, RCL_ROS_TIME};

  // stop 世代（epoch）。
  // stop を受けたらインクリメントし、古い epoch のフレームは破棄する。
  std::atomic<uint32_t> current_epoch_{1};
};

}  // namespace fv_audio_output
