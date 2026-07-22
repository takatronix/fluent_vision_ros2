#include <aspa_audio_interfaces/msg/playback_control.hpp>
#include <fv_speech_interfaces/msg/audio_frame.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "fv_tts/contracts.hpp"
#include "fv_tts/synthesis_scheduler.hpp"
#include "fv_tts/synthesis_watchdog.hpp"
#include "fv_tts/voicevox_backend.hpp"

namespace fv_tts {
namespace {

using PlaybackControl = aspa_audio_interfaces::msg::PlaybackControl;
using AudioFrame = fv_speech_interfaces::msg::AudioFrame;
using String = std_msgs::msg::String;

void validate_playback_control(const PlaybackControl &control) {
  if (control.action < PlaybackControl::PAUSE ||
      control.action > PlaybackControl::ABORT_SYSTEM) {
    throw std::invalid_argument("unsupported playback action");
  }
  if (control.release_hold > PlaybackControl::SYSTEM) {
    throw std::invalid_argument("invalid playback release_hold");
  }
  if (control.action == PlaybackControl::PAUSE ||
      control.action == PlaybackControl::RESUME) {
    if (control.minimum_agent_epoch != 0 ||
        control.release_hold != PlaybackControl::NONE ||
        !control.utterance_id.empty()) {
      throw std::invalid_argument(
          "pause/resume contains fields that must be empty");
    }
    return;
  }
  if (control.action == PlaybackControl::DISCARD) {
    if (control.release_hold != PlaybackControl::USER &&
        control.release_hold != PlaybackControl::SYSTEM) {
      throw std::invalid_argument(
          "discard requires release_hold=user or system");
    }
    if (control.release_hold == PlaybackControl::SYSTEM &&
        control.utterance_id.empty()) {
      throw std::invalid_argument("system discard requires utterance_id");
    }
    if (control.release_hold == PlaybackControl::USER &&
        !control.utterance_id.empty()) {
      throw std::invalid_argument("user discard must not specify utterance_id");
    }
    return;
  }
  if (control.utterance_id.empty() || control.minimum_agent_epoch != 0 ||
      (control.release_hold != PlaybackControl::NONE &&
       control.release_hold != PlaybackControl::SYSTEM)) {
    throw std::invalid_argument("system_abort fields are invalid");
  }
}

std::uint16_t checked_threads(std::int64_t value) {
  if (value < 0 || value > std::numeric_limits<std::uint16_t>::max()) {
    throw std::invalid_argument(
        "cpu_num_threads must be between 0 and 65535");
  }
  return static_cast<std::uint16_t>(value);
}

std::uint32_t checked_style(std::int64_t value) {
  if (value < 0 ||
      static_cast<std::uint64_t>(value) >
          std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument(
        "style_id must be a non-negative 32-bit integer");
  }
  return static_cast<std::uint32_t>(value);
}

}  // namespace

class FvTtsNode final : public rclcpp::Node {
 public:
  FvTtsNode() : Node("fv_tts") {
    VoicevoxConfig config{
        declare_parameter<std::string>("onnxruntime_filename", ""),
        declare_parameter<std::string>("open_jtalk_dict_dir", ""),
        declare_parameter<std::string>("voice_model_path", ""),
        declare_parameter<std::string>("acceleration_mode", "auto"),
        checked_threads(declare_parameter<std::int64_t>("cpu_num_threads", 0)),
        checked_style(declare_parameter<std::int64_t>("style_id", 30))};
    synthesis_timeout_ = checked_synthesis_timeout(
        declare_parameter<double>("synthesis_timeout_seconds", 60.0));
    backend_ = std::make_unique<VoicevoxCoreBackend>(config);

    // Loading a model does not prove that its selected style can synthesize.
    // Exercise the complete native path before exposing the ready service.
    {
      SynthesisWatchdog watchdog(synthesis_timeout_,
                                 "startup smoke synthesis");
      (void)backend_->synthesize("起動確認");
    }

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                         .reliable()
                         .durability_volatile();
    agent_pub_ = create_publisher<AudioFrame>("/audio/agent/frame", qos);
    system_pub_ = create_publisher<AudioFrame>("/audio/system/frame", qos);
    result_pub_ = create_publisher<String>("/aspa/tts/result", qos);

    scheduler_ = std::make_unique<SynthesisScheduler>(
        [this](const std::string &text) { return backend_->synthesize(text); },
        [this](const SayRequest &request, const SynthesizedAudio &audio) {
          publish_audio(request, audio);
        },
        [this](const SayRequest &request, const std::string &error) {
          on_synthesis_failed(request, error);
        },
        [this](const SayRequest &request) { on_synthesis_cancelled(request); },
        synthesis_timeout_);
    scheduler_health_timer_ = create_wall_timer(
        std::chrono::milliseconds(10),
        [this] { scheduler_->rethrow_if_failed(); });

    say_subscription_ = create_subscription<String>(
        "/aspa/tts/say", qos,
        [this](const String::ConstSharedPtr message) { on_say(*message); });
    playback_subscription_ = create_subscription<PlaybackControl>(
        "/audio/playback/control", qos,
        [this](const PlaybackControl::ConstSharedPtr message) {
          on_playback_control(*message);
        });

    const auto settings_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                                  .reliable()
                                  .transient_local();
    voices_pub_ = create_publisher<String>("/aspa/tts/voices", settings_qos);
    settings_subscription_ = create_subscription<String>(
        "/aspa/tts/settings", settings_qos,
        [this](const String::ConstSharedPtr message) { on_settings(*message); });
    publish_voice_catalog();

    ready_service_ = create_service<std_srvs::srv::Trigger>(
        "/aspa/tts/ready",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          scheduler_->rethrow_if_failed();
          response->success = true;
          response->message = "voicevox_core synthesis smoke passed";
        });
  }

  ~FvTtsNode() override {
    if (scheduler_) {
      scheduler_->close();
    }
  }

 private:
  void on_say(const String &message) {
    SayRequest request;
    try {
      request = parse_say_request(message.data);
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "%s", error.what());
      return;
    }
    try {
      const auto status = scheduler_->submit(request);
      if (status != SubmitStatus::kAccepted) {
        RCLCPP_WARN(get_logger(), "dropping %s TTS request '%s'",
                    submit_status_name(status), request.utterance_id.c_str());
      }
    } catch (const std::exception &error) {
      on_synthesis_failed(request, error.what());
    }
  }

  void on_settings(const String &message) {
    try {
      const auto settings = parse_tts_settings(message.data);
      const bool changed = backend_->set_style_id(settings.style_id);
      if (changed) {
        RCLCPP_INFO(get_logger(), "VOICEVOX style changed to %u",
                    settings.style_id);
      }
      publish_voice_catalog();
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "%s", error.what());
    }
  }

  void publish_voice_catalog() {
    String message;
    message.data = voice_catalog_json(backend_->style_id(), backend_->voices());
    voices_pub_->publish(message);
  }

  void on_playback_control(const PlaybackControl &control) {
    try {
      validate_playback_control(control);
      if (control.action == PlaybackControl::DISCARD) {
        const auto removed =
            scheduler_->advance_agent_floor(control.minimum_agent_epoch);
        if (!removed.empty()) {
          RCLCPP_INFO(
              get_logger(),
              "cancelled %zu stale queued agent synthesis request(s)",
              removed.size());
        }
      } else if (control.action == PlaybackControl::ABORT_SYSTEM) {
        (void)scheduler_->cancel_system(control.utterance_id);
      }
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "%s", error.what());
    }
  }

  void publish_audio(const SayRequest &request,
                     const SynthesizedAudio &audio) {
    if (audio.channels == 0 || audio.bit_depth != 16 || audio.pcm.empty() ||
        audio.pcm.size() % (static_cast<std::size_t>(audio.channels) * 2U) !=
            0U) {
      throw std::runtime_error("VOICEVOX returned empty or malformed audio");
    }
    const auto frame_count =
        audio.pcm.size() / (static_cast<std::size_t>(audio.channels) * 2U);
    if (frame_count > std::numeric_limits<std::uint32_t>::max()) {
      throw std::runtime_error("VOICEVOX audio exceeds AudioFrame capacity");
    }

    AudioFrame message;
    message.header.stamp = get_clock()->now();
    message.source_id = "fv_tts";
    message.stream_id = request.utterance_id;
    message.seq = 0;
    message.sample_index = 0;
    message.capture_time_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
    message.frame_count = static_cast<std::uint32_t>(frame_count);
    message.encoding = "PCM16LE";
    message.sample_rate_hz = audio.sample_rate_hz;
    message.channels = audio.channels;
    message.bit_depth = audio.bit_depth;
    message.layout = "interleaved";
    message.data = audio.pcm;
    message.final = true;

    if (request.kind == SpeechKind::kAgent) {
      agent_pub_->publish(message);
    } else {
      system_pub_->publish(message);
    }
    publish_result(request, ResultStatus::kCompleted);
  }

  void on_synthesis_failed(const SayRequest &request,
                           const std::string &error_value) {
    auto error = trim_unicode_whitespace(error_value);
    if (error.empty()) {
      error = "native synthesis failure";
    }
    RCLCPP_ERROR(get_logger(), "VOICEVOX synthesis failed for %s: %s",
                 request.utterance_id.c_str(), error.c_str());
    publish_result(request, ResultStatus::kFailed, error);
  }

  void on_synthesis_cancelled(const SayRequest &request) {
    publish_result(request, ResultStatus::kCancelled);
  }

  void publish_result(const SayRequest &request, ResultStatus status,
                      const std::string &error = {}) {
    String message;
    message.data =
        tts_result_json(request.kind, request.utterance_id, status, error);
    result_pub_->publish(message);
  }

  std::unique_ptr<VoicevoxCoreBackend> backend_;
  std::unique_ptr<SynthesisScheduler> scheduler_;
  SynthesisTimeout synthesis_timeout_{60.0};
  rclcpp::Publisher<AudioFrame>::SharedPtr agent_pub_;
  rclcpp::Publisher<AudioFrame>::SharedPtr system_pub_;
  rclcpp::Publisher<String>::SharedPtr result_pub_;
  rclcpp::Publisher<String>::SharedPtr voices_pub_;
  rclcpp::Subscription<String>::SharedPtr say_subscription_;
  rclcpp::Subscription<PlaybackControl>::SharedPtr playback_subscription_;
  rclcpp::Subscription<String>::SharedPtr settings_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ready_service_;
  rclcpp::TimerBase::SharedPtr scheduler_health_timer_;
};

}  // namespace fv_tts

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<fv_tts::FvTtsNode>());
  } catch (const std::exception &error) {
    std::fprintf(stderr, "fv_tts failed: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
