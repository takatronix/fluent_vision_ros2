#pragma once

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <exception>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "fv_tts/contracts.hpp"
#include "fv_tts/synthesis_watchdog.hpp"

namespace fv_tts {

struct SynthesizedAudio {
  std::vector<std::uint8_t> pcm;
  std::uint32_t sample_rate_hz;
  std::uint32_t channels;
  std::uint32_t bit_depth{16};
  std::vector<SynthesisMark> marks;
};

enum class SubmitStatus { kAccepted, kStale, kCancelled };

class SynthesisScheduler {
 public:
  using Synthesize = std::function<SynthesizedAudio(const std::string &)>;
  using Completed =
      std::function<void(const SayRequest &, const SynthesizedAudio &)>;
  using Failed = std::function<void(const SayRequest &, const std::string &)>;
  using Cancelled = std::function<void(const SayRequest &)>;

  SynthesisScheduler(Synthesize synthesize, Completed completed, Failed failed,
                     Cancelled cancelled,
                     SynthesisTimeout synthesis_timeout =
                         SynthesisTimeout{60.0});
  ~SynthesisScheduler();

  SynthesisScheduler(const SynthesisScheduler &) = delete;
  SynthesisScheduler &operator=(const SynthesisScheduler &) = delete;

  SubmitStatus submit(const SayRequest &request);
  std::vector<std::string> advance_agent_floor(std::uint64_t floor_epoch);
  bool cancel_system(const std::string &utterance_id);
  void rethrow_if_failed() const;
  void close();

 private:
  static constexpr std::size_t kMaxSystemAbortTombstones = 64;

  void run();
  bool is_current_locked(const SayRequest &request) const;
  bool commit_audio_if_current(const SayRequest &request,
                               const SynthesizedAudio &audio);
  bool commit_failure_if_current(const SayRequest &request,
                                 const std::string &error);
  void report_cancelled(const SayRequest &request) noexcept;
  void record_fatal_locked(std::exception_ptr error) noexcept;
  void remember_cancelled_system_locked(const std::string &utterance_id);
  void forget_cancelled_system_locked(const std::string &utterance_id);

  Synthesize synthesize_;
  Completed completed_;
  Failed failed_;
  Cancelled cancelled_;
  std::deque<SayRequest> agent_;
  std::deque<SayRequest> system_;
  std::uint64_t minimum_agent_epoch_{0};
  std::unordered_set<std::string> cancelled_system_;
  std::deque<std::string> cancelled_system_order_;
  std::optional<SayRequest> active_;
  std::exception_ptr fatal_error_;
  bool closed_{false};
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  SynthesisTimeout synthesis_timeout_;
  std::thread worker_;
};

const char *submit_status_name(SubmitStatus status);

}  // namespace fv_tts
