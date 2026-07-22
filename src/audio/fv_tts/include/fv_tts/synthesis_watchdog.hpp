#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

namespace fv_tts {

using SynthesisTimeout = std::chrono::duration<double>;

SynthesisTimeout checked_synthesis_timeout(double seconds);

class SynthesisWatchdog {
 public:
  SynthesisWatchdog(SynthesisTimeout timeout, std::string context);
  ~SynthesisWatchdog();

  SynthesisWatchdog(const SynthesisWatchdog &) = delete;
  SynthesisWatchdog &operator=(const SynthesisWatchdog &) = delete;

 private:
  void run();

  SynthesisTimeout timeout_;
  std::string context_;
  std::mutex mutex_;
  std::condition_variable condition_;
  bool started_{false};
  bool completed_{false};
  std::thread worker_;
};

}  // namespace fv_tts
