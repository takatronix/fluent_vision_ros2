#include "fv_tts/synthesis_watchdog.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <utility>

namespace fv_tts {

SynthesisTimeout checked_synthesis_timeout(double seconds) {
  if (!std::isfinite(seconds) || seconds <= 0.0) {
    throw std::invalid_argument(
        "synthesis_timeout_seconds must be finite and greater than zero");
  }
  return SynthesisTimeout(seconds);
}

SynthesisWatchdog::SynthesisWatchdog(SynthesisTimeout timeout,
                                     std::string context)
    : timeout_(checked_synthesis_timeout(timeout.count())),
      context_(std::move(context)) {
  if (context_.empty()) {
    throw std::invalid_argument("synthesis watchdog context must not be empty");
  }

  worker_ = std::thread(&SynthesisWatchdog::run, this);
  std::unique_lock<std::mutex> lock(mutex_);
  condition_.wait(lock, [this] { return started_; });
}

SynthesisWatchdog::~SynthesisWatchdog() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    completed_ = true;
  }
  condition_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

void SynthesisWatchdog::run() {
  std::unique_lock<std::mutex> lock(mutex_);
  started_ = true;
  condition_.notify_all();
  if (condition_.wait_for(lock, timeout_, [this] { return completed_; })) {
    return;
  }
  lock.unlock();

  std::fprintf(
      stderr,
      "fv_tts fatal: VOICEVOX Core native synthesis exceeded %.3f seconds; "
      "context=%s\n",
      timeout_.count(), context_.c_str());
  std::fflush(stderr);
  std::_Exit(EXIT_FAILURE);
}

}  // namespace fv_tts
