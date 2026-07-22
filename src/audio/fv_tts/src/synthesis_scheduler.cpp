#include "fv_tts/synthesis_scheduler.hpp"

#include <algorithm>
#include <exception>
#include <stdexcept>
#include <utility>

#include <nlohmann/json.hpp>

namespace fv_tts {
namespace {

std::string current_exception_message() {
  try {
    throw;
  } catch (const std::exception &error) {
    const std::string message = error.what();
    return message.empty() ? "native synthesis failure" : message;
  } catch (...) {
    return "unknown native synthesis failure";
  }
}

std::string request_synthesis_context(const SayRequest &request) {
  return std::string("request synthesis kind=") + speech_kind_name(request.kind) +
         " utterance_id=" + nlohmann::json(request.utterance_id).dump();
}

}  // namespace

SynthesisScheduler::SynthesisScheduler(Synthesize synthesize,
                                       Completed completed, Failed failed,
                                       Cancelled cancelled,
                                       SynthesisTimeout synthesis_timeout)
    : synthesize_(std::move(synthesize)),
      completed_(std::move(completed)),
      failed_(std::move(failed)),
      cancelled_(std::move(cancelled)),
      synthesis_timeout_(
          checked_synthesis_timeout(synthesis_timeout.count())),
      worker_(&SynthesisScheduler::run, this) {
  if (!synthesize_ || !completed_ || !failed_ || !cancelled_) {
    close();
    throw std::invalid_argument("synthesis scheduler callbacks must be set");
  }
}

SynthesisScheduler::~SynthesisScheduler() { close(); }

SubmitStatus SynthesisScheduler::submit(const SayRequest &request) {
  const auto epoch = request.kind == SpeechKind::kAgent
                         ? std::optional<std::uint64_t>(
                               agent_floor_epoch(request.utterance_id))
                         : std::nullopt;
  SubmitStatus status;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {
      throw std::runtime_error("synthesis scheduler is closed");
    }
    if (request.kind == SpeechKind::kAgent) {
      if (*epoch < minimum_agent_epoch_) {
        status = SubmitStatus::kStale;
      } else {
        agent_.push_back(request);
        status = SubmitStatus::kAccepted;
      }
    } else if (cancelled_system_.count(request.utterance_id) != 0U) {
      forget_cancelled_system_locked(request.utterance_id);
      status = SubmitStatus::kCancelled;
    } else {
      system_.push_back(request);
      status = SubmitStatus::kAccepted;
    }
    if (status == SubmitStatus::kAccepted) {
      condition_.notify_one();
    }
  }
  if (status != SubmitStatus::kAccepted) {
    report_cancelled(request);
  }
  return status;
}

std::vector<std::string> SynthesisScheduler::advance_agent_floor(
    std::uint64_t floor_epoch) {
  std::vector<SayRequest> removed;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    minimum_agent_epoch_ = std::max(minimum_agent_epoch_, floor_epoch);
    std::deque<SayRequest> kept;
    while (!agent_.empty()) {
      auto request = std::move(agent_.front());
      agent_.pop_front();
      if (agent_floor_epoch(request.utterance_id) < minimum_agent_epoch_) {
        removed.push_back(std::move(request));
      } else {
        kept.push_back(std::move(request));
      }
    }
    agent_ = std::move(kept);
  }
  std::vector<std::string> ids;
  ids.reserve(removed.size());
  for (const auto &request : removed) {
    ids.push_back(request.utterance_id);
    report_cancelled(request);
  }
  return ids;
}

bool SynthesisScheduler::cancel_system(const std::string &utterance_id) {
  if (utterance_id.empty()) {
    throw std::invalid_argument("system cancellation requires utterance_id");
  }
  std::vector<SayRequest> removed;
  bool active = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::deque<SayRequest> kept;
    while (!system_.empty()) {
      auto request = std::move(system_.front());
      system_.pop_front();
      if (request.utterance_id == utterance_id) {
        removed.push_back(std::move(request));
      } else {
        kept.push_back(std::move(request));
      }
    }
    system_ = std::move(kept);
    active = active_.has_value() && active_->kind == SpeechKind::kSystem &&
             active_->utterance_id == utterance_id;
    if (active || removed.empty()) {
      remember_cancelled_system_locked(utterance_id);
    }
  }
  for (const auto &request : removed) {
    report_cancelled(request);
  }
  return !removed.empty() || active;
}

void SynthesisScheduler::close() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!closed_) {
      closed_ = true;
      agent_.clear();
      system_.clear();
    }
    condition_.notify_one();
  }
  if (worker_.joinable()) {
    worker_.join();
  }
}

void SynthesisScheduler::rethrow_if_failed() const {
  std::exception_ptr error;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    error = fatal_error_;
  }
  if (error) {
    std::rethrow_exception(error);
  }
}

void SynthesisScheduler::run() {
  while (true) {
    SayRequest request;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock,
                      [this] { return closed_ || !system_.empty() || !agent_.empty(); });
      if (closed_) {
        return;
      }
      if (!system_.empty()) {
        request = std::move(system_.front());
        system_.pop_front();
      } else {
        request = std::move(agent_.front());
        agent_.pop_front();
      }
      active_ = request;
    }

    try {
      auto audio = [&] {
        SynthesisWatchdog watchdog(synthesis_timeout_,
                                   request_synthesis_context(request));
        return synthesize_(request.text);
      }();
      if (!commit_audio_if_current(request, audio)) {
        report_cancelled(request);
      }
    } catch (...) {
      const auto error = current_exception_message();
      if (!commit_failure_if_current(request, error)) {
        report_cancelled(request);
      }
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (request.kind == SpeechKind::kSystem) {
        forget_cancelled_system_locked(request.utterance_id);
      }
      active_.reset();
    }
  }
}

bool SynthesisScheduler::is_current_locked(const SayRequest &request) const {
  if (closed_) {
    return false;
  }
  if (request.kind == SpeechKind::kAgent) {
    return agent_floor_epoch(request.utterance_id) >= minimum_agent_epoch_;
  }
  return cancelled_system_.count(request.utterance_id) == 0U;
}

bool SynthesisScheduler::commit_audio_if_current(
    const SayRequest &request, const SynthesizedAudio &audio) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!is_current_locked(request)) {
    return false;
  }
  try {
    completed_(request, audio);
  } catch (...) {
    record_fatal_locked(std::current_exception());
  }
  return true;
}

bool SynthesisScheduler::commit_failure_if_current(const SayRequest &request,
                                                   const std::string &error) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!is_current_locked(request)) {
    return false;
  }
  try {
    failed_(request, error);
  } catch (...) {
    record_fatal_locked(std::current_exception());
  }
  return true;
}

void SynthesisScheduler::record_fatal_locked(std::exception_ptr error) noexcept {
  if (!fatal_error_) {
    fatal_error_ = std::move(error);
  }
  closed_ = true;
  agent_.clear();
  system_.clear();
  condition_.notify_all();
}

void SynthesisScheduler::report_cancelled(const SayRequest &request) noexcept {
  try {
    cancelled_(request);
  } catch (...) {
    std::lock_guard<std::mutex> lock(mutex_);
    record_fatal_locked(std::current_exception());
  }
}

void SynthesisScheduler::remember_cancelled_system_locked(
    const std::string &utterance_id) {
  if (cancelled_system_.count(utterance_id) != 0U) {
    return;
  }
  if (cancelled_system_order_.size() >= kMaxSystemAbortTombstones) {
    cancelled_system_.erase(cancelled_system_order_.front());
    cancelled_system_order_.pop_front();
  }
  cancelled_system_.insert(utterance_id);
  cancelled_system_order_.push_back(utterance_id);
}

void SynthesisScheduler::forget_cancelled_system_locked(
    const std::string &utterance_id) {
  if (cancelled_system_.erase(utterance_id) == 0U) {
    return;
  }
  const auto entry = std::find(cancelled_system_order_.begin(),
                               cancelled_system_order_.end(), utterance_id);
  if (entry != cancelled_system_order_.end()) {
    cancelled_system_order_.erase(entry);
  }
}

const char *submit_status_name(SubmitStatus status) {
  switch (status) {
    case SubmitStatus::kAccepted:
      return "accepted";
    case SubmitStatus::kStale:
      return "stale";
    case SubmitStatus::kCancelled:
      return "cancelled";
  }
  return "unknown";
}

}  // namespace fv_tts
