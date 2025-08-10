#pragma once

#include <chrono>
#include <string>
#include <unordered_map>
#include <mutex>

namespace fluent_lib::ros::time
{

class Stopwatch {
public:
  void start() {
    std::lock_guard<std::mutex> lk(m_);
    if (!running_) { t0_ = std::chrono::steady_clock::now(); running_ = true; }
  }
  // Returns elapsed ms of this interval (and accumulates)
  long stop() {
    std::lock_guard<std::mutex> lk(m_);
    if (running_) {
      auto t1 = std::chrono::steady_clock::now();
      total_ms_ += std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0_).count();
      running_ = false;
    }
    return total_ms_;
  }
  void reset() {
    std::lock_guard<std::mutex> lk(m_);
    running_ = false; total_ms_ = 0; }
  // Non-stopping read
  long elapsed_ms() const {
    std::lock_guard<std::mutex> lk(m_);
    if (!running_) return total_ms_;
    auto now = std::chrono::steady_clock::now();
    auto cur = std::chrono::duration_cast<std::chrono::milliseconds>(now - t0_).count();
    return total_ms_ + cur;
  }
  bool running() const { std::lock_guard<std::mutex> lk(m_); return running_; }

private:
  mutable std::mutex m_;
  bool running_ {false};
  std::chrono::steady_clock::time_point t0_ {};
  long total_ms_ {0};
};

class TimerRegistry {
public:
  class Ref {
  public:
    Ref(TimerRegistry* reg, std::string key) : reg_(reg), key_(std::move(key)) {}
    void start() { reg_->get(key_).start(); }
    long stop() { return reg_->get(key_).stop(); }
    void reset() { reg_->get(key_).reset(); }
    long elapsed_ms() const { return reg_->get_const(key_).elapsed_ms(); }
    bool running() const { return reg_->get_const(key_).running(); }
  private:
    TimerRegistry* reg_;
    std::string key_;
  };

  Ref operator[](const std::string& name) { return Ref(this, name); }

private:
  Stopwatch& get(const std::string& name) {
    std::lock_guard<std::mutex> lk(m_);
    return timers_[name];
  }
  const Stopwatch& get_const(const std::string& name) const {
    std::lock_guard<std::mutex> lk(m_);
    auto it = timers_.find(name);
    if (it == timers_.end()) return dummy_;
    return it->second;
  }

  mutable std::mutex m_;
  mutable Stopwatch dummy_;
  std::unordered_map<std::string, Stopwatch> timers_;
};

} // namespace fluent_lib::ros::time


