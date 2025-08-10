#pragma once

#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <atomic>
#include <thread>

namespace fluent_lib::async {

class Worker {
public:
  // idle_fn はキューが空のときに呼ばれる（不要なら空ラムダでOK）
  explicit Worker(std::function<void()> idle_fn = {}) : idle_(std::move(idle_fn)) {
    thread_ = std::thread([this]{ run(); });
  }

  ~Worker() { stop(); }

  void post(std::function<void()> job) {
    {
      std::scoped_lock lk(m_);
      q_.push(std::move(job));
    }
    cv_.notify_one();
  }

  void stop() {
    if (!stopped_.exchange(true)) {
      // C++17 std::thread has no request_stop. Use flag + cv to exit.
      cv_.notify_all();
      if (thread_.joinable()) thread_.join();
    }
  }

private:
  void run() {
    for (;;) {
      std::function<void()> job;
      {
        std::unique_lock lk(m_);
        cv_.wait(lk, [this]{ return stopped_.load() || !q_.empty(); });
        if (stopped_.load() && q_.empty()) break;
        if (!q_.empty()) { job = std::move(q_.front()); q_.pop(); }
      }
      if (job) job(); else if (idle_) idle_();
    }
  }

  std::thread thread_;
  std::queue<std::function<void()>> q_;
  std::mutex m_;
  std::condition_variable_any cv_;
  std::atomic<bool> stopped_{false};
  std::function<void()> idle_;
};

} // namespace fluent_lib::async


