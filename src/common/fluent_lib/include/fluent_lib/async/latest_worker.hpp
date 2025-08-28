#pragma once
#include <functional>
#include <memory>
#include <atomic>
#include <utility>
#include "fluent_lib/async/worker.hpp"

namespace fluent_lib::async {

// 最新タスクのみを実行するワーカー。
// 新しいsubmitが来たら直前のタスクにキャンセルフラグを立てる。
class LatestWorker {
public:
  LatestWorker() = default;
  ~LatestWorker() { stop(); }

  // job(cancelled): 途中で cancelled.load() を見て早期return可能。
  void submit(std::function<void(std::atomic<bool>&)> job) {
    auto flag = std::make_shared<std::atomic<bool>>(false);
    // 直前のキャンセル
    {
      std::shared_ptr<std::atomic<bool>> prev;
      prev.swap(cancel_flag_);
      if (prev) prev->store(true);
      cancel_flag_ = flag;
    }
    ensureWorker();
    worker_->post([flag, j = std::move(job)]{ if (j) j(*flag); });
  }

  void stop() {
    if (cancel_flag_) cancel_flag_->store(true);
    if (worker_) worker_->stop();
    worker_.reset();
    cancel_flag_.reset();
  }

private:
  void ensureWorker() {
    if (!worker_) worker_ = std::make_unique<fluent_lib::async::Worker>();
  }

  std::unique_ptr<fluent_lib::async::Worker> worker_;
  std::shared_ptr<std::atomic<bool>> cancel_flag_;
};

} // namespace fluent_lib::async


