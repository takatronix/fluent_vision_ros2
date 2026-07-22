#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "fv_tts/synthesis_scheduler.hpp"

namespace fv_tts {
namespace {

using namespace std::chrono_literals;

class Event {
 public:
  void set() {
    std::lock_guard<std::mutex> lock(mutex_);
    set_ = true;
    condition_.notify_all();
  }

  bool wait(std::chrono::milliseconds timeout = 1000ms) {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout, [this] { return set_; });
  }

 private:
  std::mutex mutex_;
  std::condition_variable condition_;
  bool set_{false};
};

SayRequest request(SpeechKind kind, std::string id) {
  return SayRequest{kind, id, std::move(id)};
}

SynthesizedAudio audio() {
  return SynthesizedAudio{{0, 0}, 24000, 1, 16};
}

template <typename Predicate>
bool wait_until(Predicate predicate,
                std::chrono::milliseconds timeout = 1000ms) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!predicate()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    std::this_thread::sleep_for(2ms);
  }
  return true;
}

bool scheduler_failed(const SynthesisScheduler &scheduler) {
  try {
    scheduler.rethrow_if_failed();
    return false;
  } catch (...) {
    return true;
  }
}

TEST(SynthesisSchedulerDeathTest, HungRequestHardExitsWithRequestContext) {
  ASSERT_EXIT(
      {
        SynthesisScheduler scheduler(
            [](const std::string &) {
              std::this_thread::sleep_for(250ms);
              return audio();
            },
            [](const SayRequest &, const SynthesizedAudio &) {},
            [](const SayRequest &, const std::string &) {},
            [](const SayRequest &) {}, SynthesisTimeout{0.02});
        scheduler.submit(request(SpeechKind::kAgent, "agent-0-hung"));
        std::this_thread::sleep_for(250ms);
        std::_Exit(99);
      },
      ::testing::ExitedWithCode(EXIT_FAILURE),
      "context=request synthesis kind=agent "
      "utterance_id=\"agent-0-hung\"");
}

TEST(SynthesisScheduler, SystemOvertakesQueuedAgent) {
  Event active;
  Event release;
  std::mutex output_mutex;
  std::vector<std::string> delivered;
  SynthesisScheduler scheduler(
      [&](const std::string &text) {
        if (text == "agent-0-active") {
          active.set();
          if (!release.wait()) {
            throw std::runtime_error("release timeout");
          }
        }
        return audio();
      },
      [&](const SayRequest &item, const SynthesizedAudio &) {
        std::lock_guard<std::mutex> lock(output_mutex);
        delivered.push_back(item.utterance_id);
      },
      [](const SayRequest &, const std::string &) {},
      [](const SayRequest &) {});

  ASSERT_EQ(scheduler.submit(request(SpeechKind::kAgent, "agent-0-active")),
            SubmitStatus::kAccepted);
  ASSERT_TRUE(active.wait());
  scheduler.submit(request(SpeechKind::kAgent, "agent-0-queued"));
  scheduler.submit(request(SpeechKind::kSystem, "system-warning"));
  release.set();
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(output_mutex);
    return delivered.size() == 3;
  }));
  std::lock_guard<std::mutex> lock(output_mutex);
  EXPECT_EQ(delivered, (std::vector<std::string>{"agent-0-active",
                                                "system-warning",
                                                "agent-0-queued"}));
}

TEST(SynthesisScheduler, FloorCancelsQueuedAndSuppressesActiveStaleAgent) {
  Event active;
  Event release;
  std::mutex output_mutex;
  std::vector<std::string> delivered;
  std::vector<std::string> cancelled;
  SynthesisScheduler scheduler(
      [&](const std::string &text) {
        if (text == "agent-0-active") {
          active.set();
          if (!release.wait()) {
            throw std::runtime_error("release timeout");
          }
        }
        return audio();
      },
      [&](const SayRequest &item, const SynthesizedAudio &) {
        std::lock_guard<std::mutex> lock(output_mutex);
        delivered.push_back(item.utterance_id);
      },
      [](const SayRequest &, const std::string &) {},
      [&](const SayRequest &item) {
        std::lock_guard<std::mutex> lock(output_mutex);
        cancelled.push_back(item.utterance_id);
      });

  scheduler.submit(request(SpeechKind::kAgent, "agent-0-active"));
  ASSERT_TRUE(active.wait());
  scheduler.submit(request(SpeechKind::kAgent, "agent-0-queued"));
  scheduler.submit(request(SpeechKind::kSystem, "system-warning"));
  EXPECT_EQ(scheduler.advance_agent_floor(1),
            (std::vector<std::string>{"agent-0-queued"}));
  EXPECT_EQ(scheduler.submit(request(SpeechKind::kAgent, "agent-0-late")),
            SubmitStatus::kStale);
  scheduler.submit(request(SpeechKind::kAgent, "agent-1-new"));
  release.set();

  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(output_mutex);
    return delivered.size() == 2 && cancelled.size() == 3;
  }));
  std::lock_guard<std::mutex> lock(output_mutex);
  EXPECT_EQ(delivered,
            (std::vector<std::string>{"system-warning", "agent-1-new"}));
  EXPECT_EQ(cancelled, (std::vector<std::string>{"agent-0-queued",
                                                "agent-0-late",
                                                "agent-0-active"}));
}

TEST(SynthesisScheduler, SystemAbortTombstoneHandlesTopicReordering) {
  std::mutex output_mutex;
  std::vector<std::string> delivered;
  std::vector<std::string> cancelled;
  SynthesisScheduler scheduler(
      [](const std::string &) { return audio(); },
      [&](const SayRequest &item, const SynthesizedAudio &) {
        std::lock_guard<std::mutex> lock(output_mutex);
        delivered.push_back(item.utterance_id);
      },
      [](const SayRequest &, const std::string &) {},
      [&](const SayRequest &item) {
        std::lock_guard<std::mutex> lock(output_mutex);
        cancelled.push_back(item.utterance_id);
      });

  EXPECT_FALSE(scheduler.cancel_system("system-delayed"));
  EXPECT_EQ(scheduler.submit(request(SpeechKind::kSystem, "system-delayed")),
            SubmitStatus::kCancelled);
  std::lock_guard<std::mutex> lock(output_mutex);
  EXPECT_TRUE(delivered.empty());
  EXPECT_EQ(cancelled, (std::vector<std::string>{"system-delayed"}));
}

TEST(SynthesisScheduler, SystemAbortCancelsQueuedSynthesis) {
  Event active;
  Event release;
  std::mutex output_mutex;
  std::vector<std::string> delivered;
  std::vector<std::string> cancelled;
  SynthesisScheduler scheduler(
      [&](const std::string &text) {
        if (text == "agent-0-active") {
          active.set();
          if (!release.wait()) {
            throw std::runtime_error("release timeout");
          }
        }
        return audio();
      },
      [&](const SayRequest &item, const SynthesizedAudio &) {
        std::lock_guard<std::mutex> lock(output_mutex);
        delivered.push_back(item.utterance_id);
      },
      [](const SayRequest &, const std::string &) {},
      [&](const SayRequest &item) {
        std::lock_guard<std::mutex> lock(output_mutex);
        cancelled.push_back(item.utterance_id);
      });

  scheduler.submit(request(SpeechKind::kAgent, "agent-0-active"));
  ASSERT_TRUE(active.wait());
  scheduler.submit(request(SpeechKind::kSystem, "system-warning"));
  EXPECT_TRUE(scheduler.cancel_system("system-warning"));
  release.set();
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(output_mutex);
    return delivered.size() == 1 && cancelled.size() == 1;
  }));
  std::lock_guard<std::mutex> lock(output_mutex);
  EXPECT_EQ(delivered, (std::vector<std::string>{"agent-0-active"}));
  EXPECT_EQ(cancelled, (std::vector<std::string>{"system-warning"}));
}

TEST(SynthesisScheduler, SystemAbortSuppressesActiveAndWorkerContinues) {
  Event active;
  Event release;
  std::mutex output_mutex;
  std::vector<std::string> delivered;
  std::vector<std::string> cancelled;
  SynthesisScheduler scheduler(
      [&](const std::string &text) {
        if (text == "system-warning") {
          active.set();
          if (!release.wait()) {
            throw std::runtime_error("release timeout");
          }
        }
        return audio();
      },
      [&](const SayRequest &item, const SynthesizedAudio &) {
        std::lock_guard<std::mutex> lock(output_mutex);
        delivered.push_back(item.utterance_id);
      },
      [](const SayRequest &, const std::string &) {},
      [&](const SayRequest &item) {
        std::lock_guard<std::mutex> lock(output_mutex);
        cancelled.push_back(item.utterance_id);
      });

  scheduler.submit(request(SpeechKind::kSystem, "system-warning"));
  ASSERT_TRUE(active.wait());
  EXPECT_TRUE(scheduler.cancel_system("system-warning"));
  scheduler.submit(request(SpeechKind::kAgent, "agent-0-next"));
  release.set();
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(output_mutex);
    return delivered.size() == 1 && cancelled.size() == 1;
  }));
  std::lock_guard<std::mutex> lock(output_mutex);
  EXPECT_EQ(delivered, (std::vector<std::string>{"agent-0-next"}));
  EXPECT_EQ(cancelled, (std::vector<std::string>{"system-warning"}));
}

TEST(SynthesisScheduler, FloorCannotReturnInsidePcmPublishCommit) {
  Event publishing;
  Event release_publish;
  std::atomic<bool> floor_returned{false};
  std::vector<std::string> delivered;
  SynthesisScheduler scheduler(
      [](const std::string &) { return audio(); },
      [&](const SayRequest &item, const SynthesizedAudio &) {
        publishing.set();
        if (!release_publish.wait()) {
          throw std::runtime_error("publish release timeout");
        }
        delivered.push_back(item.utterance_id);
      },
      [](const SayRequest &, const std::string &) {},
      [](const SayRequest &) {});

  scheduler.submit(request(SpeechKind::kAgent, "agent-0-race"));
  ASSERT_TRUE(publishing.wait());
  std::thread floor([&] {
    scheduler.advance_agent_floor(1);
    floor_returned = true;
  });
  std::this_thread::sleep_for(30ms);
  EXPECT_FALSE(floor_returned.load());
  release_publish.set();
  floor.join();
  EXPECT_TRUE(floor_returned.load());
  EXPECT_EQ(delivered, (std::vector<std::string>{"agent-0-race"}));
}

TEST(SynthesisScheduler, SystemAbortCannotReturnInsidePcmPublishCommit) {
  Event publishing;
  Event release_publish;
  std::atomic<bool> abort_returned{false};
  std::vector<std::string> delivered;
  SynthesisScheduler scheduler(
      [](const std::string &) { return audio(); },
      [&](const SayRequest &item, const SynthesizedAudio &) {
        publishing.set();
        if (!release_publish.wait()) {
          throw std::runtime_error("publish release timeout");
        }
        delivered.push_back(item.utterance_id);
      },
      [](const SayRequest &, const std::string &) {},
      [](const SayRequest &) {});

  scheduler.submit(request(SpeechKind::kSystem, "system-race"));
  ASSERT_TRUE(publishing.wait());
  std::thread abort([&] {
    scheduler.cancel_system("system-race");
    abort_returned = true;
  });
  std::this_thread::sleep_for(30ms);
  EXPECT_FALSE(abort_returned.load());
  release_publish.set();
  abort.join();
  EXPECT_TRUE(abort_returned.load());
  EXPECT_EQ(delivered, (std::vector<std::string>{"system-race"}));
}

TEST(SynthesisScheduler, FailureIsTerminalAndWorkerContinues) {
  std::mutex output_mutex;
  std::vector<std::string> delivered;
  std::vector<std::pair<std::string, std::string>> failed;
  SynthesisScheduler scheduler(
      [](const std::string &text) {
        if (text == "agent-0-broken") {
          throw std::runtime_error("native failure");
        }
        return audio();
      },
      [&](const SayRequest &item, const SynthesizedAudio &) {
        std::lock_guard<std::mutex> lock(output_mutex);
        delivered.push_back(item.utterance_id);
      },
      [&](const SayRequest &item, const std::string &error) {
        std::lock_guard<std::mutex> lock(output_mutex);
        failed.emplace_back(item.utterance_id, error);
      },
      [](const SayRequest &) {});
  scheduler.submit(request(SpeechKind::kAgent, "agent-0-broken"));
  scheduler.submit(request(SpeechKind::kAgent, "agent-0-next"));
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(output_mutex);
    return delivered.size() == 1 && failed.size() == 1;
  }));
  std::lock_guard<std::mutex> lock(output_mutex);
  EXPECT_EQ(delivered, (std::vector<std::string>{"agent-0-next"}));
  EXPECT_EQ(failed.front(),
            (std::pair<std::string, std::string>{"agent-0-broken",
                                                "native failure"}));
}

TEST(SynthesisScheduler, CompletedCallbackFailureIsFatal) {
  Event callback;
  SynthesisScheduler scheduler(
      [](const std::string &) { return audio(); },
      [&](const SayRequest &, const SynthesizedAudio &) {
        callback.set();
        throw std::runtime_error("PCM publish failed");
      },
      [](const SayRequest &, const std::string &) {},
      [](const SayRequest &) {});

  scheduler.submit(request(SpeechKind::kAgent, "agent-0-fatal"));
  ASSERT_TRUE(callback.wait());
  ASSERT_TRUE(wait_until([&] { return scheduler_failed(scheduler); }));
  EXPECT_THROW(scheduler.rethrow_if_failed(), std::runtime_error);
  EXPECT_THROW(scheduler.submit(request(SpeechKind::kAgent, "agent-0-next")),
               std::runtime_error);
}

TEST(SynthesisScheduler, FailedCallbackFailureIsFatal) {
  Event callback;
  SynthesisScheduler scheduler(
      [](const std::string &) -> SynthesizedAudio {
        throw std::runtime_error("synthesis failed");
      },
      [](const SayRequest &, const SynthesizedAudio &) {},
      [&](const SayRequest &, const std::string &) {
        callback.set();
        throw std::runtime_error("failure result publish failed");
      },
      [](const SayRequest &) {});

  scheduler.submit(request(SpeechKind::kAgent, "agent-0-fatal"));
  ASSERT_TRUE(callback.wait());
  ASSERT_TRUE(wait_until([&] { return scheduler_failed(scheduler); }));
  EXPECT_THROW(scheduler.rethrow_if_failed(), std::runtime_error);
}

TEST(SynthesisScheduler, CancelledCallbackFailureIsFatal) {
  SynthesisScheduler scheduler(
      [](const std::string &) { return audio(); },
      [](const SayRequest &, const SynthesizedAudio &) {},
      [](const SayRequest &, const std::string &) {},
      [](const SayRequest &) {
        throw std::runtime_error("cancel result publish failed");
      });

  scheduler.advance_agent_floor(1);
  EXPECT_EQ(scheduler.submit(request(SpeechKind::kAgent, "agent-0-stale")),
            SubmitStatus::kStale);
  EXPECT_THROW(scheduler.rethrow_if_failed(), std::runtime_error);
}

}  // namespace
}  // namespace fv_tts
