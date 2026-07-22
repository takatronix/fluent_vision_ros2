#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <thread>

#include "fv_tts/synthesis_watchdog.hpp"

namespace fv_tts {
namespace {

using namespace std::chrono_literals;

TEST(SynthesisWatchdogDeathTest, TimeoutPrintsContextAndHardExits) {
  ASSERT_EXIT(
      {
        SynthesisWatchdog watchdog(SynthesisTimeout{0.02},
                                   "watchdog-death-test");
        std::this_thread::sleep_for(250ms);
        std::_Exit(99);
      },
      ::testing::ExitedWithCode(EXIT_FAILURE),
      "fv_tts fatal: VOICEVOX Core native synthesis exceeded 0\\.020 "
      "seconds; context=watchdog-death-test");
}

TEST(SynthesisWatchdog, CompletionCancelsDeadline) {
  SynthesisWatchdog watchdog(SynthesisTimeout{1.0}, "completed-call");
}

TEST(SynthesisWatchdog, RejectsNonPositiveOrNonFiniteTimeout) {
  EXPECT_THROW(checked_synthesis_timeout(0.0), std::invalid_argument);
  EXPECT_THROW(checked_synthesis_timeout(-1.0), std::invalid_argument);
  EXPECT_THROW(
      checked_synthesis_timeout(std::numeric_limits<double>::infinity()),
      std::invalid_argument);
  EXPECT_THROW(
      checked_synthesis_timeout(std::numeric_limits<double>::quiet_NaN()),
      std::invalid_argument);
}

TEST(SynthesisWatchdog, RejectsEmptyContext) {
  EXPECT_THROW(SynthesisWatchdog(SynthesisTimeout{1.0}, ""),
               std::invalid_argument);
}

}  // namespace
}  // namespace fv_tts
