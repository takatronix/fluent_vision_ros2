#include "fluent_lib/ros/io_thread.hpp"

#include <gtest/gtest.h>

#include <pthread.h>
#include <sched.h>

#include <stdexcept>

namespace fluent_lib::ros
{
namespace
{

void expectAppliedConfiguration(const IoThreadConfiguration& configuration)
{
  int policy = 0;
  sched_param parameters{};
  ASSERT_EQ(pthread_getschedparam(pthread_self(), &policy, &parameters), 0);

  switch (configuration.scheduler) {
    case IoThreadScheduler::kSchedFifo:
      EXPECT_EQ(configuration.fifo_priority, 20);
      EXPECT_EQ(policy, SCHED_FIFO);
      EXPECT_EQ(parameters.sched_priority, 20);
      break;
    case IoThreadScheduler::kSchedOther:
      EXPECT_EQ(configuration.fifo_priority, 0);
      EXPECT_EQ(policy, SCHED_OTHER);
      EXPECT_EQ(parameters.sched_priority, 0);
      break;
  }
}

TEST(ThreadScheduling, ConfiguresCurrentIoThreadWithoutFailingStartup)
{
  expectAppliedConfiguration(configure_current_io_thread());
}

TEST(ThreadScheduling, RejectsInvalidFifoPriority)
{
  EXPECT_THROW(configure_current_io_thread(0), std::invalid_argument);
}

}  // namespace
}  // namespace fluent_lib::ros
