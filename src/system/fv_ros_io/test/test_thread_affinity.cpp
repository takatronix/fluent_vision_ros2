#include "cpu_contract.hpp"

#include <gtest/gtest.h>

#include <sched.h>

namespace fv::ros_io::detail
{
namespace
{

TEST(ThreadAffinity, BindsAndRestoresCallingThread)
{
  const cpu_set_t original = currentThreadAffinity();
  int target_cpu = -1;
  for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
    if (CPU_ISSET(cpu, &original)) {
      target_cpu = cpu;
      break;
    }
  }
  ASSERT_GE(target_cpu, 0);

  setCurrentThreadAffinity(target_cpu);
  const cpu_set_t bound = currentThreadAffinity();
  EXPECT_EQ(CPU_COUNT(&bound), 1);
  EXPECT_TRUE(CPU_ISSET(target_cpu, &bound));

  const int restore_result = pthread_setaffinity_np(
    pthread_self(), sizeof(original), &original);
  ASSERT_EQ(restore_result, 0);
}

}  // namespace
}  // namespace fv::ros_io::detail
