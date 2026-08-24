#include "fv_ros_io/thread_affinity.hpp"

#include "cpu_contract.hpp"

#include <sched.h>

#include <stdexcept>
#include <string>

namespace fv::ros_io
{

void bind_current_thread()
{
  const int cpu = detail::parseReservedCpu(detail::readTextFile("/proc/cmdline"));
  const auto online_cpus = detail::parseCpuList(
    detail::readTextFile("/sys/devices/system/cpu/online"));
  const auto isolated_cpus = detail::parseCpuList(
    detail::readTextFile("/sys/devices/system/cpu/isolated"));
  const cpu_set_t allowed_affinity = detail::currentThreadAffinity();
  detail::validateCpuContract(cpu, online_cpus, isolated_cpus, allowed_affinity);

  detail::setCurrentThreadAffinity(cpu);
  const cpu_set_t applied_affinity = detail::currentThreadAffinity();
  if (CPU_COUNT(&applied_affinity) != 1 || !CPU_ISSET(cpu, &applied_affinity)) {
    throw std::runtime_error(
      "thread affinity verification failed for fv_ros_io.cpu=" + std::to_string(cpu));
  }
}

}  // namespace fv::ros_io
