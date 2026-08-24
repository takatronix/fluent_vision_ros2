#pragma once

#include <sched.h>

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace fv::ros_io::detail
{

std::string readTextFile(const std::filesystem::path& path);
std::vector<int> parseCpuList(std::string_view value);
int parseReservedCpu(std::string_view kernel_command_line);
bool containsCpu(const std::vector<int>& cpus, int cpu);
cpu_set_t affinityForCpu(int cpu);
void validateCpuContract(
  int cpu,
  const std::vector<int>& online_cpus,
  const std::vector<int>& isolated_cpus,
  const cpu_set_t& allowed_affinity);
void setCurrentThreadAffinity(int cpu);
cpu_set_t currentThreadAffinity();

}  // namespace fv::ros_io::detail
