#include "cpu_contract.hpp"

#include <pthread.h>

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <cstring>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>

namespace fv::ros_io::detail
{
namespace
{

std::string trim(std::string_view value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string_view::npos) {
    return {};
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return std::string(value.substr(first, last - first + 1));
}

int parseCpuIndex(std::string_view value, std::string_view source)
{
  const std::string normalized = trim(value);
  if (normalized.empty()) {
    throw std::runtime_error(std::string(source) + ": empty CPU index");
  }

  int cpu = -1;
  const char* begin = normalized.data();
  const char* end = begin + normalized.size();
  const auto [ptr, error] = std::from_chars(begin, end, cpu);
  if (error != std::errc{} || ptr != end || cpu < 0 || cpu >= CPU_SETSIZE) {
    throw std::runtime_error(
      std::string(source) + ": invalid CPU index '" + normalized + "'");
  }
  return cpu;
}

}  // namespace

std::string readTextFile(const std::filesystem::path& path)
{
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("cannot open " + path.string());
  }

  std::ostringstream content;
  content << stream.rdbuf();
  if (stream.bad()) {
    throw std::runtime_error("cannot read " + path.string());
  }
  return content.str();
}

std::vector<int> parseCpuList(std::string_view value)
{
  const std::string normalized = trim(value);
  if (normalized.empty()) {
    return {};
  }

  std::set<int> result;
  std::size_t offset = 0;
  while (offset < normalized.size()) {
    const std::size_t comma = normalized.find(',', offset);
    const std::size_t length =
      comma == std::string::npos ? normalized.size() - offset : comma - offset;
    const std::string token = trim(std::string_view(normalized).substr(offset, length));
    if (token.empty()) {
      throw std::runtime_error("CPU list contains an empty element: '" + normalized + "'");
    }

    const std::size_t dash = token.find('-');
    if (dash == std::string::npos) {
      result.insert(parseCpuIndex(token, "CPU list"));
    } else {
      if (token.find('-', dash + 1) != std::string::npos) {
        throw std::runtime_error("CPU list contains an invalid range: '" + token + "'");
      }
      const int first = parseCpuIndex(std::string_view(token).substr(0, dash), "CPU range");
      const int last = parseCpuIndex(std::string_view(token).substr(dash + 1), "CPU range");
      if (first > last) {
        throw std::runtime_error("CPU range is descending: '" + token + "'");
      }
      for (int cpu = first; cpu <= last; ++cpu) {
        result.insert(cpu);
      }
    }

    if (comma == std::string::npos) {
      break;
    }
    offset = comma + 1;
  }

  return {result.begin(), result.end()};
}

int parseReservedCpu(std::string_view kernel_command_line)
{
  constexpr std::string_view kPrefix = "fv_ros_io.cpu=";
  std::istringstream stream{std::string(kernel_command_line)};
  std::string token;
  std::vector<int> values;
  while (stream >> token) {
    if (token.rfind(kPrefix, 0) == 0) {
      values.push_back(parseCpuIndex(
        std::string_view(token).substr(kPrefix.size()), "fv_ros_io.cpu"));
    }
  }

  if (values.size() != 1) {
    throw std::runtime_error(
      "kernel command line must contain exactly one fv_ros_io.cpu entry; found " +
      std::to_string(values.size()));
  }
  return values.front();
}

bool containsCpu(const std::vector<int>& cpus, int cpu)
{
  return std::binary_search(cpus.begin(), cpus.end(), cpu);
}

cpu_set_t affinityForCpu(int cpu)
{
  if (cpu < 0 || cpu >= CPU_SETSIZE) {
    throw std::runtime_error("CPU index is outside cpu_set_t: " + std::to_string(cpu));
  }
  cpu_set_t affinity;
  CPU_ZERO(&affinity);
  CPU_SET(cpu, &affinity);
  return affinity;
}

void validateCpuContract(
  int cpu,
  const std::vector<int>& online_cpus,
  const std::vector<int>& isolated_cpus,
  const cpu_set_t& allowed_affinity)
{
  if (!containsCpu(online_cpus, cpu)) {
    throw std::runtime_error(
      "fv_ros_io.cpu=" + std::to_string(cpu) + " is absent from online CPUs");
  }
  if (!containsCpu(isolated_cpus, cpu)) {
    throw std::runtime_error(
      "fv_ros_io.cpu=" + std::to_string(cpu) + " is absent from isolated CPUs");
  }
  if (!CPU_ISSET(cpu, &allowed_affinity)) {
    throw std::runtime_error(
      "fv_ros_io.cpu=" + std::to_string(cpu) + " is absent from the process cpuset");
  }
}

void setCurrentThreadAffinity(int cpu)
{
  cpu_set_t affinity = affinityForCpu(cpu);
  const int result = pthread_setaffinity_np(pthread_self(), sizeof(affinity), &affinity);
  if (result != 0) {
    throw std::runtime_error(
      "pthread_setaffinity_np(cpu=" + std::to_string(cpu) + ") failed: " +
      std::strerror(result));
  }
}

cpu_set_t currentThreadAffinity()
{
  cpu_set_t affinity;
  CPU_ZERO(&affinity);
  const int result = pthread_getaffinity_np(pthread_self(), sizeof(affinity), &affinity);
  if (result != 0) {
    throw std::runtime_error(
      std::string("pthread_getaffinity_np failed: ") + std::strerror(result));
  }
  return affinity;
}

}  // namespace fv::ros_io::detail
