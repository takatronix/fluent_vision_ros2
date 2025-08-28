#pragma once

#include <cstdint>
#include <cstdio>
#include <string>
#include <fstream>
#include <sstream>

namespace fluent {
namespace utils {

struct SystemStats {
  float cpu_usage_pct {0.0f};
  int mem_used_mb {0};
  int mem_total_mb {0};
};

// Header-only simple system monitor for Linux (/proc based)
class SystemMonitor {
public:
  SystemMonitor() = default;

  // Compute deltas from previous sample to get CPU usage percent
  SystemStats sample() {
    SystemStats s;
    // CPU
    {
      std::ifstream stat_file("/proc/stat");
      std::string line;
      if (std::getline(stat_file, line)) {
        std::istringstream iss(line);
        std::string cpu;
        uint64_t user=0,nice=0,system=0,idle=0,iowait=0,irq=0,softirq=0,steal=0;
        iss >> cpu >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;
        uint64_t idle_all = idle + iowait;
        uint64_t non_idle = user + nice + system + irq + softirq + steal;
        uint64_t total = idle_all + non_idle;
        uint64_t last_idle_all = last_idle_ + last_iowait_;
        uint64_t last_non_idle = last_user_ + last_nice_ + last_system_ + last_irq_ + last_softirq_ + last_steal_;
        uint64_t last_total = last_idle_all + last_non_idle;
        uint64_t totald = total > last_total ? (total - last_total) : 0;
        uint64_t idled = idle_all > last_idle_all ? (idle_all - last_idle_all) : 0;
        if (totald > 0) {
          s.cpu_usage_pct = static_cast<float>(100.0 * (totald - idled) / static_cast<double>(totald));
        }
        last_user_ = user; last_nice_ = nice; last_system_ = system; last_idle_ = idle;
        last_iowait_ = iowait; last_irq_ = irq; last_softirq_ = softirq; last_steal_ = steal;
      }
    }
    // Memory
    {
      std::ifstream meminfo("/proc/meminfo");
      std::string key; long value=0; std::string unit;
      long mem_total_kb = 0, mem_available_kb = 0;
      while (meminfo >> key >> value >> unit) {
        if (key == "MemTotal:") mem_total_kb = value;
        else if (key == "MemAvailable:") mem_available_kb = value;
        if (mem_total_kb && mem_available_kb) break;
      }
      if (mem_total_kb > 0) {
        s.mem_total_mb = static_cast<int>(mem_total_kb / 1024);
        int mem_avail_mb = static_cast<int>(mem_available_kb / 1024);
        s.mem_used_mb = s.mem_total_mb - mem_avail_mb;
      }
    }
    return s;
  }

private:
  // Last CPU counters
  uint64_t last_user_ = 0, last_nice_ = 0, last_system_ = 0, last_idle_ = 0, last_iowait_ = 0,
           last_irq_ = 0, last_softirq_ = 0, last_steal_ = 0;
};

} // namespace utils
} // namespace fluent


