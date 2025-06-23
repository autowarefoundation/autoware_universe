// Copyright 2025 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file cpu_usage_statistics.cpp
 * @brief CPU usage statistics collection and calculation implementation
 */

#include "system_monitor/cpu_monitor/cpu_usage_statistics.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

CpuUsageStatistics::CpuUsageStatistics()
  : first_call_(true), statistics_1_(), statistics_2_(),
    current_statistics_(statistics_1_), previous_statistics_(statistics_2_)
{
}

void CpuUsageStatistics::update_cpu_statistics()
{
  core_usage_info_.clear();
  // The vector is cleared, but the allocated memory won't be released.

  std::ifstream stat_file("/proc/stat");
  if (!stat_file.is_open()) {
    return;
  }

  current_statistics_.clear();  // Allocated memory area won't be released.

  std::string line;
  while (std::getline(stat_file, line)) {
    try {
      std::istringstream iss(line);
      std::string cpu_name;
      iss >> cpu_name;

      // Skip lines that don't start with "cpu"
      if (cpu_name.substr(0, 3) != "cpu") {
        continue;
      }
      if (cpu_name == "cpu") {
        cpu_name = "all";
      } else {
        cpu_name = cpu_name.substr(3);
      }

      CpuStatistics statistics;
      iss >> statistics.user >> statistics.nice >> statistics.system >> statistics.idle
        >> statistics.iowait >> statistics.irq >> statistics.softirq >> statistics.steal
        >> statistics.guest >> statistics.guest_nice;

      statistics.name = cpu_name;
      current_statistics_.push_back(statistics);
    } catch (const std::exception& e) {
      // Log error but continue processing other lines
      continue;
    }
  }
  stat_file.close();

  // If this is the first call, just store the current statistics.
  if (first_call_) {
    swap_statistics();
    first_call_ = false;
    return;
  }

  // Process each CPU's statistics
  for (const auto & core_info : current_statistics_) {
    const std::string & cpu_name = core_info.name;
    const CpuStatistics & stats = core_info;

    // Skip if we don't have previous stats for this CPU
    auto prev_iter =
      std::find_if(previous_statistics_.begin(), previous_statistics_.end(),
        [cpu_name](const CpuStatistics & statistics) { return statistics.name == cpu_name; });
    if (prev_iter == previous_statistics_.end()) {
      continue;
    }

    // Calculate deltas
    uint64_t user_delta = stats.user - prev_iter->user;
    uint64_t nice_delta = stats.nice - prev_iter->nice;
    uint64_t system_delta = stats.system - prev_iter->system;
    uint64_t idle_delta = stats.idle - prev_iter->idle;
    uint64_t iowait_delta = stats.iowait - prev_iter->iowait;
    uint64_t irq_delta = stats.irq - prev_iter->irq;
    uint64_t softirq_delta = stats.softirq - prev_iter->softirq;
    uint64_t steal_delta = stats.steal - prev_iter->steal;
    uint64_t guest_delta = stats.guest - prev_iter->guest;
    uint64_t guest_nice_delta = stats.guest_nice - prev_iter->guest_nice;

    // Calculate total time delta
    uint64_t total_delta = user_delta + nice_delta + system_delta +
                          iowait_delta + irq_delta + softirq_delta + steal_delta;
    uint64_t total_all_delta = total_delta + idle_delta;

    // Skip if total time delta is zero
    if (total_all_delta == 0) {
      continue;
    }

    float total_all_delta_float = static_cast<float>(total_all_delta);
    CoreUsageInfo core_usage;
    core_usage.name = cpu_name;
    core_usage.user_percent = (static_cast<float>(user_delta) / total_all_delta_float) * 100.0f;
    core_usage.nice_percent = (static_cast<float>(nice_delta) / total_all_delta_float) * 100.0f;
    core_usage.system_percent = (static_cast<float>(system_delta) / total_all_delta_float) * 100.0f;
    core_usage.idle_percent = (static_cast<float>(idle_delta) / total_all_delta_float) * 100.0f;
    core_usage.iowait_percent = (static_cast<float>(iowait_delta) / total_all_delta_float) * 100.0f;
    core_usage.irq_percent = (static_cast<float>(irq_delta) / total_all_delta_float) * 100.0f;
    core_usage.softirq_percent = (static_cast<float>(softirq_delta) / total_all_delta_float) * 100.0f;
    core_usage.steal_percent = (static_cast<float>(steal_delta) / total_all_delta_float) * 100.0f;
    core_usage.guest_percent = (static_cast<float>(guest_delta) / total_all_delta_float) * 100.0f;
    core_usage.guest_nice_percent = (static_cast<float>(guest_nice_delta) / total_all_delta_float) * 100.0f;
    core_usage.total_usage_percent = 100.0f - core_usage.idle_percent;

    // Add to CPU info
    core_usage_info_.push_back(core_usage);
  }

  // Store current stats for next call
  swap_statistics();
}

void CpuUsageStatistics::swap_statistics()
{
  std::swap(current_statistics_, previous_statistics_);
}

int32_t CpuUsageStatistics::get_num_cores() const
{
  return core_usage_info_.size();
}

CpuUsageStatistics::CoreUsageInfo CpuUsageStatistics::get_core_usage_info(int32_t core_index) const
{
  if (core_index < 0 || static_cast<size_t>(core_index) >= core_usage_info_.size()) {
    // To avoid unexpected segmentation fault, return empty data.
    CoreUsageInfo empty_info{};
    return empty_info;
  }
  return core_usage_info_[core_index];
}
