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
 * @file cpu_usage_statistics.hpp
 * @brief CPU usage statistics collection and calculation
 */

#ifndef SYSTEM_MONITOR__CPU_MONITOR__CPU_USAGE_STATISTICS_HPP_
#define SYSTEM_MONITOR__CPU_MONITOR__CPU_USAGE_STATISTICS_HPP_

#include <cstdint>
#include <string>
#include <vector>

class CpuUsageStatistics
{
public:
  CpuUsageStatistics();
  ~CpuUsageStatistics() = default;

  // Structure to hold detailed CPU information for each core
  struct CoreUsageInfo
  {
    std::string name;       // "all", "0", "1", etc. : short enough for "short string optimization"
    float user_percent;     // Percentage of time spent in user mode
    float nice_percent;     // Percentage of time spent in nice mode
    float system_percent;   // Percentage of time spent in system mode
    float idle_percent;     // Percentage of time spent idle
    float iowait_percent;   // Percentage of time spent waiting for I/O
    float irq_percent;      // Percentage of time spent handling IRQs
    float softirq_percent;  // Percentage of time spent handling soft IRQs
    float steal_percent;    // Percentage of time stolen by other VMs
    float guest_percent;    // Percentage of time spent running a virtual CPU
    float guest_nice_percent;   // Percentage of time spent running a virtual CPU in nice mode
    float total_usage_percent;  // Total CPU usage percentage (100% - idle%)

    // Constructor with default values
    CoreUsageInfo()
    : name(""),
      user_percent(0.0f),
      nice_percent(0.0f),
      system_percent(0.0f),
      idle_percent(0.0f),
      iowait_percent(0.0f),
      irq_percent(0.0f),
      softirq_percent(0.0f),
      steal_percent(0.0f),
      guest_percent(0.0f),
      guest_nice_percent(0.0f),
      total_usage_percent(0.0f)
    {
    }
  };

  /**
   * @brief Update CPU statistics data by comparing current and previous statistics
   */
  void update_cpu_statistics();

  /**
   * @brief Update current CPU statistics data by reading /proc/stat
   * @return true if the update is successful, false otherwise
   */
  bool update_current_cpu_statistics();

  /**
   * @brief Get the number of cores including the "all" core
   * @return The number of cores
   */
  int32_t get_num_cores() const;

  /**
   * @brief Get the CPU usage information for a specific core
   * @param [in] core_index The index of the core
   * @return The CPU usage information for the specified core
   */
  CoreUsageInfo get_core_usage_info(int32_t core_index) const;

private:
  // Delete the unnecessary default constructors and operators.
  CpuUsageStatistics(const CpuUsageStatistics &) = delete;
  CpuUsageStatistics(const CpuUsageStatistics &&) = delete;
  CpuUsageStatistics operator=(const CpuUsageStatistics &) = delete;
  CpuUsageStatistics operator=(const CpuUsageStatistics &&) = delete;

  /**
   * @brief Swap the current and previous statistics
   */
  void swap_statistics();

  // Internal structure to hold raw CPU statistics from /proc/stat
  struct CpuStatistics
  {
    std::string name;  // "all", "0", "1", etc. : short enough for "short string optimization"
    uint64_t user;
    uint64_t nice;
    uint64_t system;
    uint64_t idle;
    uint64_t iowait;
    uint64_t irq;
    uint64_t softirq;
    uint64_t steal;
    uint64_t guest;
    uint64_t guest_nice;

    // Calculate total time (excluding idle)
    uint64_t total() const { return user + nice + system + iowait + irq + softirq + steal; }

    // Calculate total time including idle
    uint64_t total_all() const { return total() + idle; }

    // Calculate CPU usage percentage
    float usage_percent(const CpuStatistics & previous_statistics) const
    {
      uint64_t total_delta = total() - previous_statistics.total();
      uint64_t total_all_delta = total_all() - previous_statistics.total_all();

      if (total_all_delta == 0) {
        return 0.0f;
      }
      return (static_cast<float>(total_delta) / static_cast<float>(total_all_delta)) * 100.0f;
    }
  };

  bool first_call_;  // Flag to indicate first call of update_cpu_statistics().
  std::vector<CpuStatistics> statistics_1_;           // CPU statistics of CPUs
  std::vector<CpuStatistics> statistics_2_;           // CPU statistics of CPUs
  std::vector<CpuStatistics> & current_statistics_;   // Reference to current CPU statistics
  std::vector<CpuStatistics> & previous_statistics_;  // Reference to previous CPU statistics

  /**
   * @brief Vector of CPU usage statistics for each core
   * @note Allocated on heap and won't be reallocated during the lifetime of the instance.
   */
  std::vector<CpuUsageStatistics::CoreUsageInfo> core_usage_info_{};
};

#endif  // SYSTEM_MONITOR__CPU_MONITOR__CPU_USAGE_STATISTICS_HPP_
