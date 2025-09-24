// Copyright 2020,2025 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file raspi_cpu_monitor.cpp
 * @brief Raspberry Pi CPU monitor class
 */

#include "system_monitor/cpu_monitor/raspi_cpu_monitor.hpp"

#include "system_monitor/system_monitor_utility.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <string>
#include <vector>

namespace fs = boost::filesystem;

CPUMonitor::CPUMonitor(const rclcpp::NodeOptions & options) : CPUMonitorBase("cpu_monitor", options)
{
}

CPUMonitor::CPUMonitor(const std::string & node_name, const rclcpp::NodeOptions & options)
: CPUMonitorBase(node_name, options)
{
}

void CPUMonitor::checkThermalThrottling()
{
  // Remember start time to measure elapsed time
  const auto t_start = std::chrono::high_resolution_clock::now();

  int level = DiagStatus::OK;
  std::vector<std::string> status;

  const fs::path path("/sys/devices/platform/soc/soc:firmware/get_throttled");
  fs::ifstream ifs(path, std::ios::in);
  if (!ifs) {
    std::lock_guard<std::mutex> lock_snapshot(mutex_snapshot_);
    thermal_throttling_data_.clear();
    thermal_throttling_data_.summary_status = DiagStatus::ERROR;
    thermal_throttling_data_.summary_message = "file open error";
    thermal_throttling_data_.error_key = "get_throttled";
    thermal_throttling_data_.error_value = "file open error";
    return;
  }

  int throttled;
  ifs >> std::hex >> throttled;
  ifs.close();

  // Consider only thermal throttling as an error
  if ((throttled & raspiThermalThrottlingMask) == raspiThermalThrottlingMask) {
    level = DiagStatus::ERROR;
  }

  while (throttled) {
    int flag = throttled & ((~throttled) + 1);
    throttled ^= flag;
    status.push_back(throttledToString(flag));
  }
  if (status.empty()) {
    status.emplace_back("All clear");
  }

  std::lock_guard<std::mutex> lock_snapshot(mutex_snapshot_);
  thermal_throttling_data_.clear();

  thermal_throttling_data_.status = boost::algorithm::join(status, ", ");

  thermal_throttling_data_.summary_status = level;
  thermal_throttling_data_.summary_message = thermal_dictionary_.at(level);

  // Measure elapsed time since start time.
  const auto t_end = std::chrono::high_resolution_clock::now();
  const float elapsed_ms = std::chrono::duration<float, std::milli>(t_end - t_start).count();
  thermal_throttling_data_.elapsed_ms = elapsed_ms;
}

void CPUMonitor::updateThermalThrottlingImpl(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock_snapshot(mutex_snapshot_);

  if (!thermal_throttling_data_.error_key.empty()) {
    stat.summary(thermal_throttling_data_.summary_status, thermal_throttling_data_.summary_message);
    stat.add(thermal_throttling_data_.error_key, thermal_throttling_data_.error_value);
    return;
  }

  stat.add("status", thermal_throttling_data_.status);
  stat.summary(thermal_throttling_data_.summary_status, thermal_throttling_data_.summary_message);
  stat.addf("execution time", "%f ms", thermal_throttling_data_.elapsed_ms);
}

int CPUMonitor::getThermalThrottlingStatus() const
{
  std::lock_guard<std::mutex> lock_snapshot(mutex_snapshot_);
  return thermal_throttling_data_.summary_status;
}

// This function is called from a locked context in the timer callback.
void CPUMonitor::getTemperatureFileNames()
{
  // thermal_zone0
  std::vector<thermal_zone> thermal_zones;
  SystemMonitorUtility::getThermalZone("cpu-thermal", &thermal_zones);

  for (const auto & zone : thermal_zones) {
    temperatures_.emplace_back(zone.label_, zone.path_);
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CPUMonitor)
