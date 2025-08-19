// Copyright 2020,2025 Tier IV, Inc.
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
 * @file tegra_cpu_monitor.h
 * @brief TEGRA CPU monitor class
 */

#ifndef SYSTEM_MONITOR__CPU_MONITOR__TEGRA_CPU_MONITOR_HPP_
#define SYSTEM_MONITOR__CPU_MONITOR__TEGRA_CPU_MONITOR_HPP_

#include "system_monitor/cpu_monitor/cpu_monitor_base.hpp"

#include <string>

class CPUMonitor : public CPUMonitorBase
{
public:
  /**
   * @brief constructor
   * @param [in] options Options associated with this node.
   */
  explicit CPUMonitor(const rclcpp::NodeOptions & options);

  /**
   * @brief constructor with node_name argument, required by TestCPUMonitor
   * @param [in] node_name The name of the node instance.
   * @param [in] options   Options associated with this node.
   */
  CPUMonitor(const std::string & node_name, const rclcpp::NodeOptions & options);

protected:
  /**
   * @brief get names for core temperature files
   */
  void getTemperatureFileNames() override;
};

#endif  // SYSTEM_MONITOR__CPU_MONITOR__TEGRA_CPU_MONITOR_HPP_
