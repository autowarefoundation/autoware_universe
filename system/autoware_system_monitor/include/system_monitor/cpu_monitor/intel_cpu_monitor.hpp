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
 * @file intel_cpu_monitor.h
 * @brief  CPU monitor class
 */

#ifndef SYSTEM_MONITOR__CPU_MONITOR__INTEL_CPU_MONITOR_HPP_
#define SYSTEM_MONITOR__CPU_MONITOR__INTEL_CPU_MONITOR_HPP_

#include "system_monitor/cpu_monitor/cpu_monitor_base.hpp"

#include <string>
#include <vector>

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
   * @brief check CPU thermal throttling
   */
  void checkThermalThrottling() override;

  /**
   * @brief update CPU thermal throttling (implementation)
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void updateThermalThrottlingImpl(
    diagnostic_updater::DiagnosticStatusWrapper & stat) override;  // NOLINT(runtime/references)

  /**
   * @brief Get CPU thermal throttling status
   */
  int getThermalThrottlingStatus() const override;

  /**
   * @brief get names for core temperature files
   */
  void getTemperatureFileNames() override;

  /**
   * @brief Add a loadable kernel module msr
   */
  void modprobeMSR();

  // The format of Thermal Throttling report depends on CPU model.
  // Therefore, Thermal Throttling report is implemented in each derived class.

  // Intel CPU uses msr_reader to get thermal throttling data.
  int msr_reader_port_;  //!< @brief port number to connect to msr_reader

  struct ThermalThrottlingData
  {
    float elapsed_ms;
    int summary_status;
    std::string summary_message;
    std::string error_key;
    std::string error_value;
    std::vector<std::pair<std::string, std::string>> core_data;

    void clear()
    {
      elapsed_ms = 0.0f;
      summary_status = 0;
      summary_message.clear();
      error_key.clear();
      error_value.clear();
      core_data.clear();  // Allocated heap memory is not released.
    }
  };

  ThermalThrottlingData thermal_throttling_data_;
};

#endif  // SYSTEM_MONITOR__CPU_MONITOR__INTEL_CPU_MONITOR_HPP_
