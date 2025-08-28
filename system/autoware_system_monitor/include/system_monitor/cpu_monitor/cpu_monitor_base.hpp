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
 * @file cpu_monitor_base.h
 * @brief CPU monitor base class
 */

#ifndef SYSTEM_MONITOR__CPU_MONITOR__CPU_MONITOR_BASE_HPP_
#define SYSTEM_MONITOR__CPU_MONITOR__CPU_MONITOR_BASE_HPP_

#include "system_monitor/cpu_monitor/cpu_information.hpp"
#include "system_monitor/cpu_monitor/cpu_usage_statistics.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_external_api_msgs/msg/cpu_status.hpp>
#include <tier4_external_api_msgs/msg/cpu_temperature.hpp>
#include <tier4_external_api_msgs/msg/cpu_usage.hpp>

#include <atomic>
#include <climits>
#include <map>
#include <mutex>
#include <string>
#include <vector>

class CPUMonitorBase : public rclcpp::Node
{
public:
  /**
   * @brief Update the diagnostic state.
   */
  void update();

protected:
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

  /**
   * @brief constructor
   * @param [in] node_name Name of the node.
   * @param [in] options Options associated with this node.
   */
  CPUMonitorBase(const std::string & node_name, const rclcpp::NodeOptions & options);

  /**
   * @brief check CPU temperature
   */
  virtual void checkTemperature();

  /**
   * @brief convert Cpu Usage To diagnostic Level
   * @param [cpu_name] cpu name (all, 0, 1, etc. : compatible with mpstat)
   * @param [usage] cpu usage value
   * @return DiagStatus::OK or WARN or ERROR
   */
  virtual int CpuUsageToLevel(const std::string & cpu_name, float usage);

  /**
   * @brief check CPU usage
   */
  virtual void checkUsage();

  /**
   * @brief check CPU load average
   */
  virtual void checkLoad();

  /**
   * @brief check CPU frequency
   */
  virtual void checkFrequency();

  /**
   * @brief check CPU thermal throttling
   * @note Data format of ThermalThrottling differs among platforms.
   * So both of checkThermalThrottling() and updateThermalThrottlingImpl() should be implemented in
   * each derived class.
   */
  virtual void checkThermalThrottling();

  /**
   * @brief update CPU temperature
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void updateTemperature(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief update CPU usage
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void updateUsage(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief update CPU load average
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void updateLoad(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief update CPU frequency
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void updateFrequency(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief update CPU thermal throttling
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void updateThermalThrottling(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief update CPU thermal throttling implementation
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   * @note Data format of ThermalThrottling differs among platforms.
   * So both of checkThermalThrottling() and updateThermalThrottlingImpl() should be implemented in
   * the derived class.
   */
  virtual void updateThermalThrottlingImpl(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief Get CPU thermal throttling status
   * @return DiagStatus::OK or WARN or ERROR
   * @note ERROR indicates that thermal throttling is active or status retrieval failed
   */
  virtual int getThermalThrottlingStatus() const;

  /**
   * @brief timer callback to collect cpu statistics
   */
  void onTimer();

  /**
   * @brief publish CPU usage as an independent topic
   * @param [in] usage CPU usage
   */
  virtual void publishCpuUsage(tier4_external_api_msgs::msg::CpuUsage usage);

  /**
   * @brief publish CPU temperature as an independent topic
   */
  void publishCpuTemperature();

  // Updater won't be changed after initialization. No need to protect it with mutex.
  diagnostic_updater::Updater updater_;  //!< @brief Updater class which advertises to /diagnostics

  std::mutex mutex_context_;  //!< @brief mutex for protecting the class context
  // Unit tests modify these variables.
  // Therefore, they should be protected with mutex_context_.
  // NOTE:
  //   Though current implementation of unit tests disables the timer callback,
  //   the context variables still should be protected by mutex_context_.
  char hostname_[HOST_NAME_MAX + 1];              //!< @brief host name
  int num_cores_;                                 //!< @brief number of cores
  std::vector<CpuTemperatureInfo> temperatures_;  //!< @brief CPU list for temperature
  std::vector<CpuFrequencyInfo> frequencies_;     //!< @brief CPU list for frequency
  std::vector<int> usage_warn_check_count_;  //!< @brief CPU list for usage over warn check counter
  std::vector<int>
    usage_error_check_count_;  //!< @brief CPU list for usage over error check counter
  // Though node parameters are read-only after initialization, unit tests modify them.
  // Therefore, they should be protected with mutex_context_, too.
  float usage_warn_;       //!< @brief CPU usage(%) to generate warning
  float usage_error_;      //!< @brief CPU usage(%) to generate error
  int usage_warn_count_;   //!< @brief continuous count over usage_warn_ to generate warning
  int usage_error_count_;  //!< @brief continuous count over usage_error_ to generate error
  bool usage_average_;     //!< @brief Check CPU usage calculated as averages among all processors
// Warning/Error about temperature used to be implemented,
// but they were removed in favor of warning/error about thermal throttling.
#ifdef ENABLE_TEMPERATURE_DIAGNOSTICS
  int temperature_warn_;   //!< @brief CPU temperature to generate warning
  int temperature_error_;  //!< @brief CPU temperature to generate error
#endif                     // ENABLE_TEMPERATURE_DIAGNOSTICS

  /**
   * @brief CPU temperature status messages
   */
  const std::map<int, const char *> temperature_dictionary_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "warm"}, {DiagStatus::ERROR, "hot"}};

  /**
   * @brief CPU usage status messages
   */
  const std::map<int, const char *> load_dictionary_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "high load"}, {DiagStatus::ERROR, "very high load"}};

  /**
   * @brief CPU thermal throttling status messages
   */
  const std::map<int, const char *> thermal_dictionary_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "unused"}, {DiagStatus::ERROR, "throttling"}};

  /**
   * @brief Class instance for collecting CPU usage statistics
   */
  CpuUsageStatistics cpu_usage_statistics_;

  // Publisher
  rclcpp::Publisher<tier4_external_api_msgs::msg::CpuUsage>::SharedPtr pub_cpu_usage_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::CpuTemperature>::SharedPtr pub_cpu_temperature_;

  rclcpp::TimerBase::SharedPtr timer_;  //!< @brief timer to collect cpu statistics
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;  //!< @brief Callback Group

  mutable std::mutex mutex_snapshot_;  //!< @brief mutex for protecting snapshot
  TemperatureData temperature_data_;   //!< @brief snapshot of CPU temperature
  UsageData usage_data_;               //!< @brief snapshot of CPU usage
  LoadData load_data_;                 //!< @brief snapshot of CPU load average
  FrequencyData frequency_data_;       //!< @brief snapshot of CPU frequency

private:
  /**
   * @brief get names of core temperature files
   */
  virtual void getTemperatureFileNames();

  /**
   * @brief get names of cpu frequency files
   */
  virtual void getFrequencyFileNames();

  // Lazy initialization.
  // File name lists are initialized at the first call of onTimer()
  // so that virtual functions can be called.
  std::atomic<bool> is_temperature_file_names_initialized_;
  std::atomic<bool> is_frequency_file_names_initialized_;
};

#endif  // SYSTEM_MONITOR__CPU_MONITOR__CPU_MONITOR_BASE_HPP_
