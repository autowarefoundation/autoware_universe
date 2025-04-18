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
 * @file process_monitor.hpp
 * @brief Process monitor class
 */

#ifndef SYSTEM_MONITOR__PROCESS_MONITOR__PROCESS_MONITOR_HPP_
#define SYSTEM_MONITOR__PROCESS_MONITOR__PROCESS_MONITOR_HPP_

#include "system_monitor/process_monitor/diag_task.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <limits.h>  // for HOST_NAME_MAX

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

struct ProcessStatistics;
struct RawProcessInfo;

class ProcessMonitor : public rclcpp::Node
{
public:
  /**
   * @brief constructor
   * @param [in]  options Options associated with this node.
   */
  explicit ProcessMonitor(const rclcpp::NodeOptions & options);

  /**
   * @brief constructor with node_name argument, required by TestProcessMonitor
   * @param [in]  node_name The name of the node instance.
   * @param [in]  options   Options associated with this node.
   */
  ProcessMonitor(const std::string & node_name, const rclcpp::NodeOptions & options);

  /**
   * @brief destructor. ProcessMonitor is inheritable for testing purposes.
   */
  virtual ~ProcessMonitor();

  /**
   * @brief get number of processes to be reported
   * @return Number of processes to be reported
   */
  int getNumOfProcs() const { return num_of_procs_; }

protected:  // For TestProcessMonitor
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;
  /**
   * @brief set root path for proc filesystem
   * @param [in]  root_path Root path
   */
  void setRoot(const std::string & root_path);

  /**
   * @brief get root path
   * @return Root path
   */
  std::string getRoot() const;

  /**
   * @brief timer callback to collect process information
   */
  void onTimer();

  diagnostic_updater::Updater updater_;  //!< @brief Updater class which advertises to /diagnostics

private:
  /**
   * @brief Common initialization for the constructors.
   */
  void initialize();

  /**
   * @brief Finalization for the destructor.
   */
  void finalize();

  /**
   * @brief monitor processes
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void monitorProcesses(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief get task summary
   * @param [in]  snapshot Process statistics snapshot
   * @param [out] stat     diagnostic message passed directly to diagnostic publish calls
   */
  void getTasksSummary(
    const ProcessStatistics & snapshot, diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief get high load processes
   * @param [in]  snapshot Process statistics snapshot
   */
  void getHighLoadProcesses(const ProcessStatistics & snapshot);

  /**
   * @brief get high memory processes
   * @param [in]  snapshot Process statistics snapshot
   */
  void getHighMemoryProcesses(const ProcessStatistics & snapshot);

  /**
   * @brief fill task information for publishing based on raw process information
   * @param [in]  raw_p            Raw process information
   * @param [in]  uptime_delta_sec System uptime delta in seconds
   * @param [out] task_p           Pointer to task information structure to be filled
   */
  void fillTaskInfo(
    const std::unique_ptr<RawProcessInfo> & raw_p, const double uptime_delta_sec,
    const std::shared_ptr<DiagTask> & task_p);

  /**
   * @brief accumulate process state count
   * @param [in]  info Raw process information
   */
  void accumulateStateCount(const RawProcessInfo & info);

  /**
   * @brief initialize process statistics
   */
  void initializeProcessStatistics();

  /**
   * @brief collect process information
   * @param [in]  pid_str Process ID in numeric string
   */
  void collectProcessInfo(const char * pid_str);

  /**
   * @brief scan proc filesystem
   * @return true if successful
   */
  bool scanProcFs();

  /**
   * @brief read memory information
   * @return true if successful
   */
  bool readMemInfo();

  /**
   * @brief update high load process ranking
   * @param [in]  info Raw process information
   */
  void updateHighLoadProcessRanking(const RawProcessInfo & info);

  /**
   * @brief update high memory process ranking
   * @param [in]  info Raw process information
   */
  void updateHighMemoryProcessRanking(const RawProcessInfo & info);

  /**
   * @brief register process information to new map
   * @param [in] pid  Process ID
   * @param [in] info Raw process information
   */
  void registerProcessInfoToNewMap(const pid_t pid, const RawProcessInfo & info);

  /**
   * @brief rotate process maps
   */
  void rotateProcessMaps();

  /**
   * @brief get system uptime
   * @param [out] uptime System uptime in seconds
   * @return true if successful
   */
  bool getUptime(double & uptime_sec) const;

  /**
   * @brief get command line from process ID
   * @param [in]  pid     Process ID
   * @param [out] command Command line
   * @return true if successful
   */
  bool getCommandLineFromPid(const std::string & pid, std::string & command) const;

  /**
   * @brief set error content
   * @param [in]  message       Error message
   * @param [in]  error_command Error command
   * @param [in]  content       Error content
   * @param [out] tasks         List of diagnostic tasks
   */
  void setErrorContent(
    const std::string & message, const std::string & error_command, const std::string & content,
    std::vector<std::shared_ptr<DiagTask>> & tasks);

  // Constant
  static constexpr int32_t kDefaultNumProcs = 5;

  // Node Parameter
  int32_t num_of_procs_;  //!< @brief number of processes to show

  char hostname_[HOST_NAME_MAX + 1]{'\0'};  //!< @brief host name
  std::string root_path_{"/"};              //!< @brief root path for proc filesystem
  // mem_total_kb_ should not be zero to avoid "divide by zero" exception.
  int64_t mem_total_kb_{1};  //!< @brief total memory size in KB
  int64_t page_size_kb_{0};  //!< @brief page size in KB
  int64_t clock_tick_{0};    //!< @brief clock tick

  std::vector<std::shared_ptr<DiagTask>>
    load_tasks_{};  //!< @brief list of diagnostics tasks for high load procs
  std::vector<std::shared_ptr<DiagTask>>
    memory_tasks_{};  //!< @brief list of diagnostics tasks for high memory procs

  std::unique_ptr<ProcessStatistics>
    work_{};  //!< @brief Unstable information being read from /proc files
  std::unordered_map<pid_t, std::shared_ptr<RawProcessInfo>> pid_map1_{};
  std::unordered_map<pid_t, std::shared_ptr<RawProcessInfo>> pid_map2_{};
  std::unordered_map<pid_t, std::shared_ptr<RawProcessInfo>> & prev_map_{pid_map1_};
  std::unordered_map<pid_t, std::shared_ptr<RawProcessInfo>> & new_map_{pid_map2_};

  std::unique_ptr<ProcessStatistics>
    snapshot_{};  //!< @brief Stable information copied from work_ within mutex_ locked scope
  double elapsed_ms_{0.0};            //!< @brief Execution time of reading from /proc files
  double uptime_prev_sec_{0.0};       //!< @brief uptime in second at previous onTimer() call
  bool error_occurred_{false};        //!< @brief true if temporaryerror occurred
  bool fatal_error_occurred_{false};  //!< @brief true if fatal error occurred
  std::mutex mutex_{};  //!< @brief mutex for protecting snapshot_, elapsed_ms_, uptime_prev_sec_

  rclcpp::TimerBase::SharedPtr timer_{};  //!< @brief timer to trigger reading from /proc files
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_{};  //!< @brief Callback Group
};

#endif  // SYSTEM_MONITOR__PROCESS_MONITOR__PROCESS_MONITOR_HPP_
