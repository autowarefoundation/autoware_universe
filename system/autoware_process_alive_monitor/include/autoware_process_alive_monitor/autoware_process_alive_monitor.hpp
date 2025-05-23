// Copyright 2025 Tier IV, Inc.
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

#ifndef AUTOWARE_PROCESS_ALIVE_MONITOR__AUTOWARE_PROCESS_ALIVE_MONITOR_HPP_
#define AUTOWARE_PROCESS_ALIVE_MONITOR__AUTOWARE_PROCESS_ALIVE_MONITOR_HPP_

#include "rclcpp/rclcpp.hpp"

#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::process_alive_monitor
{

class ProcessAliveMonitor : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for ProcessAliveMonitor
   * @param options Node options for configuration
   */
  explicit ProcessAliveMonitor(const rclcpp::NodeOptions & options);

private:
  /**
   * @brief Read and process new content appended to launch.log
   */
  void read_launch_log_diff();

  /**
   * @brief Parse a single line from the log for process death information
   * @param line The log line to parse
   */
  void parse_log_line(const std::string & line);

  /**
   * @brief Timer callback to report and manage dead node list
   */
  void on_timer();

  // Map to track dead nodes: [node_name-#] -> true
  std::unordered_map<std::string, bool> dead_nodes_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Launch log file path and read position
  std::filesystem::path launch_log_path_;
  size_t last_file_pos_{static_cast<size_t>(-1)};

  // Parameters
  std::vector<std::string> ignore_node_names_;  // Node names to exclude from monitoring
  std::vector<int64_t> ignore_exit_codes_;      // Exit codes to ignore (e.g., normal termination)
  double check_interval_{1.0};                  // Check interval in seconds
  bool enable_debug_{false};                    // Enable debug output
};

}  // namespace autoware::process_alive_monitor

#endif  // AUTOWARE_PROCESS_ALIVE_MONITOR__AUTOWARE_PROCESS_ALIVE_MONITOR_HPP_
