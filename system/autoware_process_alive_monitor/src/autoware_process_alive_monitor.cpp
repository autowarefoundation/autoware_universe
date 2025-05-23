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

#include "autoware_process_alive_monitor/autoware_process_alive_monitor.hpp"

#include <filesystem>
#include <fstream>
#include <regex>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace autoware::process_alive_monitor
{

/**
 * @brief Find the latest launch.log file
 * @return Path to latest launch.log file, empty path if not found
 */
static fs::path find_latest_launch_log()
{
  // Check ROS_LOG_DIR environment variable first
  const char * ros_log_dir_env = std::getenv("ROS_LOG_DIR");
  fs::path base_path;
  if (ros_log_dir_env) {
    base_path = fs::path(ros_log_dir_env);
  } else {
    // Fallback to ~/.ros/log
    const char * home_env = std::getenv("HOME");
    if (!home_env) {
      base_path = fs::current_path();
    } else {
      base_path = fs::path(home_env) / ".ros" / "log";
    }
  }

  if (!fs::exists(base_path) || !fs::is_directory(base_path)) {
    return fs::path();
  }

  // Find the latest directory by last write time
  fs::path latest_dir;
  auto latest_time = fs::file_time_type::min();

  for (auto & entry : fs::directory_iterator(base_path)) {
    if (entry.is_directory()) {
      auto ftime = fs::last_write_time(entry.path());
      if (ftime > latest_time) {
        latest_time = ftime;
        latest_dir = entry.path();
      }
    }
  }

  if (latest_dir.empty()) {
    return fs::path();
  }

  // Check for launch.log in the latest directory
  fs::path log_file = latest_dir / "launch.log";
  if (fs::exists(log_file) && fs::is_regular_file(log_file)) {
    return log_file;
  }
  return fs::path();
}

ProcessAliveMonitor::ProcessAliveMonitor(const rclcpp::NodeOptions & options)
: Node("autoware_process_alive_monitor", options)
{
  ignore_node_names_ =
    declare_parameter<std::vector<std::string>>("ignore_node_names", std::vector<std::string>{});
  ignore_exit_codes_ = declare_parameter<std::vector<int64_t>>("ignore_exit_codes");
  check_interval_ = declare_parameter<double>("check_interval");
  enable_debug_ = declare_parameter<bool>("enable_debug");

  // Initialize launch.log monitoring
  launch_log_path_ = find_latest_launch_log();
  if (launch_log_path_.empty()) {
    RCLCPP_WARN(get_logger(), "Could not find latest launch.log. Monitoring disabled.");
  } else {
    RCLCPP_WARN(get_logger(), "Monitoring launch.log at: %s", launch_log_path_.c_str());
  }

  // Set initial file position for differential reading
  last_file_pos_ = 0;
  if (!launch_log_path_.empty() && fs::exists(launch_log_path_)) {
    auto raw_size = fs::file_size(launch_log_path_);
    last_file_pos_ = raw_size;

    if (enable_debug_) {
      RCLCPP_WARN(
        get_logger(),
        "File size details - Raw size (uintmax_t): %ju, Stored position (size_t): %zu", raw_size,
        last_file_pos_);
    }
  }

  auto interval_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(check_interval_));
  timer_ = create_wall_timer(interval_ns, std::bind(&ProcessAliveMonitor::on_timer, this));
}

void ProcessAliveMonitor::read_launch_log_diff()
{
  if (launch_log_path_.empty()) {
    return;
  }

  std::ifstream ifs(launch_log_path_, std::ios::binary);
  if (!ifs.good()) {
    RCLCPP_WARN(get_logger(), "Failed to open launch.log: %s", launch_log_path_.c_str());
    return;
  }

  // Get file size
  ifs.seekg(0, std::ios::end);
  const std::streampos file_end = ifs.tellg();

  // Reset position to start if previous position exceeds file size (log rotation)
  if (last_file_pos_ > static_cast<size_t>(file_end)) {
    RCLCPP_WARN(
      get_logger(),
      "File size is reset. Possibly new session? Reading from top. last_file_pos_: %zu, file_end: "
      "%zu",
      last_file_pos_, static_cast<size_t>(file_end));
    last_file_pos_ = 0;
  }

  // Seek to last position
  ifs.seekg(last_file_pos_, std::ios::beg);

  if (enable_debug_) {
    RCLCPP_WARN(
      get_logger(), "[DEBUG] Reading launch.log from pos=%zu to end=%zu",
      static_cast<size_t>(last_file_pos_), static_cast<size_t>(file_end));
  }

  std::streampos last_valid_pos = static_cast<std::streampos>(last_file_pos_);

  size_t iteration = 0;
  while (rclcpp::ok()) {
    // Check current position
    std::streampos current_pos_start = ifs.tellg();
    if (current_pos_start == std::streampos(-1)) {
      if (ifs.eof()) {
        RCLCPP_DEBUG(get_logger(), "EOF reached at iteration=%zu", iteration);
      } else {
        RCLCPP_WARN(
          get_logger(), "tellg() failed at iteration=%zu. Possibly file closed?", iteration);
      }
      break;
    }

    // Read one line
    std::string line;
    if (!std::getline(ifs, line)) {
      if (ifs.eof()) {
        RCLCPP_DEBUG(get_logger(), "Reached EOF at iteration=%zu", iteration);
      } else {
        RCLCPP_WARN(get_logger(), "Error reading line at iteration=%zu", iteration);
      }
      break;
    }

    parse_log_line(line);

    // Check position after reading
    std::streampos current_pos_end = ifs.tellg();
    if (current_pos_end == std::streampos(-1)) {
      if (ifs.eof()) {
        RCLCPP_DEBUG(get_logger(), "EOF after iteration=%zu", iteration);
      } else {
        RCLCPP_WARN(get_logger(), "tellg() failed after reading line at iteration=%zu", iteration);
      }
      break;
    }

    last_valid_pos = current_pos_end;
    ++iteration;
  }

  // Update last valid position
  if (last_valid_pos != std::streampos(-1)) {
    last_file_pos_ = static_cast<size_t>(last_valid_pos);
    RCLCPP_DEBUG(get_logger(), "Set last_file_pos_=%zu after reading", last_file_pos_);
  } else {
    RCLCPP_WARN(get_logger(), "No valid position found at the end");
  }
}

void ProcessAliveMonitor::parse_log_line(const std::string & line)
{
  const std::string target_str = "process has died";
  if (line.find(target_str) == std::string::npos) {
    if (enable_debug_) {
      RCLCPP_INFO(
        get_logger(), "[DEBUG] The log line does not contain '%s': skip\nline='%s'",
        target_str.c_str(), line.c_str());
    }
    return;
  }

  // Parse exit code
  int exit_code = -1;
  {
    static const std::regex exit_code_pattern("exit code\\s+(-?[0-9]+)");
    std::smatch match_exit;
    if (std::regex_search(line, match_exit, exit_code_pattern)) {
      try {
        exit_code = std::stoi(match_exit[1]);
      } catch (...) {
        exit_code = -1;
      }

      if (enable_debug_) {
        RCLCPP_WARN(
          get_logger(), "[DEBUG] Parsed exit_code=%d from log line (line='%s')", exit_code,
          line.c_str());
      }
    } else {
      if (enable_debug_) {
        RCLCPP_WARN(
          get_logger(), "[DEBUG] Could not parse exit_code from log line (line='%s')",
          line.c_str());
      }
    }
  }

  // Filter by exit code
  if (
    std::find(ignore_exit_codes_.begin(), ignore_exit_codes_.end(), exit_code) !=
    ignore_exit_codes_.end()) {
    RCLCPP_WARN(
      get_logger(),
      "[DEBUG] Ignoring process died log (exit_code=%d is in ignore_exit_codes_). line='%s'",
      exit_code, line.c_str());
    return;
  }

  // Extract node name from log line
  static const std::regex node_name_pattern("\\[([^\\]]+)\\]\\:\\s*process has died");
  std::smatch match_node;
  if (std::regex_search(line, match_node, node_name_pattern)) {
    const std::string node_id = match_node[1];
    if (enable_debug_) {
      RCLCPP_WARN(get_logger(), "[DEBUG] Extracted node_id='%s'", node_id.c_str());
    }

    // Filter by node name
    for (const auto & ignore : ignore_node_names_) {
      if (node_id.find(ignore) != std::string::npos) {
        if (enable_debug_) {
          RCLCPP_WARN(
            get_logger(), "[DEBUG] Ignoring node death: node_id='%s' matched ignore='%s'",
            node_id.c_str(), ignore.c_str());
        }
        return;
      }
    }

    // Register dead node
    dead_nodes_[node_id] = true;

    RCLCPP_WARN(
      get_logger(), "Detected node death from launch.log: node_id='%s' (exit_code=%d)\n  line='%s'",
      node_id.c_str(), exit_code, line.c_str());
  } else {
    if (enable_debug_) {
      RCLCPP_WARN(
        get_logger(),
        "[DEBUG] Could not extract [node_name-#] (with ': process has died') from log line='%s'",
        line.c_str());
    }
  }
}

void ProcessAliveMonitor::on_timer()
{
  read_launch_log_diff();

  // Report dead nodes
  if (!dead_nodes_.empty()) {
    std::string report = "Dead nodes detected: ";
    for (const auto & kv : dead_nodes_) {
      if (kv.second) {
        report += kv.first + " ";
      }
    }
    RCLCPP_WARN(get_logger(), "%s", report.c_str());
  } else if (enable_debug_) {
    RCLCPP_WARN(get_logger(), "[DEBUG] on_timer: No dead nodes so far.");
  }
}

}  // namespace autoware::process_alive_monitor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::process_alive_monitor::ProcessAliveMonitor)
