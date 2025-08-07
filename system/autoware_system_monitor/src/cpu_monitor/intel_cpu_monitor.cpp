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
 * @file _cpu_monitor.cpp
 * @brief  CPU monitor class
 */

#include "system_monitor/cpu_monitor/intel_cpu_monitor.hpp"

#include "system_monitor/msr_reader/msr_reader.hpp"
#include "system_monitor/system_monitor_utility.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/filesystem.hpp>

#include <fmt/format.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <algorithm>
#include <regex>
#include <string>
#include <vector>

namespace fs = boost::filesystem;

CPUMonitor::CPUMonitor(const rclcpp::NodeOptions & options) : CPUMonitorBase("cpu_monitor", options)
{
  msr_reader_port_ = declare_parameter<int>("msr_reader_port", 7634);
}

CPUMonitor::CPUMonitor(const std::string & node_name, const rclcpp::NodeOptions & options)
: CPUMonitorBase(node_name, options)
{
  msr_reader_port_ = declare_parameter<int>("msr_reader_port", 7634);
}

void CPUMonitor::checkThermalThrottling()
{
  // Remember start time to measure elapsed time
  const auto t_start = std::chrono::high_resolution_clock::now();

  // Create a new socket
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    std::lock_guard<std::mutex> lock_snapshot(mutex_snapshot_);
    thermal_throttling_data_.clear();
    thermal_throttling_data_.summary_status = DiagStatus::ERROR;
    thermal_throttling_data_.summary_message = "socket error";
    thermal_throttling_data_.error_key = "socket";
    thermal_throttling_data_.error_value = strerror(errno);
    return;
  }

  // Specify the receiving timeouts until reporting an error
  struct timeval tv;
  tv.tv_sec = 10;
  tv.tv_usec = 0;
  int ret = setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  if (ret < 0) {
    std::lock_guard<std::mutex> lock_snapshot(mutex_snapshot_);
    thermal_throttling_data_.clear();
    thermal_throttling_data_.summary_status = DiagStatus::ERROR;
    thermal_throttling_data_.summary_message = "setsockopt error";
    thermal_throttling_data_.error_key = "setsockopt";
    thermal_throttling_data_.error_value = strerror(errno);
    close(sock);
    return;
  }

  // Connect the socket referred to by the file descriptor
  sockaddr_in addr;
  memset(&addr, 0, sizeof(sockaddr_in));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(msr_reader_port_);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  // cppcheck-suppress cstyleCast
  ret = connect(sock, (struct sockaddr *)&addr, sizeof(addr));
  if (ret < 0) {
    std::lock_guard<std::mutex> lock_snapshot(mutex_snapshot_);
    thermal_throttling_data_.clear();
    thermal_throttling_data_.summary_status = DiagStatus::ERROR;
    thermal_throttling_data_.summary_message = "connect error";
    thermal_throttling_data_.error_key = "connect";
    thermal_throttling_data_.error_value = strerror(errno);
    close(sock);
    return;
  }

  // Receive messages from a socket
  char buf[1024] = "";
  ret = recv(sock, buf, sizeof(buf) - 1, 0);
  if (ret < 0) {
    std::lock_guard<std::mutex> lock_snapshot(mutex_snapshot_);
    thermal_throttling_data_.clear();
    thermal_throttling_data_.summary_status = DiagStatus::ERROR;
    thermal_throttling_data_.summary_message = "recv error";
    thermal_throttling_data_.error_key = "recv";
    thermal_throttling_data_.error_value = strerror(errno);
    close(sock);
    return;
  }
  // No data received
  if (ret == 0) {
    std::lock_guard<std::mutex> lock_snapshot(mutex_snapshot_);
    thermal_throttling_data_.clear();
    thermal_throttling_data_.summary_status = DiagStatus::ERROR;
    thermal_throttling_data_.summary_message = "recv error";
    thermal_throttling_data_.error_key = "recv";
    thermal_throttling_data_.error_value = "No data received";
    close(sock);
    return;
  }

  // Close the file descriptor FD
  ret = close(sock);
  if (ret < 0) {
    std::lock_guard<std::mutex> lock_snapshot(mutex_snapshot_);
    thermal_throttling_data_.clear();
    thermal_throttling_data_.summary_status = DiagStatus::ERROR;
    thermal_throttling_data_.summary_message = "close error";
    thermal_throttling_data_.error_key = "close";
    thermal_throttling_data_.error_value = strerror(errno);
    return;
  }

  // Restore MSR information
  MSRInfo info;

  try {
    std::istringstream iss(buf);
    boost::archive::text_iarchive oa(iss);
    oa >> info;
  } catch (const std::exception & e) {
    std::lock_guard<std::mutex> lock_snapshot(mutex_snapshot_);
    thermal_throttling_data_.clear();
    thermal_throttling_data_.summary_status = DiagStatus::ERROR;
    thermal_throttling_data_.summary_message = "recv error";
    thermal_throttling_data_.error_key = "recv";
    thermal_throttling_data_.error_value = e.what();
    return;
  }

  // msr_reader returns an error
  if (info.error_code_ != 0) {
    std::lock_guard<std::mutex> lock_snapshot(mutex_snapshot_);
    thermal_throttling_data_.clear();
    thermal_throttling_data_.summary_status = DiagStatus::ERROR;
    thermal_throttling_data_.summary_message = "msr_reader error";
    thermal_throttling_data_.error_key = "msr_reader";
    thermal_throttling_data_.error_value = strerror(info.error_code_);
    return;
  }

  int whole_level = DiagStatus::OK;
  int index = 0;

  std::lock_guard<std::mutex> lock_snapshot(mutex_snapshot_);
  thermal_throttling_data_.clear();
  for (auto itr = info.pkg_thermal_status_.begin(); itr != info.pkg_thermal_status_.end();
       ++itr, ++index) {
    int level = DiagStatus::OK;
    if (*itr) {
      level = DiagStatus::ERROR;
    }

    thermal_throttling_data_.core_data.emplace_back(
      fmt::format("CPU {}: Pkg Thermal Status", index), thermal_dictionary_.at(level));

    whole_level = std::max(whole_level, level);
  }

  thermal_throttling_data_.summary_status = whole_level;
  thermal_throttling_data_.summary_message = thermal_dictionary_.at(whole_level);

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

  for (const auto & core_data : thermal_throttling_data_.core_data) {
    stat.add(core_data.first, core_data.second);
  }

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
  const fs::path root("/sys/devices/platform/coretemp.0");

  if (!fs::exists(root)) {
    return;
  }

  for (const fs::path & path : boost::make_iterator_range(
         fs::recursive_directory_iterator(root), fs::recursive_directory_iterator())) {
    if (fs::is_directory(path)) {
      continue;
    }

    std::cmatch match;
    const std::string temperature_input = path.generic_string();

    // /sys/devices/platform/coretemp.0/hwmon/hwmon[0-9]/temp[0-9]_input ?
    if (!std::regex_match(temperature_input.c_str(), match, std::regex(".*temp(\\d+)_input"))) {
      continue;
    }

    CpuTemperatureInfo temperature;
    temperature.path_ = temperature_input;
    temperature.label_ = path.filename().generic_string();

    std::string label = boost::algorithm::replace_all_copy(temperature_input, "input", "label");
    const fs::path label_path(label);
    fs::ifstream ifs(label_path, std::ios::in);
    // If the label file exists, read the label from the file.
    if (ifs) {
      std::string line;
      if (std::getline(ifs, line)) {
        temperature.label_ = line;
      }
      ifs.close();
    }
    temperatures_.push_back(temperature);
  }

  std::sort(
    temperatures_.begin(), temperatures_.end(),
    [](const CpuTemperatureInfo & c1, const CpuTemperatureInfo & c2) {
      std::smatch match;
      const std::regex filter(".*temp(\\d+)_input");
      int n1 = 0;
      int n2 = 0;
      if (std::regex_match(c1.path_, match, filter)) {
        n1 = std::stoi(match[1].str());
      }
      if (std::regex_match(c2.path_, match, filter)) {
        n2 = std::stoi(match[1].str());
      }
      return n1 < n2;
    });  // NOLINT
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CPUMonitor)
