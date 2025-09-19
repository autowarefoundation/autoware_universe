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
 * @file process_monitor.cpp
 * @brief Process monitor class
 */

#include "system_monitor/process_monitor/process_monitor.hpp"

#include "system_monitor/process_monitor/process_information.hpp"

#include <autoware_utils/system/stop_watch.hpp>

#include <dirent.h>
#include <fmt/format.h>
#include <pwd.h>
#include <sys/types.h>
#include <unistd.h>  // for gethostname()

#include <algorithm>
#include <cmath>  // for std::ceil()
#include <fstream>
#include <iostream>
#include <memory>
#include <regex>
#include <string>
#include <utility>
#include <vector>

namespace
{

std::string to7DigitString(double size_kb)
{
  constexpr double KB_MAX_KB = 9999999;
  constexpr double MB_MAX_KB = 9999.9 * 1024;  // "9999.9m" takes 7 characters.
  if (size_kb < 0.0) {
    size_kb = 0.0;  // negative value is not expected.
  }
  try {
    if (size_kb <= KB_MAX_KB) {
      return fmt::format("{:.0f}", size_kb);  // Shown as KiB.
    } else if (size_kb <= MB_MAX_KB) {
      // 9765.7m (rounding up) > 9999999 > 9765.6m (rounding down)
      double value = std::ceil(((size_kb / 1024) * 10.0)) / 10.0;
      return fmt::format("{:.1f}m", value);  // Shown as MiB.
    } else {
      // 976.6g (rounding up) > 999999m > 976.5g (rounding down)
      double value = std::ceil(((size_kb / (1024 * 1024)) * 10.0)) / 10.0;
      return fmt::format("{:.1f}g", value);  // Shown as GiB.
    }
  } catch (...) {
    return "?";
  }
}

std::string to6DigitString(double size_kb)
{
  constexpr double KB_MAX_KB = 999999;
  constexpr double MB_MAX_KB = 999.9 * 1024;  // "999.9m" takes 6 characters.
  if (size_kb < 0.0) {
    size_kb = 0.0;  // negative value is not expected.
  }
  try {
    if (size_kb <= KB_MAX_KB) {
      return fmt::format("{:.0f}", size_kb);  // Shown as KiB.
    } else if (size_kb <= MB_MAX_KB) {
      // 976.6m (rounding up) > 999999 > 976.5m (rounding down)
      double value = std::ceil(((size_kb / 1024) * 10.0)) / 10.0;
      return fmt::format("{:.1f}m", value);  // Shown as MiB.
    } else {
      // 97.7g (rounding up) > 99999m > 97.6g (rounding down)
      double value = std::ceil(((size_kb / (1024 * 1024)) * 10.0)) / 10.0;
      return fmt::format("{:.1f}g", value);  // Shown as GiB.
    }
  } catch (...) {
    return "?";
  }
}

std::string formatCpuUsage(
  const int64_t cpuUsage, const double uptime_delta_sec, const int64_t clock_tick_)
{
  try {
    return fmt::format("{:.1f}", (cpuUsage * 100 / (uptime_delta_sec * clock_tick_)));
  } catch (...) {
    return "?";
  }
}

std::string formatMemoryUsage(const int64_t resident_kb, int64_t total_kb)
{
  constexpr double MAX_USAGE = 99.9;  // Max value for fmt::format("{4.1f}")"
  constexpr double MIN_USAGE = 0.0;   // Usage can't be negative.
  try {
    if (total_kb <= 0) {
      total_kb = 1;  // Avoid division by zero
    }
    double memory_usage =
      (static_cast<double>(resident_kb) / static_cast<double>(total_kb)) * 100.0;
    if (memory_usage > MAX_USAGE) {
      memory_usage = MAX_USAGE;
    }
    if (memory_usage < MIN_USAGE) {
      memory_usage = MIN_USAGE;
    }
    return fmt::format("{:.1f}", memory_usage);
  } catch (...) {
    return "?";
  }
}

// Employ the same time format as that of scale_tics() function in "top" command.
// While "top" cares about variable window width, we don't need to care about it.
// With "top -bn1 -w 128", 9 character space is allocated for the "TIME+" column.
// Examples:
//   "999:59.99" : 999 minutes, 59.99 seconds
//   "999999:59" : 999999 minutes, 59 seconds
//   "999999,59" : 999999 hours, 59 minutes
//   "99999999h" : 99999999 hours
//   "99999999d" : 99999999 days
//   "99999999w" : 99999999 weeks
//   "?"         : Too long...
std::string formatCpuTime(int64_t time_in_tick, int64_t clock_tick)
{
  constexpr int64_t MAX_3_DIGIT = 999;
  constexpr int64_t MAX_6_DIGIT = 999999;
  constexpr int64_t MAX_8_DIGIT = 99999999;

  if (time_in_tick < 0) {
    time_in_tick = 0;
  }
  if (clock_tick <= 0) {
    clock_tick = 1;  // Avoid division by zero
  }

  try {
    int64_t seconds = time_in_tick / clock_tick;
    int64_t sub_sec = ((time_in_tick - (seconds * clock_tick)) * 100) / clock_tick;
    int64_t minutes = seconds / 60;
    seconds -= (minutes * 60);
    if (minutes <= MAX_3_DIGIT) {
      return fmt::format("{}:{:02}.{:02}", minutes, seconds, sub_sec);
    } else if (seconds <= MAX_6_DIGIT) {
      return fmt::format("{}:{:02}", minutes, seconds);
    }

    int64_t hours = minutes / 60;
    minutes -= (hours * 60);
    if (hours <= MAX_6_DIGIT) {
      return fmt::format("{},{:02}", hours, minutes);
    } else if (hours <= MAX_8_DIGIT) {
      return fmt::format("{}h", hours);
    }

    int64_t days = hours / 24;
    hours -= (days * 24);
    if (days <= MAX_8_DIGIT) {
      return fmt::format("{}d", days);
    }

    int64_t weeks = days / 7;
    days -= (weeks * 7);
    if (days <= MAX_8_DIGIT) {
      return fmt::format("{}w", weeks);
    }
    // time_in_tick is too large. There seems to be something wrong.
    return "?";
  } catch (...) {
    return "?";
  }
}

std::string convertUidToUserName(uid_t uid)
{
  struct passwd user_data;
  // The size of work buffer where getpwuid_r() stores data.
  constexpr int BUFFER_SIZE = 4096;
  char work_buf[BUFFER_SIZE];
  struct passwd * result;

  int rtn = getpwuid_r(uid, &user_data, work_buf, BUFFER_SIZE, &result);

  if ((rtn == 0) && (result == &user_data)) {
    // The corresponding user name was found.
    std::string username(user_data.pw_name);
    // Old implementation of process_monitor used "top -bn1 -w 128" to get information.
    // "top -bn1 -w 128" assigns 8 characters for the USER column.
    constexpr size_t MAX_NAME_LENGTH = 8;
    if (username.size() > MAX_NAME_LENGTH) {
      username.replace((MAX_NAME_LENGTH - 1), 1, "+", 1);
    }
    return username.substr(0, MAX_NAME_LENGTH);
  }
  // There was no such User ID in passwd or an error occurred.
  // Return uid as a string.
  return std::to_string(uid);
}

bool isNumeric(const std::string & str)
{
  return !str.empty() && (str.find_first_not_of("0123456789") == std::string::npos);
}

bool isProcessID(const char * entry)
{
  const std::string name(entry);

  // A process/thread entry has a numeric name.
  // No need to distinguish between process and thread.
  return isNumeric(name);
}

bool readStat(const std::string & proc_path, StatInfo & info)
{
  std::string stat_path = proc_path + "stat";

  std::ifstream stat_file(stat_path);
  if (!stat_file) {
    return false;
  }
  std::string line;
  if (!std::getline(stat_file, line, '\n')) {
    return false;
  }

  StatInfo info_temp{};
  try {
    std::istringstream line_stream(line);
    line_stream.exceptions(std::ios::failbit | std::ios::badbit);
    line_stream >> info_temp.pid;
    line_stream.exceptions();  // Reset to default state
  } catch (...) {
    return false;
  }

  // command may include spaces. Ex. (UVM deferred release queue)
  // command may include multiple pairs of parentheses. Ex. ((XXX))
  const std::size_t left_parenthesis_pos = line.find('(');
  const std::size_t right_parenthesis_pos = line.find_last_of(')');
  if ((left_parenthesis_pos == std::string::npos) || (right_parenthesis_pos == std::string::npos)) {
    return false;
  }

  const std::size_t command_len =
    right_parenthesis_pos - left_parenthesis_pos + 1;  // includes parentheses.
  try {  // Handle exceptions from std::string::substr(). Just in case.
    const std::string command = line.substr(left_parenthesis_pos, command_len);
    const std::string after_command = line.substr(right_parenthesis_pos + 1);
    info_temp.command = command;

    try {
      std::istringstream after_command_stream(after_command);
      after_command_stream.exceptions(std::ios::failbit | std::ios::badbit);
      after_command_stream >> info_temp.state >> info_temp.ppid >> info_temp.pgrp >>
        info_temp.session >> info_temp.tty_nr >> info_temp.tpgid >> info_temp.flags >>
        info_temp.min_flt >> info_temp.c_min_flt >> info_temp.maj_flt >> info_temp.c_maj_flt >>
        info_temp.utime_tick >> info_temp.stime_tick >> info_temp.c_utime_tick >>
        info_temp.c_stime_tick >> info_temp.priority >> info_temp.nice >> info_temp.num_threads >>
        info_temp.it_real_value >> info_temp.starttime_tick >> info_temp.vsize_byte >>
        info_temp.rss_page;
      after_command_stream.exceptions();  // Reset to default state
    } catch (...) {
      return false;  // Failed to read all values
    }
    info = info_temp;
    return true;
  } catch (...) {
    return false;
  }
}

bool readStatMemory(const std::string & proc_path, StatMemoryInfo & info)
{
  // cspell:ignore statm
  const std::string stat_memory_path = proc_path + "statm";

  std::ifstream stat_memory_file(stat_memory_path);
  if (!stat_memory_file) {
    return false;
  }
  StatMemoryInfo info_temp{};
  try {
    stat_memory_file.exceptions(std::ios::failbit | std::ios::badbit);
    stat_memory_file >> info_temp.size_page >> info_temp.resident_page >> info_temp.share_page;
    stat_memory_file.exceptions();  // Reset to default state
  } catch (...) {
    stat_memory_file.exceptions();  // Reset even if exception occurs
    return false;                   // Failed to read all values
  }
  info = info_temp;
  return true;
}

bool readStatus(const std::string & proc_path, StatusInfo & info)
{
  std::string status_path = proc_path + "status";

  std::ifstream status_file(status_path);
  if (!status_file) {
    return false;
  }
  StatusInfo info_temp{};
  constexpr uint FOUND_NAME = 0x1;
  constexpr uint FOUND_UID = 0x2;
  constexpr uint FOUND_ALL = FOUND_NAME | FOUND_UID;
  uint found_entries = 0x0;
  std::string line;
  while (std::getline(status_file, line, '\n')) {
    if (found_entries == FOUND_ALL) {
      break;
    }
    std::size_t first_delimiter_pos = line.find('\t');  // Delimiters in "status" are tabs.
    if (first_delimiter_pos == std::string::npos) {
      continue;  // Skip malformed lines
    }
    try {  // Handle exceptions thrown by std::string::substr(). Just in case.
      std::string header = line.substr(0, first_delimiter_pos);
      if (header == "Name:") {
        std::size_t cmd_pos =
          line.find_first_not_of("\t ", first_delimiter_pos);  // Not TABs or spaces
        // "Name:" line may contain multiple words delimited by spaces.
        // Ex. "Name: UVM deferred release queue"
        std::string command_line = line.substr(cmd_pos);
        info_temp.command = command_line;
        found_entries |= FOUND_NAME;
      } else if (header == "Uid:") {
        try {
          std::istringstream uid_line(line.substr(first_delimiter_pos));
          uid_line.exceptions(std::ios::failbit | std::ios::badbit);
          // Delimiters (tabs and spaces) are skipped.
          uid_line >> info_temp.real_uid >> info_temp.effective_uid >> info_temp.saved_set_uid >>
            info_temp.filesystem_uid;
          uid_line.exceptions();  // Reset to default state
        } catch (...) {
          continue;  // Skip malformed lines
        }
        found_entries |= FOUND_UID;
      }
    } catch (...) {
      return false;  // Exception is not expected. Further processing is meaningless.
    }
  }
  if (found_entries != FOUND_ALL) {
    return false;
  }
  info = info_temp;
  return true;
}

// Helper function for process ranking about CPU usage
int64_t getCpuUsage(const RawProcessInfo & info)
{
  return info.diff_info.cpu_usage;
}

// Helper function for process rankin about memory usage
int64_t getMemoryUsage(const RawProcessInfo & info)
{
  return info.stat_memory_info.resident_page;
}

void updateProcessRanking(
  const RawProcessInfo & info, std::vector<std::unique_ptr<RawProcessInfo>> & tasks,
  int64_t compare_value, int64_t (*get_value)(const RawProcessInfo &))
{
  using TaskIterator = std::vector<std::unique_ptr<RawProcessInfo>>::iterator;
  std::reverse_iterator<TaskIterator> lowest(tasks.end());
  std::reverse_iterator<TaskIterator> highest(tasks.begin());

  std::size_t order = tasks.size();
  for (std::reverse_iterator<TaskIterator> iter = lowest; iter != highest; iter++) {
    if (get_value(**iter) < compare_value) {
      order--;
    } else {
      break;
    }
  }
  // No smaller value found
  if (order >= tasks.size()) {
    return;
  }
  // Shift elements after order to make space for insertion.
  std::size_t dst_index = tasks.size() - 1;
  for (std::reverse_iterator<TaskIterator> iter = lowest; iter != highest; iter++) {
    if (dst_index == order) {
      break;
    }
    **iter = **(iter + 1);
    dst_index--;
  }
  *tasks[order] = info;
}

void invalidateRankingEntry(const std::unique_ptr<RawProcessInfo> & entry)
{
  // Clear only the members used for comparison.
  if (entry) {
    // -1 is used here so that the ranking works even if all processes' CPU/memory usage is 0.
    entry->diff_info.cpu_usage = -1;
    entry->stat_memory_info.resident_page = -1;
  }
}

const char NUM_OF_PROCS_DESCRIPTION[] =
  "Number of processes in High-load[] and High-mem[]. Cannot be changed after initialization.";

}  // namespace

ProcessMonitor::ProcessMonitor(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options),
  updater_(this),
  num_of_procs_(
    declare_parameter<int>(
      "num_of_procs", kDefaultNumProcs,
      rcl_interfaces::msg::ParameterDescriptor().set__read_only(true).set__description(
        NUM_OF_PROCS_DESCRIPTION)))
{
  initialize();
}

ProcessMonitor::ProcessMonitor(const rclcpp::NodeOptions & options)
: Node("process_monitor", options),
  updater_(this),
  num_of_procs_(
    declare_parameter<int>(
      "num_of_procs", kDefaultNumProcs,
      rcl_interfaces::msg::ParameterDescriptor().set__read_only(true).set__description(
        NUM_OF_PROCS_DESCRIPTION)))
{
  initialize();
}

ProcessMonitor::~ProcessMonitor()
{
  finalize();
}

void ProcessMonitor::initialize()
{
  using namespace std::literals::chrono_literals;

  setRoot("/");
  gethostname(hostname_, sizeof(hostname_));

  updater_.setHardwareID(hostname_);
  updater_.add("Tasks Summary", this, &ProcessMonitor::monitorProcesses);

  // As long as the number of processes is less than EXPECTED_NUM_PROCESSES,
  // the size of the maps can be a compile-time constant.
  // This is to avoid the cost of rehashing when the map is resized.
  // When the number of processes exceeds EXPECTED_NUM_PROCESSES,
  // the map will be resized, which is a costly operation.
  constexpr int32_t EXPECTED_NUM_PROCESSES = 1024;
  pid_map1_.reserve(EXPECTED_NUM_PROCESSES);
  pid_map2_.reserve(EXPECTED_NUM_PROCESSES);

  work_ = std::make_unique<ProcessStatistics>();
  snapshot_ = std::make_unique<ProcessStatistics>();

  for (int32_t index = 0; index < getNumOfProcs(); ++index) {
    auto task = std::make_shared<DiagTask>(fmt::format("High-load Proc[{}]", index));
    load_tasks_.emplace_back(task);
    updater_.add(*task);  // The life of task is managed by load_tasks_.
  }
  for (int32_t index = 0; index < getNumOfProcs(); ++index) {
    auto task = std::make_shared<DiagTask>(fmt::format("High-mem Proc[{}]", index));
    memory_tasks_.emplace_back(task);
    updater_.add(*task);  // The life of task is managed by memory_tasks_.
  }

  for (int32_t index = 0; index < getNumOfProcs(); ++index) {
    work_->load_tasks_raw.emplace_back(std::make_unique<RawProcessInfo>());
    work_->memory_tasks_raw.emplace_back(std::make_unique<RawProcessInfo>());
    snapshot_->load_tasks_raw.emplace_back(std::make_unique<RawProcessInfo>());
    snapshot_->memory_tasks_raw.emplace_back(std::make_unique<RawProcessInfo>());
  }
  uptime_prev_sec_ = 0.0;
  page_size_kb_ = ::sysconf(_SC_PAGESIZE) / 1024;
  clock_tick_ = ::sysconf(_SC_CLK_TCK);

  // Start timer to collect process information periodically.
  timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_ = rclcpp::create_timer(
    this, get_clock(), 1s, std::bind(&ProcessMonitor::onTimer, this), timer_callback_group_);
}

void ProcessMonitor::finalize()
{
  timer_->cancel();
  for (const auto & iter : load_tasks_) {
    updater_.removeByName(iter->getName());
  }
  for (const auto & iter : memory_tasks_) {
    updater_.removeByName(iter->getName());
  }
  load_tasks_.clear();
  memory_tasks_.clear();

  updater_.removeByName("Tasks Summary");
}

void ProcessMonitor::setRoot(const std::string & root_path)
{
  std::string new_root_path = root_path;
  if (new_root_path.empty() || new_root_path.back() != '/') {
    new_root_path.append(1, '/');
  }
  root_path_ = new_root_path;
  // /proc/meminfo is read only when setRoot() is called.
  // If it can't be read, /proc pseudo-filesystem may not be mounted.
  bool meminfo_error_occurred = !readMemInfo();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    fatal_error_occurred_ = meminfo_error_occurred;
  }
}

std::string ProcessMonitor::getRoot() const
{
  return root_path_;
}

void ProcessMonitor::getTasksSummary(
  const ProcessStatistics & snapshot, diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.add(
    "total", std::to_string(
               snapshot.state_running + snapshot.state_sleeping + snapshot.state_stopped +
               snapshot.state_zombie));
  stat.add("running", std::to_string(snapshot.state_running));
  stat.add("sleeping", std::to_string(snapshot.state_sleeping));
  stat.add("stopped", std::to_string(snapshot.state_stopped));
  stat.add("zombie", std::to_string(snapshot.state_zombie));
  stat.summary(DiagStatus::OK, "OK");
}

// It is not always guaranteed that /proc/[pid]/cmdline file is readable.
// Please see "man 5 proc".
bool ProcessMonitor::getCommandLineFromPid(const std::string & pid, std::string & command) const
{
  std::string commandLineFilePath = root_path_ + "proc/" + pid + "/cmdline";
  std::ifstream commandFile(commandLineFilePath, std::ios::in | std::ios::binary);

  if (!commandFile) {
    return false;
  }
  std::vector<uint8_t> buffer;
  std::copy(
    std::istream_iterator<uint8_t>(commandFile), std::istream_iterator<uint8_t>(),
    std::back_inserter(buffer));
  if (buffer.size() == 0) {  // cmdline is empty if it is a kernel process
    return false;
  }
  // 0x00 is used as delimiter in /cmdline instead of 0x20 (space)
  // Leave the last 0x00 (end-of-C-string) intact.
  std::replace(buffer.begin(), (buffer.end() - 1), '\0', ' ');
  // Make sure the last character is a null terminator.

  // Although buffer.back() is safer and more idiomatic,
  // using it here causes a compile error with -Werror=stringop-overflow.
  // Therefore, we safely access the last element by index.
  buffer[buffer.size() - 1] = '\0';
  try {
    std::string cmdline = std::string(buffer.begin(), (buffer.end() - 1));
    // Remove trailing spaces
    command = cmdline.substr(0, cmdline.find_last_not_of(' ') + 1);
    return true;
  } catch (const std::out_of_range &) {
    return false;
  }
}

void ProcessMonitor::fillTaskInfo(
  const std::unique_ptr<RawProcessInfo> & raw_p, const double uptime_delta_sec,
  const std::shared_ptr<DiagTask> & task_p)
{
  ProcessInfo info;
  info.processId = std::to_string(raw_p->stat_info.pid);
  info.userName = convertUidToUserName(raw_p->status_info.real_uid);
  // For backward compatibility with the old implementation with Linux "top" command,
  // real-time processes need exceptional handling.
  // Linux "top" command shows priority less than -99 and more than 999 as "rt", which means
  // "real-time".
  if ((raw_p->stat_info.priority < -99) || (raw_p->stat_info.priority > 999)) {
    info.priority = "rt";
  } else {
    info.priority = std::to_string(raw_p->stat_info.priority);
  }
  info.niceValue = std::to_string(raw_p->stat_info.nice);
  auto virtual_image_size_kb = raw_p->stat_memory_info.size_page * page_size_kb_;
  auto resident_size_kb = raw_p->stat_memory_info.resident_page * page_size_kb_;
  auto shared_mem_size_kb = raw_p->stat_memory_info.share_page * page_size_kb_;
  info.virtualImage = to7DigitString(virtual_image_size_kb);
  info.residentSize = to6DigitString(resident_size_kb);
  info.sharedMemSize = to6DigitString(shared_mem_size_kb);
  info.processStatus = raw_p->stat_info.state;
  double cpuUsage = raw_p->diff_info.cpu_usage;
  info.cpuUsage = formatCpuUsage(cpuUsage, uptime_delta_sec, clock_tick_);
  info.memoryUsage = formatMemoryUsage(resident_size_kb, mem_total_kb_);
  info.cpuTime =
    formatCpuTime(raw_p->stat_info.utime_tick + raw_p->stat_info.stime_tick, clock_tick_);

  std::string commandName;
  bool flag_find_command_line = getCommandLineFromPid(info.processId, commandName);
  if (flag_find_command_line) {
    info.commandName = commandName;
  } else {
    info.commandName = raw_p->status_info.command;
  }

  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;
  task_p->setDiagnosticsStatus(DiagStatus::OK, "OK");
  task_p->setProcessInformation(info);
}

void ProcessMonitor::getHighLoadProcesses(const ProcessStatistics & snapshot)
{
  for (std::size_t i = 0; i < load_tasks_.size(); i++) {
    fillTaskInfo(snapshot.load_tasks_raw[i], snapshot.uptime_delta_sec, load_tasks_[i]);
  }
}

void ProcessMonitor::getHighMemoryProcesses(const ProcessStatistics & snapshot)
{
  for (std::size_t i = 0; i < memory_tasks_.size(); i++) {
    fillTaskInfo(snapshot.memory_tasks_raw[i], snapshot.uptime_delta_sec, memory_tasks_[i]);
  }
}

void ProcessMonitor::monitorProcesses(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (error_occurred_ || fatal_error_occurred_) {
    stat.summary(DiagStatus::ERROR, "Failed to read /proc");
    setErrorContent("Failed to read /proc", "Failed to read /proc", "", load_tasks_);
    setErrorContent("Failed to read /proc", "Failed to read /proc", "", memory_tasks_);
    return;
  }
  getTasksSummary(*snapshot_, stat);
  getHighLoadProcesses(*snapshot_);
  getHighMemoryProcesses(*snapshot_);
  stat.addf("execution time", "%f ms", elapsed_ms_);
}

void ProcessMonitor::accumulateStateCount(const RawProcessInfo & info)
{
  // The definitions of states are different among Linux versions.
  // See /proc/[pid]/state of "man 5 proc".
  switch (info.stat_info.state) {
    case 'R':
      work_->state_running++;
      break;
    case 'Z':
      work_->state_zombie++;
      break;
    case 't':  // Tracing stop
    case 'T':  // Stopped
      work_->state_stopped++;
      break;
    // To reduce case evaluation, the following states are handled in the default case.
    // case 'S':  // Sleeping
    // case 'I':  // Idle kernel task, not documented in "man 5 proc"
    // case 'D':  // Disk Sleep
    // case 'X':  // Dead
    // case 'x':  // Dead
    // case 'W':  // Paging/Waking
    // case 'K':  // Wakekill
    // case 'P':  // Parked
    default:
      work_->state_sleeping++;
      break;
  }
}

void ProcessMonitor::initializeProcessStatistics()
{
  for (const std::unique_ptr<RawProcessInfo> & entry : work_->load_tasks_raw) {
    invalidateRankingEntry(entry);
  }
  for (const std::unique_ptr<RawProcessInfo> & entry : work_->memory_tasks_raw) {
    invalidateRankingEntry(entry);
  }
  work_->state_running = 0;
  work_->state_zombie = 0;
  work_->state_stopped = 0;
  work_->state_sleeping = 0;
  work_->uptime_delta_sec = 0.0;
}

// Use comparison function to update Load, Memory
void ProcessMonitor::updateHighLoadProcessRanking(const RawProcessInfo & info)
{
  updateProcessRanking(info, work_->load_tasks_raw, info.diff_info.cpu_usage, getCpuUsage);
}

void ProcessMonitor::updateHighMemoryProcessRanking(const RawProcessInfo & info)
{
  updateProcessRanking(
    info, work_->memory_tasks_raw, info.stat_memory_info.resident_page, getMemoryUsage);
}

void ProcessMonitor::registerProcessInfoToNewMap(const pid_t pid, const RawProcessInfo & info)
{
  std::shared_ptr<RawProcessInfo> prev_info{nullptr};
  try {
    prev_info = prev_map_.at(pid);
  } catch (...) {
    ;
  }
  auto rtn = new_map_.emplace(pid, std::make_shared<RawProcessInfo>(info));
  int64_t cpu_usage;
  if (prev_info) {
    cpu_usage =
      static_cast<int64_t>(info.stat_info.utime_tick + info.stat_info.stime_tick) -
      static_cast<int64_t>(prev_info->stat_info.utime_tick + prev_info->stat_info.stime_tick);
  } else {
    // Pid is a new process, which is not in prev_map_
    cpu_usage = static_cast<int64_t>(info.stat_info.utime_tick + info.stat_info.stime_tick);
  }
  if (cpu_usage < 0) {
    cpu_usage = 0;
  }
  (rtn.first->second)->diff_info.cpu_usage = cpu_usage;
}

void ProcessMonitor::rotateProcessMaps()
{
  auto temp = prev_map_;
  prev_map_ = new_map_;
  // Data of PIDs that are referenced only in prev_map_ are deleted as they are no longer in use.
  new_map_ = temp;
  new_map_.clear();
}

bool ProcessMonitor::readMemInfo()
{
  const std::string meminfoPath = root_path_ + "proc/meminfo";

  std::ifstream meminfoFile(meminfoPath);
  if (!meminfoFile) {
    // Failed to open /proc/meminfo.
    // mem_total_kb_ should not be zero to avoid "divide by zero" exception.
    mem_total_kb_ = 1;
    return false;
  }

  constexpr uint32_t FOUND_MEM_TOTAL = 0x1;
  constexpr uint32_t FOUND_ALL = FOUND_MEM_TOTAL;
  uint32_t found_entries = 0x0;
  std::string line;
  while (std::getline(meminfoFile, line, '\n')) {
    if (found_entries == FOUND_ALL) {
      break;
    }
    std::size_t first_delimiter_pos = line.find(' ');  // Delimiters in "meminfo" are spaces.
    if (first_delimiter_pos == std::string::npos) {
      continue;  // Skip malformed lines
    }
    try {  // Handle exceptions thrown by std::string::substr(). Just in case.
      std::string parameter_name = line.substr(0, first_delimiter_pos);
      if (parameter_name == "MemTotal:") {
        std::istringstream parameters(line.substr(first_delimiter_pos));
        uint64_t mem_total;
        // Assuming that the unit is "kB".
        try {
          parameters.exceptions(std::ios::failbit | std::ios::badbit);
          parameters >> mem_total;  // Ignore optional unit "kB".
          parameters.exceptions();  // Reset to default state
        } catch (...) {
          parameters.exceptions();  // Reset even if exception occurs
          continue;                 // Skip malformed lines
        }
        mem_total_kb_ = mem_total;
        found_entries |= FOUND_MEM_TOTAL;
      }
    } catch (...) {
      break;  // Exception is not expected. Further looping is meaningless.
    }
  }
  if (mem_total_kb_ == 0) {
    // mem_total_kb_ should not be zero to avoid "divide by zero" exception.
    mem_total_kb_ = 1;
    return false;
  }
  return true;
}

void ProcessMonitor::collectProcessInfo(const char * pid_str)
{
  RawProcessInfo info;

  std::string procPath(root_path_ + "proc/");
  procPath += pid_str;
  procPath += "/";

  // If any of the following functions returns false, the process is not a valid process.
  if (!readStat(procPath, info.stat_info)) {
    return;
  }
  if (!readStatMemory(procPath, info.stat_memory_info)) {
    return;
  }
  if (!readStatus(procPath, info.status_info)) {
    return;
  }

  pid_t pid;
  try {
    pid = static_cast<pid_t>(std::stoul(pid_str));
  } catch (...) {
    return;
  }
  // CPU usage calculation is not possible from static information in /proc.
  // Calculate the difference from the previous information.
  registerProcessInfoToNewMap(pid, info);
}

bool ProcessMonitor::scanProcFs()
{
  std::string proc_path = root_path_ + "proc";

  // NOTE:
  // opendir() and readdir() are not perfectly thread-safe.
  // There shouldn't be a problem as long as other threads are not accessing /proc.
  // See "man 3 readdir".
  // readdir_r() has been deprecated and can't be used.
  DIR * raw_dirp = opendir(proc_path.c_str());

  if (raw_dirp == nullptr) {
    return false;
  }
  std::unique_ptr<DIR, std::function<void(DIR *)>> directory(
    raw_dirp, [](DIR * dirp) { closedir(dirp); });

  // Scan all directory entries under /proc
  // Note that any entry may disappear after readdir() returns.
  // Read "man 3 readdir" about thread safety.
  for (;;) {
    errno = 0;
    const struct dirent * dir_entry = readdir(directory.get());
    if (dir_entry == nullptr) {
      // If errno is 0, there is no more entry to read in the directory.
      // If errno is not 0, there is an error.
      // In either case, stop further processing.
      break;
    }

    // The maximum length of string d_name is not fixed.
    // See "man 3 readdir".
    if (!isProcessID(dir_entry->d_name)) {
      continue;
    }
    collectProcessInfo(dir_entry->d_name);
  }
  // Information about all valid processes is stored in new_map_
  initializeProcessStatistics();
  for (const auto & iter : new_map_) {
    const RawProcessInfo & info = *(iter.second);
    accumulateStateCount(info);
    updateHighLoadProcessRanking(info);
    updateHighMemoryProcessRanking(info);
  }
  rotateProcessMaps();
  // /proc is closed automatically by the destructor of directory.
  return true;
}

bool ProcessMonitor::getUptime(double & uptime_sec) const
{
  std::ifstream uptimeFile(root_path_ + "proc/uptime");
  if (!uptimeFile) {
    uptime_sec = 0.0;
    return false;
  }
  try {
    double uptime_read = 0.0;
    uptimeFile.exceptions(std::ios::failbit | std::ios::badbit);
    uptimeFile >> uptime_read;
    uptime_sec = uptime_read;
  } catch (...) {
    uptime_sec = 0.0;
    return false;
  }
  return true;
}

void ProcessMonitor::setErrorContent(
  const std::string & message, const std::string & error_command, const std::string & content,
  std::vector<std::shared_ptr<DiagTask>> & tasks)
{
  for (auto itr = tasks.begin(); itr != tasks.end(); ++itr) {
    (*itr)->setDiagnosticsStatus(DiagStatus::ERROR, message);
    (*itr)->setErrorContent(error_command, content);
  }
}

void ProcessMonitor::onTimer()
{
  // Start to measure elapsed time
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic("execution_time");

  bool scan_error_occurred = !scanProcFs();

  double uptime_now_sec{0.0};
  bool uptime_error_occurred = !getUptime(uptime_now_sec);
  const double elapsed_ms = stop_watch.toc("execution_time");

  // thread-safe copy
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Execute deep copy
    for (int i = 0; i < getNumOfProcs(); i++) {
      *(snapshot_->load_tasks_raw[i]) = *(work_->load_tasks_raw[i]);
      *(snapshot_->memory_tasks_raw[i]) = *(work_->memory_tasks_raw[i]);
    }
    snapshot_->state_running = work_->state_running;
    snapshot_->state_sleeping = work_->state_sleeping;
    snapshot_->state_stopped = work_->state_stopped;
    snapshot_->state_zombie = work_->state_zombie;
    double uptime_delta_sec = uptime_now_sec - uptime_prev_sec_;

    // Following the source code of top command.
    if (uptime_delta_sec < 0.01) {
      uptime_delta_sec = 0.005;
    }
    snapshot_->uptime_delta_sec = uptime_delta_sec;

    uptime_prev_sec_ = uptime_now_sec;
    elapsed_ms_ = elapsed_ms;
    error_occurred_ = scan_error_occurred || uptime_error_occurred;
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ProcessMonitor)
