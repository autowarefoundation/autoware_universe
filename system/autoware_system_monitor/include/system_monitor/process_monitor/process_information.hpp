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

#ifndef SYSTEM_MONITOR__PROCESS_MONITOR__PROCESS_INFORMATION_HPP_
#define SYSTEM_MONITOR__PROCESS_MONITOR__PROCESS_INFORMATION_HPP_

// This file defines structures for process information.
// As the definitions are implementation dependent,
// this file should be included only from process_monitor.cpp to reduce dependencies.

#include <cstdint>  // for int32_t, int64_t
#include <memory>
#include <string>
#include <vector>

// For type and size of fields, see "man 5 proc".
struct StatInfo
{
  int32_t pid;
  std::string command;  // with parentheses, may contain spaces.
  char state;
  int32_t ppid;
  int32_t pgrp;
  int32_t session;
  int32_t tty_nr;
  int32_t tpgid;
  uint64_t flags;  // %u (%lu before Linux 2.6.22)
  uint64_t min_flt;
  uint64_t c_min_flt;
  uint64_t maj_flt;
  uint64_t c_maj_flt;
  uint64_t utime_tick;
  uint64_t stime_tick;
  int64_t c_utime_tick;
  int64_t c_stime_tick;
  int64_t priority;
  int64_t nice;
  int32_t num_threads;
  int64_t it_real_value;    // not maintained since kernel 2.6.17
  uint64_t starttime_tick;  // %llu (%lu before Linux 2.6)
  uint64_t vsize_byte;      // bytes
  int64_t rss_page;         // pages, inaccurate : See "man 5 proc"
  // The following fields are not used in this implementation.
};

struct StatMemoryInfo
{
  int64_t size_page;      // pages
  int64_t resident_page;  // pages
  int64_t share_page;     // pages
  // The following fields are not used in this implementation.
};

struct StatusInfo
{
  std::string command;
  uid_t real_uid;
  uid_t effective_uid;
  uid_t saved_set_uid;
  uid_t filesystem_uid;
  // Only necessary fields are extracted from /proc/#/status.
};

struct DiffInfo
{
  int64_t cpu_usage;  // "latest (utime + stime)" - "previous (utime + stime)"
};

struct RawProcessInfo
{
  StatInfo stat_info;
  StatMemoryInfo stat_memory_info;
  StatusInfo status_info;
  DiffInfo diff_info;
};

struct ProcessStatistics
{
  std::vector<std::unique_ptr<RawProcessInfo>> load_tasks_raw;
  std::vector<std::unique_ptr<RawProcessInfo>> memory_tasks_raw;
  int32_t state_running;
  int32_t state_sleeping;
  int32_t state_stopped;
  int32_t state_zombie;
  double uptime_delta_sec;
};

#endif  // SYSTEM_MONITOR__PROCESS_MONITOR__PROCESS_INFORMATION_HPP_
