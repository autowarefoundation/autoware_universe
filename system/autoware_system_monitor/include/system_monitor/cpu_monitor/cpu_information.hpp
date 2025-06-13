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

/**
 * @file cpu_information.h
 * @brief information about CPUs/cores
 */

#ifndef SYSTEM_MONITOR__CPU_MONITOR__CPU_INFORMATION_HPP_
#define SYSTEM_MONITOR__CPU_MONITOR__CPU_INFORMATION_HPP_

#include <string>
#include <vector>

/**
 * @brief CPU temperature information
 */
struct CpuTemperatureInfo
{
  std::string label_;  //!< @brief cpu label
  std::string path_;   //!< @brief sysfs path to cpu temperature

  CpuTemperatureInfo() : label_(), path_() {}
  CpuTemperatureInfo(const std::string & label, const std::string & path)
  : label_(label), path_(path)
  {
  }
};

/**
 * @brief CPU frequency information
 */
struct CpuFrequencyInfo
{
  int index_;         //!< @brief cpu index
  std::string path_;  //!< @brief sysfs path to cpu frequency

  CpuFrequencyInfo() : index_(0), path_() {}
  CpuFrequencyInfo(int index, const std::string & path) : index_(index), path_(path) {}
};

struct TemperatureData
{
  float elapsed_ms;
  int summary_status;
  std::string summary_message;
  struct CoreTemperature
  {
    std::string label;
    int status;  //!< @brief Error if temperature data is not available.
    int level;   //!< @brief Level of temperature.
    float temperature;
    std::string error_key;
    std::string error_value;
  };
  std::vector<CoreTemperature> core_data;

  void clear()
  {
    elapsed_ms = 0.0f;
    summary_status = 0;
    summary_message.clear();
    core_data.clear();  // Allocated heap memory is not released.
  }
};

struct UsageData
{
  float elapsed_ms;
  int summary_status;
  std::string summary_message;
  std::string error_key;
  std::string error_value;

  struct CpuUsage
  {
    std::string label;
    int status;
    float usr;
    float nice;
    float sys;
    float iowait;
    float idle;
    float total;
  };
  std::vector<CpuUsage> core_data;

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

struct LoadData
{
  float elapsed_ms;
  int summary_status;
  std::string summary_message;
  std::string error_key;
  std::string error_value;
  double load_average[3];

  void clear()
  {
    elapsed_ms = 0.0f;
    summary_status = 0;
    summary_message.clear();
    load_average[0] = 0.0;
    load_average[1] = 0.0;
    load_average[2] = 0.0;
  }
};

struct FrequencyData
{
  float elapsed_ms;
  int summary_status;
  std::string summary_message;
  struct CoreFrequency
  {
    int index;
    int status;
    int frequency_khz;
  };
  std::vector<CoreFrequency> core_data;

  void clear()
  {
    elapsed_ms = 0.0f;
    summary_status = 0;
    summary_message.clear();
    core_data.clear();  // Allocated heap memory is not released.
  }
};
#endif  // SYSTEM_MONITOR__CPU_MONITOR__CPU_INFORMATION_HPP_
