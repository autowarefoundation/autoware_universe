// Copyright 2020 Autoware Foundation
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
 * @file nvml_gpu_monitor.cpp
 * @brief GPU monitor class
 */

#include "system_monitor/gpu_monitor/nvml_gpu_monitor.hpp"

#include "system_monitor/system_monitor_utility.hpp"

#include <boost/algorithm/string.hpp>

#include <fmt/format.h>
#include <sys/time.h>

#include <algorithm>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace
{
// NVML 13 renamed clock throttle reasons to clock event reasons. Keep the
// deprecated identifiers out of the NVML 13+ implementation so this remains
// compatible when those identifiers are removed from a future SDK.
#if defined(NVML_API_VERSION) && NVML_API_VERSION >= 13
constexpr unsigned long long kClockEventReasonGpuIdle = nvmlClocksEventReasonGpuIdle;  // NOLINT
constexpr unsigned long long kClockEventReasonApplicationsClocksSetting =  // NOLINT(runtime/int)
  nvmlClocksEventReasonApplicationsClocksSetting;
constexpr unsigned long long kClockEventReasonSwPowerCap =  // NOLINT(runtime/int)
  nvmlClocksEventReasonSwPowerCap;
constexpr unsigned long long kClockEventReasonSyncBoost = nvmlClocksEventReasonSyncBoost;  // NOLINT
constexpr unsigned long long kClockEventReasonSwThermalSlowdown =  // NOLINT(runtime/int)
  nvmlClocksEventReasonSwThermalSlowdown;
constexpr unsigned long long kClockEventReasonDisplayClockSetting =  // NOLINT(runtime/int)
  nvmlClocksEventReasonDisplayClockSetting;

#else
constexpr unsigned long long kClockEventReasonGpuIdle = nvmlClocksThrottleReasonGpuIdle;  // NOLINT
constexpr unsigned long long kClockEventReasonApplicationsClocksSetting =  // NOLINT(runtime/int)
  nvmlClocksThrottleReasonApplicationsClocksSetting;
constexpr unsigned long long kClockEventReasonSwPowerCap =  // NOLINT(runtime/int)
  nvmlClocksThrottleReasonSwPowerCap;
constexpr unsigned long long kClockEventReasonSyncBoost =  // NOLINT(runtime/int)
  nvmlClocksThrottleReasonSyncBoost;
constexpr unsigned long long kClockEventReasonSwThermalSlowdown =  // NOLINT(runtime/int)
  nvmlClocksThrottleReasonSwThermalSlowdown;
constexpr unsigned long long kClockEventReasonDisplayClockSetting =  // NOLINT(runtime/int)
  nvmlClocksThrottleReasonDisplayClockSetting;
#endif

// Hardware slowdown reasons retain their NVML names in both API versions.
constexpr unsigned long long kClockEventReasonHwSlowdown =  // NOLINT(runtime/int)
  nvmlClocksThrottleReasonHwSlowdown;
constexpr unsigned long long kClockEventReasonHwThermalSlowdown =  // NOLINT(runtime/int)
  nvmlClocksThrottleReasonHwThermalSlowdown;
constexpr unsigned long long kClockEventReasonHwPowerBrakeSlowdown =  // NOLINT(runtime/int)
  nvmlClocksThrottleReasonHwPowerBrakeSlowdown;

// Added by NVML 615; guard on each macro to keep building against older SDKs.
#ifdef nvmlClocksEventReasonBoardLimit
constexpr unsigned long long kClockEventReasonBoardLimit =  // NOLINT(runtime/int)
  nvmlClocksEventReasonBoardLimit;
#endif
#ifdef nvmlClocksEventReasonReliability
constexpr unsigned long long kClockEventReasonReliability =  // NOLINT(runtime/int)
  nvmlClocksEventReasonReliability;
#endif

struct ClockEventReasonName
{
  unsigned long long reason;  // NOLINT
  const char * name;
};

constexpr ClockEventReasonName kClockEventReasonNames[] = {
  {kClockEventReasonGpuIdle, "GpuIdle"},
  {kClockEventReasonApplicationsClocksSetting, "ApplicationsClocksSetting"},
  {kClockEventReasonSwPowerCap, "SwPowerCap"},
  {kClockEventReasonHwSlowdown, "HwSlowdown"},
  {kClockEventReasonSyncBoost, "SyncBoost"},
  {kClockEventReasonSwThermalSlowdown, "SwThermalSlowdown"},
  {kClockEventReasonHwThermalSlowdown, "HwThermalSlowdown"},
  {kClockEventReasonHwPowerBrakeSlowdown, "HwPowerBrakeSlowdown"},
  {kClockEventReasonDisplayClockSetting, "DisplayClockSetting"},
#ifdef nvmlClocksEventReasonBoardLimit
  {kClockEventReasonBoardLimit, "BoardLimit"},
#endif
#ifdef nvmlClocksEventReasonReliability
  {kClockEventReasonReliability, "Reliability"},
#endif
};

const char * clockEventReasonToString(unsigned long long reason)  // NOLINT
{
  for (const auto & reason_name : kClockEventReasonNames) {
    if (reason & reason_name.reason) {
      return reason_name.name;
    }
  }
  return "UNKNOWN";
}

bool isIgnoredClockEventReason(unsigned long long reason)  // NOLINT
{
  switch (reason) {
    case kClockEventReasonGpuIdle:
    case kClockEventReasonApplicationsClocksSetting:
    case kClockEventReasonSwPowerCap:
#ifdef nvmlClocksEventReasonBoardLimit
    case kClockEventReasonBoardLimit:
#endif
#ifdef nvmlClocksEventReasonReliability
    case kClockEventReasonReliability:
#endif
      return true;
    default:
      return false;
  }
}

// NVML 13 (CUDA 13) deprecated nvmlDeviceGetTemperature() and
// nvmlDeviceGetCurrentClocksThrottleReasons() in favor of the V / Event variants.
// These thin wrappers select the appropriate API at compile time so the monitor
// keeps building on both CUDA 13+ and older toolkits.
nvmlReturn_t getGpuTemperature(nvmlDevice_t device, int * temp)
{
#if defined(NVML_API_VERSION) && NVML_API_VERSION >= 13
  nvmlTemperature_t temperature_info{};
  temperature_info.version = nvmlTemperature_v1;
  temperature_info.sensorType = NVML_TEMPERATURE_GPU;
  const nvmlReturn_t ret = nvmlDeviceGetTemperatureV(device, &temperature_info);
  if (ret == NVML_SUCCESS) {
    // nvmlTemperature_t::temperature is signed; keep it signed so a sub-zero
    // reading is not wrapped into a huge unsigned value (false over-temperature).
    *temp = temperature_info.temperature;
  }
  return ret;
#else
  unsigned int temperature = 0;
  const nvmlReturn_t ret = nvmlDeviceGetTemperature(device, NVML_TEMPERATURE_GPU, &temperature);
  if (ret == NVML_SUCCESS) {
    *temp = static_cast<int>(temperature);
  }
  return ret;
#endif
}

nvmlReturn_t getClocksEventReasons(nvmlDevice_t device, unsigned long long * reasons)  // NOLINT
{
#if defined(NVML_API_VERSION) && NVML_API_VERSION >= 13
  return nvmlDeviceGetCurrentClocksEventReasons(device, reasons);
#else
  return nvmlDeviceGetCurrentClocksThrottleReasons(device, reasons);
#endif
}
}  // namespace

GPUMonitor::GPUMonitor(const rclcpp::NodeOptions & options) : GPUMonitorBase("gpu_monitor", options)
{
  nvmlReturn_t ret = nvmlInit();
  if (ret != NVML_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize NVML: %s\n", nvmlErrorString(ret));
  }

  unsigned int deviceCount = 0;
  ret = nvmlDeviceGetCount(&deviceCount);
  if (ret != NVML_SUCCESS) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to retrieve the number of compute devices: %s",
      nvmlErrorString(ret));
  }

  for (unsigned int index = 0; index < deviceCount; ++index) {
    gpu_info info{};
    ret = nvmlDeviceGetHandleByIndex(index, &info.device);
    if (ret != NVML_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to acquire the handle for a particular device [%d]: %s", index,
        nvmlErrorString(ret));
      continue;
    }
    ret = nvmlDeviceGetName(info.device, info.name, NVML_DEVICE_NAME_BUFFER_SIZE);
    if (ret != NVML_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to retrieve the name of this device [%d]: %s", index,
        nvmlErrorString(ret));
      continue;
    }
    ret = nvmlDeviceGetPciInfo(info.device, &info.pci);
    if (ret != NVML_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to retrieve the PCI attributes [%d]: %s", index,
        nvmlErrorString(ret));
      continue;
    }
    if (!getSupportedGPUClocks(index, info.device, info.supported_gpu_clocks)) {
      continue;
    }
    gpus_.push_back(info);
  }
}

GPUMonitor::~GPUMonitor()
{
  shut_down();
}

void GPUMonitor::shut_down()
{
  // When this method is called from the destructor in the process of system shutdown,
  // all nodes are guaranteed to be inactive.
  // Therefore, it is safe to call the NVML library's shutdown function.
  nvmlReturn_t ret = nvmlShutdown();
  if (ret != NVML_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to shut down NVML: %s", nvmlErrorString(ret));
  }
}

void GPUMonitor::checkTemp(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  int level = DiagStatus::OK;
  int index = 0;
  nvmlReturn_t ret{};

  if (gpus_.empty()) {
    stat.summary(DiagStatus::ERROR, "gpu not found");
    return;
  }

  for (auto itr = gpus_.begin(); itr != gpus_.end(); ++itr, ++index) {
    int temp = 0;
    ret = getGpuTemperature(itr->device, &temp);
    if (ret != NVML_SUCCESS) {
      stat.summary(DiagStatus::ERROR, "Failed to retrieve the current temperature");
      stat.add(fmt::format("GPU {}: name", index), itr->name);
      stat.add(fmt::format("GPU {}: bus-id", index), itr->pci.busId);
      stat.add(fmt::format("GPU {}: content", index), nvmlErrorString(ret));
      return;
    }

    level = DiagStatus::OK;
    stat.addf(itr->name, "%d.0 DegC", temp);
    if (temp >= temp_error_) {
      level = std::max(level, static_cast<int>(DiagStatus::ERROR));
    } else if (temp >= temp_warn_) {
      level = std::max(level, static_cast<int>(DiagStatus::WARN));
    }
  }

  stat.summary(level, temp_dict_.at(level));

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

void GPUMonitor::checkUsage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  int whole_level = DiagStatus::OK;
  int index = 0;
  nvmlReturn_t ret{};

  if (gpus_.empty()) {
    stat.summary(DiagStatus::ERROR, "gpu not found");
    return;
  }

  for (auto itr = gpus_.begin(); itr != gpus_.end(); ++itr, ++index) {
    ret = nvmlDeviceGetUtilizationRates(itr->device, &itr->utilization);
    if (ret != NVML_SUCCESS) {
      stat.summary(DiagStatus::ERROR, "Failed to retrieve the current utilization rates");
      stat.add(fmt::format("GPU {}: name", index), itr->name);
      stat.add(fmt::format("GPU {}: bus-id", index), itr->pci.busId);
      stat.add(fmt::format("GPU {}: content", index), nvmlErrorString(ret));
      return;
    }

    const float usage = static_cast<float>(itr->utilization.gpu) / 100.0f;
    const int level = checkGpuUsageLevel(static_cast<size_t>(index), usage);

    stat.add(fmt::format("GPU {}: status", index), load_dict_.at(level));
    stat.add(fmt::format("GPU {}: name", index), itr->name);
    stat.addf(fmt::format("GPU {}: usage", index), "%d.0%%", itr->utilization.gpu);

    addProcessUsage(index, *itr, stat);

    whole_level = std::max(whole_level, level);
  }

  stat.summary(whole_level, load_dict_.at(whole_level));

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

std::vector<GPUMonitor::ProcessUsageSummary> GPUMonitor::aggregateProcessUtilization(
  const std::set<unsigned int> & running_pids, const nvmlProcessUtilizationSample_t * samples,
  unsigned int sample_count,
  unsigned long long & last_timestamp)  // NOLINT(runtime/int)
{
  struct SampleAccumulator
  {
    uint64_t sum = 0;
    uint32_t count = 0;
  };

  unsigned long long max_timestamp = last_timestamp;  // NOLINT(runtime/int)
  std::map<unsigned int, SampleAccumulator> stats_by_pid;

  if (samples != nullptr) {
    for (unsigned int i = 0; i < sample_count; ++i) {
      if (samples[i].timeStamp > max_timestamp) {
        max_timestamp = samples[i].timeStamp;
      }
      if (samples[i].pid == 0U || running_pids.find(samples[i].pid) == running_pids.end()) {
        continue;
      }
      const unsigned int sm_util = (samples[i].smUtil != UINT32_MAX) ? samples[i].smUtil : 0U;
      stats_by_pid[samples[i].pid].sum += sm_util;
      stats_by_pid[samples[i].pid].count += 1U;
    }
  }

  last_timestamp = max_timestamp;

  std::vector<ProcessUsageSummary> summaries;
  summaries.reserve(running_pids.size());
  for (const auto pid : running_pids) {
    if (pid == 0U) {
      continue;
    }
    unsigned int avg_sm = 0U;
    const auto it = stats_by_pid.find(pid);
    if (it != stats_by_pid.end() && it->second.count > 0U) {
      avg_sm =
        static_cast<unsigned int>((it->second.sum + it->second.count / 2U) / it->second.count);
    }
    summaries.push_back(ProcessUsageSummary{pid, avg_sm});
  }

  return summaries;
}

void GPUMonitor::addProcessUsage(
  int index, gpu_info & gpu, diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  nvmlReturn_t ret{};
  std::set<unsigned int> running_pids;

  auto collect_running_pids = [&](auto query_fn, const char * api_name) -> bool {
    unsigned int info_count = MAX_ARRAY_SIZE;
    auto infos = std::make_unique<nvmlProcessInfo_t[]>(info_count);
    ret = query_fn(gpu.device, &info_count, infos.get());
    if (ret == NVML_ERROR_INSUFFICIENT_SIZE && info_count > 0U) {
      infos = std::make_unique<nvmlProcessInfo_t[]>(info_count);
      ret = query_fn(gpu.device, &info_count, infos.get());
    }
    if (ret != NVML_SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "Failed to %s NVML: %s", api_name, nvmlErrorString(ret));
      return false;
    }
    for (unsigned int cnt = 0; cnt < info_count; ++cnt) {
      if (infos[cnt].pid != 0U) {
        running_pids.insert(infos[cnt].pid);
      }
    }
    return true;
  };

  if (!collect_running_pids(
        nvmlDeviceGetComputeRunningProcesses, "nvmlDeviceGetComputeRunningProcesses")) {
    return;
  }
  if (!collect_running_pids(
        nvmlDeviceGetGraphicsRunningProcesses, "nvmlDeviceGetGraphicsRunningProcesses")) {
    return;
  }

  if (running_pids.empty()) {
    return;
  }

  // Query sample buffer size since gpu.last_process_timestamp
  unsigned int util_count = 0;
  ret =
    nvmlDeviceGetProcessUtilization(gpu.device, nullptr, &util_count, gpu.last_process_timestamp);

  std::vector<ProcessUsageSummary> summaries;
  if (ret == NVML_ERROR_NOT_FOUND || (ret == NVML_SUCCESS && util_count == 0U)) {
    // No non-zero utilization samples since last query; report 0% for active GPU processes.
    summaries = aggregateProcessUtilization(running_pids, nullptr, 0U, gpu.last_process_timestamp);
  } else if (ret == NVML_ERROR_INSUFFICIENT_SIZE && util_count > 0U) {
    auto utils = std::make_unique<nvmlProcessUtilizationSample_t[]>(util_count);
    ret = nvmlDeviceGetProcessUtilization(
      gpu.device, utils.get(), &util_count, gpu.last_process_timestamp);
    if (ret == NVML_SUCCESS) {
      summaries = aggregateProcessUtilization(
        running_pids, utils.get(), util_count, gpu.last_process_timestamp);
    } else if (ret == NVML_ERROR_NOT_FOUND) {
      summaries =
        aggregateProcessUtilization(running_pids, nullptr, 0U, gpu.last_process_timestamp);
    } else {
      RCLCPP_WARN(
        this->get_logger(), "Failed to nvmlDeviceGetProcessUtilization(2nd) NVML: %s",
        nvmlErrorString(ret));
      return;
    }
  } else {
    RCLCPP_WARN(
      this->get_logger(), "Failed to nvmlDeviceGetProcessUtilization(1st) NVML: %s",
      nvmlErrorString(ret));
    return;
  }

  int add_cnt = 0;
  for (const auto & proc : summaries) {
    char name[MAX_NAME_LENGTH + 1] = {};
    nvmlSystemGetProcessName(proc.pid, name, MAX_NAME_LENGTH);
    stat.add(fmt::format("GPU {0}: process {1}: pid", index, add_cnt), proc.pid);
    stat.add(fmt::format("GPU {0}: process {1}: name", index, add_cnt), name);
    stat.addf(fmt::format("GPU {0}: process {1}: usage", index, add_cnt), "%u.0%%", proc.sm_util);
    ++add_cnt;
  }
}

void GPUMonitor::checkMemoryUsage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  int whole_level = DiagStatus::OK;
  int index = 0;
  nvmlReturn_t ret{};

  if (gpus_.empty()) {
    stat.summary(DiagStatus::ERROR, "gpu not found");
    return;
  }

  for (auto itr = gpus_.begin(); itr != gpus_.end(); ++itr, ++index) {
    nvmlMemory_t memory;
    ret = nvmlDeviceGetMemoryInfo(itr->device, &memory);
    if (ret != NVML_SUCCESS) {
      stat.summary(
        DiagStatus::ERROR, "Failed to retrieve the amount of used, free and total memory");
      stat.add(fmt::format("GPU {}: name", index), itr->name);
      stat.add(fmt::format("GPU {}: bus-id", index), itr->pci.busId);
      stat.add(fmt::format("GPU {}: content", index), nvmlErrorString(ret));
      return;
    }
    if (memory.total == 0) {
      stat.summary(DiagStatus::ERROR, "Total memory is 0");
      stat.add(fmt::format("GPU {}: name", index), itr->name);
      stat.add(fmt::format("GPU {}: bus-id", index), itr->pci.busId);
      return;
    }

    int level = DiagStatus::OK;
    const auto used_memory = static_cast<double>(memory.used);
    const auto total_memory = static_cast<double>(memory.total);
    const auto usage = static_cast<float>(used_memory / total_memory);
    if (usage >= memory_usage_error_) {
      level = std::max(level, static_cast<int>(DiagStatus::ERROR));
    } else if (usage >= memory_usage_warn_) {
      level = std::max(level, static_cast<int>(DiagStatus::WARN));
    }

    stat.add(fmt::format("GPU {}: status", index), load_dict_.at(level));
    stat.add(fmt::format("GPU {}: name", index), itr->name);
    stat.addf(fmt::format("GPU {}: usage", index), "%.1f%%", usage * 100.0f);
    stat.add(fmt::format("GPU {}: total", index), toHumanReadable(memory.total));
    stat.add(fmt::format("GPU {}: used", index), toHumanReadable(memory.used));
    stat.add(fmt::format("GPU {}: free", index), toHumanReadable(memory.free));

    whole_level = std::max(whole_level, level);
  }

  stat.summary(whole_level, load_dict_.at(whole_level));

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

void GPUMonitor::checkThrottling(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  int level = DiagStatus::OK;
  int whole_level = DiagStatus::OK;
  int index = 0;
  nvmlReturn_t ret{};
  std::vector<std::string> reasons;

  if (gpus_.empty()) {
    stat.summary(DiagStatus::ERROR, "gpu not found");
    return;
  }

  for (auto itr = gpus_.begin(); itr != gpus_.end(); ++itr, ++index) {
    unsigned int clock = 0;
    ret = nvmlDeviceGetClockInfo(itr->device, NVML_CLOCK_GRAPHICS, &clock);
    if (ret != NVML_SUCCESS) {
      stat.summary(DiagStatus::ERROR, "Failed to retrieve the current clock speeds");
      stat.add(fmt::format("GPU {}: name", index), itr->name);
      stat.add(fmt::format("GPU {}: bus-id", index), itr->pci.busId);
      stat.add(fmt::format("GPU {}: content", index), nvmlErrorString(ret));
      return;
    }

    unsigned long long clocksEventReasons = 0LL;  // NOLINT
    ret = getClocksEventReasons(itr->device, &clocksEventReasons);
    if (ret != NVML_SUCCESS) {
      stat.summary(DiagStatus::ERROR, "Failed to retrieve current clock event reasons");
      stat.add(fmt::format("GPU {}: name", index), itr->name);
      stat.add(fmt::format("GPU {}: bus-id", index), itr->pci.busId);
      stat.add(fmt::format("GPU {}: content", index), nvmlErrorString(ret));
      return;
    }

    while (clocksEventReasons) {
      unsigned long long flag = clocksEventReasons & ((~clocksEventReasons) + 1);  // NOLINT
      clocksEventReasons ^= flag;
      reasons.emplace_back(clockEventReasonToString(flag));

      if (!isIgnoredClockEventReason(flag)) {
        level = DiagStatus::ERROR;
      }
    }

    stat.add(fmt::format("GPU {}: status", index), throttling_dict_.at(level));
    stat.add(fmt::format("GPU {}: name", index), itr->name);
    stat.addf(fmt::format("GPU {}: graphics clock", index), "%d MHz", clock);

    if (reasons.empty()) {
      reasons.emplace_back("ReasonNone");
    }

    stat.add(fmt::format("GPU {}: reasons", index), boost::algorithm::join(reasons, ", "));

    whole_level = std::max(whole_level, level);
  }

  stat.summary(whole_level, throttling_dict_.at(whole_level));

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

std::string GPUMonitor::toHumanReadable(unsigned long long size)  // NOLINT
{
  const char * units[] = {"B", "K", "M", "G", "T"};
  int count = 0;
  double dsize = size;

  while (dsize > 1024) {
    dsize /= 1024;
    ++count;
  }
  const char * format = (dsize > 0 && dsize < 10) ? "{:.1f}{}" : "{:.0f}{}";
  return fmt::format(format, dsize, units[count]);
}

void GPUMonitor::checkFrequency(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  int whole_level = DiagStatus::OK;
  int index = 0;
  nvmlReturn_t ret{};

  if (gpus_.empty()) {
    stat.summary(DiagStatus::ERROR, "gpu not found");
    return;
  }

  for (auto itr = gpus_.begin(); itr != gpus_.end(); ++itr, ++index) {
    int level = DiagStatus::OK;
    unsigned int clock = 0;
    ret = nvmlDeviceGetClockInfo(itr->device, NVML_CLOCK_GRAPHICS, &clock);
    if (ret != NVML_SUCCESS) {
      stat.summary(DiagStatus::ERROR, "Failed to retrieve the current clock speeds");
      stat.add(fmt::format("GPU {}: name", index), itr->name);
      stat.add(fmt::format("GPU {}: bus-id", index), itr->pci.busId);
      stat.add(fmt::format("GPU {}: content", index), nvmlErrorString(ret));
      return;
    }

    if (itr->supported_gpu_clocks.find(clock) == itr->supported_gpu_clocks.end()) {
      level = DiagStatus::WARN;
    }

    stat.add(fmt::format("GPU {}: status", index), frequency_dict_.at(level));
    stat.add(fmt::format("GPU {}: name", index), itr->name);
    stat.addf(fmt::format("GPU {}: graphics clock", index), "%d MHz", clock);

    whole_level = std::max(whole_level, level);
  }

  stat.summary(whole_level, frequency_dict_.at(whole_level));

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

std::vector<GPUMonitorBase::GpuStatus> GPUMonitor::getGPUStatus() const
{
  std::vector<GpuStatus> gpu_status_list;

  int index = 0;
  for (auto itr = gpus_.begin(); itr != gpus_.end(); ++itr, ++index) {
    nvmlReturn_t ret{};
    nvmlUtilization_t utilization;
    ret = nvmlDeviceGetUtilizationRates(itr->device, &utilization);
    if (ret != NVML_SUCCESS) {
      continue;
    }

    unsigned int clock = 0;
    ret = nvmlDeviceGetClockInfo(itr->device, NVML_CLOCK_GRAPHICS, &clock);
    if (ret != NVML_SUCCESS) {
      continue;
    }

    int temp = 0;
    ret = getGpuTemperature(itr->device, &temp);
    if (ret != NVML_SUCCESS) {
      continue;
    }

    unsigned long long clocksEventReasons = 0LL;  // NOLINT
    ret = getClocksEventReasons(itr->device, &clocksEventReasons);
    if (ret != NVML_SUCCESS) {
      continue;
    }

    int thermal_throttling = DiagStatus::OK;
    while (clocksEventReasons) {
      unsigned long long flag = clocksEventReasons & ((~clocksEventReasons) + 1);  // NOLINT
      clocksEventReasons ^= flag;

      if (!isIgnoredClockEventReason(flag)) {
        thermal_throttling = DiagStatus::ERROR;
      }
    }

    GpuStatus gpu_status;
    gpu_status.name = itr->name;
    gpu_status.usage = static_cast<float>(utilization.gpu);
    gpu_status.clock = static_cast<int>(clock);
    gpu_status.temperature = temp;
    gpu_status.thermal_throttling = thermal_throttling;
    gpu_status_list.push_back(gpu_status);
  }

  return gpu_status_list;
}

bool GPUMonitor::getSupportedGPUClocks(
  int index, nvmlDevice_t & device, std::set<unsigned int> & supported_gpu_clocks)
{
  unsigned int mem_clock_count = 0;
  nvmlReturn_t ret{};

  ret = nvmlDeviceGetSupportedMemoryClocks(device, &mem_clock_count, nullptr);
  if (ret != NVML_ERROR_INSUFFICIENT_SIZE) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to retrieve the count of possible memory clocks [%d]: %s", index,
      nvmlErrorString(ret));
    return false;
  }

  std::shared_ptr<unsigned int[]> mem_clocks(new unsigned int[mem_clock_count]);
  ret = nvmlDeviceGetSupportedMemoryClocks(device, &mem_clock_count, mem_clocks.get());
  if (ret != NVML_SUCCESS) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to retrieve the list of possible memory clocks [%d]: %s", index,
      nvmlErrorString(ret));
    return false;
  }

  for (unsigned int mem_clock_index = 0; mem_clock_index < mem_clock_count; mem_clock_index++) {
    unsigned int gpu_clock_count = 0;

    ret = nvmlDeviceGetSupportedGraphicsClocks(
      device, mem_clocks[mem_clock_index], &gpu_clock_count, nullptr);
    if (ret != NVML_ERROR_INSUFFICIENT_SIZE) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to retrieve the count of possible graphics clocks for %d MHz memory clock [%d]: %s",
        mem_clocks[mem_clock_index], index, nvmlErrorString(ret));
      return false;
    }

    std::shared_ptr<unsigned int[]> gpu_clocks(new unsigned int[gpu_clock_count]);
    ret = nvmlDeviceGetSupportedGraphicsClocks(
      device, mem_clocks[mem_clock_index], &gpu_clock_count, gpu_clocks.get());
    if (ret != NVML_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to retrieve the list of possible graphics clocks for %d MHz memory clock [%d]: %s",
        mem_clocks[mem_clock_index], index, nvmlErrorString(ret));
      return false;
    }
    for (unsigned int gpu_clock_index = 0; gpu_clock_index < gpu_clock_count; gpu_clock_index++) {
      supported_gpu_clocks.insert(gpu_clocks[gpu_clock_index]);
    }
  }
  return true;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(GPUMonitor)
