// Copyright 2020 Tier IV, Inc.
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
 * @file nvml_gpu_monitor.h
 * @brief NVML GPU monitor class
 */

#ifndef SYSTEM_MONITOR__GPU_MONITOR__NVML_GPU_MONITOR_HPP_
#define SYSTEM_MONITOR__GPU_MONITOR__NVML_GPU_MONITOR_HPP_

#include "system_monitor/gpu_monitor/gpu_monitor_base.hpp"

#include <nvml.h>

#include <map>
#include <set>
#include <string>
#include <vector>

/**
 * @brief GPU information
 */
struct gpu_info
{
  nvmlDevice_t device;                          //!< @brief handle for a particular device
  char name[NVML_DEVICE_NAME_BUFFER_SIZE];      //!< @brief name of device
  nvmlPciInfo_t pci;                            //!< @brief PCI information about a GPU device
  nvmlUtilization_t utilization;                //!< @brief Utilization information for a device
  std::set<unsigned int> supported_gpu_clocks;  //!< @brief list of supported GPU clocks
  unsigned long long last_process_timestamp =   // NOLINT(runtime/int)
    0;  //!< @brief latest sample timestamp[usec] for nvmlDeviceGetProcessUtilization()
};

class GPUMonitor : public GPUMonitorBase
{
public:
  /**
   * @brief constructor
   * @param [in] node_name Name of the node.
   * @param [in] options Options associated with this node.
   */
  explicit GPUMonitor(const rclcpp::NodeOptions & options);

  /**
   * @brief destructor
   */
  ~GPUMonitor() override;

protected:
  /**
   * @brief Terminate the node, log final statements. An independent function is preferred to allow
   * an explicit way to operate actions that require a valid rclcpp context. By default this method
   * does nothing.
   * @note This class uses the NVML library to get GPU information. The NVML library needs to be
   * shut down in this function.
   */
  void shut_down() override;

  /**
   * @brief check GPU temperature
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkTemp(
    diagnostic_updater::DiagnosticStatusWrapper & stat) override;  // NOLINT(runtime/references)

  /**
   * @brief check GPU usage
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkUsage(
    diagnostic_updater::DiagnosticStatusWrapper & stat) override;  // NOLINT(runtime/references)

  struct ProcessUsageSummary
  {
    unsigned int pid;
    unsigned int sm_util;
  };

  /**
   * @brief aggregate per-process SM utilization samples and update last seen timestamp
   * @param [in] running_pids active compute/graphics PIDs on the GPU
   * @param [in] samples utilization samples returned by NVML (may be nullptr when sample_count is
   * 0)
   * @param [in] sample_count number of entries in samples
   * @param [in,out] last_timestamp last seen sample timestamp in microseconds
   * @return aggregated SM utilization per active PID
   */
  static std::vector<ProcessUsageSummary> aggregateProcessUtilization(
    const std::set<unsigned int> & running_pids, const nvmlProcessUtilizationSample_t * samples,
    unsigned int sample_count,
    unsigned long long & last_timestamp);  // NOLINT(runtime/int)

  /**
   * @brief add stat of GPU usage per process
   * @param [in] index GPU index
   * @param [in,out] gpu GPU device info including per-device last sample timestamp
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   */
  void addProcessUsage(
    int index, gpu_info & gpu, diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief check GPU memory usage
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkMemoryUsage(
    diagnostic_updater::DiagnosticStatusWrapper & stat) override;  // NOLINT(runtime/references)

  /**
   * @brief check GPU throttling
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkThrottling(
    diagnostic_updater::DiagnosticStatusWrapper & stat) override;  // NOLINT(runtime/references)

  /**
   * @brief get human-readable output for memory size
   * @param [in] size size with bytes
   * @return human-readable output
   * @note NOLINT syntax is needed since struct nvmlMemory_t has unsigned long long values to return
   * memory size.
   */
  std::string toHumanReadable(unsigned long long size);  // NOLINT(runtime/int)

  /**
   * @brief check GPU frequency
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkFrequency(
    diagnostic_updater::DiagnosticStatusWrapper & stat) override;  // NOLINT(runtime/references)

  /**
   * @brief get GPU status
   * @return GPU status list
   */
  std::vector<GpuStatus> getGPUStatus() const override;

  /**
   * @brief get supported GPU clocks
   * @param [in] index GPU index
   * @param [in] device GPU device
   * @param [out] list of supported GPU clocks
   * @return result of getting supported GPU clocks
   */
  bool getSupportedGPUClocks(
    int index, nvmlDevice_t & device, std::set<unsigned int> & supported_gpu_clocks);

  static const size_t MAX_ARRAY_SIZE = 64;
  static const size_t MAX_NAME_LENGTH = 128;

  std::vector<gpu_info> gpus_;  //!< @brief list of gpus

  /**
   * @brief GPU frequency status messages
   */
  const std::map<int, const char *> frequency_dict_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "unsupported clock"}};
};

#endif  // SYSTEM_MONITOR__GPU_MONITOR__NVML_GPU_MONITOR_HPP_
