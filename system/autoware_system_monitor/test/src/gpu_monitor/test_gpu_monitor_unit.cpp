// Copyright 2026 Tier IV, Inc.
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

#include "system_monitor/gpu_monitor/gpu_monitor_base.hpp"

#ifdef _GPU_NVML_
#include "system_monitor/gpu_monitor/nvml_gpu_monitor.hpp"
#endif

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <climits>
#include <cstddef>
#include <set>
#include <string>
#include <vector>

namespace
{
using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

class TestableGPUMonitorBase : public GPUMonitorBase
{
public:
  explicit TestableGPUMonitorBase(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : GPUMonitorBase("test_gpu_monitor_base", options)
  {
  }

  int evaluateUsage(size_t gpu_index, float usage) { return checkGpuUsageLevel(gpu_index, usage); }
};

#ifdef _GPU_NVML_
class NVMLProcessUtilAccessor : public GPUMonitor
{
public:
  using GPUMonitor::aggregateProcessUtilization;
  using GPUMonitor::ProcessUsageSummary;
};
#endif

class GPUMonitorUnitTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(GPUMonitorUnitTest, DefaultThresholdsAndConsecutiveCounts)
{
  TestableGPUMonitorBase monitor;

  // Below warning threshold (0.90) -> OK
  EXPECT_EQ(monitor.evaluateUsage(0, 0.70f), DiagStatus::OK);
  EXPECT_EQ(monitor.evaluateUsage(0, 0.89f), DiagStatus::OK);

  // Reaching warning threshold (default gpu_usage_warn_count = 1) -> WARN immediately
  EXPECT_EQ(monitor.evaluateUsage(0, 0.90f), DiagStatus::WARN);

  // Single-sample spike to 100% (default gpu_usage_error_count = 2) -> stays WARN, not ERROR
  EXPECT_EQ(monitor.evaluateUsage(0, 1.00f), DiagStatus::WARN);

  // Dropping back below thresholds resets consecutive counters -> OK
  EXPECT_EQ(monitor.evaluateUsage(0, 0.70f), DiagStatus::OK);

  // Another single-sample 100% spike after recovery -> still WARN
  EXPECT_EQ(monitor.evaluateUsage(0, 1.00f), DiagStatus::WARN);

  // Second consecutive sample at 100% -> escalates to ERROR
  EXPECT_EQ(monitor.evaluateUsage(0, 1.00f), DiagStatus::ERROR);

  // Recovery clears both warn and error counters
  EXPECT_EQ(monitor.evaluateUsage(0, 0.65f), DiagStatus::OK);
}

TEST_F(GPUMonitorUnitTest, CustomWarnAndErrorCounts)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("gpu_usage_warn", 0.85);
  options.append_parameter_override("gpu_usage_error", 0.95);
  options.append_parameter_override("gpu_usage_warn_count", 2);
  options.append_parameter_override("gpu_usage_error_count", 3);

  TestableGPUMonitorBase monitor(options);

  // 1st sample >= 0.85 stays OK because warn_count = 2
  EXPECT_EQ(monitor.evaluateUsage(0, 0.88f), DiagStatus::OK);
  // 2nd consecutive sample >= 0.85 -> WARN
  EXPECT_EQ(monitor.evaluateUsage(0, 0.88f), DiagStatus::WARN);

  // 1st & 2nd samples >= 0.95 stay WARN because error_count = 3
  EXPECT_EQ(monitor.evaluateUsage(0, 0.98f), DiagStatus::WARN);
  EXPECT_EQ(monitor.evaluateUsage(0, 0.99f), DiagStatus::WARN);
  // 3rd consecutive sample >= 0.95 -> ERROR
  EXPECT_EQ(monitor.evaluateUsage(0, 1.00f), DiagStatus::ERROR);
}

TEST_F(GPUMonitorUnitTest, IndependentPerGpuCounters)
{
  TestableGPUMonitorBase monitor;

  // Alternating 100% spikes on GPU 0 and GPU 1 must not trigger ERROR on either GPU
  EXPECT_EQ(monitor.evaluateUsage(0, 1.00f), DiagStatus::WARN);
  EXPECT_EQ(monitor.evaluateUsage(1, 0.50f), DiagStatus::OK);

  EXPECT_EQ(monitor.evaluateUsage(0, 0.50f), DiagStatus::OK);
  EXPECT_EQ(monitor.evaluateUsage(1, 1.00f), DiagStatus::WARN);

  // GPU 1 hits 100% for a second consecutive check -> GPU 1 becomes ERROR while GPU 0 stays OK
  EXPECT_EQ(monitor.evaluateUsage(0, 0.50f), DiagStatus::OK);
  EXPECT_EQ(monitor.evaluateUsage(1, 1.00f), DiagStatus::ERROR);
}

#ifdef _GPU_NVML_
TEST_F(GPUMonitorUnitTest, AggregateProcessUtilizationAveragesPerProcess)
{
  const std::set<unsigned int> running_pids = {8445U, 8754U, 9999U};
  const nvmlProcessUtilizationSample_t samples[] = {
    {8445U, 1000ULL, 68U, 10U, 0U, 0U},
    {8445U, 1200ULL, 73U, 10U, 0U, 0U},        // avg for 8445 = round((68 + 73) / 2) = 71
    {8754U, 1150ULL, UINT32_MAX, 0U, 0U, 0U},  // UINT32_MAX treated as 0
    {8754U, 1350ULL, 25U, 5U, 0U, 0U},         // avg for 8754 = round((0 + 25) / 2) = 13
    {0U, 1500ULL, 99U, 0U, 0U, 0U},            // illegal PID 0 ignored for stats, timestamp tracked
    {1234U, 1400ULL, 80U, 0U, 0U, 0U},         // non-running PID ignored
  };

  unsigned long long last_ts = 900ULL;  // NOLINT(runtime/int)
  const auto result =
    NVMLProcessUtilAccessor::aggregateProcessUtilization(running_pids, samples, 6U, last_ts);

  EXPECT_EQ(last_ts, 1500ULL);
  ASSERT_EQ(result.size(), 3U);

  EXPECT_EQ(result[0].pid, 8445U);
  EXPECT_EQ(result[0].sm_util, 71U);

  EXPECT_EQ(result[1].pid, 8754U);
  EXPECT_EQ(result[1].sm_util, 13U);

  // Active process with no samples in window is reported with 0%
  EXPECT_EQ(result[2].pid, 9999U);
  EXPECT_EQ(result[2].sm_util, 0U);
}

TEST_F(GPUMonitorUnitTest, AggregateProcessUtilizationEmptySamplesPreservesTimestamp)
{
  const std::set<unsigned int> running_pids = {9105U, 9424U};
  unsigned long long last_ts = 42000ULL;  // NOLINT(runtime/int)

  const auto result =
    NVMLProcessUtilAccessor::aggregateProcessUtilization(running_pids, nullptr, 0U, last_ts);

  EXPECT_EQ(last_ts, 42000ULL);
  ASSERT_EQ(result.size(), 2U);
  EXPECT_EQ(result[0].pid, 9105U);
  EXPECT_EQ(result[0].sm_util, 0U);
  EXPECT_EQ(result[1].pid, 9424U);
  EXPECT_EQ(result[1].sm_util, 0U);
}
#endif

}  // namespace
