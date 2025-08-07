// Copyright 2025 TIER IV, Inc.
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

#pragma once

#include <autoware/pointcloud_preprocessor/diagnostics/diagnostics_base.hpp>
#include <autoware/pointcloud_preprocessor/diagnostics/format_utils.hpp>
#include <autoware_utils/ros/diagnostics_interface.hpp>
#include <rclcpp/time.hpp>

#include <string>

namespace autoware::pointcloud_preprocessor
{

class LatencyDiagnostics : public DiagnosticsBase
{
public:
  LatencyDiagnostics(
    const rclcpp::Time & cloud_header_timestamp, double processing_time_ms,
    double pipeline_latency_ms, double processing_time_threshold_ms)
  : cloud_header_timestamp_(cloud_header_timestamp),
    processing_time_ms_(processing_time_ms),
    pipeline_latency_ms_(pipeline_latency_ms),
    processing_time_threshold_ms_(processing_time_threshold_ms)
  {
  }

  void add_to_interface(autoware_utils::DiagnosticsInterface & interface) const override
  {
    interface.add_key_value(
      "Pointcloud header timestamp", format_timestamp(cloud_header_timestamp_.seconds()));
    interface.add_key_value("Processing time (ms)", processing_time_ms_);
    interface.add_key_value("Pipeline latency (ms)", pipeline_latency_ms_);
  }

  [[nodiscard]] std::optional<std::pair<int, std::string>> evaluate_status() const override
  {
    if (processing_time_ms_ > processing_time_threshold_ms_) {
      return std::make_pair(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "[WARN]: Processing time " + std::to_string(processing_time_ms_) +
          " ms exceeded threshold of " + std::to_string(processing_time_threshold_ms_) + " ms");
    }
    return std::nullopt;
  }

private:
  rclcpp::Time cloud_header_timestamp_;
  double processing_time_ms_;
  double pipeline_latency_ms_;
  double processing_time_threshold_ms_;
};

}  // namespace autoware::pointcloud_preprocessor
