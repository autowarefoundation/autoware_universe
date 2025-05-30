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

namespace autoware::pointcloud_preprocessor
{

class LatencyDiagnostics : public DiagnosticsBase
{
public:
  LatencyDiagnostics(
    const rclcpp::Time & cloud_header_timestamp, double processing_time_ms,
    double pipeline_latency_ms)
  : cloud_header_timestamp(cloud_header_timestamp),
    processing_time_ms(processing_time_ms),
    pipeline_latency_ms(pipeline_latency_ms)
  {
  }

  void add_to_interface(autoware_utils::DiagnosticsInterface & interface) const override
  {
    interface.add_key_value(
      "cloud_header_timestamp", format_timestamp(cloud_header_timestamp.seconds()));
    interface.add_key_value("processing_time_ms", processing_time_ms);
    interface.add_key_value("pipeline_latency_ms", pipeline_latency_ms);
  }

  rclcpp::Time cloud_header_timestamp;
  double processing_time_ms;
  double pipeline_latency_ms;
};

}  // namespace autoware::pointcloud_preprocessor
