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
#include <autoware_utils/ros/diagnostics_interface.hpp>

#include <string>
namespace autoware::pointcloud_preprocessor
{

class PassRateDiagnostics : public DiagnosticsBase
{
public:
  PassRateDiagnostics(int input_point_count, int output_point_count)
  : input_point_count_(input_point_count),
    output_point_count_(output_point_count),
    pass_rate_(
      input_point_count > 0 ? static_cast<double>(output_point_count) / input_point_count : 0.0)
  {
  }

  void add_to_interface(autoware_utils::DiagnosticsInterface & interface) const override
  {
    interface.add_key_value("Input point count", input_point_count_);
    interface.add_key_value("Output point count", output_point_count_);
    interface.add_key_value("Pass rate", pass_rate_);
  }

  [[nodiscard]] std::optional<std::pair<int, std::string>> evaluate_status() const override
  {
    if (output_point_count_ == 0) {
      return std::make_pair(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR, "[ERROR]: No valid output points");
    }
    return std::nullopt;
  }

private:
  int input_point_count_;
  int output_point_count_;
  double pass_rate_;
};

}  // namespace autoware::pointcloud_preprocessor
