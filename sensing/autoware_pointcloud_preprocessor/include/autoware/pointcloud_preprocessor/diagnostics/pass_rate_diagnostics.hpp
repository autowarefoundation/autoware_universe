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
namespace autoware::pointcloud_preprocessor
{

class PassRateDiagnostics : public DiagnosticsBase
{
public:
  PassRateDiagnostics(int input_point_count, int output_point_count)
  : input_point_count(input_point_count),
    output_point_count(output_point_count),
    pass_rate(
      input_point_count > 0 ? static_cast<double>(output_point_count) / input_point_count : 0.0)
  {
  }

  void add_to_interface(autoware_utils::DiagnosticsInterface & interface) const override
  {
    interface.add_key_value("input_point_count", input_point_count);
    interface.add_key_value("output_point_count", output_point_count);
    interface.add_key_value("pass_rate", pass_rate);
  }

  int input_point_count;
  int output_point_count;
  double pass_rate;
};

}  // namespace autoware::pointcloud_preprocessor
