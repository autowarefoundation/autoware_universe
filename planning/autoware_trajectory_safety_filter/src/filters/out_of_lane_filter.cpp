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

#include "autoware/trajectory_safety_filter/filters/out_of_lane_filter.hpp"

#include <memory>
#include <vector>

namespace autoware::trajectory_safety_filter::plugin
{

bool OutOfLaneFilter::filter_trajectory(TrajectoryPoints & points)
{
  if (points.size() < 2) return false;

  const auto is_finite = std::all_of(
    points.begin(), points.end(), [this](const auto & point) { return check_finite(point); });

  return is_finite;
}

void OutOfLaneFilter::set_up_params()
{
}

rcl_interfaces::msg::SetParametersResult OutOfLaneFilter::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  (void)parameters;  // Suppress unused parameter warning
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

}  // namespace autoware::trajectory_safety_filter::plugin
