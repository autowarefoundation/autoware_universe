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

#ifndef AUTOWARE__TRAJECTORY_SAFETY_FILTER__FILTERS__OUT_OF_LANE_FILTER_HPP_
#define AUTOWARE__TRAJECTORY_SAFETY_FILTER__FILTERS__OUT_OF_LANE_FILTER_HPP_

#include "autoware/trajectory_safety_filter/safety_filter_interface.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::trajectory_safety_filter::plugin
{

class OutOfLaneFilter : public SafetyFilterInterface
{
public:
  OutOfLaneFilter() : SafetyFilterInterface("OutOfLaneFilter") {}

  bool filter_trajectory(TrajectoryPoints & traj_points) override;
  void set_up_params() override;
  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters) override;
};

}  // namespace autoware::trajectory_safety_filter::plugin

#endif  // AUTOWARE__TRAJECTORY_SAFETY_FILTER__FILTERS__OUT_OF_LANE_FILTER_HPP_
