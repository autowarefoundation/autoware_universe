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

#ifndef AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__STOP_POINT_FIXER_HPP_
#define AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__STOP_POINT_FIXER_HPP_

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/trajectory_modifier_plugin_base.hpp"

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_modifier::plugin
{

class StopPointFixer : public TrajectoryModifierPluginBase
{
public:
  StopPointFixer(
    const std::string & name, rclcpp::Node * node_ptr,
    const std::shared_ptr<autoware_utils_debug::TimeKeeper> & time_keeper,
    const TrajectoryModifierParams & params);

  void modify_trajectory(
    TrajectoryPoints & traj_points, const TrajectoryModifierParams & params,
    const TrajectoryModifierData & data) override;
  void set_up_params() override;
  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters) override;
  bool is_trajectory_modification_required(
    const TrajectoryPoints & traj_points, const TrajectoryModifierParams & params,
    const TrajectoryModifierData & data) const override;

private:
  struct Parameters
  {
    double velocity_threshold_mps{0.1};
    double min_distance_threshold_m{1.0};
  };

  Parameters params_;
};

}  // namespace autoware::trajectory_modifier::plugin

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__STOP_POINT_FIXER_HPP_
