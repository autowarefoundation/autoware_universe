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

// NOLINTNEXTLINE
#ifndef AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_OPTIMIZER_PLUGIN_BASE_HPP_
// NOLINTNEXTLINE
#define AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_OPTIMIZER_PLUGIN_BASE_HPP_
#include "autoware/trajectory_optimizer/trajectory_optimizer_structs.hpp"

#include <autoware_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_optimizer::plugin
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

class TrajectoryOptimizerPluginBase
{
public:
  TrajectoryOptimizerPluginBase(
    const std::string name, rclcpp::Node * node_ptr,
    const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
    [[maybe_unused]] const TrajectoryOptimizerParams & params)
  : name_(name), node_ptr_(node_ptr), time_keeper_(time_keeper)
  {
    std::cerr << "instantiated TrajectoryOptimizerPluginBase: " << name_ << std::endl;
  }
  virtual ~TrajectoryOptimizerPluginBase() = default;

  // Main optimization function
  // params: Contains activation flags and shared configuration
  // data: Contains runtime vehicle state (odometry, acceleration)
  virtual void optimize_trajectory(
    TrajectoryPoints & traj_points, const TrajectoryOptimizerParams & params,
    const TrajectoryOptimizerData & data) = 0;

  // Plugin parameter setup - plugins declare their own parameters here
  virtual void set_up_params() = 0;

  // Plugin parameter update callback - plugins update their own parameters here
  virtual rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters) = 0;
  std::string get_name() const { return name_; }
  rclcpp::Node * get_node_ptr() const { return node_ptr_; }
  std::shared_ptr<autoware_utils_debug::TimeKeeper> get_time_keeper() const { return time_keeper_; }

private:
  std::string name_;
  rclcpp::Node * node_ptr_;
  mutable std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_{nullptr};
};
}  // namespace autoware::trajectory_optimizer::plugin

// NOLINTNEXTLINE
#endif  // AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_OPTIMIZER_PLUGIN_BASE_HPP_
