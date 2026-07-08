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
#ifndef AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_OPTIMIZER_PLUGIN_BASE_HPP_
// NOLINTNEXTLINE
#define AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_OPTIMIZER_PLUGIN_BASE_HPP_
#include "autoware/trajectory_processor/plugin_base.hpp"
#include "autoware/trajectory_processor/trajectory_optimizer_structs.hpp"

#include <autoware_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_optimizer::plugin
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

class TrajectoryOptimizerPluginBase : public autoware::trajectory_processor::plugin::PluginBase
{
public:
  TrajectoryOptimizerPluginBase() = default;

  virtual ~TrajectoryOptimizerPluginBase() = default;

  // Main optimization function
  // params: Contains activation flags and shared configuration
  // data: Contains runtime vehicle state (odometry, acceleration)
  virtual void optimize_trajectory(
    TrajectoryPoints & traj_points, const TrajectoryOptimizerParams & params,
    TrajectoryOptimizerData & data) = 0;

  bool modify_trajectory(
    autoware::trajectory_processor::plugin::TrajectoryPoints & traj_points,
    const autoware::trajectory_processor::plugin::InputData & input) override
  {
    if (!input.current_odometry || !input.current_acceleration) {
      return false;
    }

    TrajectoryOptimizerData data;
    data.current_odometry = *input.current_odometry;
    data.current_acceleration = *input.current_acceleration;
    if (input.semantic_speed_tracker) {
      data.semantic_speed_tracker = *input.semantic_speed_tracker;
    }

    optimize_trajectory(traj_points, params_, data);

    if (input.semantic_speed_tracker) {
      *input.semantic_speed_tracker = data.semantic_speed_tracker;
    }
    return true;
  }

  // Plugin parameter setup - plugins declare their own parameters here
  virtual void set_up_params() = 0;

  // Plugin parameter update callback - plugins update their own parameters here
  virtual rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters) = 0;

  // Initialize plugin with node context (for pluginlib-loaded plugins)
  virtual void initialize(
    const std::string & name, rclcpp::Node * node_ptr,
    const std::shared_ptr<autoware_utils_debug::TimeKeeper> & time_keeper)
  {
    auto context = std::make_shared<autoware::trajectory_processor::plugin::NodeContext>();
    context->node_ptr = node_ptr;
    context->time_keeper = time_keeper;
    autoware::trajectory_processor::plugin::PluginBase::initialize(name, context);
    set_up_params();
    std::cerr << "initialized TrajectoryOptimizerPlugin: " << get_name() << std::endl;
  }

  void initialize(
    const std::string & name,
    std::shared_ptr<autoware::trajectory_processor::plugin::NodeContext> context) override
  {
    autoware::trajectory_processor::plugin::PluginBase::initialize(name, std::move(context));
    set_up_params();
    std::cerr << "initialized TrajectoryOptimizerPlugin: " << get_name() << std::endl;
  }

  void update_params(const TrajectoryOptimizerParams & params) override { params_ = params; }

protected:
  using autoware::trajectory_processor::plugin::PluginBase::get_node_ptr;
  using autoware::trajectory_processor::plugin::PluginBase::get_time_keeper;

private:
  TrajectoryOptimizerParams params_;
};
}  // namespace autoware::trajectory_optimizer::plugin

// NOLINTNEXTLINE
#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_OPTIMIZER_PLUGIN_BASE_HPP_
