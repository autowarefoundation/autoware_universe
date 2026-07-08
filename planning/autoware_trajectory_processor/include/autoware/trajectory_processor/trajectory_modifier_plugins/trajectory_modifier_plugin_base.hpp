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
#ifndef AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_MODIFIER_PLUGINS__TRAJECTORY_MODIFIER_PLUGIN_BASE_HPP_
// NOLINTNEXTLINE
#define AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_MODIFIER_PLUGINS__TRAJECTORY_MODIFIER_PLUGIN_BASE_HPP_
#include "autoware/trajectory_processor/plugin_base.hpp"
#include "autoware/trajectory_processor/trajectory_modifier_context.hpp"
#include "autoware/trajectory_processor/trajectory_modifier_plugins/input_data.hpp"

#include <autoware_trajectory_processor/trajectory_modifier_param.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_modifier::plugin
{
using autoware_internal_planning_msgs::msg::PlanningFactor;
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using TrajectoryModifierParams = trajectory_modifier_params::Params;
using NodeContext = autoware::trajectory_processor::plugin::NodeContext;

class TrajectoryModifierPluginBase : public autoware::trajectory_processor::plugin::PluginBase
{
public:
  TrajectoryModifierPluginBase() = default;

  void initialize(
    std::string name, rclcpp::Node * node_ptr,
    const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
    const std::shared_ptr<TrajectoryModifierContext> & context,
    [[maybe_unused]] const TrajectoryModifierParams & params)
  {
    auto node_context = std::make_shared<NodeContext>();
    node_context->node_ptr = node_ptr;
    node_context->time_keeper = time_keeper;
    node_context->vehicle_info = context->vehicle_info;
    node_context->tf_buffer = &context->tf_buffer;
    autoware::trajectory_processor::plugin::PluginBase::initialize(name, node_context);
    modifier_context_ = context;
    RCLCPP_DEBUG(
      node_ptr->get_logger(), "instantiated TrajectoryModifierPluginBase: %s", name.c_str());
    on_initialize(params);
  }

  virtual ~TrajectoryModifierPluginBase() = default;
  using autoware::trajectory_processor::plugin::PluginBase::modify_trajectory;
  virtual void update_params(const TrajectoryModifierParams & params) = 0;

protected:
  virtual void on_initialize(const TrajectoryModifierParams & params) = 0;
  std::shared_ptr<TrajectoryModifierContext> modifier_context_;
  bool enabled_{true};
  double trajectory_time_step_{0.1};
};
}  // namespace autoware::trajectory_modifier::plugin

// NOLINTNEXTLINE
#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_MODIFIER_PLUGINS__TRAJECTORY_MODIFIER_PLUGIN_BASE_HPP_
