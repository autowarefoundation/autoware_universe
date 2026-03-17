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
#ifndef AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__TRAJECTORY_MODIFIER_PLUGIN_BASE_HPP_
// NOLINTNEXTLINE
#define AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__TRAJECTORY_MODIFIER_PLUGIN_BASE_HPP_
#include "autoware/trajectory_modifier/trajectory_modifier_structs.hpp"

#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <autoware_trajectory_modifier/trajectory_modifier_param.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <iostream>
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

class TrajectoryModifierPluginBase
{
public:
  TrajectoryModifierPluginBase() = default;

  void initialize(
    std::string name, rclcpp::Node * node_ptr,
    const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
    const std::shared_ptr<TrajectoryModifierData> & data,
    [[maybe_unused]] const TrajectoryModifierParams & params)
  {
    name_ = std::invoke([&name]() {
      const auto npos = name.find_last_of(':');
      return npos != std::string::npos ? name.substr(npos + 1) : name;
    });
    node_ptr_ = node_ptr;
    time_keeper_ = time_keeper;
    data_ = data;
    RCLCPP_DEBUG(
      node_ptr_->get_logger(), "instantiated TrajectoryModifierPluginBase: %s", name_.c_str());
    on_initialize(params);
  }

  virtual ~TrajectoryModifierPluginBase() = default;
  virtual bool modify_trajectory(TrajectoryPoints & traj_points) = 0;
  virtual bool is_trajectory_modification_required(const TrajectoryPoints & traj_points) = 0;
  std::string get_name() const { return name_; }
  rclcpp::Node * get_node_ptr() const { return node_ptr_; }
  std::shared_ptr<autoware_utils_debug::TimeKeeper> get_time_keeper() const { return time_keeper_; }
  virtual void update_params(const TrajectoryModifierParams & params) = 0;

  virtual void publish_debug_data([[maybe_unused]] const std::string & ns) const {}

  virtual void publish_planning_factor()
  {
    if (planning_factor_interface_) {
      planning_factor_interface_->publish();
    }
  }
  std::vector<PlanningFactor> get_planning_factors() const
  {
    if (planning_factor_interface_ != nullptr) {
      return planning_factor_interface_->get_factors();
    }
    return {};
  }

protected:
  virtual void on_initialize(const TrajectoryModifierParams & params) = 0;
  std::unique_ptr<autoware::planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface_;
  std::shared_ptr<TrajectoryModifierData> data_;
  bool enabled_{true};
  double trajectory_time_step_{0.1};

  rclcpp::Clock::SharedPtr get_clock() const { return node_ptr_->get_clock(); }

private:
  std::string name_;
  rclcpp::Node * node_ptr_;
  mutable std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_{nullptr};
};
}  // namespace autoware::trajectory_modifier::plugin

// NOLINTNEXTLINE
#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__TRAJECTORY_MODIFIER_PLUGIN_BASE_HPP_
