// Copyright 2023 TIER IV, Inc.
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

#include "autoware/behavior_path_side_shift_module/manager.hpp"

#include "autoware/behavior_path_side_shift_module/validation.hpp"
#include "autoware_utils/ros/update_param.hpp"

#include <autoware_common_msgs/msg/response_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{

void SideShiftModuleManager::init(rclcpp::Node * node)
{
  // init manager interface
  initInterface(node, {});

  SideShiftParameters p{};

  const std::string ns = "side_shift.";
  p.min_distance_to_start_shifting =
    node->declare_parameter<double>(ns + "min_distance_to_start_shifting");
  p.time_to_start_shifting = node->declare_parameter<double>(ns + "time_to_start_shifting");
  p.shifting_lateral_jerk = node->declare_parameter<double>(ns + "shifting_lateral_jerk");
  p.min_shifting_distance = node->declare_parameter<double>(ns + "min_shifting_distance");
  p.min_shifting_speed = node->declare_parameter<double>(ns + "min_shifting_speed");
  p.shift_request_time_limit = node->declare_parameter<double>(ns + "shift_request_time_limit");
  p.publish_debug_marker = node->declare_parameter<bool>(ns + "publish_debug_marker");
  p.direction_shift_amount = node->declare_parameter<double>(ns + "direction_shift_amount");

  parameters_ = std::make_shared<SideShiftParameters>(p);
  inserted_lateral_offset_state_ = std::make_shared<InsertedLateralOffsetState>();

  set_lateral_offset_srv_ = node->create_service<autoware_planning_msgs::srv::SetLateralOffset>(
    "~/set_lateral_offset",
    std::bind(&SideShiftModuleManager::onSetLateralOffset, this, std::placeholders::_1, std::placeholders::_2));

  lateral_offset_publisher_ =
    node->create_publisher<tier4_planning_msgs::msg::LateralOffset>("~/output/lateral_offset", 1);

  const auto period = std::chrono::milliseconds(100);  // 10 Hz
  lateral_offset_publish_timer_ =
    node->create_wall_timer(period, std::bind(&SideShiftModuleManager::publishInsertedLateralOffsetTimerCallback, this));
}

void SideShiftModuleManager::onSetLateralOffset(
  const autoware_planning_msgs::srv::SetLateralOffset::Request::SharedPtr request,
  autoware_planning_msgs::srv::SetLateralOffset::Response::SharedPtr response)
{
  const double current_inserted =
    inserted_lateral_offset_state_ ? inserted_lateral_offset_state_->value.load() : 0.0;

  const auto lateral_offset_opt =
    validateAndComputeLateralOffset(*request, current_inserted, parameters_->direction_shift_amount);

  if (!lateral_offset_opt) {
    response->status.success = false;
    response->status.code = autoware_common_msgs::msg::ResponseStatus::PARAMETER_ERROR;
    response->status.message = "SetLateralOffset: validation failed (invalid shift_mode, shift_value, or shift_direction_value)";
    return;
  }

  const double lateral_offset = *lateral_offset_opt;

  tier4_planning_msgs::msg::LateralOffset msg;
  msg.stamp = node_->now();
  msg.lateral_offset = static_cast<float>(lateral_offset);
  planner_data_->set_lateral_offset(
    std::make_shared<tier4_planning_msgs::msg::LateralOffset>(msg));

  response->status.success = true;
  response->status.code = 0;
  response->status.message = "";
}

void SideShiftModuleManager::publishInsertedLateralOffsetTimerCallback()
{
  if (!lateral_offset_publisher_ || !inserted_lateral_offset_state_) {
    return;
  }
  tier4_planning_msgs::msg::LateralOffset msg;
  msg.stamp = node_->now();
  msg.lateral_offset = static_cast<float>(inserted_lateral_offset_state_->value.load());
  lateral_offset_publisher_->publish(msg);
}

void SideShiftModuleManager::updateModuleParams(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;

  auto p = parameters_;
  const std::string ns = "side_shift.";
  update_param<double>(parameters, ns + "direction_shift_amount", p->direction_shift_amount);

  std::for_each(observers_.begin(), observers_.end(), [&p](const auto & observer) {
    if (!observer.expired()) observer.lock()->updateModuleParams(p);
  });
}

}  // namespace autoware::behavior_path_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_path_planner::SideShiftModuleManager,
  autoware::behavior_path_planner::SceneModuleManagerInterface)
