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

#include <rclcpp/create_timer.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_common_msgs/msg/response_status.hpp>

#include <algorithm>
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
  p.max_shift_magnitude = node->declare_parameter<double>(ns + "max_shift_magnitude");
  p.min_shift_gap = node->declare_parameter<double>(ns + "min_shift_gap");
  p.unit_shift_amount = node->declare_parameter<double>(ns + "unit_shift_amount");
  p.publish_debug_marker = node->declare_parameter<bool>(ns + "publish_debug_marker");

  parameters_ = std::make_shared<SideShiftParameters>(p);
  inserted_lateral_offset_state_ = std::make_shared<InsertedLateralOffsetState>();
  requested_lateral_offset_state_ = std::make_shared<RequestedLateralOffsetState>();

  set_lateral_offset_srv_ = node->create_service<SetLateralOffset>(
    "~/set_lateral_offset", std::bind(
                              &SideShiftModuleManager::onSetLateralOffset, this,
                              std::placeholders::_1, std::placeholders::_2));

  lateral_offset_publisher_ =
    node->create_publisher<tier4_planning_msgs::msg::LateralOffset>("~/output/lateral_offset", 1);

  lateral_offset_sub_ = node->create_subscription<tier4_planning_msgs::msg::LateralOffset>(
    "~/input/lateral_offset", rclcpp::QoS{1},
    std::bind(&SideShiftModuleManager::onLateralOffset, this, std::placeholders::_1));

  const auto period = std::chrono::milliseconds(100);  // 10 Hz
  lateral_offset_publish_timer_ = rclcpp::create_timer(
    node_, node_->get_clock(), period,
    std::bind(&SideShiftModuleManager::publishInsertedLateralOffsetTimerCallback, this));
}

void SideShiftModuleManager::onSetLateralOffset(
  const SetLateralOffset::Request::SharedPtr request,
  SetLateralOffset::Response::SharedPtr response)
{
  if (!planner_data_ || !planner_data_->route_handler->isHandlerReady()) {
    response->status.success = false;
    response->status.code = autoware_common_msgs::msg::ResponseStatus::SERVICE_UNREADY;
    response->status.message = getStatusMessage(response->status.code);
    return;
  }

  const double current_inserted =
    inserted_lateral_offset_state_ ? inserted_lateral_offset_state_->value.load() : 0.0;

  const auto [status_code, lateral_offset] =
    validateAndComputeLateralOffset(*request, current_inserted, parameters_);

  // WARN_EXCEEDED_LIMIT will be treated as success since the shift itself will be performed
  response->status.success =
    (status_code == SetLateralOffset::Response::SUCCESS ||
     status_code == SetLateralOffset::Response::WARN_EXCEEDED_LIMIT);
  response->status.code = status_code;
  response->status.message = getStatusMessage(status_code);

  if (response->status.success) {
    requested_lateral_offset_state_->value.store(lateral_offset);
  }
}

void SideShiftModuleManager::publishInsertedLateralOffsetTimerCallback()
{
  tier4_planning_msgs::msg::LateralOffset msg;
  msg.stamp = node_->now();
  msg.lateral_offset = static_cast<float>(inserted_lateral_offset_state_->value.load());
  lateral_offset_publisher_->publish(msg);
}

void SideShiftModuleManager::onLateralOffset(
  const tier4_planning_msgs::msg::LateralOffset::ConstSharedPtr msg)
{
  const auto new_offset = static_cast<double>(msg->lateral_offset);

  const auto validation_result =
    validateRawValue(new_offset, inserted_lateral_offset_state_->value.load(), parameters_);

  requested_lateral_offset_state_->value.store(validation_result.second);
}

void SideShiftModuleManager::updateModuleParams(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;

  [[maybe_unused]] auto p = parameters_;

  [[maybe_unused]] const std::string ns = "side_shift.";
  // update_param<bool>(parameters, ns + ..., ...);

  std::for_each(observers_.begin(), observers_.end(), [&p](const auto & observer) {
    if (!observer.expired()) observer.lock()->updateModuleParams(p);
  });
}

}  // namespace autoware::behavior_path_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_path_planner::SideShiftModuleManager,
  autoware::behavior_path_planner::SceneModuleManagerInterface)
