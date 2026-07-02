// Copyright 2022 Tier IV, Inc.
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

#ifndef MRM_COMFORTABLE_STOP_OPERATOR_CORE_HPP_
#define MRM_COMFORTABLE_STOP_OPERATOR_CORE_HPP_

// Core
#include <memory>
#include <vector>

// Autoware
#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit_clear_command.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit_constraints.hpp>
#include <tier4_system_msgs/msg/driving_mode_info.hpp>
#include <tier4_system_msgs/msg/driving_mode_mrm_state.hpp>
#include <tier4_system_msgs/msg/driving_mode_request.hpp>
#include <tier4_system_msgs/msg/mrm_behavior_status.hpp>
#include <tier4_system_msgs/srv/operate_mrm.hpp>

// ROS 2 core
#include <autoware/agnocast_wrapper/node.hpp>
#include <rclcpp/rclcpp.hpp>

namespace autoware::mrm_comfortable_stop_operator
{

using tier4_system_msgs::msg::DrivingModeInfo;
using tier4_system_msgs::msg::DrivingModeMrmState;
using tier4_system_msgs::msg::DrivingModeRequest;
using tier4_system_msgs::msg::MrmBehaviorStatus;

struct Parameters
{
  int update_rate;          // [Hz]
  double min_acceleration;  // [m/s^2]
  double max_jerk;          // [m/s^3]
  double min_jerk;          // [m/s^3]
};

class MrmComfortableStopOperator : public autoware::agnocast_wrapper::Node
{
public:
  explicit MrmComfortableStopOperator(const rclcpp::NodeOptions & node_options);

private:
  // Parameters
  Parameters params_;

  // Server
  AUTOWARE_SERVICE_PTR(tier4_system_msgs::srv::OperateMrm) service_operation_;

  void operateComfortableStop(
    AUTOWARE_SERVER_REQUEST_PTR(tier4_system_msgs::srv::OperateMrm) request,
    AUTOWARE_SERVER_RESPONSE_PTR(tier4_system_msgs::srv::OperateMrm) response);

  rcl_interfaces::msg::SetParametersResult onParameter(
    const std::vector<rclcpp::Parameter> & parameters);

  // Publisher
  AUTOWARE_PUBLISHER_PTR(tier4_system_msgs::msg::MrmBehaviorStatus) pub_status_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_planning_msgs::msg::VelocityLimit) pub_velocity_limit_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_planning_msgs::msg::VelocityLimitClearCommand)
  pub_velocity_limit_clear_command_;

  void publishStatus() const;
  void publishVelocityLimit() const;
  void publishVelocityLimitClearCommand() const;

  // Timer
  AUTOWARE_TIMER_PTR timer_;

  void onTimer() const;

  // States
  tier4_system_msgs::msg::MrmBehaviorStatus status_;

  // Parameter callback
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  // Driving mode interface
  AUTOWARE_SUBSCRIPTION_PTR(DrivingModeRequest) sub_driving_mode_request_;
  AUTOWARE_SUBSCRIPTION_PTR(DrivingModeInfo) sub_driving_mode_info_;
  AUTOWARE_PUBLISHER_PTR(DrivingModeMrmState) pub_mrm_state_;
  void onDrivingModeRequest(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(DrivingModeRequest) & msg);
  void onDrivingModeInfo(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(DrivingModeInfo) & msg);
  void publishMrmState() const;
  std::optional<uint32_t> driving_mode_id_;  // Refer to the driving_mode_manager for this ID.
};

}  // namespace autoware::mrm_comfortable_stop_operator

#endif  // MRM_COMFORTABLE_STOP_OPERATOR_CORE_HPP_
