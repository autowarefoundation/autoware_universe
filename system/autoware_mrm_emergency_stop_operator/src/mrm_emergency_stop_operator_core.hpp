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

#ifndef MRM_EMERGENCY_STOP_OPERATOR_CORE_HPP_
#define MRM_EMERGENCY_STOP_OPERATOR_CORE_HPP_

// Core
#include <functional>
#include <memory>

// Autoware
#include <autoware_control_msgs/msg/control.hpp>
#include <tier4_system_msgs/msg/driving_mode_info.hpp>
#include <tier4_system_msgs/msg/driving_mode_mrm_state.hpp>
#include <tier4_system_msgs/msg/driving_mode_request.hpp>
#include <tier4_system_msgs/msg/mrm_behavior_status.hpp>
#include <tier4_system_msgs/srv/operate_mrm.hpp>

// ROS 2 core
#include <autoware/agnocast_wrapper/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <vector>
namespace autoware::mrm_emergency_stop_operator
{
using autoware_control_msgs::msg::Control;
using tier4_system_msgs::msg::DrivingModeInfo;
using tier4_system_msgs::msg::DrivingModeMrmState;
using tier4_system_msgs::msg::DrivingModeRequest;
using tier4_system_msgs::msg::MrmBehaviorStatus;
using tier4_system_msgs::srv::OperateMrm;

struct Parameters
{
  int update_rate;             // [Hz]
  double target_acceleration;  // [m/s^2]
  double target_jerk;          // [m/s^3]
};

class MrmEmergencyStopOperator : public autoware::agnocast_wrapper::Node
{
public:
  explicit MrmEmergencyStopOperator(const rclcpp::NodeOptions & node_options);

private:
  // Parameters
  Parameters params_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  rcl_interfaces::msg::SetParametersResult onParameter(
    const std::vector<rclcpp::Parameter> & parameters);

  // Subscriber
  AUTOWARE_SUBSCRIPTION_PTR(Control) sub_control_cmd_;

  void onControlCommand(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(Control) & msg);

  // Server
  AUTOWARE_SERVICE_PTR(OperateMrm) service_operation_;

  void operateEmergencyStop(
    const AUTOWARE_SERVER_REQUEST_PTR(OperateMrm) request,
    AUTOWARE_SERVER_RESPONSE_PTR(OperateMrm) response);

  // Publisher
  AUTOWARE_PUBLISHER_PTR(MrmBehaviorStatus) pub_status_;
  AUTOWARE_PUBLISHER_PTR(Control) pub_control_cmd_;

  void publishStatus() const;
  void publishControlCommand(const Control & command) const;

  // Timer
  AUTOWARE_TIMER_PTR timer_;

  void onTimer();

  // States
  MrmBehaviorStatus status_;
  Control prev_control_cmd_;
  bool is_prev_control_cmd_subscribed_;

  // Driving mode interface
  AUTOWARE_SUBSCRIPTION_PTR(DrivingModeRequest) sub_driving_mode_request_;
  AUTOWARE_SUBSCRIPTION_PTR(DrivingModeInfo) sub_driving_mode_info_;
  AUTOWARE_PUBLISHER_PTR(DrivingModeMrmState) pub_mrm_state_;
  void onDrivingModeRequest(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(DrivingModeRequest) & msg);
  void onDrivingModeInfo(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(DrivingModeInfo) & msg);
  void publishMrmState() const;
  std::optional<uint32_t> driving_mode_id_;  // Refer to the driving_mode_manager for this ID.

  // Algorithm
  Control calcTargetAcceleration(const Control & prev_control_cmd) const;
};

}  // namespace autoware::mrm_emergency_stop_operator

#endif  // MRM_EMERGENCY_STOP_OPERATOR_CORE_HPP_
