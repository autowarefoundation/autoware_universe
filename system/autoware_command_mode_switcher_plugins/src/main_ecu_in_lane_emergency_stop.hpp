//  Copyright 2025 The Autoware Contributors
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#ifndef MAIN_ECU_IN_LANE_EMERGENCY_STOP_HPP_
#define MAIN_ECU_IN_LANE_EMERGENCY_STOP_HPP_

#include <autoware_command_mode_switcher/command_plugin.hpp>
#include <autoware_command_mode_types/modes.hpp>
#include <autoware_command_mode_types/sources.hpp>
#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_control_msgs/msg/constant_jerk_deceleration_trigger.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_system_msgs/srv/change_topic_relay_control.hpp>

namespace autoware::command_mode_switcher
{

using tier4_control_msgs::msg::ConstantJerkDecelerationTrigger;

class MainEcuInLaneEmergencyStopSwitcher : public CommandPlugin
{
public:
  uint16_t mode() const override
  {
    return autoware::command_mode_types::modes::main_ecu_in_lane_emergency_stop;
  }
  uint16_t source() const override { return autoware::command_mode_types::sources::in_lane_stop; }
  bool autoware_control() const override { return true; }
  void initialize() override;

  SourceState update_source_state(bool request) override;
  MrmState update_mrm_state() override;

private:
  void publish_constant_jerk_deceleration_trigger(bool turn_on);
  void request_topic_relay_control(
    bool relay_on, rclcpp::Client<tier4_system_msgs::srv::ChangeTopicRelayControl>::SharedPtr client,
    const std::string & srv_name);
  bool is_stopped();

  struct Params
  {
    double target_acceleration;
    double target_jerk;
    bool enable_trajectory_relay;
    bool enable_pose_with_covariance_relay;
    int64_t service_timeout_ms;
  };

  rclcpp::Publisher<ConstantJerkDecelerationTrigger>::SharedPtr pub_trigger_;
  std::unique_ptr<autoware_utils_rclcpp::InterProcessPollingSubscriber<nav_msgs::msg::Odometry>>
    sub_odom_;
  rclcpp::CallbackGroup::SharedPtr client_relay_trajectory_group_;
  rclcpp::Client<tier4_system_msgs::srv::ChangeTopicRelayControl>::SharedPtr
    client_relay_trajectory_;
  rclcpp::CallbackGroup::SharedPtr client_relay_pose_with_covariance_group_;
  rclcpp::Client<tier4_system_msgs::srv::ChangeTopicRelayControl>::SharedPtr
    client_relay_pose_with_covariance_;

  MrmState mrm_state_;
  Params params_;
};

}  // namespace autoware::command_mode_switcher

#endif  // MAIN_ECU_IN_LANE_EMERGENCY_STOP_HPP_
