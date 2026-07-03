// Copyright 2026 The Autoware Contributors
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

#ifndef MRM_IN_LANE_STOP_OPERATOR_HPP_
#define MRM_IN_LANE_STOP_OPERATOR_HPP_

#include <optional>
#include <string>
#include <vector>

#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_control_msgs/msg/constant_jerk_deceleration_trigger.hpp>
#include <tier4_system_msgs/msg/driving_mode_flag.hpp>
#include <tier4_system_msgs/msg/driving_mode_info.hpp>
#include <tier4_system_msgs/msg/driving_mode_mrm_state.hpp>
#include <tier4_system_msgs/msg/driving_mode_request.hpp>
#include <tier4_system_msgs/srv/change_topic_relay_control.hpp>

namespace autoware::mrm_in_lane_stop_operator
{

using ChangeTopicRelayControl = tier4_system_msgs::srv::ChangeTopicRelayControl;
using ConstantJerkDecelerationTrigger = tier4_control_msgs::msg::ConstantJerkDecelerationTrigger;
using DrivingModeFlag = tier4_system_msgs::msg::DrivingModeFlag;
using DrivingModeInfo = tier4_system_msgs::msg::DrivingModeInfo;
using DrivingModeMrmState = tier4_system_msgs::msg::DrivingModeMrmState;
using DrivingModeRequest = tier4_system_msgs::msg::DrivingModeRequest;
using Odometry = nav_msgs::msg::Odometry;

struct ModeConfig
{
  std::string name;
  double target_acceleration;
  double target_jerk;
  std::optional<uint32_t> mode_id;
};

class MrmInLaneStopOperator : public rclcpp::Node
{
public:
  explicit MrmInLaneStopOperator(const rclcpp::NodeOptions & node_options);

private:
  std::vector<ModeConfig> modes_;
  std::optional<uint32_t> active_mode_id_;
  int64_t service_timeout_ms_;
  std::string relay_service_name_;

  rclcpp::CallbackGroup::SharedPtr relay_group_;
  rclcpp::Client<ChangeTopicRelayControl>::SharedPtr relay_client_;

  rclcpp::Publisher<ConstantJerkDecelerationTrigger>::SharedPtr pub_trigger_;
  rclcpp::Publisher<DrivingModeMrmState>::SharedPtr pub_mrm_state_;
  rclcpp::Publisher<DrivingModeFlag>::SharedPtr pub_driving_mode_active_;
  rclcpp::Subscription<DrivingModeRequest>::SharedPtr sub_request_;
  rclcpp::Subscription<DrivingModeInfo>::SharedPtr sub_info_;
  std::unique_ptr<autoware_utils_rclcpp::InterProcessPollingSubscriber<Odometry>>
    sub_kinematic_state_;
  rclcpp::TimerBase::SharedPtr timer_;

  void on_request(DrivingModeRequest::ConstSharedPtr msg);
  void on_info(DrivingModeInfo::ConstSharedPtr msg);
  void on_timer();

  bool execute(ModeConfig & mode);
  void cancel(ModeConfig & mode);
  void publish_trigger(bool turn_on, double target_acceleration, double target_jerk);
  bool call_relay(bool relay_on);
  void publishDrivingModeActive() const;
  void publish_mrm_state() const;
  bool is_vehicle_stopped() const;
};

}  // namespace autoware::mrm_in_lane_stop_operator

#endif  // MRM_IN_LANE_STOP_OPERATOR_HPP_
