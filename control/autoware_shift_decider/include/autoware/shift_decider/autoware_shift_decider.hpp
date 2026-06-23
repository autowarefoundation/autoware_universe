// Copyright 2020 Tier IV, Inc.
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

#ifndef AUTOWARE__SHIFT_DECIDER__AUTOWARE_SHIFT_DECIDER_HPP_
#define AUTOWARE__SHIFT_DECIDER__AUTOWARE_SHIFT_DECIDER_HPP_

#include "shift_decider_parameters.hpp"

#include <autoware/agnocast_wrapper/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_system_msgs/msg/autoware_state.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>

#include <memory>
#include <string>

namespace autoware::shift_decider
{

class ShiftDecider : public autoware::agnocast_wrapper::Node
{
public:
  explicit ShiftDecider(const rclcpp::NodeOptions & node_options);

private:
  void onTimer();
  void updateCurrentShiftCmd();
  void initTimer(double period_s);

  template <typename MessageT>
  AUTOWARE_POLLING_SUBSCRIBER_PTR(MessageT)
    create_polling_sub(const std::string & topic, const rclcpp::QoS & qos)
  {
#ifdef USE_AGNOCAST_ENABLED
    return this->create_polling_subscriber<MessageT>(topic, qos);
#else
    return AUTOWARE_CREATE_POLLING_SUBSCRIBER(MessageT, topic, qos);
#endif
  }

  AUTOWARE_PUBLISHER_PTR(autoware_vehicle_msgs::msg::GearCommand) pub_shift_cmd_;
  AUTOWARE_POLLING_SUBSCRIBER_PTR(autoware_control_msgs::msg::Control) sub_control_cmd_;
  AUTOWARE_POLLING_SUBSCRIBER_PTR(autoware_system_msgs::msg::AutowareState) sub_autoware_state_;
  AUTOWARE_POLLING_SUBSCRIBER_PTR(autoware_vehicle_msgs::msg::GearReport) sub_current_gear_;

  AUTOWARE_TIMER_PTR timer_;

  AUTOWARE_MESSAGE_CONST_SHARED_PTR(autoware_control_msgs::msg::Control) control_cmd_;
  AUTOWARE_MESSAGE_CONST_SHARED_PTR(autoware_system_msgs::msg::AutowareState) autoware_state_;
  autoware_vehicle_msgs::msg::GearCommand shift_cmd_;
  AUTOWARE_MESSAGE_CONST_SHARED_PTR(autoware_vehicle_msgs::msg::GearReport) current_gear_ptr_;
  uint8_t prev_shift_command = autoware_vehicle_msgs::msg::GearCommand::PARK;

  std::shared_ptr<::shift_decider::ParamListener> param_listener_;
  ::shift_decider::Params param_;
};
}  // namespace autoware::shift_decider

#endif  // AUTOWARE__SHIFT_DECIDER__AUTOWARE_SHIFT_DECIDER_HPP_
