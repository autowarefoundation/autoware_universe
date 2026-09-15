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

#ifndef NODE_HPP_
#define NODE_HPP_

// include
#include <autoware/agnocast_wrapper/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>

namespace autoware::hazard_lights_selector
{

struct Parameters
{
  int update_rate;  // [Hz]
};

class HazardLightsSelector : public autoware::agnocast_wrapper::Node
{
public:
  explicit HazardLightsSelector(const rclcpp::NodeOptions & node_options);

private:
  // Parameter
  Parameters params_;

  // Subscriber
  AUTOWARE_SUBSCRIPTION_PTR(autoware_vehicle_msgs::msg::HazardLightsCommand)
  sub_hazard_lights_command_from_planning_;
  AUTOWARE_SUBSCRIPTION_PTR(autoware_vehicle_msgs::msg::HazardLightsCommand)
  sub_hazard_lights_command_from_system_;

  void on_hazard_lights_command_from_planning(
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(autoware_vehicle_msgs::msg::HazardLightsCommand) & msg);
  void on_hazard_lights_command_from_system(
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(autoware_vehicle_msgs::msg::HazardLightsCommand) & msg);

  // Publisher
  AUTOWARE_PUBLISHER_PTR(autoware_vehicle_msgs::msg::HazardLightsCommand)
  pub_hazard_lights_command_;

  // Timer
  AUTOWARE_TIMER_PTR timer_;

  void on_timer();

  // State
  AUTOWARE_MESSAGE_CONST_SHARED_PTR(autoware_vehicle_msgs::msg::HazardLightsCommand)
  hazard_lights_command_from_planning_;
  AUTOWARE_MESSAGE_CONST_SHARED_PTR(autoware_vehicle_msgs::msg::HazardLightsCommand)
  hazard_lights_command_from_system_;
};
}  // namespace autoware::hazard_lights_selector

#endif  // NODE_HPP_
