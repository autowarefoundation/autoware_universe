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
#include <rclcpp/rclcpp.hpp>

#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>

namespace autoware::hazard_lights_selector
{

struct Parameters
{
  int update_rate;  // [Hz]
};

class HazardLightsSelector : public rclcpp::Node
{
public:
  explicit HazardLightsSelector(const rclcpp::NodeOptions & node_options);

private:
  // Parameter
  Parameters params_;

  // Subscriber
  rclcpp::Subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr
    sub_hazard_lights_command_from_planning_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr
    sub_hazard_lights_command_from_system_;

  void on_hazard_lights_command_from_planning(
    const autoware_vehicle_msgs::msg::HazardLightsCommand::SharedPtr msg);
  void on_hazard_lights_command_from_system(
    const autoware_vehicle_msgs::msg::HazardLightsCommand::SharedPtr msg);

  // Publisher
  rclcpp::Publisher<autoware_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr
    pub_hazard_lights_command_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  void on_timer();

  // State
  autoware_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr
    hazard_lights_command_from_planning_;
  autoware_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr
    hazard_lights_command_from_system_;
};
}  // namespace autoware::hazard_lights_selector

#endif  // NODE_HPP_
