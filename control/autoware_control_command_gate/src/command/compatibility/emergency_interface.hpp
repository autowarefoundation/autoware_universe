// Copyright 2022 TIER IV, Inc.
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

#ifndef COMMAND__COMPATIBILITY__EMERGENCY_INTERFACE_HPP_
#define COMMAND__COMPATIBILITY__EMERGENCY_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <tier4_external_api_msgs/msg/emergency.hpp>
#include <tier4_external_api_msgs/srv/set_emergency.hpp>

namespace autoware::control_command_gate
{

class EmergencyInterface
{
private:
  using Emergency = tier4_external_api_msgs::msg::Emergency;
  using SetEmergency = tier4_external_api_msgs::srv::SetEmergency;

public:
  explicit EmergencyInterface(rclcpp::Node * node);
  bool is_emergency() const { return is_emergency_; }
  void publish();

private:
  bool is_emergency_ = false;

  rclcpp::Node * node_;
  rclcpp::Publisher<Emergency>::SharedPtr pub_external_emergency_;
  rclcpp::Service<SetEmergency>::SharedPtr srv_external_emergency_;

  void on_service(
    const SetEmergency::Request::SharedPtr req, const SetEmergency::Response::SharedPtr res);
};

}  // namespace autoware::control_command_gate

#endif  // COMMAND__COMPATIBILITY__EMERGENCY_INTERFACE_HPP_
