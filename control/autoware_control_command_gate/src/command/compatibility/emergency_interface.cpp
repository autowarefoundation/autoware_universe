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

#include "emergency_interface.hpp"

namespace autoware::control_command_gate
{

EmergencyInterface::EmergencyInterface(rclcpp::Node * node) : node_(node)
{
  pub_external_emergency_ = node_->create_publisher<Emergency>(
    "/api/autoware/get/emergency", rclcpp::QoS(1).transient_local());
  srv_external_emergency_ = node_->create_service<SetEmergency>(
    "/api/autoware/set/emergency",
    std::bind(&EmergencyInterface::on_service, this, std::placeholders::_1, std::placeholders::_2));
}

void EmergencyInterface::publish()
{
  Emergency msg;
  msg.stamp = node_->now();
  msg.emergency = is_emergency_;
  pub_external_emergency_->publish(msg);
}

void EmergencyInterface::on_service(
  const SetEmergency::Request::SharedPtr req, const SetEmergency::Response::SharedPtr res)
{
  is_emergency_ = req->emergency;
  res->status.code = tier4_external_api_msgs::msg::ResponseStatus::SUCCESS;
}

}  // namespace autoware::control_command_gate
