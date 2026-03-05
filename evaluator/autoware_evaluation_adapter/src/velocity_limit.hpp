// Copyright 2021 TIER IV, Inc.
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

#ifndef VELOCITY_LIMIT_HPP_
#define VELOCITY_LIMIT_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
#include <tier4_external_api_msgs/srv/set_velocity_limit.hpp>

namespace autoware::evaluation_adapter
{
class VelocityLimit : public rclcpp::Node
{
public:
  explicit VelocityLimit(const rclcpp::NodeOptions & options);

private:
  using VelocityLimitService = tier4_external_api_msgs::srv::SetVelocityLimit;
  using VelocityLimitStatus = autoware_internal_planning_msgs::msg::VelocityLimit;

  rclcpp::Service<VelocityLimitService>::SharedPtr srv_api_velocity_;
  rclcpp::Publisher<VelocityLimitStatus>::SharedPtr pub_planning_velocity_;
  rclcpp::Subscription<VelocityLimitStatus>::SharedPtr sub_planning_velocity_;

  void on_message(const VelocityLimitStatus::SharedPtr msg);
  void on_service(
    const VelocityLimitService::Request::SharedPtr req,
    const VelocityLimitService::Response::SharedPtr res);

  bool is_ready_ = false;
};

}  // namespace autoware::evaluation_adapter

#endif  // VELOCITY_LIMIT_HPP_
