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

#include "velocity_limit.hpp"

#include "utils/response.hpp"

namespace autoware::evaluation_adapter
{
VelocityLimit::VelocityLimit(const rclcpp::NodeOptions & options) : Node("velocity_limit", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  srv_api_velocity_ = create_service<VelocityLimitService>(
    "/api/autoware/set/velocity_limit", std::bind(&VelocityLimit::on_service, this, _1, _2));

  pub_planning_velocity_ = create_publisher<VelocityLimitStatus>(
    "/planning/scenario_planning/max_velocity_default", rclcpp::QoS(1).transient_local());
  sub_planning_velocity_ = create_subscription<VelocityLimitStatus>(
    "/planning/scenario_planning/current_max_velocity", rclcpp::QoS(1).transient_local(),
    std::bind(&VelocityLimit::on_message, this, _1));
}

void VelocityLimit::on_message(const VelocityLimitStatus::SharedPtr)
{
  is_ready_ = true;
}

void VelocityLimit::on_service(
  const VelocityLimitService ::Request::SharedPtr req,
  const VelocityLimitService::Response::SharedPtr res)
{
  if (!is_ready_) {
    res->status = utils::response_error("It is not ready to set velocity.");
    return;
  }

  VelocityLimitStatus msg;
  msg.stamp = now();
  msg.max_velocity = req->velocity;
  pub_planning_velocity_->publish(msg);

  res->status = utils::response_success();
}

}  // namespace autoware::evaluation_adapter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::evaluation_adapter::VelocityLimit)
