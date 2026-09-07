// Copyright 2026 TIER IV, Inc.
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

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <tier4_control_msgs/msg/constant_jerk_deceleration_trigger.hpp>

namespace autoware::in_lane_mrm_planner
{

class MrmTriggerRelayNode : public rclcpp::Node
{
public:
  explicit MrmTriggerRelayNode(const rclcpp::NodeOptions & options)
  : Node("mrm_trigger_relay", options)
  {
    pub_trigger_ = create_publisher<std_msgs::msg::Bool>("~/output/trigger", rclcpp::QoS{1});

    sub_trigger_ = create_subscription<tier4_control_msgs::msg::ConstantJerkDecelerationTrigger>(
      "~/input/constant_jerk_deceleration_trigger", rclcpp::QoS{1},
      [this](const tier4_control_msgs::msg::ConstantJerkDecelerationTrigger::ConstSharedPtr msg) {
        std_msgs::msg::Bool out;
        out.data = msg->trigger;
        pub_trigger_->publish(out);
      });
  }

private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_trigger_;
  rclcpp::Subscription<tier4_control_msgs::msg::ConstantJerkDecelerationTrigger>::SharedPtr
    sub_trigger_;
};

}  // namespace autoware::in_lane_mrm_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::in_lane_mrm_planner::MrmTriggerRelayNode)
