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

#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>

namespace autoware::in_lane_mrm_planner
{

class MrmInLaneStopAuxiliaryCommandsNode : public rclcpp::Node
{
public:
  explicit MrmInLaneStopAuxiliaryCommandsNode(const rclcpp::NodeOptions & options)
  : Node("mrm_in_lane_stop_auxiliary_commands", options)
  {
    const double publish_hz = declare_parameter<double>("publish_hz", 10.0);

    pub_gear_ = create_publisher<autoware_vehicle_msgs::msg::GearCommand>(
      "~/output/gear_command", rclcpp::QoS{1});
    pub_hazard_ = create_publisher<autoware_vehicle_msgs::msg::HazardLightsCommand>(
      "~/output/hazard_lights_command", rclcpp::QoS{1});
    pub_turn_ = create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>(
      "~/output/turn_indicators_command", rclcpp::QoS{1});

    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Rate(publish_hz).period(),
      std::bind(&MrmInLaneStopAuxiliaryCommandsNode::on_timer, this));
  }

private:
  void on_timer()
  {
    const auto stamp = now();

    autoware_vehicle_msgs::msg::GearCommand gear;
    gear.stamp = stamp;
    gear.command = autoware_vehicle_msgs::msg::GearCommand::DRIVE;
    pub_gear_->publish(gear);

    autoware_vehicle_msgs::msg::HazardLightsCommand hazard;
    hazard.stamp = stamp;
    hazard.command = autoware_vehicle_msgs::msg::HazardLightsCommand::ENABLE;
    pub_hazard_->publish(hazard);

    autoware_vehicle_msgs::msg::TurnIndicatorsCommand turn;
    turn.stamp = stamp;
    turn.command = autoware_vehicle_msgs::msg::TurnIndicatorsCommand::DISABLE;
    pub_turn_->publish(turn);
  }

  rclcpp::Publisher<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr pub_gear_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr pub_hazard_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr pub_turn_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autoware::in_lane_mrm_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::in_lane_mrm_planner::MrmInLaneStopAuxiliaryCommandsNode)
