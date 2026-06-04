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

#ifndef DUMMY_TRAFFIC_LIGHT_PUBLISHER_NODE_HPP_
#define DUMMY_TRAFFIC_LIGHT_PUBLISHER_NODE_HPP_

#include "dummy_traffic_light.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>

#include <memory>

namespace autoware::dummy_traffic_light_publisher
{

class DummyTrafficLightPublisherNode : public rclcpp::Node
{
public:
  explicit DummyTrafficLightPublisherNode(const rclcpp::NodeOptions & options);

private:
  void onTimer();
  void try_update_vector_map();

  std::unique_ptr<DummyTrafficLight> dummy_traffic_light_;

  rclcpp::Publisher<autoware_perception_msgs::msg::TrafficLightGroupArray>::SharedPtr pub_;
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr sub_vector_map_;
  rclcpp::Subscription<autoware_perception_msgs::msg::TrafficLightGroupArray>::SharedPtr sub_input_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr manual_group_;
};

}  // namespace autoware::dummy_traffic_light_publisher

#endif  // DUMMY_TRAFFIC_LIGHT_PUBLISHER_NODE_HPP_
