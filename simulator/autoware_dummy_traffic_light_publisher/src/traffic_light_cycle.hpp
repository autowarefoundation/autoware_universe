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

#ifndef TRAFFIC_LIGHT_CYCLE_HPP_
#define TRAFFIC_LIGHT_CYCLE_HPP_

#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/traffic_light_element.hpp>

#include <optional>

namespace autoware::dummy_traffic_light_publisher
{

class TrafficLightCycle
{
public:
  TrafficLightCycle(double green_duration, double yellow_duration, double red_duration);

  autoware_perception_msgs::msg::TrafficLightElement update(const rclcpp::Time & now);

private:
  double green_duration_;
  double yellow_duration_;
  double red_duration_;
  std::optional<rclcpp::Time> start_time_;
};

}  // namespace autoware::dummy_traffic_light_publisher

#endif  // TRAFFIC_LIGHT_CYCLE_HPP_
