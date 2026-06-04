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

#ifndef DUMMY_TRAFFIC_LIGHT_HPP_
#define DUMMY_TRAFFIC_LIGHT_HPP_

#include "traffic_light_cycle.hpp"

#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/traffic_light_element.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace autoware::dummy_traffic_light_publisher
{

enum class Mode { Standalone, Empty, Fixed };

class DummyTrafficLight
{
public:
  struct Config
  {
    Mode mode;
    double passthrough_timeout;
    // Color published in Fixed mode (a TrafficLightElement color constant). Unused in other modes.
    uint8_t fixed_color = autoware_perception_msgs::msg::TrafficLightElement::UNKNOWN;
  };

  DummyTrafficLight(const Config & config, std::unique_ptr<TrafficLightCycle> cycle);

  void set_traffic_light_ids(const std::vector<int64_t> & ids);
  void update_input_signals(
    const autoware_perception_msgs::msg::TrafficLightGroupArray & msg, const rclcpp::Time & now);
  autoware_perception_msgs::msg::TrafficLightGroupArray create_message(const rclcpp::Time & now);

  size_t traffic_light_count() const;

private:
  autoware_perception_msgs::msg::TrafficLightGroupArray build_standalone_message(
    const rclcpp::Time & now);
  autoware_perception_msgs::msg::TrafficLightGroupArray build_fixed_message(
    const rclcpp::Time & now) const;
  static autoware_perception_msgs::msg::TrafficLightGroupArray build_empty_message(
    const rclcpp::Time & now);
  autoware_perception_msgs::msg::TrafficLightGroupArray build_groups_message(
    const rclcpp::Time & now,
    const autoware_perception_msgs::msg::TrafficLightElement & element) const;

  Config config_;
  std::unique_ptr<TrafficLightCycle> cycle_;
  std::vector<int64_t> traffic_light_ids_;
  std::optional<autoware_perception_msgs::msg::TrafficLightGroupArray> last_input_signals_;
  std::optional<rclcpp::Time> last_input_time_;
};

}  // namespace autoware::dummy_traffic_light_publisher

#endif  // DUMMY_TRAFFIC_LIGHT_HPP_
