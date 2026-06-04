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

#include "dummy_traffic_light.hpp"

#include <memory>
#include <utility>
#include <vector>

namespace autoware::dummy_traffic_light_publisher
{

DummyTrafficLight::DummyTrafficLight(
  const Config & config, std::unique_ptr<TrafficLightCycle> cycle)
: config_(config), cycle_(std::move(cycle))
{
}

void DummyTrafficLight::set_traffic_light_ids(const std::vector<int64_t> & ids)
{
  traffic_light_ids_ = ids;
}

void DummyTrafficLight::update_input_signals(
  const autoware_perception_msgs::msg::TrafficLightGroupArray & msg, const rclcpp::Time & now)
{
  last_input_signals_ = msg;
  last_input_time_ = now;
}

autoware_perception_msgs::msg::TrafficLightGroupArray DummyTrafficLight::create_message(
  const rclcpp::Time & now)
{
  if (last_input_signals_ && last_input_time_) {
    const double elapsed = (now - *last_input_time_).seconds();
    if (elapsed < config_.passthrough_timeout) {
      return *last_input_signals_;
    }
    last_input_signals_.reset();
    last_input_time_.reset();
  }

  if (config_.mode == Mode::Empty || traffic_light_ids_.empty()) {
    return build_empty_message(now);
  }

  if (config_.mode == Mode::Fixed) {
    return build_fixed_message(now);
  }

  return build_standalone_message(now);
}

size_t DummyTrafficLight::traffic_light_count() const
{
  return traffic_light_ids_.size();
}

autoware_perception_msgs::msg::TrafficLightGroupArray DummyTrafficLight::build_standalone_message(
  const rclcpp::Time & now)
{
  return build_groups_message(now, cycle_->update(now));
}

autoware_perception_msgs::msg::TrafficLightGroupArray DummyTrafficLight::build_fixed_message(
  const rclcpp::Time & now) const
{
  autoware_perception_msgs::msg::TrafficLightElement element;
  element.color = config_.fixed_color;
  element.shape = autoware_perception_msgs::msg::TrafficLightElement::CIRCLE;
  element.status = autoware_perception_msgs::msg::TrafficLightElement::SOLID_ON;
  element.confidence = 1.0;
  return build_groups_message(now, element);
}

autoware_perception_msgs::msg::TrafficLightGroupArray DummyTrafficLight::build_empty_message(
  const rclcpp::Time & now)
{
  autoware_perception_msgs::msg::TrafficLightGroupArray output;
  output.stamp = now;
  return output;
}

autoware_perception_msgs::msg::TrafficLightGroupArray DummyTrafficLight::build_groups_message(
  const rclcpp::Time & now,
  const autoware_perception_msgs::msg::TrafficLightElement & element) const
{
  autoware_perception_msgs::msg::TrafficLightGroupArray output;
  output.stamp = now;

  for (const auto id : traffic_light_ids_) {
    autoware_perception_msgs::msg::TrafficLightGroup group;
    group.traffic_light_group_id = id;
    group.elements.push_back(element);
    output.traffic_light_groups.push_back(group);
  }

  return output;
}

}  // namespace autoware::dummy_traffic_light_publisher
