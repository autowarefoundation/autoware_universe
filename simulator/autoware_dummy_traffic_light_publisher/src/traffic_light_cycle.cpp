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

#include "autoware/dummy_traffic_light_publisher/traffic_light_cycle.hpp"

#include <array>

namespace autoware::dummy_traffic_light_publisher
{

TrafficLightCycle::TrafficLightCycle(
  double green_duration, double yellow_duration, double red_duration)
: green_duration_(green_duration), yellow_duration_(yellow_duration), red_duration_(red_duration)
{
}

autoware_perception_msgs::msg::TrafficLightElement TrafficLightCycle::update(
  const rclcpp::Time & now)
{
  using Element = autoware_perception_msgs::msg::TrafficLightElement;

  static constexpr std::array<Phase, 3> kNextPhase = {Phase::Yellow, Phase::Red, Phase::Green};
  static constexpr std::array<uint8_t, 3> kColor = {Element::GREEN, Element::AMBER, Element::RED};
  const std::array<double, 3> durations = {green_duration_, yellow_duration_, red_duration_};

  // advance phase
  if (!initialized_) {
    last_transition_time_ = now;
    initialized_ = true;
  } else {
    const auto i = static_cast<int>(phase_);
    if ((now - last_transition_time_).seconds() >= durations[i]) {
      last_transition_time_ = now;
      phase_ = kNextPhase[i];
    }
  }

  // build element
  Element element;
  element.color = kColor[static_cast<int>(phase_)];
  element.shape = Element::CIRCLE;
  element.status = Element::SOLID_ON;
  element.confidence = 1.0;
  return element;
}

}  // namespace autoware::dummy_traffic_light_publisher
