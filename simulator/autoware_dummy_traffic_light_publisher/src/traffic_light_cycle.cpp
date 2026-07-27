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

#include "traffic_light_cycle.hpp"

#include <cmath>
#include <stdexcept>

namespace autoware::dummy_traffic_light_publisher
{

TrafficLightCycle::TrafficLightCycle(
  double green_duration, double yellow_duration, double red_duration)
: green_duration_(green_duration), yellow_duration_(yellow_duration), red_duration_(red_duration)
{
  if (green_duration_ <= 0.0 || yellow_duration_ <= 0.0 || red_duration_ <= 0.0) {
    throw std::invalid_argument("TrafficLightCycle durations must be positive");
  }
}

autoware_perception_msgs::msg::TrafficLightElement TrafficLightCycle::update(
  const rclcpp::Time & now)
{
  using Element = autoware_perception_msgs::msg::TrafficLightElement;

  if (!start_time_) {
    start_time_ = now;
  }

  const double total = green_duration_ + yellow_duration_ + red_duration_;
  double elapsed = std::fmod((now - *start_time_).seconds(), total);
  if (elapsed < 0.0) {
    elapsed += total;
  }

  uint8_t color;
  if (elapsed < green_duration_) {
    color = Element::GREEN;
  } else if (elapsed < green_duration_ + yellow_duration_) {
    color = Element::AMBER;
  } else {
    color = Element::RED;
  }

  Element element;
  element.color = color;
  element.shape = Element::CIRCLE;
  element.status = Element::SOLID_ON;
  element.confidence = 1.0;
  return element;
}

}  // namespace autoware::dummy_traffic_light_publisher
