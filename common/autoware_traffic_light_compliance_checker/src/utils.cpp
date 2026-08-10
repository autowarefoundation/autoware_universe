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

#include "autoware/traffic_light_compliance_checker/utils.hpp"

#include <autoware/lanelet2_utils/intersection.hpp>
#include <autoware/traffic_light_utils/traffic_light_utils.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::traffic_light_compliance_checker
{
using autoware_perception_msgs::msg::TrafficLightElement;

bool is_equal(
  const autoware_perception_msgs::msg::TrafficLightElement & a,
  const autoware_perception_msgs::msg::TrafficLightElement & b)
{
  return a.color == b.color && a.shape == b.shape && a.status == b.status;
}

bool is_equal(
  const std::vector<autoware_perception_msgs::msg::TrafficLightElement> & a,
  const std::vector<autoware_perception_msgs::msg::TrafficLightElement> & b)
{
  if (a.size() != b.size()) {
    return false;
  }

  for (size_t i = 0; i < a.size(); ++i) {
    if (!is_equal(a[i], b[i])) {
      return false;
    }
  }

  return true;
}

bool is_shape_and_color(
  const std::vector<TrafficLightElement> & elements,
  const TrafficLightElement::_shape_type & shape,
  const TrafficLightElement::_color_type & color)
{
  return autoware::traffic_light_utils::hasTrafficLightShapeAndColor(elements, shape, color);
}

bool has_amber_circle(
  const std::vector<TrafficLightElement> & elements)
{
  return is_shape_and_color(
    elements, TrafficLightElement::CIRCLE, TrafficLightElement::AMBER);
}

bool has_green_circle(
  const std::vector<TrafficLightElement> & elements)
{
  return is_shape_and_color(elements, TrafficLightElement::CIRCLE, TrafficLightElement::GREEN);
}

bool has_red_circle(
  const std::vector<TrafficLightElement> & elements)
{
  return is_shape_and_color(elements, TrafficLightElement::CIRCLE, TrafficLightElement::RED);
}

bool has_unknown(
  const std::vector<TrafficLightElement> & elements)
{
  return autoware::traffic_light_utils::hasTrafficLightColor(elements, TrafficLightElement::UNKNOWN);
}

bool has_static_arrow(
  const std::shared_ptr<const lanelet::autoware::AutowareTrafficLight> & reg_elem)
{
  if (!reg_elem) {
    return false;
  }

  for (const auto & light : reg_elem->trafficLights()) {
    const auto & attributes = light.attributes();
    if (attributes.find("subtype") != attributes.end()) {
      const std::string subtype = attributes.at("subtype").value();
      if (subtype.find("arrow") != std::string::npos) {
        return true;
      }
    }
  }

  for (const auto & light_bulb_ls : reg_elem->lightBulbs()) {
    for (const auto & node : light_bulb_ls) {
      const auto & attributes = node.attributes();
      if (attributes.find("arrow") != attributes.end()) {
        return true;
      }
    }
  }

  return false;
}

bool is_arrow_aware_amber_pass(
  const lanelet::ConstLanelet & lanelet,
  const autoware_perception_msgs::msg::TrafficLightGroup & signal,
  const std::shared_ptr<const lanelet::autoware::AutowareTrafficLight> & reg_elem,
  const YellowState yellow_transition_state)
{
  if (!has_amber_circle(signal.elements)) {
    return false;
  }
  if (yellow_transition_state != YellowState::kFromGreen) {
    return false;
  }

  const bool is_turn_lane = autoware::experimental::lanelet2_utils::is_left_direction(lanelet) ||
                            autoware::experimental::lanelet2_utils::is_right_direction(lanelet);
  if (!is_turn_lane) {
    return false;
  }

  return has_static_arrow(reg_elem);
}

}  // namespace autoware::traffic_light_compliance_checker
