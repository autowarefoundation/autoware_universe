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

#ifndef AUTOWARE__TRAFFIC_LIGHT_COMPLIANCE_CHECKER__UTILS_HPP_
#define AUTOWARE__TRAFFIC_LIGHT_COMPLIANCE_CHECKER__UTILS_HPP_

#include "autoware/traffic_light_compliance_checker/structs.hpp"
#include "autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp"

#include <autoware_perception_msgs/msg/traffic_light_element.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group.hpp>

#include <lanelet2_core/primitives/Lanelet.h>

#include <memory>
#include <vector>

namespace autoware::traffic_light_compliance_checker
{

[[nodiscard]] bool is_equal(
  const autoware_perception_msgs::msg::TrafficLightElement & a,
  const autoware_perception_msgs::msg::TrafficLightElement & b);

[[nodiscard]] bool is_equal(
  const std::vector<autoware_perception_msgs::msg::TrafficLightElement> & a,
  const std::vector<autoware_perception_msgs::msg::TrafficLightElement> & b);

[[nodiscard]] bool has_amber_circle(
  const std::vector<autoware_perception_msgs::msg::TrafficLightElement> & elements);

[[nodiscard]] bool has_green_circle(
  const std::vector<autoware_perception_msgs::msg::TrafficLightElement> & elements);

[[nodiscard]] bool has_red_circle(
  const std::vector<autoware_perception_msgs::msg::TrafficLightElement> & elements);

[[nodiscard]] bool has_unknown(
  const std::vector<autoware_perception_msgs::msg::TrafficLightElement> & elements);

[[nodiscard]] bool has_static_arrow(
  const std::shared_ptr<const lanelet::autoware::AutowareTrafficLight> & reg_elem);

[[nodiscard]] bool is_arrow_aware_amber_pass(
  const lanelet::ConstLanelet & lanelet,
  const autoware_perception_msgs::msg::TrafficLightGroup & signal,
  const std::shared_ptr<const lanelet::autoware::AutowareTrafficLight> & reg_elem,
  const AmberState amber_transition_state);

}  // namespace autoware::traffic_light_compliance_checker

#endif  // AUTOWARE__TRAFFIC_LIGHT_COMPLIANCE_CHECKER__UTILS_HPP_
