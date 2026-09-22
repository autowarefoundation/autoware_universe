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

#ifndef AUTOWARE__TRAFFIC_LIGHT_MULTI_CAMERA_FUSION__DETAIL__TYPES_HPP_
#define AUTOWARE__TRAFFIC_LIGHT_MULTI_CAMERA_FUSION__DETAIL__TYPES_HPP_

#include <tier4_perception_msgs/msg/traffic_light.hpp>

#include <utility>
#include <vector>

namespace autoware::traffic_light
{

using StateKey = std::vector<std::pair<
  tier4_perception_msgs::msg::TrafficLightElement::_color_type,
  tier4_perception_msgs::msg::TrafficLightElement::_shape_type>>;

}  // namespace autoware::traffic_light

#endif  // AUTOWARE__TRAFFIC_LIGHT_MULTI_CAMERA_FUSION__DETAIL__TYPES_HPP_
