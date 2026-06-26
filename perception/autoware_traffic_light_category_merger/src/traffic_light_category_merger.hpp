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

#ifndef TRAFFIC_LIGHT_CATEGORY_MERGER_HPP_
#define TRAFFIC_LIGHT_CATEGORY_MERGER_HPP_

#include <tier4_perception_msgs/msg/traffic_light_array.hpp>

namespace autoware::traffic_light
{
using tier4_perception_msgs::msg::TrafficLightArray;

/// ROS-free core of the category merger.
///
/// Concatenates the car and pedestrian traffic-light arrays into a single array.
/// The result carries the car array's header, followed by the car signals and
/// then the pedestrian signals. The merge holds no state, so it is exposed as a
/// static function.
class TrafficLightCategoryMerger
{
public:
  static TrafficLightArray merge(
    const TrafficLightArray & car_signals, const TrafficLightArray & pedestrian_signals);
};

}  // namespace autoware::traffic_light

#endif  // TRAFFIC_LIGHT_CATEGORY_MERGER_HPP_
