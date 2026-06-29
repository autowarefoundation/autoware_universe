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

#ifndef TRAFFIC_LIGHT_SELECTOR_HPP_
#define TRAFFIC_LIGHT_SELECTOR_HPP_

#include "traffic_light_selector_utils.hpp"

#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <map>
#include <vector>

namespace autoware::traffic_light
{
using sensor_msgs::msg::RegionOfInterest;
using tier4_perception_msgs::msg::DetectedObjectsWithFeature;
using tier4_perception_msgs::msg::TrafficLightRoiArray;

class TrafficLightSelector
{
public:
  TrafficLightSelector() = default;

  TrafficLightRoiArray select(
    const DetectedObjectsWithFeature & detected_traffic_light_msg,
    const TrafficLightRoiArray & rough_rois_msg, const TrafficLightRoiArray & expected_rois_msg,
    uint32_t image_width, uint32_t image_height) const;
};

}  // namespace autoware::traffic_light

#endif  // TRAFFIC_LIGHT_SELECTOR_HPP_
