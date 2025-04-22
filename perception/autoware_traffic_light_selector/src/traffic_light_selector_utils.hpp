// Copyright 2025 TIER IV, Inc.
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

#ifndef TRAFFIC_LIGHT_SELECTOR_UTILS_HPP_
#define TRAFFIC_LIGHT_SELECTOR_UTILS_HPP_

#include <sensor_msgs/msg/region_of_interest.hpp>

#include <vector>

namespace autoware::traffic_light
{
namespace utils
{
using sensor_msgs::msg::RegionOfInterest;

bool isInsideRoughRoi(const RegionOfInterest & detected_roi, const RegionOfInterest & rough_roi);

void computeCenterOffset(
  const RegionOfInterest & source, const RegionOfInterest & target, int32_t & shift_x,
  int32_t & shift_y);

RegionOfInterest getShiftedRoi(
  const RegionOfInterest & input_ROI, const uint32_t & image_width, const uint32_t & image_height,
  const int32_t & shift_x, const int32_t & shift_y);

double getIoU(const RegionOfInterest & bbox1, const RegionOfInterest & bbox2);

double getGenIoU(const RegionOfInterest & bbox1, const RegionOfInterest & bbox2);

}  // namespace utils
}  // namespace autoware::traffic_light

#endif  // TRAFFIC_LIGHT_SELECTOR_UTILS_HPP_
