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

#include "traffic_light_multi_camera_fusion_utils.hpp"
#include <rclcpp/rclcpp.hpp>

namespace autoware::traffic_light
{
namespace utils
{

using T4Elem = tier4_perception_msgs::msg::TrafficLightElement;
using AutowareElem = autoware_perception_msgs::msg::TrafficLightElement;
const std::unordered_map<T4Elem::_color_type, AutowareElem::_color_type> color_map(
  {{T4Elem::RED, AutowareElem::RED},
    {T4Elem::AMBER, AutowareElem::AMBER},
    {T4Elem::GREEN, AutowareElem::GREEN},
    {T4Elem::WHITE, AutowareElem::WHITE}});
const std::unordered_map<T4Elem::_shape_type, AutowareElem::_shape_type> shape_map(
  {{T4Elem::CIRCLE, AutowareElem::CIRCLE},
    {T4Elem::LEFT_ARROW, AutowareElem::LEFT_ARROW},
    {T4Elem::RIGHT_ARROW, AutowareElem::RIGHT_ARROW},
    {T4Elem::UP_ARROW, AutowareElem::UP_ARROW},
    {T4Elem::UP_LEFT_ARROW, AutowareElem::UP_LEFT_ARROW},
    {T4Elem::UP_RIGHT_ARROW, AutowareElem::UP_RIGHT_ARROW},
    {T4Elem::DOWN_ARROW, AutowareElem::DOWN_ARROW},
    {T4Elem::DOWN_LEFT_ARROW, AutowareElem::DOWN_LEFT_ARROW},
    {T4Elem::DOWN_RIGHT_ARROW, AutowareElem::DOWN_RIGHT_ARROW},
    {T4Elem::CROSS, AutowareElem::CROSS}});
const std::unordered_map<T4Elem::_status_type, AutowareElem::_status_type> status_map(
  {{T4Elem::SOLID_OFF, AutowareElem::SOLID_OFF},
    {T4Elem::SOLID_ON, AutowareElem::SOLID_ON},
    {T4Elem::FLASHING, AutowareElem::FLASHING}});

int compareRecord(
  const autoware::traffic_light::FusionRecord & r1,
  const autoware::traffic_light::FusionRecord & r2)
{
  /*
  r1 will be checked
  return 1 if r1 is better than r2
  return -1 if r1 is worse than r2
  return 0 if r1 is equal to r2
  */
  /*
  if both records are from the same sensor but different stamp, trust the latest one
  */
  double t1 = rclcpp::Time(r1.header.stamp).seconds();
  double t2 = rclcpp::Time(r2.header.stamp).seconds();
  const double dt_thres = 1e-3;
  if (r1.header.frame_id == r2.header.frame_id && std::abs(t1 - t2) >= dt_thres) {
    return t1 < t2 ? -1 : 1;
  }
  bool r1_is_unknown = isUnknown(r1.signal);
  bool r2_is_unknown = isUnknown(r2.signal);
  /*
  if both are unknown, they are of the same priority
  */
  if (r1_is_unknown && r2_is_unknown) {
    return 0;
  } else if (r1_is_unknown ^ r2_is_unknown) {
    /*
    if either is unknown, the unknown is of lower priority
    */
    return r1_is_unknown ? -1 : 1;
  }
  int visible_score_1 = calVisibleScore(r1);
  int visible_score_2 = calVisibleScore(r2);
  if (visible_score_1 == visible_score_2) {
    double confidence_1 = r1.signal.elements[0].confidence;
    double confidence_2 = r2.signal.elements[0].confidence;
    return confidence_1 < confidence_2 ? -1 : 1;
  } else {
    return visible_score_1 < visible_score_2 ? -1 : 1;
  }
}

autoware_perception_msgs::msg::TrafficLightElement convert(
  const tier4_perception_msgs::msg::TrafficLightElement & input)
{
  // clang-format on

  AutowareElem output;
  output.color = at_or(color_map, input.color, AutowareElem::UNKNOWN);
  output.shape = at_or(shape_map, input.shape, AutowareElem::UNKNOWN);
  output.status = at_or(status_map, input.status, AutowareElem::UNKNOWN);
  output.confidence = input.confidence;
  return output;
}

int calVisibleScore(const autoware::traffic_light::FusionRecord & record)
{
  const uint32_t boundary = 5;
  uint32_t x1 = record.roi.roi.x_offset;
  uint32_t x2 = record.roi.roi.x_offset + record.roi.roi.width;
  uint32_t y1 = record.roi.roi.y_offset;
  uint32_t y2 = record.roi.roi.y_offset + record.roi.roi.height;
  if (
    x1 <= boundary || (record.cam_info.width - x2) <= boundary || y1 <= boundary ||
    (record.cam_info.height - y2) <= boundary) {
    return 0;
  } else {
    return 1;
  }
}

}  // namespace utils
}  // namespace autoware::traffic_light
