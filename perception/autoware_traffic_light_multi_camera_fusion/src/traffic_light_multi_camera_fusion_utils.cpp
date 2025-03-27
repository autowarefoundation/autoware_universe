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

int compareRecord(
  const autoware::traffic_light::FusionRecord & r1,
  const autoware::traffic_light::FusionRecord & r2)
{
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
  using OldElem = tier4_perception_msgs::msg::TrafficLightElement;
  using NewElem = autoware_perception_msgs::msg::TrafficLightElement;
  static const std::unordered_map<OldElem::_color_type, NewElem::_color_type> color_map(
    {{OldElem::RED, NewElem::RED},
     {OldElem::AMBER, NewElem::AMBER},
     {OldElem::GREEN, NewElem::GREEN},
     {OldElem::WHITE, NewElem::WHITE}});
  static const std::unordered_map<OldElem::_shape_type, NewElem::_shape_type> shape_map(
    {{OldElem::CIRCLE, NewElem::CIRCLE},
     {OldElem::LEFT_ARROW, NewElem::LEFT_ARROW},
     {OldElem::RIGHT_ARROW, NewElem::RIGHT_ARROW},
     {OldElem::UP_ARROW, NewElem::UP_ARROW},
     {OldElem::UP_LEFT_ARROW, NewElem::UP_LEFT_ARROW},
     {OldElem::UP_RIGHT_ARROW, NewElem::UP_RIGHT_ARROW},
     {OldElem::DOWN_ARROW, NewElem::DOWN_ARROW},
     {OldElem::DOWN_LEFT_ARROW, NewElem::DOWN_LEFT_ARROW},
     {OldElem::DOWN_RIGHT_ARROW, NewElem::DOWN_RIGHT_ARROW},
     {OldElem::CROSS, NewElem::CROSS}});
  static const std::unordered_map<OldElem::_status_type, NewElem::_status_type> status_map(
    {{OldElem::SOLID_OFF, NewElem::SOLID_OFF},
     {OldElem::SOLID_ON, NewElem::SOLID_ON},
     {OldElem::FLASHING, NewElem::FLASHING}});
  // clang-format on

  NewElem output;
  output.color = at_or(color_map, input.color, NewElem::UNKNOWN);
  output.shape = at_or(shape_map, input.shape, NewElem::UNKNOWN);
  output.status = at_or(status_map, input.status, NewElem::UNKNOWN);
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

}  // namespace
}  // namespace autoware::traffic_light
