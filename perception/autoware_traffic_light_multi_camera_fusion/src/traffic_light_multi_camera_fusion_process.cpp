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

#include "traffic_light_multi_camera_fusion_process.hpp"

#include <rclcpp/rclcpp.hpp>

#include <unordered_map>

namespace autoware::traffic_light
{
namespace utils
{

const std::unordered_map<
  tier4_perception_msgs::msg::TrafficLightElement::_color_type,
  autoware_perception_msgs::msg::TrafficLightElement::_color_type>
  color_map(
    {{tier4_perception_msgs::msg::TrafficLightElement::RED,
      autoware_perception_msgs::msg::TrafficLightElement::RED},
     {tier4_perception_msgs::msg::TrafficLightElement::AMBER,
      autoware_perception_msgs::msg::TrafficLightElement::AMBER},
     {tier4_perception_msgs::msg::TrafficLightElement::GREEN,
      autoware_perception_msgs::msg::TrafficLightElement::GREEN},
     {tier4_perception_msgs::msg::TrafficLightElement::WHITE,
      autoware_perception_msgs::msg::TrafficLightElement::WHITE}});

const std::unordered_map<
  tier4_perception_msgs::msg::TrafficLightElement::_shape_type,
  autoware_perception_msgs::msg::TrafficLightElement::_shape_type>
  shape_map(
    {{tier4_perception_msgs::msg::TrafficLightElement::CIRCLE,
      autoware_perception_msgs::msg::TrafficLightElement::CIRCLE},
     {tier4_perception_msgs::msg::TrafficLightElement::LEFT_ARROW,
      autoware_perception_msgs::msg::TrafficLightElement::LEFT_ARROW},
     {tier4_perception_msgs::msg::TrafficLightElement::RIGHT_ARROW,
      autoware_perception_msgs::msg::TrafficLightElement::RIGHT_ARROW},
     {tier4_perception_msgs::msg::TrafficLightElement::UP_ARROW,
      autoware_perception_msgs::msg::TrafficLightElement::UP_ARROW},
     {tier4_perception_msgs::msg::TrafficLightElement::UP_LEFT_ARROW,
      autoware_perception_msgs::msg::TrafficLightElement::UP_LEFT_ARROW},
     {tier4_perception_msgs::msg::TrafficLightElement::UP_RIGHT_ARROW,
      autoware_perception_msgs::msg::TrafficLightElement::UP_RIGHT_ARROW},
     {tier4_perception_msgs::msg::TrafficLightElement::DOWN_ARROW,
      autoware_perception_msgs::msg::TrafficLightElement::DOWN_ARROW},
     {tier4_perception_msgs::msg::TrafficLightElement::DOWN_LEFT_ARROW,
      autoware_perception_msgs::msg::TrafficLightElement::DOWN_LEFT_ARROW},
     {tier4_perception_msgs::msg::TrafficLightElement::DOWN_RIGHT_ARROW,
      autoware_perception_msgs::msg::TrafficLightElement::DOWN_RIGHT_ARROW},
     {tier4_perception_msgs::msg::TrafficLightElement::CROSS,
      autoware_perception_msgs::msg::TrafficLightElement::CROSS}});

const std::unordered_map<
  tier4_perception_msgs::msg::TrafficLightElement::_status_type,
  autoware_perception_msgs::msg::TrafficLightElement::_status_type>
  status_map(
    {{tier4_perception_msgs::msg::TrafficLightElement::SOLID_OFF,
      autoware_perception_msgs::msg::TrafficLightElement::SOLID_OFF},
     {tier4_perception_msgs::msg::TrafficLightElement::SOLID_ON,
      autoware_perception_msgs::msg::TrafficLightElement::SOLID_ON},
     {tier4_perception_msgs::msg::TrafficLightElement::FLASHING,
      autoware_perception_msgs::msg::TrafficLightElement::FLASHING}});

int compareRecord(const FusionRecord & r1, const FusionRecord & r2)
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

autoware_perception_msgs::msg::TrafficLightElement convertT4toAutoware(
  const tier4_perception_msgs::msg::TrafficLightElement & input)
{
  // clang-format on

  autoware_perception_msgs::msg::TrafficLightElement output;
  output.color =
    at_or(color_map, input.color, autoware_perception_msgs::msg::TrafficLightElement::UNKNOWN);
  output.shape =
    at_or(shape_map, input.shape, autoware_perception_msgs::msg::TrafficLightElement::UNKNOWN);
  output.status =
    at_or(status_map, input.status, autoware_perception_msgs::msg::TrafficLightElement::UNKNOWN);
  output.confidence = input.confidence;
  return output;
}

int calVisibleScore(const FusionRecord & record)
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
