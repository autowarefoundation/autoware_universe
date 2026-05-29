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

#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <rclcpp/time.hpp>

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

double get_min_confidence(const tier4_perception_msgs::msg::TrafficLight & signal)
{
  // Normally, we should check whether the elements are empty,
  // but we already know they are not, so we skip the check here
  return std::min_element(
           signal.elements.begin(), signal.elements.end(),
           [](const auto & a, const auto & b) { return a.confidence < b.confidence; })
    ->confidence;
}

bool has_higher_or_equal_priority(const FusionRecord & candidate, const FusionRecord & existing)
{
  /*
  Records are ranked by a fixed priority order. Ties favor the candidate so that a
  newly arrived record wins over an equally-ranked existing one.
  */

  // 1. records from the same camera are ranked by timestamp; trust the latest one
  const double candidate_time = rclcpp::Time(candidate.header.stamp).seconds();
  const double existing_time = rclcpp::Time(existing.header.stamp).seconds();
  const double dt_thres = 1e-3;
  if (
    candidate.header.frame_id == existing.header.frame_id &&
    std::abs(candidate_time - existing_time) >= dt_thres) {
    return candidate_time >= existing_time;
  }

  // 2. a recognized signal outranks an unknown one
  const bool candidate_is_unknown = is_signal_unknown(candidate.signal);
  const bool existing_is_unknown = is_signal_unknown(existing.signal);
  if (candidate_is_unknown && existing_is_unknown) {
    return true;  // both unknown -> equal priority (favor the candidate)
  }
  if (candidate_is_unknown) {
    return false;  // only the candidate is unknown -> it loses
  }
  if (existing_is_unknown) {
    return true;  // only the existing record is unknown -> the candidate wins
  }

  // 3. a fully visible signal outranks a truncated one
  const bool candidate_is_visible = is_fully_visible(candidate);
  const bool existing_is_visible = is_fully_visible(existing);
  if (candidate_is_visible != existing_is_visible) {
    return candidate_is_visible;
  }

  // 4. otherwise the higher confidence wins
  return get_min_confidence(candidate.signal) >= get_min_confidence(existing.signal);
}

autoware_perception_msgs::msg::TrafficLightElement convert_t4_to_autoware(
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

bool is_fully_visible(const FusionRecord & record)
{
  const uint32_t boundary = 5;
  const uint32_t x1 = record.roi.roi.x_offset;
  const uint32_t x2 = record.roi.roi.x_offset + record.roi.roi.width;
  const uint32_t y1 = record.roi.roi.y_offset;
  const uint32_t y2 = record.roi.roi.y_offset + record.roi.roi.height;
  const bool is_truncated = x1 <= boundary || (record.cam_info.width - x2) <= boundary ||
                            y1 <= boundary || (record.cam_info.height - y2) <= boundary;
  return !is_truncated;
}

FusionRecord generate_failsafe_record(FusionRecord base_record)
{
  // return unknown record for a fail safe
  FusionRecord fail_safe_signal;
  fail_safe_signal.header = base_record.header;
  fail_safe_signal.cam_info = base_record.cam_info;
  fail_safe_signal.roi = base_record.roi;
  traffic_light_utils::setSignalUnknown(fail_safe_signal.signal, 0.0);

  return fail_safe_signal;
}

}  // namespace utils
}  // namespace autoware::traffic_light
