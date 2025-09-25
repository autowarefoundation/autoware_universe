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

#ifndef TRAFFIC_LIGHT_MULTI_CAMERA_FUSION_PROCESS_HPP_
#define TRAFFIC_LIGHT_MULTI_CAMERA_FUSION_PROCESS_HPP_

#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/traffic_light_element.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>
#include <tier4_perception_msgs/msg/traffic_light.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_element.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <unordered_map>

namespace autoware::traffic_light
{
namespace utils
{

struct FusionRecord
{
  std_msgs::msg::Header header;
  sensor_msgs::msg::CameraInfo cam_info;
  tier4_perception_msgs::msg::TrafficLightRoi roi;
  tier4_perception_msgs::msg::TrafficLight signal;
};

struct FusionRecordArr
{
  std_msgs::msg::Header header;
  sensor_msgs::msg::CameraInfo cam_info;
  tier4_perception_msgs::msg::TrafficLightRoiArray rois;
  tier4_perception_msgs::msg::TrafficLightArray signals;
  bool operator<(const FusionRecordArr & array) const
  {
    return rclcpp::Time(header.stamp) < rclcpp::Time(array.header.stamp);
  }
};

inline bool isUnknown(const tier4_perception_msgs::msg::TrafficLight & signal)
{
  return signal.elements.size() == 1 &&
         signal.elements[0].color == tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN &&
         signal.elements[0].shape == tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
}

template <class K, class V>
V at_or(const std::unordered_map<K, V> & map, const K & key, const V & value)
{
  return map.count(key) ? map.at(key) : value;
}

int compareRecord(const FusionRecord & r1, const FusionRecord & r2);

autoware_perception_msgs::msg::TrafficLightElement convertT4toAutoware(
  const tier4_perception_msgs::msg::TrafficLightElement & input);

/**
 * @brief Currently the visible score only considers the truncation.
 * If the detection roi is very close to the image boundary, it would be considered as truncated.
 *
 * @param record    fusion record
 * @return 0 if traffic light is truncated, otherwise 1
 */
int calVisibleScore(const FusionRecord & record);

}  // namespace utils
}  // namespace autoware::traffic_light

#endif  // TRAFFIC_LIGHT_MULTI_CAMERA_FUSION_PROCESS_HPP_
