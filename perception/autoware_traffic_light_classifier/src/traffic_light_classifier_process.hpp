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

#ifndef TRAFFIC_LIGHT_CLASSIFIER_PROCESS_HPP_
#define TRAFFIC_LIGHT_CLASSIFIER_PROCESS_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <tier4_perception_msgs/msg/traffic_light_element.hpp>

#include <string>
#include <unordered_map>

namespace autoware::traffic_light
{

namespace utils
{

template <class K, class V>
V at_or(const std::unordered_map<K, V> & map, const K & key, const V & value)
{
  return map.count(key) ? map.at(key) : value;
}

tier4_perception_msgs::msg::TrafficLightElement::_color_type convertColorStringtoT4(
  const std::string & label);

tier4_perception_msgs::msg::TrafficLightElement::_shape_type convertShapeStringtoT4(
  const std::string & label);

std::string convertColorT4toString(
  const tier4_perception_msgs::msg::TrafficLightElement::_color_type & label);

std::string convertShapeT4toString(
  const tier4_perception_msgs::msg::TrafficLightElement::_shape_type & label);

bool isColorLabel(const std::string & label);

bool is_harsh_backlight(const cv::Mat & img, const double backlight_threshold);

}  // namespace utils
}  // namespace autoware::traffic_light

#endif  // TRAFFIC_LIGHT_CLASSIFIER_PROCESS_HPP_
