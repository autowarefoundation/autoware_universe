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

#include "traffic_light_classifier_process.hpp"

#include <string>
#include <unordered_map>

namespace autoware::traffic_light
{
namespace utils
{

const std::unordered_map<tier4_perception_msgs::msg::TrafficLightElement::_color_type, std::string>
  color2string(
    {{tier4_perception_msgs::msg::TrafficLightElement::RED, "red"},
     {tier4_perception_msgs::msg::TrafficLightElement::AMBER, "yellow"},
     {tier4_perception_msgs::msg::TrafficLightElement::GREEN, "green"},
     {tier4_perception_msgs::msg::TrafficLightElement::WHITE, "white"}});

const std::unordered_map<tier4_perception_msgs::msg::TrafficLightElement::_shape_type, std::string>
  shape2string(
    {{tier4_perception_msgs::msg::TrafficLightElement::CIRCLE, "circle"},
     {tier4_perception_msgs::msg::TrafficLightElement::LEFT_ARROW, "left"},
     {tier4_perception_msgs::msg::TrafficLightElement::RIGHT_ARROW, "right"},
     {tier4_perception_msgs::msg::TrafficLightElement::UP_ARROW, "straight"},
     {tier4_perception_msgs::msg::TrafficLightElement::UP_LEFT_ARROW, "up_left"},
     {tier4_perception_msgs::msg::TrafficLightElement::UP_RIGHT_ARROW, "up_right"},
     {tier4_perception_msgs::msg::TrafficLightElement::DOWN_ARROW, "down"},
     {tier4_perception_msgs::msg::TrafficLightElement::DOWN_LEFT_ARROW, "down_left"},
     {tier4_perception_msgs::msg::TrafficLightElement::DOWN_RIGHT_ARROW, "down_right"},
     {tier4_perception_msgs::msg::TrafficLightElement::CROSS, "cross"}});

const std::unordered_map<std::string, tier4_perception_msgs::msg::TrafficLightElement::_color_type>
  string2color(
    {{"red", tier4_perception_msgs::msg::TrafficLightElement::RED},
     {"yellow", tier4_perception_msgs::msg::TrafficLightElement::AMBER},
     {"green", tier4_perception_msgs::msg::TrafficLightElement::GREEN},
     {"white", tier4_perception_msgs::msg::TrafficLightElement::WHITE}});

const std::unordered_map<std::string, tier4_perception_msgs::msg::TrafficLightElement::_shape_type>
  string2shape(
    {{"circle", tier4_perception_msgs::msg::TrafficLightElement::CIRCLE},
     {"left", tier4_perception_msgs::msg::TrafficLightElement::LEFT_ARROW},
     {"right", tier4_perception_msgs::msg::TrafficLightElement::RIGHT_ARROW},
     {"straight", tier4_perception_msgs::msg::TrafficLightElement::UP_ARROW},
     {"up_left", tier4_perception_msgs::msg::TrafficLightElement::UP_LEFT_ARROW},
     {"up_right", tier4_perception_msgs::msg::TrafficLightElement::UP_RIGHT_ARROW},
     {"down", tier4_perception_msgs::msg::TrafficLightElement::DOWN_ARROW},
     {"down_left", tier4_perception_msgs::msg::TrafficLightElement::DOWN_LEFT_ARROW},
     {"down_right", tier4_perception_msgs::msg::TrafficLightElement::DOWN_RIGHT_ARROW},
     {"cross", tier4_perception_msgs::msg::TrafficLightElement::CROSS}});

tier4_perception_msgs::msg::TrafficLightElement::_color_type convertColorStringtoT4(
  const std::string & label)
{
  return at_or(string2color, label, tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN);
}

tier4_perception_msgs::msg::TrafficLightElement::_shape_type convertShapeStringtoT4(
  const std::string & label)
{
  return at_or(string2shape, label, tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN);
}

std::string convertColorT4toString(
  const tier4_perception_msgs::msg::TrafficLightElement::_color_type & label)
{
  return at_or(color2string, label, std::string("unknown"));
}

std::string convertShapeT4toString(
  const tier4_perception_msgs::msg::TrafficLightElement::_shape_type & label)
{
  return at_or(shape2string, label, std::string("unknown"));
}

bool isColorLabel(const std::string & label)
{
  return convertColorStringtoT4(label) != tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
}

bool is_harsh_backlight(const cv::Mat & img, const double backlight_threshold)
{
  if (img.empty()) {
    return false;
  }
  cv::Mat y_cr_cb;
  cv::cvtColor(img, y_cr_cb, cv::COLOR_RGB2YCrCb);

  const cv::Scalar mean_values = cv::mean(y_cr_cb);
  const double intensity = (mean_values[0] - 112.5) / 112.5;

  return backlight_threshold <= intensity;
}

}  // namespace utils
}  // namespace autoware::traffic_light
