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

#include "../src/traffic_light_multi_camera_fusion_utils.hpp"

#include <gtest/gtest.h>

TEST(isUnknown, normal)
{
  tier4_perception_msgs::msg::TrafficLight signal;
  tier4_perception_msgs::msg::TrafficLightElement element;
  {
    element.color = tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
    signal.elements.push_back(element);
  }
  EXPECT_TRUE(autoware::traffic_light::utils::isUnknown(signal));

  {
    signal.elements.clear();
    element.color = tier4_perception_msgs::msg::TrafficLightElement::RED;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    signal.elements.push_back(element);
  }
  EXPECT_FALSE(autoware::traffic_light::utils::isUnknown(signal));
}