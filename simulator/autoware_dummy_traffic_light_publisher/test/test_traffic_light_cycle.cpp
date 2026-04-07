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

#include "autoware/dummy_traffic_light_publisher/traffic_light_cycle.hpp"

#include <gtest/gtest.h>

using autoware::dummy_traffic_light_publisher::TrafficLightCycle;
using autoware_perception_msgs::msg::TrafficLightElement;

class TrafficLightCycleTest : public ::testing::Test
{
protected:
  static constexpr double kGreenDuration = 30.0;
  static constexpr double kYellowDuration = 3.0;
  static constexpr double kRedDuration = 30.0;

  TrafficLightCycle cycle{kGreenDuration, kYellowDuration, kRedDuration};

  rclcpp::Time makeTime(double seconds) const
  {
    return rclcpp::Time(static_cast<int64_t>(seconds * 1e9));
  }
};

TEST_F(TrafficLightCycleTest, InitialOutputIsGreen)
{
  const auto element = cycle.update(makeTime(0.0));
  EXPECT_EQ(element.color, TrafficLightElement::GREEN);
  EXPECT_EQ(element.shape, TrafficLightElement::CIRCLE);
  EXPECT_EQ(element.status, TrafficLightElement::SOLID_ON);
  EXPECT_DOUBLE_EQ(element.confidence, 1.0);
}

TEST_F(TrafficLightCycleTest, StaysGreenBeforeDuration)
{
  cycle.update(makeTime(0.0));
  const auto element = cycle.update(makeTime(kGreenDuration - 0.1));
  EXPECT_EQ(element.color, TrafficLightElement::GREEN);
}

TEST_F(TrafficLightCycleTest, TransitionsToYellow)
{
  cycle.update(makeTime(0.0));
  const auto element = cycle.update(makeTime(kGreenDuration));
  EXPECT_EQ(element.color, TrafficLightElement::AMBER);
}

TEST_F(TrafficLightCycleTest, TransitionsToRed)
{
  cycle.update(makeTime(0.0));
  cycle.update(makeTime(kGreenDuration));
  const auto element = cycle.update(makeTime(kGreenDuration + kYellowDuration));
  EXPECT_EQ(element.color, TrafficLightElement::RED);
}

TEST_F(TrafficLightCycleTest, CyclesBackToGreen)
{
  cycle.update(makeTime(0.0));
  cycle.update(makeTime(kGreenDuration));
  cycle.update(makeTime(kGreenDuration + kYellowDuration));
  const auto element = cycle.update(makeTime(kGreenDuration + kYellowDuration + kRedDuration));
  EXPECT_EQ(element.color, TrafficLightElement::GREEN);
}

TEST_F(TrafficLightCycleTest, SecondCycleWorks)
{
  const double full_cycle = kGreenDuration + kYellowDuration + kRedDuration;
  cycle.update(makeTime(0.0));
  cycle.update(makeTime(kGreenDuration));
  cycle.update(makeTime(kGreenDuration + kYellowDuration));
  cycle.update(makeTime(full_cycle));
  const auto element = cycle.update(makeTime(full_cycle + kGreenDuration));
  EXPECT_EQ(element.color, TrafficLightElement::AMBER);
}
