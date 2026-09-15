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

#include "traffic_light_cycle.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

using autoware::dummy_traffic_light_publisher::TrafficLightCycle;
using autoware_perception_msgs::msg::TrafficLightElement;

class TrafficLightCycleTest : public ::testing::Test
{
protected:
  static constexpr double kGreenDuration = 30.0;
  static constexpr double kYellowDuration = 3.0;
  static constexpr double kRedDuration = 30.0;

  TrafficLightCycle cycle{kGreenDuration, kYellowDuration, kRedDuration};

  rclcpp::Time timeFromSec(double seconds) const
  {
    return rclcpp::Time(static_cast<int64_t>(seconds * 1e9));
  }
};

TEST_F(TrafficLightCycleTest, InitialOutputIsGreen)
{
  const auto element = cycle.update(timeFromSec(0.0));
  EXPECT_EQ(element.color, TrafficLightElement::GREEN);
  EXPECT_EQ(element.shape, TrafficLightElement::CIRCLE);
  EXPECT_EQ(element.status, TrafficLightElement::SOLID_ON);
  EXPECT_DOUBLE_EQ(element.confidence, 1.0);
}

TEST_F(TrafficLightCycleTest, StaysGreenBeforeDuration)
{
  cycle.update(timeFromSec(0.0));
  const auto element = cycle.update(timeFromSec(kGreenDuration - 0.1));
  EXPECT_EQ(element.color, TrafficLightElement::GREEN);
}

TEST_F(TrafficLightCycleTest, TransitionsToYellow)
{
  cycle.update(timeFromSec(0.0));
  const auto element = cycle.update(timeFromSec(kGreenDuration));
  EXPECT_EQ(element.color, TrafficLightElement::AMBER);
}

TEST_F(TrafficLightCycleTest, TransitionsToRed)
{
  cycle.update(timeFromSec(0.0));
  const auto element = cycle.update(timeFromSec(kGreenDuration + kYellowDuration));
  EXPECT_EQ(element.color, TrafficLightElement::RED);
}

TEST_F(TrafficLightCycleTest, CyclesBackToGreen)
{
  cycle.update(timeFromSec(0.0));
  const auto element = cycle.update(timeFromSec(kGreenDuration + kYellowDuration + kRedDuration));
  EXPECT_EQ(element.color, TrafficLightElement::GREEN);
}

TEST_F(TrafficLightCycleTest, MidGreenStaysGreen)
{
  cycle.update(timeFromSec(0.0));
  const auto element = cycle.update(timeFromSec(15.0));
  EXPECT_EQ(element.color, TrafficLightElement::GREEN);
}

TEST_F(TrafficLightCycleTest, MidYellowStaysYellow)
{
  cycle.update(timeFromSec(0.0));
  const auto element = cycle.update(timeFromSec(31.5));
  EXPECT_EQ(element.color, TrafficLightElement::AMBER);
}

TEST_F(TrafficLightCycleTest, MidRedStaysRed)
{
  cycle.update(timeFromSec(0.0));
  const auto element = cycle.update(timeFromSec(50.0));
  EXPECT_EQ(element.color, TrafficLightElement::RED);
}

TEST_F(TrafficLightCycleTest, SecondCycleGreen)
{
  const double full_cycle = kGreenDuration + kYellowDuration + kRedDuration;
  cycle.update(timeFromSec(0.0));
  const auto element = cycle.update(timeFromSec(full_cycle + 1.0));
  EXPECT_EQ(element.color, TrafficLightElement::GREEN);
}

TEST_F(TrafficLightCycleTest, SecondCycleYellow)
{
  const double full_cycle = kGreenDuration + kYellowDuration + kRedDuration;
  cycle.update(timeFromSec(0.0));
  const auto element = cycle.update(timeFromSec(full_cycle + kGreenDuration));
  EXPECT_EQ(element.color, TrafficLightElement::AMBER);
}

TEST_F(TrafficLightCycleTest, SecondCycleRed)
{
  const double full_cycle = kGreenDuration + kYellowDuration + kRedDuration;
  cycle.update(timeFromSec(0.0));
  const auto element = cycle.update(timeFromSec(full_cycle + kGreenDuration + kYellowDuration));
  EXPECT_EQ(element.color, TrafficLightElement::RED);
}

TEST_F(TrafficLightCycleTest, LargeTimeJump)
{
  cycle.update(timeFromSec(0.0));
  // 1000s = 15 full cycles + 55s remainder (55 >= 33 → Red phase)
  const auto element = cycle.update(timeFromSec(1000.0));
  EXPECT_EQ(element.color, TrafficLightElement::RED);
}

TEST_F(TrafficLightCycleTest, TimeRewindWrapsAround)
{
  cycle.update(timeFromSec(10.0));
  // Rewind: now < start_time_ → negative elapsed → corrected by adding total
  const auto element = cycle.update(timeFromSec(5.0));
  // (5 - 10) = -5, fmod(-5, 63) = -5, +63 = 58 → Red
  EXPECT_EQ(element.color, TrafficLightElement::RED);
}

TEST_F(TrafficLightCycleTest, NonZeroStartTime)
{
  cycle.update(timeFromSec(100.0));
  // 15s after start → still Green
  EXPECT_EQ(cycle.update(timeFromSec(115.0)).color, TrafficLightElement::GREEN);
  // 30s after start → Yellow
  EXPECT_EQ(cycle.update(timeFromSec(130.0)).color, TrafficLightElement::AMBER);
  // 33s after start → Red
  EXPECT_EQ(cycle.update(timeFromSec(133.0)).color, TrafficLightElement::RED);
}

TEST(TrafficLightCycleConstruction, ThrowsOnNonPositiveDuration)
{
  // Non-positive durations make the cycle total zero/negative and would break fmod, so reject them.
  EXPECT_THROW(TrafficLightCycle(0.0, 3.0, 30.0), std::invalid_argument);
  EXPECT_THROW(TrafficLightCycle(30.0, -1.0, 30.0), std::invalid_argument);
  EXPECT_THROW(TrafficLightCycle(30.0, 3.0, 0.0), std::invalid_argument);
}

TEST(TrafficLightCycleConstruction, AcceptsPositiveDurations)
{
  EXPECT_NO_THROW(TrafficLightCycle(30.0, 3.0, 30.0));
}
