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

#include "dummy_traffic_light.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>

using autoware::dummy_traffic_light_publisher::DummyTrafficLight;
using autoware::dummy_traffic_light_publisher::Mode;
using autoware::dummy_traffic_light_publisher::TrafficLightCycle;
using autoware_perception_msgs::msg::TrafficLightElement;
using autoware_perception_msgs::msg::TrafficLightGroupArray;

class DummyTrafficLightTest : public ::testing::Test
{
protected:
  static constexpr double kGreenDuration = 30.0;
  static constexpr double kYellowDuration = 3.0;
  static constexpr double kRedDuration = 30.0;
  static constexpr double kPassthroughTimeout = 1.0;

  std::unique_ptr<DummyTrafficLight> createStandalone()
  {
    return std::make_unique<DummyTrafficLight>(
      DummyTrafficLight::Config{Mode::Standalone, kPassthroughTimeout},
      std::make_unique<TrafficLightCycle>(kGreenDuration, kYellowDuration, kRedDuration));
  }

  std::unique_ptr<DummyTrafficLight> createEmpty()
  {
    return std::make_unique<DummyTrafficLight>(
      DummyTrafficLight::Config{Mode::Empty, kPassthroughTimeout},
      std::make_unique<TrafficLightCycle>(kGreenDuration, kYellowDuration, kRedDuration));
  }

  std::unique_ptr<DummyTrafficLight> createFixed(uint8_t color)
  {
    return std::make_unique<DummyTrafficLight>(
      DummyTrafficLight::Config{Mode::Fixed, kPassthroughTimeout, color},
      std::make_unique<TrafficLightCycle>(kGreenDuration, kYellowDuration, kRedDuration));
  }

  rclcpp::Time timeFromSec(double seconds) const
  {
    return rclcpp::Time(static_cast<int64_t>(seconds * 1e9));
  }
};

TEST_F(DummyTrafficLightTest, EmptyModePublishesEmptyMessage)
{
  auto logic = createEmpty();
  const auto msg = logic->create_message(timeFromSec(0.0));
  EXPECT_TRUE(msg.traffic_light_groups.empty());
}

TEST_F(DummyTrafficLightTest, StandaloneWithoutIdsPublishesEmpty)
{
  auto logic = createStandalone();
  const auto msg = logic->create_message(timeFromSec(0.0));
  EXPECT_TRUE(msg.traffic_light_groups.empty());
}

TEST_F(DummyTrafficLightTest, StandaloneWithIdsPublishesGroups)
{
  auto logic = createStandalone();
  logic->set_traffic_light_ids({100, 200, 300});

  const auto msg = logic->create_message(timeFromSec(0.0));
  ASSERT_EQ(msg.traffic_light_groups.size(), 3u);
  EXPECT_EQ(msg.traffic_light_groups[0].traffic_light_group_id, 100);
  EXPECT_EQ(msg.traffic_light_groups[1].traffic_light_group_id, 200);
  EXPECT_EQ(msg.traffic_light_groups[2].traffic_light_group_id, 300);

  // All groups should have the same element (initial = GREEN)
  for (const auto & group : msg.traffic_light_groups) {
    ASSERT_EQ(group.elements.size(), 1u);
    EXPECT_EQ(group.elements[0].color, TrafficLightElement::GREEN);
    EXPECT_EQ(group.elements[0].shape, TrafficLightElement::CIRCLE);
    EXPECT_EQ(group.elements[0].status, TrafficLightElement::SOLID_ON);
  }
  EXPECT_EQ(logic->traffic_light_count(), 3u);
}

TEST_F(DummyTrafficLightTest, FixedModePublishesFixedColorForAllIds)
{
  auto logic = createFixed(TrafficLightElement::RED);
  logic->set_traffic_light_ids({100, 200, 300});

  const auto msg = logic->create_message(timeFromSec(0.0));
  ASSERT_EQ(msg.traffic_light_groups.size(), 3u);
  for (const auto & group : msg.traffic_light_groups) {
    ASSERT_EQ(group.elements.size(), 1u);
    EXPECT_EQ(group.elements[0].color, TrafficLightElement::RED);
    EXPECT_EQ(group.elements[0].shape, TrafficLightElement::CIRCLE);
    EXPECT_EQ(group.elements[0].status, TrafficLightElement::SOLID_ON);
  }
}

TEST_F(DummyTrafficLightTest, FixedModeWithoutIdsPublishesEmpty)
{
  auto logic = createFixed(TrafficLightElement::GREEN);
  const auto msg = logic->create_message(timeFromSec(0.0));
  EXPECT_TRUE(msg.traffic_light_groups.empty());
}

TEST_F(DummyTrafficLightTest, FixedModeColorDoesNotChangeOverTime)
{
  auto logic = createFixed(TrafficLightElement::GREEN);
  logic->set_traffic_light_ids({1});

  // Times that would move a standalone cycle into other phases must not affect the fixed color.
  const double times[] = {0.0, 31.0, 35.0, 100.0};
  for (const double sec : times) {
    const auto msg = logic->create_message(timeFromSec(sec));
    ASSERT_EQ(msg.traffic_light_groups.size(), 1u);
    EXPECT_EQ(msg.traffic_light_groups[0].elements[0].color, TrafficLightElement::GREEN);
  }
}

TEST_F(DummyTrafficLightTest, PassthroughRelaysInput)
{
  auto logic = createStandalone();

  TrafficLightGroupArray input;
  input.stamp = timeFromSec(1.0);
  autoware_perception_msgs::msg::TrafficLightGroup group;
  group.traffic_light_group_id = 42;
  TrafficLightElement element;
  element.color = TrafficLightElement::RED;
  element.shape = TrafficLightElement::CIRCLE;
  element.status = TrafficLightElement::SOLID_ON;
  element.confidence = 1.0;
  group.elements.push_back(element);
  input.traffic_light_groups.push_back(group);

  logic->update_input_signals(input, timeFromSec(1.0));

  const auto msg = logic->create_message(timeFromSec(1.5));
  ASSERT_EQ(msg.traffic_light_groups.size(), 1u);
  EXPECT_EQ(msg.traffic_light_groups[0].traffic_light_group_id, 42);
  EXPECT_EQ(msg.traffic_light_groups[0].elements[0].color, TrafficLightElement::RED);
  // Passthrough returns the original input stamp, not the requested time
  EXPECT_EQ(msg.stamp, timeFromSec(1.0));
}

TEST_F(DummyTrafficLightTest, PassthroughRelaysInputMultipleCalls)
{
  auto logic = createStandalone();

  TrafficLightGroupArray input;
  input.stamp = timeFromSec(1.0);
  autoware_perception_msgs::msg::TrafficLightGroup group;
  group.traffic_light_group_id = 50;
  input.traffic_light_groups.push_back(group);

  logic->update_input_signals(input, timeFromSec(1.0));

  // Both calls within timeout should return the same input
  const auto msg1 = logic->create_message(timeFromSec(1.3));
  const auto msg2 = logic->create_message(timeFromSec(1.6));
  ASSERT_EQ(msg1.traffic_light_groups.size(), 1u);
  ASSERT_EQ(msg2.traffic_light_groups.size(), 1u);
  EXPECT_EQ(msg1.traffic_light_groups[0].traffic_light_group_id, 50);
  EXPECT_EQ(msg2.traffic_light_groups[0].traffic_light_group_id, 50);
}

TEST_F(DummyTrafficLightTest, PassthroughTimeoutFallsBack)
{
  auto logic = createStandalone();

  TrafficLightGroupArray input;
  input.stamp = timeFromSec(1.0);
  autoware_perception_msgs::msg::TrafficLightGroup group;
  group.traffic_light_group_id = 99;
  input.traffic_light_groups.push_back(group);

  logic->update_input_signals(input, timeFromSec(1.0));

  // Within timeout: passthrough
  const auto msg_within = logic->create_message(timeFromSec(1.5));
  ASSERT_EQ(msg_within.traffic_light_groups.size(), 1u);
  EXPECT_EQ(msg_within.traffic_light_groups[0].traffic_light_group_id, 99);

  // After timeout: falls back to standalone (no IDs = empty)
  const auto msg_after = logic->create_message(timeFromSec(2.5));
  EXPECT_TRUE(msg_after.traffic_light_groups.empty());
}

TEST_F(DummyTrafficLightTest, EmptyModeStillAllowsPassthrough)
{
  auto logic = createEmpty();

  TrafficLightGroupArray input;
  input.stamp = timeFromSec(1.0);
  autoware_perception_msgs::msg::TrafficLightGroup group;
  group.traffic_light_group_id = 10;
  input.traffic_light_groups.push_back(group);

  logic->update_input_signals(input, timeFromSec(1.0));

  // Even in empty mode, passthrough takes priority within timeout
  const auto msg_within = logic->create_message(timeFromSec(1.5));
  ASSERT_EQ(msg_within.traffic_light_groups.size(), 1u);

  // After timeout, empty mode returns empty
  const auto msg_after = logic->create_message(timeFromSec(2.5));
  EXPECT_TRUE(msg_after.traffic_light_groups.empty());
}

TEST_F(DummyTrafficLightTest, MessageStampMatchesRequestedTime)
{
  auto logic = createEmpty();
  const auto now = timeFromSec(5.0);
  const auto msg = logic->create_message(now);
  EXPECT_EQ(msg.stamp, now);
}

TEST_F(DummyTrafficLightTest, SetIdsOverwritesPrevious)
{
  auto logic = createStandalone();
  logic->set_traffic_light_ids({1, 2, 3});
  logic->set_traffic_light_ids({10, 20});

  const auto msg = logic->create_message(timeFromSec(0.0));
  ASSERT_EQ(msg.traffic_light_groups.size(), 2u);
  EXPECT_EQ(msg.traffic_light_groups[0].traffic_light_group_id, 10);
  EXPECT_EQ(msg.traffic_light_groups[1].traffic_light_group_id, 20);
}

TEST_F(DummyTrafficLightTest, SetIdsToEmptyFallsBackToEmpty)
{
  auto logic = createStandalone();
  logic->set_traffic_light_ids({1, 2});

  const auto msg1 = logic->create_message(timeFromSec(0.0));
  ASSERT_EQ(msg1.traffic_light_groups.size(), 2u);

  logic->set_traffic_light_ids({});

  const auto msg2 = logic->create_message(timeFromSec(1.0));
  EXPECT_TRUE(msg2.traffic_light_groups.empty());
}

TEST_F(DummyTrafficLightTest, PassthroughInputOverwrittenByNewInput)
{
  auto logic = createStandalone();

  TrafficLightGroupArray input1;
  input1.stamp = timeFromSec(1.0);
  autoware_perception_msgs::msg::TrafficLightGroup group1;
  group1.traffic_light_group_id = 10;
  input1.traffic_light_groups.push_back(group1);

  logic->update_input_signals(input1, timeFromSec(1.0));

  TrafficLightGroupArray input2;
  input2.stamp = timeFromSec(1.5);
  autoware_perception_msgs::msg::TrafficLightGroup group2;
  group2.traffic_light_group_id = 20;
  input2.traffic_light_groups.push_back(group2);

  logic->update_input_signals(input2, timeFromSec(1.5));

  // New input replaces old, and timeout resets from 1.5s
  const auto msg = logic->create_message(timeFromSec(2.0));
  ASSERT_EQ(msg.traffic_light_groups.size(), 1u);
  EXPECT_EQ(msg.traffic_light_groups[0].traffic_light_group_id, 20);
}

TEST_F(DummyTrafficLightTest, StandaloneColorChangesOverTime)
{
  auto logic = createStandalone();
  logic->set_traffic_light_ids({1});

  const auto msg_green = logic->create_message(timeFromSec(0.0));
  EXPECT_EQ(msg_green.traffic_light_groups[0].elements[0].color, TrafficLightElement::GREEN);

  const auto msg_yellow = logic->create_message(timeFromSec(30.0));
  EXPECT_EQ(msg_yellow.traffic_light_groups[0].elements[0].color, TrafficLightElement::AMBER);

  const auto msg_red = logic->create_message(timeFromSec(33.0));
  EXPECT_EQ(msg_red.traffic_light_groups[0].elements[0].color, TrafficLightElement::RED);
}

TEST_F(DummyTrafficLightTest, PassthroughExactTimeoutExpires)
{
  auto logic = createStandalone();

  TrafficLightGroupArray input;
  input.stamp = timeFromSec(1.0);
  autoware_perception_msgs::msg::TrafficLightGroup group;
  group.traffic_light_group_id = 5;
  input.traffic_light_groups.push_back(group);

  logic->update_input_signals(input, timeFromSec(1.0));

  // elapsed == passthrough_timeout (1.0s): condition is '<', so this should timeout
  const auto msg = logic->create_message(timeFromSec(2.0));
  EXPECT_TRUE(msg.traffic_light_groups.empty());
}

TEST_F(DummyTrafficLightTest, StandaloneMessageStampMatchesNow)
{
  auto logic = createStandalone();
  logic->set_traffic_light_ids({1});

  const auto now = timeFromSec(5.0);
  const auto msg = logic->create_message(now);
  EXPECT_EQ(msg.stamp, now);
}
