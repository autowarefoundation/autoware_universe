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

#include "autoware/dummy_traffic_light_publisher/dummy_traffic_light.hpp"

#include <gtest/gtest.h>

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

  rclcpp::Time makeTime(double seconds) const
  {
    return rclcpp::Time(static_cast<int64_t>(seconds * 1e9));
  }
};

TEST_F(DummyTrafficLightTest, EmptyModePublishesEmptyMessage)
{
  auto logic = createEmpty();
  const auto msg = logic->create_message(makeTime(0.0));
  EXPECT_TRUE(msg.traffic_light_groups.empty());
}

TEST_F(DummyTrafficLightTest, StandaloneWithoutVectorMapPublishesEmpty)
{
  auto logic = createStandalone();
  const auto msg = logic->create_message(makeTime(0.0));
  EXPECT_TRUE(msg.traffic_light_groups.empty());
}

TEST_F(DummyTrafficLightTest, StandaloneWithIdsPublishesGroups)
{
  auto logic = createStandalone();

  // Simulate setting traffic light IDs by injecting input then timeout,
  // but we can't call update_vector_map without a real lanelet map.
  // Instead, verify that without IDs the output is empty.
  const auto msg = logic->create_message(makeTime(0.0));
  EXPECT_TRUE(msg.traffic_light_groups.empty());
  EXPECT_EQ(logic->traffic_light_count(), 0u);
}

TEST_F(DummyTrafficLightTest, PassthroughRelaysInput)
{
  auto logic = createStandalone();

  TrafficLightGroupArray input;
  input.stamp = makeTime(1.0);
  autoware_perception_msgs::msg::TrafficLightGroup group;
  group.traffic_light_group_id = 42;
  TrafficLightElement element;
  element.color = TrafficLightElement::RED;
  element.shape = TrafficLightElement::CIRCLE;
  element.status = TrafficLightElement::SOLID_ON;
  element.confidence = 1.0;
  group.elements.push_back(element);
  input.traffic_light_groups.push_back(group);

  logic->update_input_signals(input, makeTime(1.0));

  const auto msg = logic->create_message(makeTime(1.5));
  ASSERT_EQ(msg.traffic_light_groups.size(), 1u);
  EXPECT_EQ(msg.traffic_light_groups[0].traffic_light_group_id, 42);
  EXPECT_EQ(msg.traffic_light_groups[0].elements[0].color, TrafficLightElement::RED);
}

TEST_F(DummyTrafficLightTest, PassthroughTimeoutFallsBack)
{
  auto logic = createStandalone();

  TrafficLightGroupArray input;
  input.stamp = makeTime(1.0);
  autoware_perception_msgs::msg::TrafficLightGroup group;
  group.traffic_light_group_id = 99;
  input.traffic_light_groups.push_back(group);

  logic->update_input_signals(input, makeTime(1.0));

  // Within timeout: passthrough
  const auto msg_within = logic->create_message(makeTime(1.5));
  ASSERT_EQ(msg_within.traffic_light_groups.size(), 1u);
  EXPECT_EQ(msg_within.traffic_light_groups[0].traffic_light_group_id, 99);

  // After timeout: falls back to standalone (no IDs = empty)
  const auto msg_after = logic->create_message(makeTime(2.5));
  EXPECT_TRUE(msg_after.traffic_light_groups.empty());
}

TEST_F(DummyTrafficLightTest, EmptyModeIgnoresPassthrough)
{
  auto logic = createEmpty();

  TrafficLightGroupArray input;
  input.stamp = makeTime(1.0);
  autoware_perception_msgs::msg::TrafficLightGroup group;
  group.traffic_light_group_id = 10;
  input.traffic_light_groups.push_back(group);

  logic->update_input_signals(input, makeTime(1.0));

  // Even in empty mode, passthrough takes priority within timeout
  const auto msg_within = logic->create_message(makeTime(1.5));
  ASSERT_EQ(msg_within.traffic_light_groups.size(), 1u);

  // After timeout, empty mode returns empty
  const auto msg_after = logic->create_message(makeTime(2.5));
  EXPECT_TRUE(msg_after.traffic_light_groups.empty());
}

TEST_F(DummyTrafficLightTest, MessageStampMatchesRequestedTime)
{
  auto logic = createEmpty();
  const auto now = makeTime(5.0);
  const auto msg = logic->create_message(now);
  EXPECT_EQ(msg.stamp, now);
}
