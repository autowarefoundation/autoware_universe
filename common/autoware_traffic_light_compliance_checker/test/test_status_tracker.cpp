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

#include "autoware/traffic_light_compliance_checker/traffic_light_status_tracker.hpp"

#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/traffic_light_element.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>

#include <gtest/gtest.h>

namespace
{
using autoware::traffic_light_compliance_checker::AmberState;
using autoware::traffic_light_compliance_checker::StatusTrackerParameters;
using autoware::traffic_light_compliance_checker::TrafficLightStatusTracker;
using autoware_perception_msgs::msg::TrafficLightElement;
using autoware_perception_msgs::msg::TrafficLightGroup;
using autoware_perception_msgs::msg::TrafficLightGroupArray;

TrafficLightElement make_element(uint8_t color, uint8_t shape = TrafficLightElement::CIRCLE)
{
  TrafficLightElement element;
  element.color = color;
  element.shape = shape;
  element.status = TrafficLightElement::SOLID_ON;
  element.confidence = 1.0f;
  return element;
}

TrafficLightGroupArray make_signals(int64_t id, const TrafficLightElement & element)
{
  TrafficLightGroup group;
  group.traffic_light_group_id = id;
  group.elements.push_back(element);

  TrafficLightGroupArray signals;
  signals.traffic_light_groups.push_back(group);
  return signals;
}

StatusTrackerParameters make_params()
{
  StatusTrackerParameters params{};
  params.stable_duration_threshold_red = 0.0;
  params.stable_duration_threshold_amber = 0.0;
  params.stable_duration_threshold_unknown = 0.0;
  return params;
}

void update_tracker(
  TrafficLightStatusTracker & tracker, const TrafficLightGroupArray & signals,
  const rclcpp::Time & time)
{
  [[maybe_unused]] const auto filtered = tracker.filter_signals(signals, time, false);
}
}  // namespace

TEST(TrafficLightStatusTrackerTest, GreenToAmberSetsFromGreen)
{
  TrafficLightStatusTracker tracker(make_params());
  const rclcpp::Time t0(100, 0, RCL_ROS_TIME);
  const rclcpp::Time t1(100, 100000000, RCL_ROS_TIME);
  constexpr int64_t id = 42;

  update_tracker(tracker, make_signals(id, make_element(TrafficLightElement::GREEN)), t0);
  EXPECT_EQ(tracker.get_amber_transition_state(id), AmberState::kNotAmber);

  update_tracker(tracker, make_signals(id, make_element(TrafficLightElement::AMBER)), t1);
  EXPECT_EQ(tracker.get_amber_transition_state(id), AmberState::kFromGreen);
}

TEST(TrafficLightStatusTrackerTest, RedToAmberSetsFromNonGreen)
{
  TrafficLightStatusTracker tracker(make_params());
  const rclcpp::Time t0(100, 0, RCL_ROS_TIME);
  const rclcpp::Time t1(100, 100000000, RCL_ROS_TIME);
  constexpr int64_t id = 42;

  update_tracker(tracker, make_signals(id, make_element(TrafficLightElement::RED)), t0);
  update_tracker(tracker, make_signals(id, make_element(TrafficLightElement::AMBER)), t1);
  EXPECT_EQ(tracker.get_amber_transition_state(id), AmberState::kFromNonGreen);
}

TEST(TrafficLightStatusTrackerTest, AmberWithoutHistoryStaysNotAmber)
{
  TrafficLightStatusTracker tracker(make_params());
  const rclcpp::Time t0(100, 0, RCL_ROS_TIME);
  constexpr int64_t id = 42;

  update_tracker(tracker, make_signals(id, make_element(TrafficLightElement::AMBER)), t0);
  EXPECT_EQ(tracker.get_amber_transition_state(id), AmberState::kNotAmber);
}

TEST(TrafficLightStatusTrackerTest, StateResetsWhenAmberEnds)
{
  TrafficLightStatusTracker tracker(make_params());
  const rclcpp::Time t0(100, 0, RCL_ROS_TIME);
  const rclcpp::Time t1(100, 100000000, RCL_ROS_TIME);
  const rclcpp::Time t2(100, 200000000, RCL_ROS_TIME);
  constexpr int64_t id = 42;

  update_tracker(tracker, make_signals(id, make_element(TrafficLightElement::GREEN)), t0);
  update_tracker(tracker, make_signals(id, make_element(TrafficLightElement::AMBER)), t1);
  EXPECT_EQ(tracker.get_amber_transition_state(id), AmberState::kFromGreen);

  update_tracker(tracker, make_signals(id, make_element(TrafficLightElement::RED)), t2);
  EXPECT_EQ(tracker.get_amber_transition_state(id), AmberState::kNotAmber);
}

TEST(TrafficLightStatusTrackerTest, AmberStatePersistsAcrossFrames)
{
  TrafficLightStatusTracker tracker(make_params());
  const rclcpp::Time t0(100, 0, RCL_ROS_TIME);
  const rclcpp::Time t1(100, 100000000, RCL_ROS_TIME);
  const rclcpp::Time t2(100, 200000000, RCL_ROS_TIME);
  constexpr int64_t id = 42;

  update_tracker(tracker, make_signals(id, make_element(TrafficLightElement::GREEN)), t0);
  update_tracker(tracker, make_signals(id, make_element(TrafficLightElement::AMBER)), t1);
  update_tracker(tracker, make_signals(id, make_element(TrafficLightElement::AMBER)), t2);
  EXPECT_EQ(tracker.get_amber_transition_state(id), AmberState::kFromGreen);
}
