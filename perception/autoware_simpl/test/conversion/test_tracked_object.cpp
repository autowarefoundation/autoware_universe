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

#include "autoware/simpl/archetype/agent.hpp"
#include "autoware/simpl/conversion/tracked_object.hpp"

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>

#include <gtest/gtest.h>

#include <vector>

namespace autoware::simpl
{
namespace
{
std::vector<autoware_perception_msgs::msg::ObjectClassification> construct_classification()
{
  std::vector<autoware_perception_msgs::msg::ObjectClassification> classifications;
  autoware_perception_msgs::msg::ObjectClassification class1;
  class1.probability = 0.5;
  class1.label = autoware_perception_msgs::msg::ObjectClassification::CAR;
  classifications.emplace_back(class1);

  autoware_perception_msgs::msg::ObjectClassification class2;
  class2.probability = 1.0;
  class2.label = autoware_perception_msgs::msg::ObjectClassification::BUS;
  classifications.emplace_back(class2);

  return classifications;
}

autoware_perception_msgs::msg::TrackedObject construct_object()
{
  autoware_perception_msgs::msg::TrackedObject object;

  // classification
  object.classification = construct_classification();

  // position
  object.kinematics.pose_with_covariance.pose.position.x = 1.0;
  object.kinematics.pose_with_covariance.pose.position.y = 1.0;
  object.kinematics.pose_with_covariance.pose.position.z = 1.0;

  // orientation
  object.kinematics.pose_with_covariance.pose.orientation.w = 1.0;
  object.kinematics.pose_with_covariance.pose.orientation.x = 0.0;
  object.kinematics.pose_with_covariance.pose.orientation.y = 0.0;
  object.kinematics.pose_with_covariance.pose.orientation.z = 0.0;

  // velocity
  object.kinematics.twist_with_covariance.twist.linear.x = 1.0;
  object.kinematics.twist_with_covariance.twist.linear.y = 1.0;
  object.kinematics.twist_with_covariance.twist.linear.z = 1.0;

  return object;
}
}  // namespace

TEST(TestTrackedObject, testToLabel)
{
  const auto object = construct_object();

  const auto result = conversion::to_label(object);

  // label with the highest probability should be choosen
  EXPECT_EQ(result, archetype::AgentLabel::LARGE_VEHICLE);
}

TEST(TestTrackedObject, testToState)
{
  const auto object = construct_object();

  const auto result = conversion::to_state(object, true);

  EXPECT_DOUBLE_EQ(result.x, 1.0);
  EXPECT_DOUBLE_EQ(result.y, 1.0);
  EXPECT_DOUBLE_EQ(result.z, 1.0);
  EXPECT_DOUBLE_EQ(result.yaw, 0.0);
  EXPECT_DOUBLE_EQ(result.vx, 1.0);
  EXPECT_DOUBLE_EQ(result.vy, 1.0);
  EXPECT_EQ(result.label, archetype::AgentLabel::LARGE_VEHICLE);
  EXPECT_TRUE(result.is_valid);
}
}  // namespace autoware::simpl
