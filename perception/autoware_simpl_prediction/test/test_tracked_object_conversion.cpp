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

#include "autoware/simpl_prediction/conversion/tracked_object.hpp"

#include <autoware_perception_msgs/msg/detail/object_classification__struct.hpp>
#include <autoware_perception_msgs/msg/detail/tracked_object__struct.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtest/gtest.h>

#include <cmath>

namespace autoware::simpl_prediction::test
{
namespace
{
bool is_close(double a, double b, double tol = 1e-6)
{
  return std::abs(a - b) < tol;
}
}  // namespace

using autoware::simpl_prediction::archetype::AgentLabel;
using autoware::simpl_prediction::archetype::AgentState;
using autoware::simpl_prediction::conversion::to_agent_label;
using autoware::simpl_prediction::conversion::to_agent_state;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::TrackedObject;
using nav_msgs::msg::Odometry;

TEST(TestTrackedObjectConversion, LabelConversionCar)
{
  TrackedObject obj;
  ObjectClassification classification;
  classification.label = ObjectClassification::CAR;
  classification.probability = 1.0;
  obj.classification.push_back(classification);

  EXPECT_EQ(to_agent_label(obj), AgentLabel::VEHICLE);
}

TEST(TestTracedObjectConversion, LabelConversionTruck)
{
  TrackedObject obj;
  ObjectClassification classification;
  classification.label = ObjectClassification::TRUCK;
  classification.probability = 1.0;
  obj.classification.push_back(classification);

  EXPECT_EQ(to_agent_label(obj), AgentLabel::LARGE_VEHICLE);
}

TEST(TestTrackedObjectConversion, BasicConversion)
{
  TrackedObject obj;
  obj.kinematics.pose_with_covariance.pose.position.x = 1.0;
  obj.kinematics.pose_with_covariance.pose.position.y = 2.0;
  obj.kinematics.pose_with_covariance.pose.position.z = 3.0;
  obj.kinematics.pose_with_covariance.pose.orientation.w = 1.0;

  obj.kinematics.twist_with_covariance.twist.linear.x = 4.0;
  obj.kinematics.twist_with_covariance.twist.linear.y = 0.0;

  ObjectClassification classification;
  classification.label = ObjectClassification::CAR;
  classification.probability = 1.0;
  obj.classification.push_back(classification);

  AgentState state = to_agent_state(obj);

  EXPECT_TRUE(is_close(state.x, 1.0));
  EXPECT_TRUE(is_close(state.y, 2.0));
  EXPECT_TRUE(is_close(state.z, 3.0));
  EXPECT_TRUE(is_close(state.vx, 4.0));  // no rotation
  EXPECT_TRUE(is_close(state.vy, 0.0));
  EXPECT_TRUE(is_close(state.yaw, 0.0));
  EXPECT_TRUE(state.is_valid);
}

TEST(TestOdometryConversion, BasicConversion)
{
  Odometry odom;
  odom.pose.pose.position.x = -1.0;
  odom.pose.pose.position.y = -2.0;
  odom.pose.pose.position.z = -3.0;
  odom.pose.pose.orientation.w = 1.0;

  odom.twist.twist.linear.x = 2.0;
  odom.twist.twist.linear.y = 0.0;

  AgentState state = to_agent_state(odom);

  EXPECT_TRUE(is_close(state.x, -1.0));
  EXPECT_TRUE(is_close(state.y, -2.0));
  EXPECT_TRUE(is_close(state.z, -3.0));
  EXPECT_TRUE(is_close(state.vx, 2.0));  // no rotation
  EXPECT_TRUE(is_close(state.vy, 0.0));
  EXPECT_TRUE(is_close(state.yaw, 0.0));
  EXPECT_TRUE(state.is_valid);
}
}  // namespace autoware::simpl_prediction::test
