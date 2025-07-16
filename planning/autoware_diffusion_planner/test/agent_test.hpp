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

#ifndef AGENT_TEST_HPP_
#define AGENT_TEST_HPP_

#include "autoware/diffusion_planner/conversion/agent.hpp"

#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

namespace autoware::diffusion_planner::test
{
using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjects;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Vector3;

class AgentTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize a sample TrackedObject
    tracked_object_.kinematics.pose_with_covariance.pose.position.x = 1.0;
    tracked_object_.kinematics.pose_with_covariance.pose.position.y = 2.0;
    tracked_object_.kinematics.pose_with_covariance.pose.position.z = 0.0;
    tracked_object_.kinematics.twist_with_covariance.twist.linear.x = 3.0;
    tracked_object_.kinematics.twist_with_covariance.twist.linear.y = 4.0;
    tracked_object_.shape.dimensions.x = 5.0;
    tracked_object_.shape.dimensions.y = 2.0;
    tracked_object_.shape.dimensions.z = 1.5;

    tracked_objects_.objects.push_back(tracked_object_);
    tracked_objects_.header.stamp.sec = 100;
    tracked_objects_.header.stamp.nanosec = 0;
  }

  TrackedObject tracked_object_;
  TrackedObjects tracked_objects_;
};

}  // namespace autoware::diffusion_planner::test

#endif  // AGENT_TEST_HPP_
