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

#include "agent_test.hpp"

#include <autoware_utils/ros/uuid_helper.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

namespace autoware::diffusion_planner::test
{

namespace
{
TrackedObject make_object(
  const unique_identifier_msgs::msg::UUID & uuid, const double x, const double yaw, const double vx,
  const double vy = 0.0)
{
  TrackedObject object;
  object.object_id = uuid;
  object.kinematics.pose_with_covariance.pose.position.x = x;
  object.kinematics.pose_with_covariance.pose.orientation =
    autoware_utils_geometry::create_quaternion_from_yaw(yaw);
  object.kinematics.twist_with_covariance.twist.linear.x = vx;
  object.kinematics.twist_with_covariance.twist.linear.y = vy;
  object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  autoware_perception_msgs::msg::ObjectClassification classification;
  classification.label = autoware_perception_msgs::msg::ObjectClassification::CAR;
  classification.probability = 0.9;
  object.classification.push_back(classification);
  return object;
}
}  // namespace

// Velocity features carry the signed body-frame twist rotated into the pose frame: a reversing
// vehicle keeps its heading while its velocity points backward.
TEST(AgentStateEncodingTest, ReversingVehicleVelocityOpposesHeading)
{
  const auto uuid = autoware_utils_uuid::generate_uuid();
  const AgentState state(make_object(uuid, 0.0, 0.0, -2.0), rclcpp::Time(100, 0));

  const auto array = state.as_array();
  EXPECT_NEAR(array[2], 1.0, 1e-6);   // cos_yaw
  EXPECT_NEAR(array[3], 0.0, 1e-6);   // sin_yaw
  EXPECT_NEAR(array[4], -2.0, 1e-6);  // velocity_x
  EXPECT_NEAR(array[5], 0.0, 1e-6);   // velocity_y
}

// The two heading re-expressions (yaw, +v) and (yaw + pi, -v) describe the same motion: velocity
// features coincide while the heading features negate.
TEST(AgentStateEncodingTest, FlippedReexpressionKeepsVelocityFeatures)
{
  const auto uuid = autoware_utils_uuid::generate_uuid();
  const double yaw = 0.7;
  const AgentState forward(make_object(uuid, 0.0, yaw, 5.0, 1.0), rclcpp::Time(100, 0));
  const AgentState flipped(make_object(uuid, 0.0, yaw + M_PI, -5.0, -1.0), rclcpp::Time(100, 0));

  const auto a = forward.as_array();
  const auto b = flipped.as_array();
  EXPECT_NEAR(a[2], -b[2], 1e-6);  // cos_yaw
  EXPECT_NEAR(a[3], -b[3], 1e-6);  // sin_yaw
  EXPECT_NEAR(a[4], b[4], 1e-6);   // velocity_x
  EXPECT_NEAR(a[5], b[5], 1e-6);   // velocity_y
}

// An ingested heading flip is a tracker orientation correction: every buffered state is
// re-expressed to the incoming convention, so the stored history holds one coherent heading.
TEST(AgentHistoryFlipTest, FlipIngestReexpressesBufferedStates)
{
  const auto uuid = autoware_utils_uuid::generate_uuid();
  AgentHistory history(NEIGHBOR_HISTORY_BUFFER_SIZE);
  history.update(make_object(uuid, 0.0, 0.0, 5.0), rclcpp::Time(100, 0));
  history.update(make_object(uuid, 0.5, 0.0, 5.0), rclcpp::Time(100, 100000000));
  // Flipped re-expression of the same motion arrives: heading rotated by pi, speed negated.
  history.update(make_object(uuid, 1.0, M_PI, -5.0), rclcpp::Time(100, 200000000));

  const auto & states = history.states();
  ASSERT_EQ(states.size(), 3u);
  for (const auto & state : states) {
    EXPECT_NEAR(state.pose(0, 0), -1.0, 1e-6);  // yaw = pi
    EXPECT_NEAR(state.original_info.kinematics.twist_with_covariance.twist.linear.x, -5.0, 1e-6);
  }
  // Positions and timestamps are untouched by the re-expression.
  EXPECT_NEAR(states[0].pose(0, 3), 0.0, 1e-6);
  EXPECT_NEAR(states[1].pose(0, 3), 0.5, 1e-6);
  EXPECT_EQ(states[0].timestamp, rclcpp::Time(100, 0));
}

// A genuine reverse observation (no heading jump) leaves the buffered states as they are.
TEST(AgentHistoryFlipTest, ReverseMotionWithoutFlipKeepsBufferedStates)
{
  const auto uuid = autoware_utils_uuid::generate_uuid();
  AgentHistory history(NEIGHBOR_HISTORY_BUFFER_SIZE);
  history.update(make_object(uuid, 0.0, 0.0, 5.0), rclcpp::Time(100, 0));
  history.update(make_object(uuid, 0.4, 0.0, -0.5), rclcpp::Time(100, 100000000));

  const auto & states = history.states();
  ASSERT_EQ(states.size(), 2u);
  EXPECT_NEAR(states[0].pose(0, 0), 1.0, 1e-6);  // yaw = 0
  EXPECT_NEAR(states[0].original_info.kinematics.twist_with_covariance.twist.linear.x, 5.0, 1e-6);
  EXPECT_NEAR(states[1].original_info.kinematics.twist_with_covariance.twist.linear.x, -0.5, 1e-6);
}

}  // namespace autoware::diffusion_planner::test
