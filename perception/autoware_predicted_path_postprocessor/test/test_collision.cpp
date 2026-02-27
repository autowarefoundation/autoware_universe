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

#include "autoware/predicted_path_postprocessor/processor/collision.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_path.hpp>

#include <gtest/gtest.h>

#include <array>
#include <vector>

namespace autoware::predicted_path_postprocessor::testing
{
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedPath;
using processor::find_collision;

// =============================================================================
// Test Utilities
// =============================================================================

namespace
{
constexpr double kDefaultTimeStep = 0.1;
const geometry_msgs::msg::Vector3 kDefaultDimensions = []() {
  return geometry_msgs::build<geometry_msgs::msg::Vector3>().x(2.0).y(2.0).z(1.0);
}();

unique_identifier_msgs::msg::UUID make_uuid(uint8_t id)
{
  return unique_identifier_msgs::build<unique_identifier_msgs::msg::UUID>().uuid({id});
}

geometry_msgs::msg::Pose make_pose(double x, double y)
{
  return geometry_msgs::build<geometry_msgs::msg::Pose>()
    .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(x).y(y).z(0.0))
    .orientation(
      geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.0).w(1.0));
}

PredictedObject make_target(
  uint8_t id, const std::vector<std::array<double, 2>> & waypoints,
  const geometry_msgs::msg::Vector3 & dimensions = kDefaultDimensions)
{
  PredictedObject obj;
  obj.object_id = make_uuid(id);
  obj.shape.dimensions = dimensions;

  if (!waypoints.empty()) {
    obj.kinematics.initial_pose_with_covariance.pose = make_pose(waypoints[0][0], waypoints[0][1]);
  }

  PredictedPath path;
  path.time_step = rclcpp::Duration::from_seconds(kDefaultTimeStep);
  for (const auto & wp : waypoints) {
    path.path.push_back(make_pose(wp[0], wp[1]));
  }
  obj.kinematics.predicted_paths.push_back(path);

  return obj;
}

PredictedObject make_obstacle(
  uint8_t id, double x, double y, double speed = 0.0,
  const geometry_msgs::msg::Vector3 & dimensions = kDefaultDimensions)
{
  PredictedObject obs;
  obs.object_id = make_uuid(id);
  obs.shape.dimensions = dimensions;
  obs.kinematics.initial_pose_with_covariance.pose = make_pose(x, y);
  obs.kinematics.initial_twist_with_covariance.twist.linear.x = speed;
  return obs;
}
}  // namespace

// =============================================================================
// Tests
// =============================================================================

class CollisionTest : public ::testing::Test
{
protected:
  // Standard straight path: (0,0) -> (5,0) -> (10,0) -> (15,0)
  PredictedObject target = make_target(1, {{0, 0}, {5, 0}, {10, 0}, {15, 0}});
};

// -----------------------------------------------------------------------------
// Basic collision detection
// -----------------------------------------------------------------------------

TEST_F(CollisionTest, NoCollision_WhenNoObstacles)
{
  std::vector<PredictedObject> obstacles;
  auto result = find_collision(target, 0, obstacles);
  EXPECT_FALSE(result.has_value());
}

TEST_F(CollisionTest, NoCollision_WhenObstacleFarAway)
{
  std::vector<PredictedObject> obstacles = {make_obstacle(2, 100.0, 100.0)};
  auto result = find_collision(target, 0, obstacles);
  EXPECT_FALSE(result.has_value());
}

TEST_F(CollisionTest, DetectsCollision_AtWaypoint)
{
  // Obstacle at waypoint 1 (5, 0)
  std::vector<PredictedObject> obstacles = {make_obstacle(2, 5.0, 0.0)};
  auto result = find_collision(target, 0, obstacles);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->segment_index, 1);
  EXPECT_NEAR(result->distance, 5.0, 1e-6);
}

TEST_F(CollisionTest, ReturnsFirstCollision_WhenMultipleExist)
{
  std::vector<PredictedObject> obstacles = {
    make_obstacle(2, 10.0, 0.0),  // Later collision
    make_obstacle(3, 0.0, 0.0),   // First collision (at origin)
    make_obstacle(4, 5.0, 0.0),   // Middle collision
  };
  auto result = find_collision(target, 0, obstacles);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->segment_index, 0);
  EXPECT_NEAR(result->distance, 0.0, 1e-6);
}

// -----------------------------------------------------------------------------
// Speed threshold filtering
// -----------------------------------------------------------------------------

TEST_F(CollisionTest, SkipsObstacle_WhenSpeedExceedsThreshold)
{
  // High-speed obstacle (speed=10) with threshold=5
  std::vector<PredictedObject> obstacles = {make_obstacle(2, 5.0, 0.0, /*speed=*/10.0)};
  auto result = find_collision(target, 0, obstacles, /*speed_threshold=*/5.0);
  EXPECT_FALSE(result.has_value());
}

TEST_F(CollisionTest, DetectsCollision_WhenSpeedBelowThreshold)
{
  // Low-speed obstacle (speed=2) with threshold=5
  std::vector<PredictedObject> obstacles = {make_obstacle(2, 5.0, 0.0, /*speed=*/2.0)};
  auto result = find_collision(target, 0, obstacles, /*speed_threshold=*/5.0);
  ASSERT_TRUE(result.has_value());
}

TEST_F(CollisionTest, UsesAbsoluteSpeed_ForNegativeValues)
{
  // Negative speed obstacle (speed=-10), |speed| > threshold
  std::vector<PredictedObject> obstacles = {make_obstacle(2, 5.0, 0.0, /*speed=*/-10.0)};
  auto result = find_collision(target, 0, obstacles, /*speed_threshold=*/5.0);
  EXPECT_FALSE(result.has_value());
}

// -----------------------------------------------------------------------------
// Self-collision filtering
// -----------------------------------------------------------------------------

TEST_F(CollisionTest, SkipsSelfCollision)
{
  // Obstacle with same UUID as target
  std::vector<PredictedObject> obstacles = {make_obstacle(1, 5.0, 0.0)};  // UUID = 1
  auto result = find_collision(target, 0, obstacles);
  EXPECT_FALSE(result.has_value());
}

// -----------------------------------------------------------------------------
// Edge cases
// -----------------------------------------------------------------------------

TEST_F(CollisionTest, HandlesEmptyPath)
{
  auto empty_target = make_target(1, {});
  std::vector<PredictedObject> obstacles = {make_obstacle(2, 0.0, 0.0)};
  auto result = find_collision(empty_target, 0, obstacles);
  EXPECT_FALSE(result.has_value());
}

TEST_F(CollisionTest, HandlesSingleWaypointPath)
{
  auto single_point_target = make_target(1, {{0, 0}});
  std::vector<PredictedObject> obstacles = {make_obstacle(2, 0.0, 0.0)};
  // Single waypoint = no segments, so no collision detection possible
  auto result = find_collision(single_point_target, 0, obstacles);
  EXPECT_FALSE(result.has_value());
}

TEST_F(CollisionTest, SkipsZeroLengthSegments)
{
  // Path with duplicate waypoint (zero-length segment at index 0)
  auto target = make_target(1, {{0, 0}, {0, 0}, {5, 0}});
  // Obstacle at origin overlaps with segment 1's OBB (centered at second (0,0))
  std::vector<PredictedObject> obstacles = {make_obstacle(2, 0.0, 0.0)};

  auto result = find_collision(target, 0, obstacles);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->segment_index, 1);  // Collision on second segment (first valid segment)
}

// -----------------------------------------------------------------------------
// Multiple predicted paths (mode index)
// -----------------------------------------------------------------------------

TEST_F(CollisionTest, ChecksCorrectModeIndex)
{
  PredictedObject target;
  target.object_id = make_uuid(1);
  target.shape.dimensions = kDefaultDimensions;

  // Mode 0: horizontal path (0,0) -> (10,0)
  PredictedPath path0;
  path0.time_step = rclcpp::Duration::from_seconds(kDefaultTimeStep);
  path0.path = {make_pose(0, 0), make_pose(10, 0)};
  target.kinematics.predicted_paths.push_back(path0);

  // Mode 1: vertical path (0,5) -> (0,15)
  PredictedPath path1;
  path1.time_step = rclcpp::Duration::from_seconds(kDefaultTimeStep);
  path1.path = {make_pose(0, 5), make_pose(0, 15)};
  target.kinematics.predicted_paths.push_back(path1);

  // Obstacle at (0, 5) - only overlaps with mode 1's starting position
  std::vector<PredictedObject> obstacles = {make_obstacle(2, 0.0, 5.0)};

  EXPECT_FALSE(find_collision(target, 0, obstacles).has_value());
  EXPECT_TRUE(find_collision(target, 1, obstacles).has_value());
}

// -----------------------------------------------------------------------------
// Distance calculation
// -----------------------------------------------------------------------------

TEST_F(CollisionTest, CalculatesCorrectDistance_OnDiagonalPath)
{
  // Diagonal path: (0,0) -> (3,4) creates segment of length 5
  auto target = make_target(1, {{0, 0}, {3, 4}, {6, 8}});
  std::vector<PredictedObject> obstacles = {make_obstacle(2, 3.0, 4.0)};

  auto result = find_collision(target, 0, obstacles);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->segment_index, 1);
  EXPECT_NEAR(result->distance, 5.0, 1e-6);  // sqrt(3^2 + 4^2) = 5
}

}  // namespace autoware::predicted_path_postprocessor::testing
