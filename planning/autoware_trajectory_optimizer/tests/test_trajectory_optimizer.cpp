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

#include "autoware/trajectory_optimizer/trajectory_optimizer_structs.hpp"
#include "autoware/trajectory_optimizer/utils.hpp"
#include "autoware/velocity_smoother/smoother/jerk_filtered_smoother.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <vector>

using autoware::trajectory_optimizer::TrajectoryOptimizerParams;
using autoware::trajectory_optimizer::utils::TrajectoryPoints;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;

class TrajectoryOptimizerUtilsTest : public ::testing::Test
{
protected:
  TrajectoryPoints create_sample_trajectory(double resolution = 1.0, double offset = 0.0)
  {
    TrajectoryPoints points;
    for (int i = 0; i < 10; ++i) {
      TrajectoryPoint point;
      point.pose.position.x = i * resolution + offset;
      point.pose.position.y = i * resolution + offset;
      point.longitudinal_velocity_mps = 1.0;
      point.acceleration_mps2 = 0.1;
      points.push_back(point);
    }
    return points;
  }
};

TEST_F(TrajectoryOptimizerUtilsTest, RemoveInvalidPoints)
{
  TrajectoryPoints points = create_sample_trajectory();
  const auto points_size = points.size();
  autoware::trajectory_optimizer::utils::remove_invalid_points(points);
  ASSERT_EQ(points.size(), points_size);
}

TEST_F(TrajectoryOptimizerUtilsTest, RemoveCloseProximityPoints)
{
  TrajectoryPoints points = create_sample_trajectory();
  const auto points_size = points.size();

  autoware::trajectory_optimizer::utils::remove_close_proximity_points(points, 1E-2);
  ASSERT_EQ(points.size(), points_size);

  autoware::trajectory_optimizer::utils::remove_close_proximity_points(
    points, std::numeric_limits<double>::max());
  ASSERT_EQ(points.size(), 1);
}

TEST_F(TrajectoryOptimizerUtilsTest, ClampVelocities)
{
  TrajectoryPoints points = create_sample_trajectory();
  autoware::trajectory_optimizer::utils::clamp_velocities(points, 2.0f, 0.5f);
  for (const auto & point : points) {
    ASSERT_GE(point.longitudinal_velocity_mps, 2.0f);
    ASSERT_GE(point.acceleration_mps2, 0.5f);
  }
}

TEST_F(TrajectoryOptimizerUtilsTest, SetMaxVelocity_BasicCapping)
{
  // Create trajectory with varying velocities
  TrajectoryPoints points;
  for (int i = 0; i < 5; ++i) {
    TrajectoryPoint point;
    point.pose.position.x = i * 1.0;
    point.pose.position.y = 0.0;
    point.longitudinal_velocity_mps = 5.0f + i;  // [5, 6, 7, 8, 9]
    point.acceleration_mps2 = 0.5f;
    points.push_back(point);
  }

  const float max_velocity = 7.0f;
  autoware::trajectory_optimizer::utils::set_max_velocity(points, max_velocity);

  // Check all velocities are capped
  for (const auto & point : points) {
    ASSERT_LE(point.longitudinal_velocity_mps, max_velocity);
  }

  // Check specific values
  EXPECT_FLOAT_EQ(points[0].longitudinal_velocity_mps, 5.0f);  // Below max, unchanged
  EXPECT_FLOAT_EQ(points[1].longitudinal_velocity_mps, 6.0f);  // Below max, unchanged
  EXPECT_FLOAT_EQ(points[2].longitudinal_velocity_mps, 7.0f);  // At max, unchanged
  EXPECT_FLOAT_EQ(points[3].longitudinal_velocity_mps, 7.0f);  // Capped from 8
  EXPECT_FLOAT_EQ(points[4].longitudinal_velocity_mps, 7.0f);  // Capped from 9
}

TEST_F(TrajectoryOptimizerUtilsTest, SetMaxVelocity_InteriorAccelerationsZero)
{
  // Create trajectory with segment of constant velocity after capping
  TrajectoryPoints points;
  for (int i = 0; i < 5; ++i) {
    TrajectoryPoint point;
    point.pose.position.x = i * 1.0;
    point.pose.position.y = 0.0;
    point.longitudinal_velocity_mps = 10.0f;  // All exceed max
    point.acceleration_mps2 = 0.5f;
    points.push_back(point);
  }

  const float max_velocity = 5.0f;
  autoware::trajectory_optimizer::utils::set_max_velocity(points, max_velocity);

  // All interior points should have zero acceleration (constant velocity)
  // Points 0-3 should have zero acceleration (transition to next point at same velocity)
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_FLOAT_EQ(points[i].acceleration_mps2, 0.0f)
      << "Interior point " << i << " should have zero acceleration";
  }

  // Last point always has zero acceleration
  EXPECT_FLOAT_EQ(points[4].acceleration_mps2, 0.0f);
}

TEST_F(TrajectoryOptimizerUtilsTest, SetMaxVelocity_BoundaryAccelerations)
{
  // Create trajectory: [low, HIGH, HIGH, HIGH, low]
  TrajectoryPoints points;
  std::vector<float> velocities = {3.0f, 10.0f, 10.0f, 10.0f, 4.0f};

  for (size_t i = 0; i < velocities.size(); ++i) {
    TrajectoryPoint point;
    point.pose.position.x = i * 1.0;
    point.pose.position.y = 0.0;
    point.longitudinal_velocity_mps = velocities[i];
    point.acceleration_mps2 = 1.0f;  // Will be used to compute dt
    points.push_back(point);
  }

  const float max_velocity = 5.0f;
  autoware::trajectory_optimizer::utils::set_max_velocity(points, max_velocity);

  // After capping: [3.0, 5.0, 5.0, 5.0, 4.0]
  // Segment is [1, 3]

  // Point 0 (before segment): acceleration should be recalculated for transition INTO segment
  // a[0] = (v[1] - v[0]) / dt[0] = (5.0 - 3.0) / dt[0] > 0
  EXPECT_GT(points[0].acceleration_mps2, 0.0f)
    << "Point before segment should have positive acceleration";

  // Points 1-2 (interior): should have zero acceleration
  EXPECT_FLOAT_EQ(points[1].acceleration_mps2, 0.0f);
  EXPECT_FLOAT_EQ(points[2].acceleration_mps2, 0.0f);

  // Point 3 (end of segment): acceleration for transition OUT of segment
  // a[3] = (v[4] - v[3]) / dt[3] = (4.0 - 5.0) / dt[3] < 0
  EXPECT_LT(points[3].acceleration_mps2, 0.0f)
    << "Last point of segment should have negative acceleration";

  // Last point always zero
  EXPECT_FLOAT_EQ(points[4].acceleration_mps2, 0.0f);
}

TEST_F(TrajectoryOptimizerUtilsTest, SetMaxVelocity_SegmentAtStart)
{
  // Create trajectory starting with offending segment: [HIGH, HIGH, low, low]
  TrajectoryPoints points;
  std::vector<float> velocities = {10.0f, 10.0f, 3.0f, 3.0f};

  for (size_t i = 0; i < velocities.size(); ++i) {
    TrajectoryPoint point;
    point.pose.position.x = i * 1.0;
    point.pose.position.y = 0.0;
    point.longitudinal_velocity_mps = velocities[i];
    point.acceleration_mps2 = 1.0f;
    points.push_back(point);
  }

  const float max_velocity = 5.0f;
  autoware::trajectory_optimizer::utils::set_max_velocity(points, max_velocity);

  // After capping: [5.0, 5.0, 3.0, 3.0]
  // Segment is [0, 1]

  // Point 0: interior of segment, should have zero acceleration
  EXPECT_FLOAT_EQ(points[0].acceleration_mps2, 0.0f);

  // Point 1: end of segment, should have negative acceleration (transition to lower velocity)
  EXPECT_LT(points[1].acceleration_mps2, 0.0f);

  // Last point always zero
  EXPECT_FLOAT_EQ(points[3].acceleration_mps2, 0.0f);
}

TEST_F(TrajectoryOptimizerUtilsTest, SetMaxVelocity_SegmentAtEnd)
{
  // Create trajectory ending with offending segment: [low, low, HIGH, HIGH]
  TrajectoryPoints points;
  std::vector<float> velocities = {3.0f, 3.0f, 10.0f, 10.0f};

  for (size_t i = 0; i < velocities.size(); ++i) {
    TrajectoryPoint point;
    point.pose.position.x = i * 1.0;
    point.pose.position.y = 0.0;
    point.longitudinal_velocity_mps = velocities[i];
    point.acceleration_mps2 = 1.0f;
    points.push_back(point);
  }

  const float max_velocity = 5.0f;
  autoware::trajectory_optimizer::utils::set_max_velocity(points, max_velocity);

  // After capping: [3.0, 3.0, 5.0, 5.0]
  // Segment is [2, 3]

  // Point 1: before segment, should have positive acceleration
  EXPECT_GT(points[1].acceleration_mps2, 0.0f);

  // Point 2: interior of segment, should have zero acceleration
  EXPECT_FLOAT_EQ(points[2].acceleration_mps2, 0.0f);

  // Point 3: last point of trajectory, must have zero acceleration
  EXPECT_FLOAT_EQ(points[3].acceleration_mps2, 0.0f);
}

TEST_F(TrajectoryOptimizerUtilsTest, SetMaxVelocity_SinglePointSegment)
{
  // Create trajectory with single offending point: [low, HIGH, low]
  TrajectoryPoints points;
  std::vector<float> velocities = {3.0f, 10.0f, 3.0f};

  for (size_t i = 0; i < velocities.size(); ++i) {
    TrajectoryPoint point;
    point.pose.position.x = i * 1.0;
    point.pose.position.y = 0.0;
    point.longitudinal_velocity_mps = velocities[i];
    point.acceleration_mps2 = 1.0f;
    points.push_back(point);
  }

  const float max_velocity = 5.0f;
  autoware::trajectory_optimizer::utils::set_max_velocity(points, max_velocity);

  // After capping: [3.0, 5.0, 3.0]
  // Segment is [1, 1]

  EXPECT_FLOAT_EQ(points[1].longitudinal_velocity_mps, 5.0f);

  // Point 0: transition into segment, positive acceleration
  EXPECT_GT(points[0].acceleration_mps2, 0.0f);

  // Point 1: transition out of segment, negative acceleration
  EXPECT_LT(points[1].acceleration_mps2, 0.0f);

  // Last point always zero
  EXPECT_FLOAT_EQ(points[2].acceleration_mps2, 0.0f);
}

TEST_F(TrajectoryOptimizerUtilsTest, SetMaxVelocity_EntireTrajectoryOffending)
{
  // All points exceed max velocity
  TrajectoryPoints points;
  for (int i = 0; i < 5; ++i) {
    TrajectoryPoint point;
    point.pose.position.x = i * 1.0;
    point.pose.position.y = 0.0;
    point.longitudinal_velocity_mps = 10.0f;
    point.acceleration_mps2 = 0.5f;
    points.push_back(point);
  }

  const float max_velocity = 5.0f;
  autoware::trajectory_optimizer::utils::set_max_velocity(points, max_velocity);

  // All velocities should be capped
  for (const auto & point : points) {
    EXPECT_FLOAT_EQ(point.longitudinal_velocity_mps, max_velocity);
  }

  // All accelerations should be zero (constant velocity throughout)
  for (const auto & point : points) {
    EXPECT_FLOAT_EQ(point.acceleration_mps2, 0.0f);
  }
}

TEST_F(TrajectoryOptimizerUtilsTest, SetMaxVelocity_NoOffendingSegments)
{
  // All points below max velocity
  TrajectoryPoints points;
  for (int i = 0; i < 5; ++i) {
    TrajectoryPoint point;
    point.pose.position.x = i * 1.0;
    point.pose.position.y = 0.0;
    point.longitudinal_velocity_mps = 3.0f;
    point.acceleration_mps2 = 0.5f;
    points.push_back(point);
  }

  const float max_velocity = 10.0f;
  autoware::trajectory_optimizer::utils::set_max_velocity(points, max_velocity);

  // Velocities should remain unchanged
  for (const auto & point : points) {
    EXPECT_FLOAT_EQ(point.longitudinal_velocity_mps, 3.0f);
  }

  // Last point should have zero acceleration
  EXPECT_FLOAT_EQ(points.back().acceleration_mps2, 0.0f);
}

TEST_F(TrajectoryOptimizerUtilsTest, SetMaxVelocity_MultipleSegments)
{
  // Create trajectory with multiple offending segments: [HIGH, HIGH, low, HIGH, HIGH, low]
  TrajectoryPoints points;
  std::vector<float> velocities = {10.0f, 10.0f, 3.0f, 10.0f, 10.0f, 3.0f};

  for (size_t i = 0; i < velocities.size(); ++i) {
    TrajectoryPoint point;
    point.pose.position.x = i * 1.0;
    point.pose.position.y = 0.0;
    point.longitudinal_velocity_mps = velocities[i];
    point.acceleration_mps2 = 1.0f;
    points.push_back(point);
  }

  const float max_velocity = 5.0f;
  autoware::trajectory_optimizer::utils::set_max_velocity(points, max_velocity);

  // After capping: [5.0, 5.0, 3.0, 5.0, 5.0, 3.0]
  // Segments are [0, 1] and [3, 4]

  // First segment [0, 1]
  EXPECT_FLOAT_EQ(points[0].acceleration_mps2, 0.0f);  // Interior
  EXPECT_LT(points[1].acceleration_mps2, 0.0f);        // Transition out

  // Between segments
  EXPECT_GT(points[2].acceleration_mps2, 0.0f);  // Transition into second segment

  // Second segment [3, 4]
  EXPECT_FLOAT_EQ(points[3].acceleration_mps2, 0.0f);  // Interior
  EXPECT_LT(points[4].acceleration_mps2, 0.0f);        // Transition out

  // Last point always zero
  EXPECT_FLOAT_EQ(points[5].acceleration_mps2, 0.0f);
}

TEST_F(TrajectoryOptimizerUtilsTest, SetMaxVelocity_EmptyTrajectory)
{
  TrajectoryPoints points;
  autoware::trajectory_optimizer::utils::set_max_velocity(points, 5.0f);
  EXPECT_TRUE(points.empty());
}

TEST_F(TrajectoryOptimizerUtilsTest, SetMaxVelocity_SinglePointTrajectory)
{
  TrajectoryPoints points;
  TrajectoryPoint point;
  point.pose.position.x = 0.0;
  point.pose.position.y = 0.0;
  point.longitudinal_velocity_mps = 10.0f;
  point.acceleration_mps2 = 0.5f;
  points.push_back(point);

  const float max_velocity = 5.0f;
  autoware::trajectory_optimizer::utils::set_max_velocity(points, max_velocity);

  EXPECT_FLOAT_EQ(points[0].longitudinal_velocity_mps, max_velocity);
  EXPECT_FLOAT_EQ(points[0].acceleration_mps2, 0.0f);
}

TEST_F(TrajectoryOptimizerUtilsTest, ValidatePoint)
{
  TrajectoryPoint valid_point;
  valid_point.longitudinal_velocity_mps = 1.0;
  valid_point.acceleration_mps2 = 0.1;
  valid_point.pose.position.x = 1.0;
  valid_point.pose.position.y = 1.0;
  valid_point.pose.position.z = 1.0;
  valid_point.pose.orientation.x = 0.0;
  valid_point.pose.orientation.y = 0.0;
  valid_point.pose.orientation.z = 0.0;
  valid_point.pose.orientation.w = 1.0;
  ASSERT_TRUE(autoware::trajectory_optimizer::utils::validate_point(valid_point));

  TrajectoryPoint invalid_point;
  invalid_point.pose.position.x = std::nan("");
  ASSERT_FALSE(autoware::trajectory_optimizer::utils::validate_point(invalid_point));
}

TEST_F(TrajectoryOptimizerUtilsTest, ApplySpline)
{
  TrajectoryPoints points = create_sample_trajectory();
  const double interpolation_resolution_m = 0.1;
  const double max_distance_discrepancy_m = 5.0;
  const bool preserve_original_orientation = true;
  autoware::trajectory_optimizer::utils::apply_spline(
    points, interpolation_resolution_m, max_distance_discrepancy_m, preserve_original_orientation);
  ASSERT_GE(points.size(), 2);
}

TEST_F(TrajectoryOptimizerUtilsTest, AddEgoStateToTrajectory)
{
  TrajectoryPoints points = create_sample_trajectory();
  Odometry current_odometry;
  current_odometry.pose.pose.position.x = 1.0;
  current_odometry.pose.pose.position.y = 1.0;
  const double nearest_dist_threshold_m = 1.5;
  const double nearest_yaw_threshold_rad = 1.0;
  const double backward_trajectory_extension_m = 5.0;
  autoware::trajectory_optimizer::utils::add_ego_state_to_trajectory(
    points, current_odometry, nearest_dist_threshold_m, nearest_yaw_threshold_rad,
    backward_trajectory_extension_m);
  ASSERT_FALSE(points.empty());
}

TEST_F(TrajectoryOptimizerUtilsTest, ExpandTrajectoryWithEgoHistory)
{
  TrajectoryPoints points = create_sample_trajectory();
  TrajectoryPoints ego_history_points = create_sample_trajectory(1.0, -10.0);
  Odometry current_odometry;
  current_odometry.pose.pose.position.x = points.front().pose.position.x;
  current_odometry.pose.pose.position.y = points.front().pose.position.y;
  autoware::trajectory_optimizer::utils::expand_trajectory_with_ego_history(
    points, ego_history_points, current_odometry);
  ASSERT_GE(points.size(), 20);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
