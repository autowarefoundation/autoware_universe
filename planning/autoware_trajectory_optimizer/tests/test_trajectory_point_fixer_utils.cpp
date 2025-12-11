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

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/plugin_utils/trajectory_point_fixer_utils.hpp"
#include "test_utils.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

using autoware::trajectory_optimizer::plugin::trajectory_point_fixer_utils::
  calculate_cluster_reference_yaw;
using autoware::trajectory_optimizer::plugin::trajectory_point_fixer_utils::
  compute_cluster_arc_lengths;
using autoware::trajectory_optimizer::plugin::trajectory_point_fixer_utils::
  create_ego_point_from_odometry;
using autoware::trajectory_optimizer::plugin::trajectory_point_fixer_utils::
  get_close_proximity_clusters;
using autoware::trajectory_optimizer::plugin::trajectory_point_fixer_utils::normalize_values;
using autoware::trajectory_optimizer::plugin::trajectory_point_fixer_utils::
  remove_close_proximity_points;
using autoware::trajectory_optimizer::plugin::trajectory_point_fixer_utils::remove_invalid_points;
using autoware::trajectory_optimizer::plugin::trajectory_point_fixer_utils::
  resample_close_proximity_points;
using autoware::trajectory_optimizer::plugin::trajectory_point_fixer_utils::resample_single_cluster;
using autoware_planning_msgs::msg::TrajectoryPoint;
using nav_msgs::msg::Odometry;
using trajectory_optimizer_test_utils::create_odometry;
using trajectory_optimizer_test_utils::create_point;
using trajectory_optimizer_test_utils::create_point_with_yaw;

class PointFixerUtilsTest : public ::testing::Test
{
};

// Tests for get_close_proximity_clusters
TEST_F(PointFixerUtilsTest, GetCloseProximityClusters_NoClusters)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0));
  points.push_back(create_point(1.0, 0.0));
  points.push_back(create_point(2.0, 0.0));

  const auto clusters = get_close_proximity_clusters(points, 0.5);

  EXPECT_TRUE(clusters.empty());
}

TEST_F(PointFixerUtilsTest, GetCloseProximityClusters_SingleCluster)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0));
  points.push_back(create_point(0.01, 0.0));  // Very close
  points.push_back(create_point(0.02, 0.0));  // Still close
  points.push_back(create_point(2.0, 0.0));   // Far away

  const auto clusters = get_close_proximity_clusters(points, 0.1);

  ASSERT_EQ(clusters.size(), 1);
  EXPECT_EQ(clusters[0].size(), 3);
  EXPECT_EQ(clusters[0][0], 0);
  EXPECT_EQ(clusters[0][1], 1);
  EXPECT_EQ(clusters[0][2], 2);
}

TEST_F(PointFixerUtilsTest, GetCloseProximityClusters_MultipleClusters)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0));
  points.push_back(create_point(0.01, 0.0));  // Cluster 1
  points.push_back(create_point(2.0, 0.0));   // Gap
  points.push_back(create_point(2.01, 0.0));  // Cluster 2
  points.push_back(create_point(2.02, 0.0));  // Cluster 2

  const auto clusters = get_close_proximity_clusters(points, 0.1);

  ASSERT_EQ(clusters.size(), 2);
  EXPECT_EQ(clusters[0].size(), 2);
  EXPECT_EQ(clusters[1].size(), 3);
}

TEST_F(PointFixerUtilsTest, GetCloseProximityClusters_EmptyTrajectory)
{
  std::vector<TrajectoryPoint> points;
  const auto clusters = get_close_proximity_clusters(points, 0.1);
  EXPECT_TRUE(clusters.empty());
}

// Tests for create_ego_point_from_odometry
TEST_F(PointFixerUtilsTest, CreateEgoPointFromOdometry_Basic)
{
  const auto odom = create_odometry(1.0, 2.0, M_PI_4, 5.0);
  const auto ego_point = create_ego_point_from_odometry(odom);

  EXPECT_DOUBLE_EQ(ego_point.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(ego_point.pose.position.y, 2.0);
  EXPECT_FLOAT_EQ(ego_point.longitudinal_velocity_mps, 5.0f);
  EXPECT_EQ(ego_point.time_from_start.sec, 0);
  EXPECT_EQ(ego_point.time_from_start.nanosec, 0u);
}

// Tests for calculate_cluster_reference_yaw
TEST_F(PointFixerUtilsTest, CalculateClusterReferenceYaw_FromPreviousPoints)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0));
  points.push_back(create_point(1.0, 0.0));
  points.push_back(create_point(2.0, 0.0));  // Cluster starts here
  points.push_back(create_point(2.01, 0.0));

  std::vector<size_t> cluster{2, 3};
  const auto ego_point = create_point(0.0, 0.0);

  const double yaw = calculate_cluster_reference_yaw(cluster, points, ego_point);

  // Should be 0 (pointing along x-axis)
  EXPECT_NEAR(yaw, 0.0, 1e-6);
}

TEST_F(PointFixerUtilsTest, CalculateClusterReferenceYaw_FallbackToEgo)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0));  // Cluster at start
  points.push_back(create_point(0.01, 0.0));

  std::vector<size_t> cluster{0, 1};
  const auto ego_point = create_point_with_yaw(0.0, 0.0, M_PI_4);

  const double yaw = calculate_cluster_reference_yaw(cluster, points, ego_point);

  EXPECT_NEAR(yaw, M_PI_4, 1e-6);
}

// Tests for compute_cluster_arc_lengths
TEST_F(PointFixerUtilsTest, ComputeClusterArcLengths_Basic)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0));
  points.push_back(create_point(1.0, 0.0));
  points.push_back(create_point(2.0, 0.0));

  std::vector<size_t> cluster{0, 1, 2};
  const auto arc_lengths = compute_cluster_arc_lengths(cluster, points);

  ASSERT_EQ(arc_lengths.size(), 3);
  EXPECT_DOUBLE_EQ(arc_lengths[0], 0.0);
  EXPECT_NEAR(arc_lengths[1], 1.0, 1e-6);
  EXPECT_NEAR(arc_lengths[2], 2.0, 1e-6);
}

TEST_F(PointFixerUtilsTest, ComputeClusterArcLengths_NonUniformSpacing)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0));
  points.push_back(create_point(0.5, 0.0));
  points.push_back(create_point(2.0, 0.0));

  std::vector<size_t> cluster{0, 1, 2};
  const auto arc_lengths = compute_cluster_arc_lengths(cluster, points);

  ASSERT_EQ(arc_lengths.size(), 3);
  EXPECT_DOUBLE_EQ(arc_lengths[0], 0.0);
  EXPECT_NEAR(arc_lengths[1], 0.5, 1e-6);
  EXPECT_NEAR(arc_lengths[2], 2.0, 1e-6);
}

// Tests for normalize_values
TEST_F(PointFixerUtilsTest, NormalizeValues_Basic)
{
  std::vector<double> values{0.0, 1.0, 2.0, 4.0};
  const auto normalized = normalize_values(values);

  ASSERT_EQ(normalized.size(), 4);
  EXPECT_DOUBLE_EQ(normalized[0], 0.0);
  EXPECT_DOUBLE_EQ(normalized[1], 0.25);
  EXPECT_DOUBLE_EQ(normalized[2], 0.5);
  EXPECT_DOUBLE_EQ(normalized[3], 1.0);
}

TEST_F(PointFixerUtilsTest, NormalizeValues_EmptyInput)
{
  std::vector<double> values;
  const auto normalized = normalize_values(values);
  EXPECT_TRUE(normalized.empty());
}

TEST_F(PointFixerUtilsTest, NormalizeValues_ZeroLength)
{
  std::vector<double> values{0.0, 0.0, 0.0};
  const auto normalized = normalize_values(values);

  ASSERT_EQ(normalized.size(), 3);
  for (const auto & val : normalized) {
    EXPECT_DOUBLE_EQ(val, 0.0);
  }
}

// Tests for resample_single_cluster
TEST_F(PointFixerUtilsTest, ResampleSingleCluster_RedistributesPoints)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0));
  points.push_back(create_point(1.0, 0.0));
  points.push_back(create_point(1.01, 0.0));  // Very close
  points.push_back(create_point(1.02, 0.0));  // Very close
  points.push_back(create_point(5.0, 0.0));

  std::vector<size_t> cluster{2, 3};
  const auto ego_point = create_point(0.0, 0.0);

  resample_single_cluster(cluster, points, ego_point);

  // Points should still exist and have valid positions
  EXPECT_EQ(points.size(), 5);
  EXPECT_TRUE(std::isfinite(points[2].pose.position.x));
  EXPECT_TRUE(std::isfinite(points[3].pose.position.x));

  // Points should maintain relative ordering
  EXPECT_LE(points[2].pose.position.x, points[3].pose.position.x);
}

TEST_F(PointFixerUtilsTest, ResampleSingleCluster_EmptyCluster)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0));

  std::vector<size_t> cluster;
  const auto ego_point = create_point(0.0, 0.0);

  // Should not crash
  resample_single_cluster(cluster, points, ego_point);
  EXPECT_EQ(points.size(), 1);
}

// Tests for resample_close_proximity_points
TEST_F(PointFixerUtilsTest, ResampleCloseProximityPoints_NoClosePoints)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0));
  points.push_back(create_point(1.0, 0.0));
  points.push_back(create_point(2.0, 0.0));

  const auto odom = create_odometry(0.0, 0.0, 0.0, 1.0);
  const auto original_points = points;

  resample_close_proximity_points(points, odom, 0.5);

  // Points should be unchanged
  ASSERT_EQ(points.size(), original_points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    EXPECT_NEAR(points[i].pose.position.x, original_points[i].pose.position.x, 1e-6);
    EXPECT_NEAR(points[i].pose.position.y, original_points[i].pose.position.y, 1e-6);
  }
}

TEST_F(PointFixerUtilsTest, ResampleCloseProximityPoints_WithClosePoints)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0));
  points.push_back(create_point(0.01, 0.0));
  points.push_back(create_point(0.02, 0.0));
  points.push_back(create_point(2.0, 0.0));

  const auto odom = create_odometry(0.0, 0.0, 0.0, 1.0);

  resample_close_proximity_points(points, odom, 0.5);

  // Points should still exist and have valid positions
  EXPECT_EQ(points.size(), 4);
  for (const auto & pt : points) {
    EXPECT_TRUE(std::isfinite(pt.pose.position.x));
    EXPECT_TRUE(std::isfinite(pt.pose.position.y));
    EXPECT_TRUE(std::isfinite(pt.pose.orientation.w));
  }

  // Points should maintain relative ordering along x-axis
  for (size_t i = 1; i < points.size(); ++i) {
    EXPECT_LE(points[i - 1].pose.position.x, points[i].pose.position.x);
  }
}

TEST_F(PointFixerUtilsTest, ResampleCloseProximityPoints_EmptyTrajectory)
{
  std::vector<TrajectoryPoint> points;
  const auto odom = create_odometry(0.0, 0.0, 0.0, 1.0);

  resample_close_proximity_points(points, odom, 0.1);

  EXPECT_TRUE(points.empty());
}

TEST_F(PointFixerUtilsTest, ResampleCloseProximityPoints_SinglePoint)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0));

  const auto odom = create_odometry(0.0, 0.0, 0.0, 1.0);

  resample_close_proximity_points(points, odom, 0.1);

  EXPECT_EQ(points.size(), 1);
}

// Tests for remove_invalid_points
TEST_F(PointFixerUtilsTest, RemoveInvalidPoints_AllValid)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0));
  points.push_back(create_point(1.0, 0.0));
  points.push_back(create_point(2.0, 0.0));

  const size_t original_size = points.size();
  remove_invalid_points(points);

  EXPECT_EQ(points.size(), original_size);
}

TEST_F(PointFixerUtilsTest, RemoveInvalidPoints_WithNaN)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0));

  auto invalid_point = create_point(1.0, 0.0);
  invalid_point.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  points.push_back(invalid_point);

  points.push_back(create_point(2.0, 0.0));

  remove_invalid_points(points);

  EXPECT_EQ(points.size(), 2);
  EXPECT_DOUBLE_EQ(points[0].pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(points[1].pose.position.x, 2.0);
}

TEST_F(PointFixerUtilsTest, RemoveInvalidPoints_WithInf)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0));

  auto invalid_point = create_point(1.0, 0.0);
  invalid_point.longitudinal_velocity_mps = std::numeric_limits<float>::infinity();
  points.push_back(invalid_point);

  points.push_back(create_point(2.0, 0.0));

  remove_invalid_points(points);

  EXPECT_EQ(points.size(), 2);
}

TEST_F(PointFixerUtilsTest, RemoveInvalidPoints_InvalidOrientation)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0));

  auto invalid_point = create_point(1.0, 0.0);
  invalid_point.pose.orientation.w = std::numeric_limits<double>::quiet_NaN();
  points.push_back(invalid_point);

  points.push_back(create_point(2.0, 0.0));

  remove_invalid_points(points);

  EXPECT_EQ(points.size(), 2);
}

// Tests for remove_close_proximity_points
TEST_F(PointFixerUtilsTest, RemoveCloseProximityPoints_NoClosePoints)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0));
  points.push_back(create_point(1.0, 0.0));
  points.push_back(create_point(2.0, 0.0));

  const size_t original_size = points.size();
  remove_close_proximity_points(points, 0.1);

  EXPECT_EQ(points.size(), original_size);
}

TEST_F(PointFixerUtilsTest, RemoveCloseProximityPoints_WithClosePoints)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0));
  points.push_back(create_point(0.001, 0.0));  // Very close
  points.push_back(create_point(1.0, 0.0));

  remove_close_proximity_points(points, 0.01);

  EXPECT_EQ(points.size(), 2);
  EXPECT_DOUBLE_EQ(points[0].pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(points[1].pose.position.x, 1.0);
}

TEST_F(PointFixerUtilsTest, RemoveCloseProximityPoints_AllClose)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0));
  points.push_back(create_point(0.001, 0.0));
  points.push_back(create_point(0.002, 0.0));
  points.push_back(create_point(0.003, 0.0));

  remove_close_proximity_points(points, 0.01);

  // Should keep only the first point
  EXPECT_EQ(points.size(), 1);
  EXPECT_DOUBLE_EQ(points[0].pose.position.x, 0.0);
}

TEST_F(PointFixerUtilsTest, RemoveCloseProximityPoints_EmptyTrajectory)
{
  std::vector<TrajectoryPoint> points;
  remove_close_proximity_points(points, 0.1);
  EXPECT_TRUE(points.empty());
}

TEST_F(PointFixerUtilsTest, RemoveCloseProximityPoints_SinglePoint)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0));

  remove_close_proximity_points(points, 0.1);

  EXPECT_EQ(points.size(), 1);
}

TEST_F(PointFixerUtilsTest, RemoveCloseProximityPoints_DefaultThreshold)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0));
  points.push_back(create_point(0.001, 0.0));  // Within default 1E-2
  points.push_back(create_point(1.0, 0.0));

  // Use default threshold
  remove_close_proximity_points(points);

  EXPECT_EQ(points.size(), 2);
}
