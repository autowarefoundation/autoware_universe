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

#include "autoware/boundary_departure_checker/detail/boundary_segment_finder.hpp"
#include "autoware/boundary_departure_checker/detail/geometry_projection.hpp"
#include "autoware/boundary_departure_checker/detail/severity_evaluator.hpp"
#include "autoware/boundary_departure_checker/detail/type_alias.hpp"
#include "autoware/boundary_departure_checker/detail/uncrossable_boundaries_rtree.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

namespace autoware::boundary_departure_checker
{
// ==============================================================================
// 1. geometry_projection Tests
// ==============================================================================

TEST(UncrossableBoundaryUtilsTest, TestSegmentToSegmentProjection)
{
  // Arrange:
  Segment2d ego_seg{{0.0, 0.0}, {0.0, 2.0}};
  Segment2d boundary_seg{{1.0, 0.0}, {1.0, 2.0}};

  // Act:
  auto result = geometry_projection::calc_nearest_projection(ego_seg, boundary_seg, 0);

  // Assert:
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->lat_dist, 1.0, 1e-6);
}

TEST(UncrossableBoundaryUtilsTest, TestIntersectionDetection)
{
  // Arrange:
  Segment2d ego_seg{{0.0, 0.0}, {1.0, 2.0}};
  Segment2d boundary_seg{{1.0, 0.0}, {1.0, 2.0}};

  // Act:
  auto result = geometry_projection::calc_nearest_projection(ego_seg, boundary_seg, 0);

  // Assert:
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->lat_dist, 0.0);
}

TEST(UncrossableBoundaryUtilsTest, TestPointBeyondSegmentEnd)
{
  // Arrange:
  Segment2d ego_seg{{0.0, 0.0}, {1.0, 0.0}};
  Segment2d boundary_seg{{3.0, 2.0}, {4.0, 2.0}};

  // Act:
  auto result = geometry_projection::calc_nearest_projection(ego_seg, boundary_seg, 0);

  // Assert:
  ASSERT_FALSE(result.has_value());
}

TEST(UncrossableBoundaryUtilsTest, TestFindClosestSegmentRearIntersection)
{
  // Arrange:
  Segment2d ego_side_seg{{0.0, 4.0}, {0.0, 2.0}};
  Segment2d ego_rear_seg{{0.0, 0.0}, {2.0, 0.0}};
  Segment2d ego_front_seg{{0.0, 4.0}, {2.0, 4.0}};
  size_t curr_fp_idx = 42;

  std::vector<SegmentWithIdx> boundary_segments;
  IdxForRTreeSegment id{1, 0, 1};
  boundary_segments.emplace_back(Segment2d{{1.0, -1.0}, {1.0, 1.0}}, id);

  // Act:
  auto result = geometry_projection::find_closest_segment(
    ego_side_seg, ego_front_seg, ego_rear_seg, curr_fp_idx, boundary_segments);

  // Assert:
  EXPECT_EQ(result.pose_index, curr_fp_idx);
  EXPECT_DOUBLE_EQ(result.lat_dist, 0.0);
  EXPECT_DOUBLE_EQ(result.pt_on_ego.x(), 1.0);
  EXPECT_DOUBLE_EQ(result.pt_on_ego.y(), 0.0);
}

TEST(UncrossableBoundaryUtilsTest, TestFindClosestSegmentFrontIntersection)
{
  // Arrange:
  Segment2d ego_side_seg{{0.0, 4.0}, {0.0, 2.0}};
  Segment2d ego_rear_seg{{0.0, -4.0}, {2.0, -4.0}};
  Segment2d ego_front_seg{{0.0, 4.0}, {2.0, 4.0}};
  size_t curr_fp_idx = 42;

  std::vector<SegmentWithIdx> boundary_segments;
  IdxForRTreeSegment id{1, 0, 1};
  boundary_segments.emplace_back(Segment2d{{1.0, 5.0}, {1.0, 3.0}}, id);

  // Act:
  auto result = geometry_projection::find_closest_segment(
    ego_side_seg, ego_front_seg, ego_rear_seg, curr_fp_idx, boundary_segments);

  // Assert:
  EXPECT_EQ(result.pose_index, curr_fp_idx);
  EXPECT_DOUBLE_EQ(result.lat_dist, 0.0);
  EXPECT_DOUBLE_EQ(result.pt_on_ego.x(), 1.0);
  EXPECT_DOUBLE_EQ(result.pt_on_ego.y(), 4.0);
}

TEST(UncrossableBoundaryUtilsTest, TestFindClosestSegmentFallback)
{
  // Arrange:
  Segment2d ego_side_seg{{0.0, 4.0}, {0.0, 2.0}};
  Segment2d ego_rear_seg{{0.0, 0.0}, {2.0, 0.0}};
  Segment2d ego_front_seg{{0.0, 4.0}, {2.0, 4.0}};

  size_t curr_fp_idx = 99;

  std::vector<SegmentWithIdx> boundary_segments;
  IdxForRTreeSegment id{1, 0, 1};
  boundary_segments.emplace_back(Segment2d{{4.0, -2.0}, {5.0, -2.0}}, id);

  // Act:
  auto result = geometry_projection::find_closest_segment(
    ego_side_seg, ego_front_seg, ego_rear_seg, curr_fp_idx, boundary_segments);

  // Assert:
  EXPECT_EQ(result.pose_index, curr_fp_idx);
  EXPECT_DOUBLE_EQ(result.lat_dist, std::numeric_limits<double>::max());
}

TEST(UncrossableBoundaryUtilsTest, TestCalcSignedLateralDistanceToBoundary)
{
  // Arrange:
  geometry_msgs::msg::Pose ego_pose;
  ego_pose.position.x = 0.0;
  ego_pose.position.y = 0.0;
  ego_pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(0.0);

  // Act & Assert:
  lanelet::LineString3d empty_ls(lanelet::utils::getId());
  EXPECT_FALSE(
    geometry_projection::calc_signed_lateral_distance_to_boundary(empty_ls, ego_pose).has_value());

  lanelet::Point3d p1(lanelet::utils::getId(), -5.0, 5.0, 0.0);
  lanelet::LineString3d single_pt_ls(lanelet::utils::getId(), {p1});
  EXPECT_FALSE(
    geometry_projection::calc_signed_lateral_distance_to_boundary(single_pt_ls, ego_pose)
      .has_value());

  lanelet::Point3d p2(lanelet::utils::getId(), 5.0, 5.0, 0.0);
  lanelet::LineString3d left_boundary(lanelet::utils::getId(), {p1, p2});
  auto dist_left =
    geometry_projection::calc_signed_lateral_distance_to_boundary(left_boundary, ego_pose);
  ASSERT_TRUE(dist_left.has_value());
  EXPECT_DOUBLE_EQ(dist_left.value(), 5.0);

  lanelet::Point3d p3(lanelet::utils::getId(), -5.0, -3.0, 0.0);
  lanelet::Point3d p4(lanelet::utils::getId(), 5.0, -3.0, 0.0);
  lanelet::LineString3d right_boundary(lanelet::utils::getId(), {p3, p4});
  auto dist_right =
    geometry_projection::calc_signed_lateral_distance_to_boundary(right_boundary, ego_pose);
  ASSERT_TRUE(dist_right.has_value());
  EXPECT_DOUBLE_EQ(dist_right.value(), -3.0);

  lanelet::Point3d p5(lanelet::utils::getId(), 2.0, 5.0, 0.0);
  lanelet::Point3d p6(lanelet::utils::getId(), 10.0, 5.0, 0.0);
  lanelet::LineString3d missed_boundary(lanelet::utils::getId(), {p5, p6});
  auto dist_miss =
    geometry_projection::calc_signed_lateral_distance_to_boundary(missed_boundary, ego_pose);
  EXPECT_FALSE(dist_miss.has_value());

  lanelet::Point3d p7(lanelet::utils::getId(), 5.0, -10.0, 0.0);
  lanelet::Point3d p8(lanelet::utils::getId(), 5.0, 10.0, 0.0);
  lanelet::LineString3d parallel_boundary(lanelet::utils::getId(), {p7, p8});
  auto dist_parallel =
    geometry_projection::calc_signed_lateral_distance_to_boundary(parallel_boundary, ego_pose);
  EXPECT_FALSE(dist_parallel.has_value());

  lanelet::Point3d p9(lanelet::utils::getId(), -10.0, 8.0, 0.0);
  lanelet::Point3d p10(lanelet::utils::getId(), 10.0, 8.0, 0.0);
  lanelet::Point3d p11(lanelet::utils::getId(), 10.0, 4.0, 0.0);
  lanelet::Point3d p12(lanelet::utils::getId(), -10.0, 4.0, 0.0);
  lanelet::LineString3d multi_boundary(lanelet::utils::getId(), {p9, p10, p11, p12});
  auto dist_multi =
    geometry_projection::calc_signed_lateral_distance_to_boundary(multi_boundary, ego_pose);
  ASSERT_TRUE(dist_multi.has_value());
  EXPECT_DOUBLE_EQ(dist_multi.value(), 4.0);
}

// ==============================================================================
// 2. UncrossableBoundariesRTree Tests
// ==============================================================================

TEST(UncrossableBoundaryUtilsTest, TestGetSegment3DFromId)
{
  // Arrange:
  auto map = std::make_shared<lanelet::LaneletMap>();
  lanelet::Point3d p1(lanelet::utils::getId(), 1.0, 2.0, 3.0);
  lanelet::Point3d p2(lanelet::utils::getId(), 4.0, 5.0, 6.0);
  lanelet::Point3d p3(lanelet::utils::getId(), 7.0, 8.0, 9.0);
  lanelet::LineString3d ls(lanelet::utils::getId(), {p1, p2, p3});
  ls.attributes()[lanelet::AttributeName::Type] = "road_border";
  map->add(ls);

  UncrossableBoundariesRTree rtree(map, {"road_border"});

  // Act & Assert:
  IdxForRTreeSegment id1{ls.id(), 0, 1};
  auto seg1 = rtree.get_segment_3d_from_id(id1);
  EXPECT_DOUBLE_EQ(seg1.first.x(), 1.0);
  EXPECT_DOUBLE_EQ(seg1.second.x(), 4.0);

  IdxForRTreeSegment id2{ls.id(), 1, 2};
  auto seg2 = rtree.get_segment_3d_from_id(id2);
  EXPECT_DOUBLE_EQ(seg2.first.x(), 4.0);
  EXPECT_DOUBLE_EQ(seg2.second.x(), 7.0);
}

// ==============================================================================
// 3. boundary_segment_finder Tests
// ==============================================================================

TEST(UncrossableBoundaryUtilsTest, TestIsClosestToBoundarySegment)
{
  // Arrange:
  Segment2d left_side{{0.0, 2.0}, {2.0, 2.0}};
  Segment2d right_side{{0.0, -2.0}, {2.0, -2.0}};

  // Act & Assert:
  Segment2d bound_left{{0.0, 3.0}, {2.0, 3.0}};
  EXPECT_TRUE(
    boundary_segment_finder::is_closest_to_boundary_segment(bound_left, left_side, right_side));

  Segment2d bound_right{{0.0, -3.0}, {2.0, -3.0}};
  EXPECT_TRUE(
    boundary_segment_finder::is_closest_to_boundary_segment(bound_right, right_side, left_side));

  Segment2d bound_center{{0.0, 0.0}, {2.0, 0.0}};
  EXPECT_TRUE(
    boundary_segment_finder::is_closest_to_boundary_segment(bound_center, left_side, right_side));
}

TEST(UncrossableBoundaryUtilsTest, TestIsSegmentWithinEgoHeight)
{
  // Arrange:
  const double ego_z = 0.0;
  const double ego_height = 2.5;

  // Act & Assert:
  Segment3d seg_within{{0.0, 0.0, 1.0}, {1.0, 0.0, 1.5}};
  EXPECT_TRUE(boundary_segment_finder::is_segment_within_ego_height(seg_within, ego_z, ego_height));

  Segment3d seg_above{{0.0, 0.0, 3.0}, {1.0, 0.0, 4.0}};
  EXPECT_FALSE(boundary_segment_finder::is_segment_within_ego_height(seg_above, ego_z, ego_height));

  Segment3d seg_below{{0.0, 0.0, -3.0}, {1.0, 0.0, -4.0}};
  EXPECT_FALSE(boundary_segment_finder::is_segment_within_ego_height(seg_below, ego_z, ego_height));
}

// ==============================================================================
// 4. severity_evaluator Tests
// ==============================================================================

TEST(UncrossableBoundaryUtilsTest, TestIsCritical)
{
  // Arrange:
  Side<std::optional<DeparturePointPair>> projections;

  // Act & Assert:
  EXPECT_FALSE(severity_evaluator::is_critical(projections));

  DeparturePointPair pair_crit;
  pair_crit.physical_departure_point.departure_type = DepartureType::CRITICAL;
  projections.left = pair_crit;
  EXPECT_TRUE(severity_evaluator::is_critical(projections));
}

TEST(UncrossableBoundaryUtilsTest, TestIsNearBoundary)
{
  // Arrange:
  Side<std::optional<DeparturePointPair>> projections;

  // Act & Assert:
  EXPECT_FALSE(severity_evaluator::is_near_boundary(projections));

  DeparturePointPair pair_none;
  pair_none.physical_departure_point.departure_type = DepartureType::NONE;
  projections.left = pair_none;
  EXPECT_FALSE(severity_evaluator::is_near_boundary(projections));

  DeparturePointPair pair_near;
  pair_near.physical_departure_point.departure_type = DepartureType::NEAR_BOUNDARY;
  projections.right = pair_near;
  EXPECT_TRUE(severity_evaluator::is_near_boundary(projections));

  // A critical departure is not reported as near boundary.
  projections.right->physical_departure_point.departure_type = DepartureType::CRITICAL;
  EXPECT_FALSE(severity_evaluator::is_near_boundary(projections));
}

TEST(UncrossableBoundaryUtilsTest, TestGetMinLateralDistanceToBound)
{
  // Arrange:
  Side<std::optional<DeparturePointPair>> projections;

  // Act & Assert: without any projection, the distance is infinity so consumers can filter it out.
  EXPECT_TRUE(std::isinf(severity_evaluator::get_min_lateral_distance_to_bound(projections)));

  // A projection whose segment lookup failed keeps the sentinel default and is ignored.
  DeparturePointPair pair_without_segment;
  projections.left = pair_without_segment;
  EXPECT_TRUE(std::isinf(severity_evaluator::get_min_lateral_distance_to_bound(projections)));

  DeparturePointPair pair_left;
  pair_left.physical_departure_point.lat_dist = 0.8;
  projections.left = pair_left;
  EXPECT_DOUBLE_EQ(severity_evaluator::get_min_lateral_distance_to_bound(projections), 0.8);

  DeparturePointPair pair_right;
  pair_right.physical_departure_point.lat_dist = 0.3;
  projections.right = pair_right;
  EXPECT_DOUBLE_EQ(severity_evaluator::get_min_lateral_distance_to_bound(projections), 0.3);
}

TEST(UncrossableBoundaryUtilsTest, TestAssignDepartureType)
{
  // Arrange:
  DepartureCheckThresholds thresholds;
  thresholds.min_braking_distance = 10.0;
  thresholds.cutoff_time = 2.0;
  thresholds.th_lat_critical = 0.1;
  thresholds.th_near_boundary = 0.5;

  const auto make_metrics = [](double lat_dist, double lon_dist, double time) {
    ProjectionEvaluationMetrics metrics;
    metrics.lat_dist = lat_dist;
    metrics.lon_dist_to_departure = lon_dist;
    metrics.time_from_start = time;
    return metrics;
  };

  // Act & Assert: within the critical margin and unable to stop in time.
  EXPECT_EQ(
    severity_evaluator::assign_departure_type(make_metrics(0.05, 5.0, 3.0), thresholds),
    DepartureType::CRITICAL);

  // Within the critical margin but far enough ahead to stop: still near the boundary.
  EXPECT_EQ(
    severity_evaluator::assign_departure_type(make_metrics(0.05, 20.0, 3.0), thresholds),
    DepartureType::NEAR_BOUNDARY);

  // Between the two lateral thresholds.
  EXPECT_EQ(
    severity_evaluator::assign_departure_type(make_metrics(0.3, 20.0, 3.0), thresholds),
    DepartureType::NEAR_BOUNDARY);

  // Beyond the near boundary threshold.
  EXPECT_EQ(
    severity_evaluator::assign_departure_type(make_metrics(0.6, 20.0, 3.0), thresholds),
    DepartureType::NONE);
}

TEST(UncrossableBoundaryUtilsTest, TestNearBoundaryThresholdBelowCriticalMargin)
{
  // Verifies that a near boundary threshold configured below the critical margin does not leave a
  // projection inside the critical margin classified as NONE.

  // Arrange:
  UncrossableBoundaryDepartureParam param;
  param.critical_departure_lateral_th_m = 0.5;
  param.near_boundary_lateral_th_m = 0.1;
  param.time_to_departure_cutoff_s = 2.0;

  ProjectionToBound projection(0);
  projection.lat_dist = 0.3;  // inside the critical margin, outside the near boundary threshold
  projection.dist_along_trajectory_m = 20.0;
  projection.ego_front_to_proj_offset_m = 0.0;
  projection.time_from_start = 3.0;

  // Act:
  const auto out = severity_evaluator::filter_and_assign_departure_types({projection}, param, 10.0);

  // Assert:
  ASSERT_EQ(out.size(), 1U);
  EXPECT_EQ(out.front().departure_type, DepartureType::NEAR_BOUNDARY);
}
}  // namespace autoware::boundary_departure_checker
