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

#include "autoware/boundary_departure_checker/boundary_segment_finder.hpp"
#include "autoware/boundary_departure_checker/geometry_projection.hpp"
#include "autoware/boundary_departure_checker/severity_evaluator.hpp"
#include "autoware/boundary_departure_checker/type_alias.hpp"
#include "autoware/boundary_departure_checker/uncrossable_boundaries_rtree.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace
{
using autoware::boundary_departure_checker::ProjectionToBound;
using autoware_utils_geometry::Segment2d;
}  // namespace

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
  size_t curr_fp_idx = 42;

  std::vector<SegmentWithIdx> boundary_segments;
  IdxForRTreeSegment id{1, 0, 1};
  boundary_segments.emplace_back(Segment2d{{1.0, -1.0}, {1.0, 1.0}}, id);

  // Act:
  auto result = geometry_projection::find_closest_segment(
    ego_side_seg, ego_rear_seg, curr_fp_idx, boundary_segments);

  // Assert:
  EXPECT_EQ(result.pose_index, curr_fp_idx);
  EXPECT_DOUBLE_EQ(result.lat_dist, 0.0);
  EXPECT_DOUBLE_EQ(result.pt_on_ego.x(), 1.0);
  EXPECT_DOUBLE_EQ(result.pt_on_ego.y(), 0.0);
}

TEST(UncrossableBoundaryUtilsTest, TestFindClosestSegmentFallback)
{
  // Arrange:
  Segment2d ego_side_seg{{0.0, 4.0}, {0.0, 2.0}};
  Segment2d ego_rear_seg{{0.0, 0.0}, {2.0, 0.0}};
  size_t curr_fp_idx = 99;

  std::vector<SegmentWithIdx> boundary_segments;
  IdxForRTreeSegment id{1, 0, 1};
  boundary_segments.emplace_back(Segment2d{{4.0, -2.0}, {5.0, -2.0}}, id);

  // Act:
  auto result = geometry_projection::find_closest_segment(
    ego_side_seg, ego_rear_seg, curr_fp_idx, boundary_segments);

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

  UncrossableBoundariesRTree rtree(*map, {"road_border"});

  // Act & Assert:
  IdxForRTreeSegment id1{ls.id(), 0, 1};
  auto seg1 = rtree.get_segment_3d_from_id(map, id1);
  EXPECT_DOUBLE_EQ(seg1.first.x(), 1.0);
  EXPECT_DOUBLE_EQ(seg1.second.x(), 4.0);

  IdxForRTreeSegment id2{ls.id(), 1, 2};
  auto seg2 = rtree.get_segment_3d_from_id(map, id2);
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
  Side<std::optional<CriticalPointPair>> projections;

  // Act & Assert:
  EXPECT_FALSE(severity_evaluator::is_critical(projections));

  CriticalPointPair pair_crit;
  pair_crit.physical_departure_point.departure_type = DepartureType::CRITICAL;
  projections.left = pair_crit;
  EXPECT_TRUE(severity_evaluator::is_critical(projections));
}
}  // namespace autoware::boundary_departure_checker
