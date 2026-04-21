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

#include "autoware/boundary_departure_checker/type_alias.hpp"
#include "autoware/boundary_departure_checker/utils.hpp"

#include <autoware/motion_utils/distance/distance.hpp>

#include <gtest/gtest.h>

#include <string>
#include <utility>
#include <vector>

namespace autoware::boundary_departure_checker
{
namespace
{
TrajectoryPoint make_trajectory_point(double x, double y, double time)
{
  TrajectoryPoint p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.time_from_start.sec = static_cast<int32_t>(time);
  p.time_from_start.nanosec = static_cast<uint32_t>((time - p.time_from_start.sec) * 1e9);
  return p;
}

FootprintSideSegments make_footprint_sides(Segment2d left, Segment2d right)
{
  FootprintSideSegments s;
  s.left = std::move(left);
  s.right = std::move(right);
  return s;
}

BoundarySegmentsBySide make_boundaries(
  const std::vector<Segment2d> & left_segments, const std::vector<Segment2d> & right_segments = {})
{
  BoundarySegmentsBySide boundaries;
  for (const auto & seg : left_segments) {
    boundaries.left.emplace_back(seg, IdxForRTreeSegment{1, 0, 1});
  }
  for (const auto & seg : right_segments) {
    boundaries.right.emplace_back(seg, IdxForRTreeSegment{2, 0, 1});
  }
  return boundaries;
}
}  // namespace

// clang-format off
/**
 * TestParallelSegments:
 *
 *    Y ^
 *  1.0 |       [ Boundary Segment ] (X: 1.0 to 3.0)
 *      |       +------------------+
 *      |       : <--- lat_dist=1.0
 *  0.0 | +------------------+             ---> X
 *      | [   Ego Segment    ] (X: 0.0 to 2.0)
 *
 * Verifies shortest distance between two parallel segments with overlap.
 */
// clang-format on
TEST(UncrossableBoundaryTest, TestParallelSegments)
{
  // Verifies projection distance between parallel segments with partial longitudinal overlap.

  // Arrange:
  Segment2d ego_seg{{0.0, 0.0}, {2.0, 0.0}};
  Segment2d boundary_seg{{1.0, 1.0}, {3.0, 1.0}};

  // Act:
  auto result = utils::calc_nearest_projection(ego_seg, boundary_seg, 0);

  // Assert:
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->lat_dist, 1.0, 1e-6);
  EXPECT_GE(result->pt_on_ego.x(), 0.0);
  EXPECT_LE(result->pt_on_ego.x(), 2.0);
}

// clang-format off
/**
 * TestPerpendicularNonIntersecting:
 *
 *    Y ^
 *  1.0 |               | [ Boundary ]
 *      |               | (X=2.0, Y: -1 to 1)
 *  0.0 | +-------+     | lat_dist=1.0     ---> X
 *      | (Ego)         |
 * -1.0 | (X: 0 to 1)   |
 *
 * Verifies shortest distance between perpendicular segments.
 */
// clang-format on
TEST(UncrossableBoundaryTest, TestPerpendicularNonIntersecting)
{
  // Verifies that perpendicular non-intersecting segments yield correct shortest distance.

  // Arrange:
  Segment2d ego_seg{{0.0, 0.0}, {1.0, 0.0}};
  Segment2d boundary_seg{{2.0, -1.0}, {2.0, 1.0}};

  // Act:
  auto result = utils::calc_nearest_projection(ego_seg, boundary_seg, 0);

  // Assert:
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->lat_dist, 1.0, 1e-6);
  EXPECT_DOUBLE_EQ(result->pt_on_ego.x(), 1.0);
  EXPECT_DOUBLE_EQ(result->pt_on_bound.x(), 2.0);
}

// clang-format off
/**
 * TestCollinearSegments:
 *
 *    Y ^
 *      |
 *  0.0 | +-------+     +-------+          ---> X
 *      |  (Ego)         (Boundary)
 *      |  (X:0 to 1)    (X:2 to 3)
 *      |         (Gap: No Projection)
 *
 * Verifies that no projection is found for collinear separated segments.
 */
// clang-format on
TEST(UncrossableBoundaryTest, TestCollinearSegments)
{
  // Verifies that no projection is found for collinear but longitudinally separated segments.

  // Arrange:
  Segment2d ego_seg{{0.0, 0.0}, {1.0, 0.0}};
  Segment2d boundary_seg{{2.0, 0.0}, {3.0, 0.0}};

  // Act:
  auto result = utils::calc_nearest_projection(ego_seg, boundary_seg, 0);

  // Assert:
  ASSERT_FALSE(result.has_value());
}

TEST(UncrossableBoundaryTest, TestMiddleOfSegmentCrossingForLonDist)
{
  // Verifies accurate longitudinal distance calculation when boundary crosses in the middle of an
  // ego segment.

  // Arrange:
  TrajectoryPoints ego_pred_traj = {
    make_trajectory_point(0.0, 0.0, 0.0), make_trajectory_point(10.0, 0.0, 1.0),
    make_trajectory_point(20.0, 0.0, 2.0)};

  FootprintSideSegmentsArray ego_sides = {
    make_footprint_sides(Segment2d{{2.0, 1.0}, {-2.0, 1.0}}, Segment2d{{2.0, -1.0}, {-2.0, -1.0}}),
    make_footprint_sides(Segment2d{{12.0, 1.0}, {8.0, 1.0}}, Segment2d{{12.0, -1.0}, {8.0, -1.0}}),
    make_footprint_sides(
      Segment2d{{22.0, 1.0}, {18.0, 1.0}}, Segment2d{{22.0, -1.0}, {18.0, -1.0}})};

  BoundarySegmentsBySide boundaries = make_boundaries({Segment2d{{10.0, 0.0}, {10.0, 2.0}}});

  // Act:
  auto result =
    utils::get_closest_boundary_segments_from_side(ego_pred_traj, boundaries, ego_sides);

  // Assert:
  ASSERT_EQ(result.left.size(), 3);
  const auto & proj_at_i = result.left[1];
  EXPECT_DOUBLE_EQ(proj_at_i.lat_dist, 0.0);
  EXPECT_DOUBLE_EQ(proj_at_i.pt_on_ego.x(), 10.0);
  EXPECT_DOUBLE_EQ(proj_at_i.ego_front_to_proj_offset_m, 2.0);
  EXPECT_DOUBLE_EQ(proj_at_i.dist_along_trajectory_m, 10.0);
}

TEST(UncrossableBoundaryTest, TestRealisticLaneDeparture)
{
  // Verifies signed lateral distance transitions during a realistic left-side departure.

  // Arrange:
  TrajectoryPoints ego_pred_traj = {
    make_trajectory_point(0.0, 0.0, 0.0), make_trajectory_point(24.0, 7.0, 1.0),
    make_trajectory_point(48.0, 14.0, 2.0), make_trajectory_point(72.0, 21.0, 3.0),
    make_trajectory_point(96.0, 28.0, 4.0)};

  FootprintSideSegmentsArray ego_sides = {
    make_footprint_sides(Segment2d{{4.1, 3.8}, {-5.5, 1.0}}, Segment2d{{5.5, -1.0}, {-4.1, -3.8}}),
    make_footprint_sides(Segment2d{{28.1, 10.8}, {18.5, 8.0}}, Segment2d{{29.5, 6.0}, {19.9, 3.2}}),
    make_footprint_sides(
      Segment2d{{52.1, 17.8}, {42.5, 15.0}}, Segment2d{{53.5, 13.0}, {43.9, 10.2}}),
    make_footprint_sides(
      Segment2d{{76.1, 24.8}, {66.5, 22.0}}, Segment2d{{77.5, 20.0}, {67.9, 17.2}}),
    make_footprint_sides(
      Segment2d{{100.1, 31.8}, {90.5, 29.0}}, Segment2d{{101.5, 27.0}, {91.9, 24.2}})};

  BoundarySegmentsBySide boundaries = make_boundaries(
    {Segment2d{{-10.0, 9.4}, {120.0, 9.4}}}, {Segment2d{{-10.0, -5.0}, {120.0, -5.0}}});

  // Act:
  auto result =
    utils::get_closest_boundary_segments_from_side(ego_pred_traj, boundaries, ego_sides);

  // Assert:
  EXPECT_NEAR(result.left[0].lat_dist, 5.6, 1e-6);
  EXPECT_NEAR(result.left[1].lat_dist, 0.0, 1e-6);
  EXPECT_NEAR(result.left[2].lat_dist, -5.6, 1e-6);
}

TEST(UncrossableBoundaryTest, TestRealisticRightLaneDeparture)
{
  // Verifies signed lateral distance transitions during a realistic right-side departure.

  // Arrange:
  TrajectoryPoints ego_pred_traj = {
    make_trajectory_point(0.0, 0.0, 0.0), make_trajectory_point(24.0, -7.0, 1.0),
    make_trajectory_point(48.0, -14.0, 2.0), make_trajectory_point(72.0, -21.0, 3.0),
    make_trajectory_point(96.0, -28.0, 4.0)};

  FootprintSideSegmentsArray ego_sides = {
    make_footprint_sides(Segment2d{{5.5, 1.0}, {-4.1, 3.8}}, Segment2d{{4.1, -3.8}, {-5.5, -1.0}}),
    make_footprint_sides(
      Segment2d{{29.5, -6.0}, {19.9, -3.2}}, Segment2d{{28.1, -10.8}, {18.5, -8.0}}),
    make_footprint_sides(
      Segment2d{{53.5, -13.0}, {43.9, -10.2}}, Segment2d{{52.1, -17.8}, {42.5, -15.0}}),
    make_footprint_sides(
      Segment2d{{77.5, -20.0}, {67.9, -17.2}}, Segment2d{{76.1, -24.8}, {66.5, -22.0}}),
    make_footprint_sides(
      Segment2d{{101.5, -27.0}, {91.9, -24.2}}, Segment2d{{100.1, -31.8}, {90.5, -29.0}})};

  BoundarySegmentsBySide boundaries = make_boundaries(
    {Segment2d{{-10.0, 5.0}, {120.0, 5.0}}}, {Segment2d{{-10.0, -9.4}, {120.0, -9.4}}});

  // Act:
  auto result =
    utils::get_closest_boundary_segments_from_side(ego_pred_traj, boundaries, ego_sides);

  // Assert:
  EXPECT_NEAR(result.right[0].lat_dist, 5.6, 1e-6);
  EXPECT_NEAR(result.right[1].lat_dist, 0.0, 1e-6);
  EXPECT_NEAR(result.right[2].lat_dist, -5.6, 1e-6);
}

TEST(UncrossableBoundaryUtilsTest, TestCalcJudgeLineDist)
{
  // Verifies braking distance calculation with jerk and acceleration limits.

  // Arrange:
  constexpr double acceleration = 0.0;
  constexpr double max_stop_accel = -4.0;
  constexpr double max_stop_jerk = -10.0;
  constexpr double delay_time = 1.0;
  constexpr double v_test = 10.0;

  // Act:
  const auto dist_opt = autoware::motion_utils::calculate_stop_distance(
    v_test, acceleration, max_stop_accel, max_stop_jerk, delay_time);

  // Assert:
  ASSERT_TRUE(dist_opt.has_value());
  EXPECT_GT(*dist_opt, 22.5);
}

TEST(UncrossableBoundaryUtilsTest, TestPointToSegmentProjection)
{
  // Verifies point-to-segment projection and resulting distance.

  // Arrange:
  autoware_utils_geometry::Point2d p{0.5, 1.0};
  Segment2d segment{{0.0, 0.0}, {1.0, 0.0}};

  // Act:
  auto result = utils::point_to_segment_projection(p, segment);

  // Assert:
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->second, 1.0);
}

TEST(UncrossableBoundaryUtilsTest, TestIsUncrossableType)
{
  // Verifies boundary type filtering logic.

  // Arrange:
  lanelet::LineString3d ls(lanelet::utils::getId());
  ls.attributes()[lanelet::AttributeName::Type] = "road_border";
  std::vector<std::string> types = {"road_border", "curb"};

  // Act & Assert:
  EXPECT_TRUE(utils::is_uncrossable_type(types, ls));
  ls.attributes()[lanelet::AttributeName::Type] = "lane_divider";
  EXPECT_FALSE(utils::is_uncrossable_type(types, ls));
}

TEST(UncrossableBoundaryUtilsTest, TestEvaluateProjectionsSeverityBackwardBuffer)
{
  // Verifies that backward buffering correctly identifies the start of a critical departure zone.

  // Arrange:
  Side<ProjectionsToBound> input;
  UncrossableBoundaryDepartureParam param;
  param.lateral_margin_m = 0.5;
  param.time_to_departure_cutoff_s = 2.0;
  param.longitudinal_margin_m = 1.0;
  double min_braking_dist = 10.0;

  auto create_pt = [](size_t idx, double s, double time) {
    ProjectionToBound pt(idx);
    pt.lat_dist = 0.1;
    pt.dist_along_trajectory_m = s;
    pt.ego_front_to_proj_offset_m = 0.0;
    pt.time_from_start = time;
    pt.pt_on_ego = {s, 0.1};
    pt.pt_on_bound = {s, 0.0};
    return pt;
  };

  input.left.push_back(create_pt(0, 12.0, 2.4));
  input.left.push_back(create_pt(1, 13.0, 2.6));
  input.left.push_back(create_pt(2, 14.0, 2.8));
  input.left.push_back(create_pt(3, 15.0, 1.9));

  // Act:
  auto result = input.transform_each_side([&](const auto & side_value) {
    const auto min_to_bounds =
      utils::filter_and_assign_departure_types(side_value, param, min_braking_dist);
    return utils::apply_backward_buffer_and_filter(min_to_bounds, param);
  });

  // Assert:
  ASSERT_TRUE(result.left.has_value());
  EXPECT_DOUBLE_EQ(result.left->physical_departure_point.dist_along_trajectory_m, 15.0);
  EXPECT_DOUBLE_EQ(result.left->safety_buffer_start.dist_along_trajectory_m, 14.0);
}

TEST(UncrossableBoundaryUtilsTest, TestBuildUncrossableBoundariesRTree)
{
  // Verifies the construction and querying of an R-tree for uncrossable boundary segments.

  // Arrange:
  lanelet::LaneletMap map;
  lanelet::Point3d p1(lanelet::utils::getId(), 0.0, 0.0, 0.0);
  lanelet::Point3d p2(lanelet::utils::getId(), 1.0, 0.0, 0.0);
  lanelet::Point3d p3(lanelet::utils::getId(), 2.0, 0.0, 0.0);
  lanelet::LineString3d ls1(lanelet::utils::getId(), {p1, p2, p3});
  ls1.attributes()[lanelet::AttributeName::Type] = "road_border";
  map.add(ls1);
  std::vector<std::string> types = {"road_border"};

  // Act:
  auto rtree = utils::build_uncrossable_boundaries_rtree(map, types);

  // Assert:
  EXPECT_EQ(rtree.size(), 2);
  std::vector<SegmentWithIdx> results;
  rtree.query(
    boost::geometry::index::nearest(lanelet::BasicPoint2d(0.5, 0.0), 1),
    std::back_inserter(results));
  ASSERT_EQ(results.size(), 1);
  EXPECT_EQ(results.front().second.linestring_id, ls1.id());
}
}  // namespace autoware::boundary_departure_checker
