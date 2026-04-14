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
#include "test_plot_utils.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace
{
using autoware::boundary_departure_checker::ProjectionToBound;
using autoware_utils_geometry::Segment2d;

void plot_separate_segment(
  [[maybe_unused]] autoware::pyplot::PyPlot & plt, [[maybe_unused]] const Segment2d & segment,
  [[maybe_unused]] const std::string & color, [[maybe_unused]] const std::string & label)
{
#ifdef EXPORT_TEST_PLOT_FIGURE
  plt.plot(
    Args(
      std::vector<double>{segment.first.x(), segment.second.x()},
      std::vector<double>{segment.first.y(), segment.second.y()}),
    Kwargs("color"_a = color, "label"_a = label));
#endif
}

void plot_projection_line(
  [[maybe_unused]] autoware::pyplot::PyPlot & plt,
  [[maybe_unused]] const ProjectionToBound & projection)
{
#ifdef EXPORT_TEST_PLOT_FIGURE
  plt.plot(
    Args(
      std::vector<double>{projection.pt_on_ego.x(), projection.pt_on_bound.x()},
      std::vector<double>{projection.pt_on_ego.y(), projection.pt_on_bound.y()}),
    Kwargs("color"_a = "green", "linestyle"_a = "--", "label"_a = "Shortest Projection"));
#endif
}

void plot_ego_and_boundary(
  [[maybe_unused]] const Segment2d & ego_seg, [[maybe_unused]] const Segment2d & boundary_seg,
  [[maybe_unused]] const tl::expected<ProjectionToBound, std::string> & projection_opt)
{
#ifdef EXPORT_TEST_PLOT_FIGURE
  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();
    plot_separate_segment(plt, ego_seg, "blue", "Ego Side");
    plot_separate_segment(plt, boundary_seg, "red", "Boundary");
    if (projection_opt) {
      plot_projection_line(plt, *projection_opt);
    }
    plt.legend();
    plt.axis(Args("equal"));
    autoware::boundary_departure_checker::save_figure(plt, "test_boundary_utils");
  });
#endif
}

void plot_find_closest_segment(
  [[maybe_unused]] const Segment2d & ego_side_seg, [[maybe_unused]] const Segment2d & ego_rear_seg,
  [[maybe_unused]] const Segment2d & boundary_seg,
  [[maybe_unused]] const ProjectionToBound & result)
{
#ifdef EXPORT_TEST_PLOT_FIGURE
  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();
    std::vector<double> ego_box_x = {0.0, 0.0, 2.0, 2.0, 0.0};
    std::vector<double> ego_box_y = {4.0, 0.0, 0.0, 4.0, 4.0};
    plt.plot(
      Args(ego_box_x, ego_box_y),
      Kwargs(
        "color"_a = "gray", "linestyle"_a = "--", "alpha"_a = 0.5, "label"_a = "Full Ego Vehicle"));

    plot_separate_segment(plt, ego_side_seg, "blue", "Evaluated Side Sub-segment");
    plot_separate_segment(plt, ego_rear_seg, "cyan", "Evaluated Ego Rear");
    plot_separate_segment(plt, boundary_seg, "red", "Boundary Segment");

    if (result.lat_dist != std::numeric_limits<double>::max()) {
      plt.scatter(
        Args(std::vector<double>{result.pt_on_ego.x()}, std::vector<double>{result.pt_on_ego.y()}),
        Kwargs("color"_a = "orange", "label"_a = "Intersection/Projection Point", "zorder"_a = 5));
    }

    plt.legend();
    plt.axis(Args("equal"));
    autoware::boundary_departure_checker::save_figure(plt, "test_boundary_utils");
  });
#endif
}

void plot_lateral_raycast(
  [[maybe_unused]] const std::vector<double> & bound_x,
  [[maybe_unused]] const std::vector<double> & bound_y, [[maybe_unused]] const double dist_multi)
{
#ifdef EXPORT_TEST_PLOT_FIGURE
  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();
    plt.plot(Args(bound_x, bound_y), Kwargs("color"_a = "red", "label"_a = "U-Shaped Boundary"));
    plt.scatter(
      Args(std::vector<double>{0.0}, std::vector<double>{0.0}),
      Kwargs("color"_a = "black", "s"_a = 100, "marker"_a = "o", "label"_a = "Ego Pose"));

    std::vector<double> ray_x = {0.0, 0.0};
    std::vector<double> ray_y = {0.0, dist_multi};
    plt.plot(
      Args(ray_x, ray_y),
      Kwargs("color"_a = "green", "linestyle"_a = "--", "label"_a = "Lateral Ray (Selected)"));

    std::vector<double> ray_miss_x = {0.0, 0.0};
    std::vector<double> ray_miss_y = {dist_multi, 8.0};
    plt.plot(
      Args(ray_miss_x, ray_miss_y),
      Kwargs("color"_a = "gray", "linestyle"_a = ":", "label"_a = "Ignored Further Intersection"));

    plt.axis(Args("equal"));
    plt.legend();
    plt.title(Args("Signed Lateral Raycast to Closest Boundary"));
    autoware::boundary_departure_checker::save_figure(plt, "test_boundary_utils");
  });
#endif
}
}  // namespace

namespace autoware::boundary_departure_checker
{
// ==============================================================================
// 1. calc_nearest_projection Tests
// ==============================================================================

TEST(UncrossableBoundaryUtilsTest, TestSegmentToSegmentProjection)
{
  // 1-line summary: Verifies shortest distance between two parallel segments.

  // Arrange:
  Segment2d ego_seg{{0.0, 0.0}, {0.0, 2.0}};
  Segment2d boundary_seg{{1.0, 0.0}, {1.0, 2.0}};

  // Act:
  auto result = utils::calc_nearest_projection(ego_seg, boundary_seg, 0);

  // Assert:
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->lat_dist, 1.0, 1e-6);
  plot_ego_and_boundary(ego_seg, boundary_seg, result);
}

TEST(UncrossableBoundaryUtilsTest, TestIntersectionDetection)
{
  // 1-line summary: Verifies that intersecting segments yield zero lateral distance.

  // Arrange:
  Segment2d ego_seg{{0.0, 0.0}, {1.0, 2.0}};
  Segment2d boundary_seg{{1.0, 0.0}, {1.0, 2.0}};

  // Act:
  auto result = utils::calc_nearest_projection(ego_seg, boundary_seg, 0);

  // Assert:
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->lat_dist, 0.0);
  plot_ego_and_boundary(ego_seg, boundary_seg, result);
}

TEST(UncrossableBoundaryUtilsTest, TestPointBeyondSegmentEnd)
{
  // 1-line summary: Verifies that no projection is found when segments are longitudinally
  // separated.

  // Arrange:
  Segment2d ego_seg{{0.0, 0.0}, {1.0, 0.0}};
  Segment2d boundary_seg{{3.0, 2.0}, {4.0, 2.0}};

  // Act:
  auto result = utils::calc_nearest_projection(ego_seg, boundary_seg, 0);

  // Assert:
  ASSERT_FALSE(result.has_value());
  plot_ego_and_boundary(ego_seg, boundary_seg, result);
}

// ==============================================================================
// 2. find_closest_segment Tests
// ==============================================================================

TEST(UncrossableBoundaryUtilsTest, TestFindClosestSegmentRearIntersection)
{
  // 1-line summary: Verifies fallback to rear intersection when lateral projection fails.

  // Arrange:
  Segment2d ego_side_seg{{0.0, 4.0}, {0.0, 2.0}};
  Segment2d ego_rear_seg{{0.0, 0.0}, {2.0, 0.0}};
  size_t curr_fp_idx = 42;

  std::vector<SegmentWithIdx> boundary_segments;
  IdxForRTreeSegment id{1, 0, 1};
  boundary_segments.emplace_back(Segment2d{{1.0, -1.0}, {1.0, 1.0}}, id);

  // Act:
  auto result =
    utils::find_closest_segment(ego_side_seg, ego_rear_seg, curr_fp_idx, boundary_segments);

  // Assert:
  EXPECT_EQ(result.pose_index, curr_fp_idx);
  EXPECT_DOUBLE_EQ(result.lat_dist, 0.0);
  EXPECT_DOUBLE_EQ(result.pt_on_ego.x(), 1.0);
  EXPECT_DOUBLE_EQ(result.pt_on_ego.y(), 0.0);
  plot_find_closest_segment(ego_side_seg, ego_rear_seg, boundary_segments.front().first, result);
}

TEST(UncrossableBoundaryUtilsTest, TestFindClosestSegmentFallback)
{
  // 1-line summary: Verifies that max distance is returned when no projection or intersection
  // exists.

  // Arrange:
  Segment2d ego_side_seg{{0.0, 4.0}, {0.0, 2.0}};
  Segment2d ego_rear_seg{{0.0, 0.0}, {2.0, 0.0}};
  size_t curr_fp_idx = 99;

  std::vector<SegmentWithIdx> boundary_segments;
  IdxForRTreeSegment id{1, 0, 1};
  boundary_segments.emplace_back(Segment2d{{4.0, -2.0}, {5.0, -2.0}}, id);

  // Act:
  auto result =
    utils::find_closest_segment(ego_side_seg, ego_rear_seg, curr_fp_idx, boundary_segments);

  // Assert:
  EXPECT_EQ(result.pose_index, curr_fp_idx);
  EXPECT_DOUBLE_EQ(result.lat_dist, std::numeric_limits<double>::max());
  plot_find_closest_segment(ego_side_seg, ego_rear_seg, boundary_segments.front().first, result);
}

TEST(UncrossableBoundaryUtilsTest, TestCalcSignedLateralDistanceToBoundary)
{
  // 1-line summary: Verifies signed lateral raycast distance to various boundary configurations.

  // Arrange:
  geometry_msgs::msg::Pose ego_pose;
  ego_pose.position.x = 0.0;
  ego_pose.position.y = 0.0;
  ego_pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(0.0);

  // Act & Assert:
  lanelet::LineString3d empty_ls(lanelet::utils::getId());
  EXPECT_FALSE(utils::calc_signed_lateral_distance_to_boundary(empty_ls, ego_pose).has_value());

  lanelet::Point3d p1(lanelet::utils::getId(), -5.0, 5.0, 0.0);
  lanelet::LineString3d single_pt_ls(lanelet::utils::getId(), {p1});
  EXPECT_FALSE(utils::calc_signed_lateral_distance_to_boundary(single_pt_ls, ego_pose).has_value());

  lanelet::Point3d p2(lanelet::utils::getId(), 5.0, 5.0, 0.0);
  lanelet::LineString3d left_boundary(lanelet::utils::getId(), {p1, p2});
  auto dist_left = utils::calc_signed_lateral_distance_to_boundary(left_boundary, ego_pose);
  ASSERT_TRUE(dist_left.has_value());
  EXPECT_DOUBLE_EQ(dist_left.value(), 5.0);

  lanelet::Point3d p3(lanelet::utils::getId(), -5.0, -3.0, 0.0);
  lanelet::Point3d p4(lanelet::utils::getId(), 5.0, -3.0, 0.0);
  lanelet::LineString3d right_boundary(lanelet::utils::getId(), {p3, p4});
  auto dist_right = utils::calc_signed_lateral_distance_to_boundary(right_boundary, ego_pose);
  ASSERT_TRUE(dist_right.has_value());
  EXPECT_DOUBLE_EQ(dist_right.value(), -3.0);

  lanelet::Point3d p5(lanelet::utils::getId(), 2.0, 5.0, 0.0);
  lanelet::Point3d p6(lanelet::utils::getId(), 10.0, 5.0, 0.0);
  lanelet::LineString3d missed_boundary(lanelet::utils::getId(), {p5, p6});
  auto dist_miss = utils::calc_signed_lateral_distance_to_boundary(missed_boundary, ego_pose);
  EXPECT_FALSE(dist_miss.has_value());

  lanelet::Point3d p7(lanelet::utils::getId(), 5.0, -10.0, 0.0);
  lanelet::Point3d p8(lanelet::utils::getId(), 5.0, 10.0, 0.0);
  lanelet::LineString3d parallel_boundary(lanelet::utils::getId(), {p7, p8});
  auto dist_parallel = utils::calc_signed_lateral_distance_to_boundary(parallel_boundary, ego_pose);
  EXPECT_FALSE(dist_parallel.has_value());

  lanelet::Point3d p9(lanelet::utils::getId(), -10.0, 8.0, 0.0);
  lanelet::Point3d p10(lanelet::utils::getId(), 10.0, 8.0, 0.0);
  lanelet::Point3d p11(lanelet::utils::getId(), 10.0, 4.0, 0.0);
  lanelet::Point3d p12(lanelet::utils::getId(), -10.0, 4.0, 0.0);
  lanelet::LineString3d multi_boundary(lanelet::utils::getId(), {p9, p10, p11, p12});
  auto dist_multi = utils::calc_signed_lateral_distance_to_boundary(multi_boundary, ego_pose);
  ASSERT_TRUE(dist_multi.has_value());
  EXPECT_DOUBLE_EQ(dist_multi.value(), 4.0);

  plot_lateral_raycast(
    {p9.x(), p10.x(), p11.x(), p12.x()}, {p9.y(), p10.y(), p11.y(), p12.y()}, dist_multi.value());
}

// ==============================================================================
// 3. Extracted Utility Function Tests
// ==============================================================================

TEST(UncrossableBoundaryUtilsTest, TestGetSegment3DFromId)
{
  // 1-line summary: Verifies extraction of 3D segments from LaneletMap using IDs.

  // Arrange:
  auto map = std::make_shared<lanelet::LaneletMap>();
  lanelet::Point3d p1(lanelet::utils::getId(), 1.0, 2.0, 3.0);
  lanelet::Point3d p2(lanelet::utils::getId(), 4.0, 5.0, 6.0);
  lanelet::Point3d p3(lanelet::utils::getId(), 7.0, 8.0, 9.0);
  lanelet::LineString3d ls(lanelet::utils::getId(), {p1, p2, p3});
  map->add(ls);

  // Act & Assert:
  IdxForRTreeSegment id1{ls.id(), 0, 1};
  auto seg1 = utils::get_segment_3d_from_id(map, id1);
  EXPECT_DOUBLE_EQ(seg1.first.x(), 1.0);
  EXPECT_DOUBLE_EQ(seg1.second.x(), 4.0);

  IdxForRTreeSegment id2{ls.id(), 1, 2};
  auto seg2 = utils::get_segment_3d_from_id(map, id2);
  EXPECT_DOUBLE_EQ(seg2.first.x(), 4.0);
  EXPECT_DOUBLE_EQ(seg2.second.x(), 7.0);
}

TEST(UncrossableBoundaryUtilsTest, TestIsClosestToBoundarySegment)
{
  // 1-line summary: Verifies logic for determining which ego side is closer to a boundary segment.

  // Arrange:
  Segment2d left_side{{0.0, 2.0}, {2.0, 2.0}};
  Segment2d right_side{{0.0, -2.0}, {2.0, -2.0}};

  // Act & Assert:
  Segment2d bound_left{{0.0, 3.0}, {2.0, 3.0}};
  EXPECT_TRUE(utils::is_closest_to_boundary_segment(bound_left, left_side, right_side));

  Segment2d bound_right{{0.0, -3.0}, {2.0, -3.0}};
  EXPECT_TRUE(utils::is_closest_to_boundary_segment(bound_right, right_side, left_side));

  Segment2d bound_center{{0.0, 0.0}, {2.0, 0.0}};
  EXPECT_TRUE(utils::is_closest_to_boundary_segment(bound_center, left_side, right_side));
}

TEST(UncrossableBoundaryUtilsTest, TestIsSegmentWithinEgoHeight)
{
  // 1-line summary: Verifies vertical filtering of boundary segments based on vehicle height.

  // Arrange:
  const double ego_z = 0.0;
  const double ego_height = 2.5;

  // Act & Assert:
  Segment3d seg_within{{0.0, 0.0, 1.0}, {1.0, 0.0, 1.5}};
  EXPECT_TRUE(utils::is_segment_within_ego_height(seg_within, ego_z, ego_height));

  Segment3d seg_above{{0.0, 0.0, 3.0}, {1.0, 0.0, 4.0}};
  EXPECT_FALSE(utils::is_segment_within_ego_height(seg_above, ego_z, ego_height));

  Segment3d seg_below{{0.0, 0.0, -3.0}, {1.0, 0.0, -4.0}};
  EXPECT_FALSE(utils::is_segment_within_ego_height(seg_below, ego_z, ego_height));
}

TEST(UncrossableBoundaryUtilsTest, TestIsCritical)
{
  // 1-line summary: Verifies the logic for identifying a critical departure from either side.

  // Arrange:
  Side<std::optional<CriticalPointPair>> projections;

  // Act & Assert:
  EXPECT_FALSE(utils::is_critical(projections));

  CriticalPointPair pair_crit;
  pair_crit.physical_departure_point.departure_type = DepartureType::CRITICAL;
  projections.left = pair_crit;
  EXPECT_TRUE(utils::is_critical(projections));
}
}  // namespace autoware::boundary_departure_checker
