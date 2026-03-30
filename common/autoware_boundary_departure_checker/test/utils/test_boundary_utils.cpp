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
#ifdef EXPORT_TEST_PLOT_FIGURE
using autoware::boundary_departure_checker::ProjectionToBound;
using autoware_utils_geometry::Segment2d;

void plot_separate_segment(
  autoware::pyplot::PyPlot & plt, const Segment2d & segment, const std::string & color,
  const std::string & label)
{
  plt.plot(
    Args(
      std::vector<double>{segment.first.x(), segment.second.x()},
      std::vector<double>{segment.first.y(), segment.second.y()}),
    Kwargs("color"_a = color, "label"_a = label));
}

void plot_projection_line(autoware::pyplot::PyPlot & plt, const ProjectionToBound & projection)
{
  plt.plot(
    Args(
      std::vector<double>{projection.pt_on_ego.x(), projection.pt_on_bound.x()},
      std::vector<double>{projection.pt_on_ego.y(), projection.pt_on_bound.y()}),
    Kwargs("color"_a = "green", "linestyle"_a = "--", "label"_a = "Shortest Projection"));
}

void plot_ego_and_boundary(
  autoware::pyplot::PyPlot & plt, const Segment2d & ego_seg, const Segment2d & boundary_seg,
  const tl::expected<ProjectionToBound, std::string> & projection_opt)
{
  plot_separate_segment(plt, ego_seg, "blue", "Ego Side");
  plot_separate_segment(plt, boundary_seg, "red", "Boundary");
  if (projection_opt) {
    plot_projection_line(plt, *projection_opt);
  }
  plt.legend();
  plt.axis(Args("equal"));
}

void plot_find_closest_segment(
  autoware::pyplot::PyPlot & plt, const Segment2d & ego_side_seg, const Segment2d & ego_rear_seg,
  const Segment2d & boundary_seg, const ProjectionToBound & result)
{
  // Draw the full "Ghost" Ego Vehicle for context
  std::vector<double> ego_box_x = {0.0, 0.0, 2.0, 2.0, 0.0};
  std::vector<double> ego_box_y = {4.0, 0.0, 0.0, 4.0, 4.0};
  plt.plot(
    Args(ego_box_x, ego_box_y),
    Kwargs(
      "color"_a = "gray", "linestyle"_a = "--", "alpha"_a = 0.5, "label"_a = "Full Ego Vehicle"));

  plot_separate_segment(plt, ego_side_seg, "blue", "Evaluated Side Sub-segment");
  plot_separate_segment(plt, ego_rear_seg, "cyan", "Evaluated Ego Rear");
  plot_separate_segment(plt, boundary_seg, "red", "Boundary Segment");

  // Only plot the point if a valid projection/intersection was actually found
  if (result.lat_dist != std::numeric_limits<double>::max()) {
    plt.scatter(
      Args(std::vector<double>{result.pt_on_ego.x()}, std::vector<double>{result.pt_on_ego.y()}),
      Kwargs("color"_a = "orange", "label"_a = "Intersection/Projection Point", "zorder"_a = 5));
  }

  plt.legend();
  plt.axis(Args("equal"));
}
#endif
}  // namespace

namespace autoware::boundary_departure_checker
{
constexpr const char * export_folder = "test_boundary_utils";

// ==============================================================================
// 1. calc_nearest_projection Tests
// ==============================================================================

TEST(UncrossableBoundaryUtilsTest, TestSegmentToSegmentProjection)
{
  // 1. Setup PyPlot context

  // Define segments
  Segment2d ego_seg{{0.0, 0.0}, {0.0, 2.0}};
  Segment2d boundary_seg{{1.0, 0.0}, {1.0, 2.0}};

  auto result = utils::calc_nearest_projection(ego_seg, boundary_seg, 0);

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->lat_dist, 1.0, 1e-6);
  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();
    plot_ego_and_boundary(plt, ego_seg, boundary_seg, result);
    save_figure(plt, export_folder);
  });
}

TEST(UncrossableBoundaryUtilsTest, TestIntersectionDetection)
{
  // Ego side segment crossing the boundary
  Segment2d ego_seg{{0.0, 0.0}, {1.0, 2.0}};
  Segment2d boundary_seg{{1.0, 0.0}, {1.0, 2.0}};

  auto result = utils::calc_nearest_projection(ego_seg, boundary_seg, 0);

  ASSERT_TRUE(result.has_value());
  // Distance should be 0.0 because they intersect
  EXPECT_DOUBLE_EQ(result->lat_dist, 0.0);

  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();
    plot_ego_and_boundary(plt, ego_seg, boundary_seg, result);
    save_figure(plt, export_folder);
  });
}

TEST(UncrossableBoundaryUtilsTest, TestPointBeyondSegmentEnd)
{
  // Boundary segment is short and far ahead of ego
  Segment2d ego_seg{{0.0, 0.0}, {1.0, 0.0}};
  Segment2d boundary_seg{{3.0, 2.0}, {4.0, 2.0}};

  auto result = utils::calc_nearest_projection(ego_seg, boundary_seg, 0);

  ASSERT_FALSE(result.has_value());

  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();
    plot_ego_and_boundary(plt, ego_seg, boundary_seg, result);
    save_figure(plt, export_folder);
  });
}

// ==============================================================================
// 2. find_closest_segment Tests (Includes Coverage for Fallbacks)
// ==============================================================================

TEST(UncrossableBoundaryUtilsTest, TestFindClosestSegmentRearIntersection)
{
  // Simulate a 2x4m Ego Vehicle
  // We evaluate the FRONT-HALF of the left side to intentionally evade projection
  Segment2d ego_side_seg{{0.0, 4.0}, {0.0, 2.0}};

  // The rear of the vehicle
  Segment2d ego_rear_seg{{0.0, 0.0}, {2.0, 0.0}};
  size_t curr_fp_idx = 42;

  // A boundary segment that crosses exactly through the middle of the vehicle's rear
  // It spans Y from -1 to +1. Because our side segment stops at Y=2, they cannot project!
  std::vector<SegmentWithIdx> boundary_segments;
  IdxForRTreeSegment id{1, 0, 1};
  boundary_segments.emplace_back(Segment2d{{1.0, -1.0}, {1.0, 1.0}}, id);

  auto result =
    utils::find_closest_segment(ego_side_seg, ego_rear_seg, curr_fp_idx, boundary_segments);

  // Assert it successfully fell through the logic and hit the rear intersection branch
  EXPECT_EQ(result.pose_index, curr_fp_idx);
  EXPECT_DOUBLE_EQ(result.lat_dist, 0.0);
  EXPECT_DOUBLE_EQ(result.pt_on_ego.x(), 1.0);
  EXPECT_DOUBLE_EQ(result.pt_on_ego.y(), 0.0);

  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();
    plot_find_closest_segment(
      plt, ego_side_seg, ego_rear_seg, boundary_segments.front().first, result);
    save_figure(plt, export_folder);
  });
}

TEST(UncrossableBoundaryUtilsTest, TestFindClosestSegmentFallback)
{
  // 1. Simulate a 2x4m Ego Vehicle
  // Ego side is strictly X=0, Y in [2.0, 4.0]
  Segment2d ego_side_seg{{0.0, 4.0}, {0.0, 2.0}};

  // Ego rear is strictly Y=0, X in [0.0, 2.0]
  Segment2d ego_rear_seg{{0.0, 0.0}, {2.0, 0.0}};
  size_t curr_fp_idx = 99;

  // 2. A boundary that completely misses the vehicle in both projection and intersection
  // We place it at Y=-2.0, X in [4.0, 5.0].
  // - It misses the side segment's Y-band [2.0, 4.0] (no projection)
  // - It misses the rear segment's X-band [0.0, 2.0] (no intersection)
  std::vector<SegmentWithIdx> boundary_segments;
  IdxForRTreeSegment id{1, 0, 1};
  boundary_segments.emplace_back(Segment2d{{4.0, -2.0}, {5.0, -2.0}}, id);

  auto result =
    utils::find_closest_segment(ego_side_seg, ego_rear_seg, curr_fp_idx, boundary_segments);

  // 3. Since there's no projection and no intersection, it hits the final fallback return
  // statement.
  EXPECT_EQ(result.pose_index, curr_fp_idx);
  EXPECT_DOUBLE_EQ(result.lat_dist, std::numeric_limits<double>::max());

  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();
    plot_find_closest_segment(
      plt, ego_side_seg, ego_rear_seg, boundary_segments.front().first, result);
    save_figure(plt, export_folder);
  });
}

TEST(UncrossableBoundaryUtilsTest, TestCalcSignedLateralDistanceToBoundary)
{
  // 1. Setup Ego Pose (At Origin, Yaw = 0.0)
  // With Yaw = 0, the vehicle points along the +X axis.
  // Its lateral Y-axis ray points along the +Y axis (Left) and -Y axis (Right).
  geometry_msgs::msg::Pose ego_pose;
  ego_pose.position.x = 0.0;
  ego_pose.position.y = 0.0;
  ego_pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(0.0);

  // 2. Invalid boundary (empty or 1 point)
  lanelet::LineString3d empty_ls(lanelet::utils::getId());
  EXPECT_FALSE(utils::calc_signed_lateral_distance_to_boundary(empty_ls, ego_pose).has_value());

  lanelet::Point3d p1(lanelet::utils::getId(), -5.0, 5.0, 0.0);
  lanelet::LineString3d single_pt_ls(lanelet::utils::getId(), {p1});
  EXPECT_FALSE(utils::calc_signed_lateral_distance_to_boundary(single_pt_ls, ego_pose).has_value());

  // 3. Boundary to the LEFT (Positive distance)
  // Spans from X=-5 to X=5 at Y=5. The lateral ray (X=0) intersects it exactly at Y=5.
  lanelet::Point3d p2(lanelet::utils::getId(), 5.0, 5.0, 0.0);
  lanelet::LineString3d left_boundary(lanelet::utils::getId(), {p1, p2});

  auto dist_left = utils::calc_signed_lateral_distance_to_boundary(left_boundary, ego_pose);
  ASSERT_TRUE(dist_left.has_value());
  EXPECT_DOUBLE_EQ(dist_left.value(), 5.0);

  // 4. Boundary to the RIGHT (Negative distance)
  // Spans from X=-5 to X=5 at Y=-3. The lateral ray (X=0) intersects it exactly at Y=-3.
  lanelet::Point3d p3(lanelet::utils::getId(), -5.0, -3.0, 0.0);
  lanelet::Point3d p4(lanelet::utils::getId(), 5.0, -3.0, 0.0);
  lanelet::LineString3d right_boundary(lanelet::utils::getId(), {p3, p4});

  auto dist_right = utils::calc_signed_lateral_distance_to_boundary(right_boundary, ego_pose);
  ASSERT_TRUE(dist_right.has_value());
  EXPECT_DOUBLE_EQ(dist_right.value(), -3.0);

  // 5. Ray misses the segment (Intersection occurs outside segment bounds)
  // Boundary spans from X=2 to X=10 at Y=5.
  // The lateral ray is at X=0, which completely misses the start of the segment.
  lanelet::Point3d p5(lanelet::utils::getId(), 2.0, 5.0, 0.0);
  lanelet::Point3d p6(lanelet::utils::getId(), 10.0, 5.0, 0.0);
  lanelet::LineString3d missed_boundary(lanelet::utils::getId(), {p5, p6});

  auto dist_miss = utils::calc_signed_lateral_distance_to_boundary(missed_boundary, ego_pose);
  EXPECT_FALSE(dist_miss.has_value());

  // 6. Parallel Boundary (No intersection)
  // Boundary is a vertical line at X=5, spanning Y from -10 to 10.
  // This is perfectly parallel to the lateral Y-axis ray.
  lanelet::Point3d p7(lanelet::utils::getId(), 5.0, -10.0, 0.0);
  lanelet::Point3d p8(lanelet::utils::getId(), 5.0, 10.0, 0.0);
  lanelet::LineString3d parallel_boundary(lanelet::utils::getId(), {p7, p8});

  auto dist_parallel = utils::calc_signed_lateral_distance_to_boundary(parallel_boundary, ego_pose);
  EXPECT_FALSE(dist_parallel.has_value());

  // 7. Multiple segments, picks the closest
  // A U-shaped boundary: Top line at Y=8, Vertical line at X=10, Bottom line at Y=4.
  // The lateral ray (X=0) intersects BOTH the Top (Y=8) and Bottom (Y=4) lines.
  // It must correctly select the shortest distance (4.0).
  lanelet::Point3d p9(lanelet::utils::getId(), -10.0, 8.0, 0.0);
  lanelet::Point3d p10(lanelet::utils::getId(), 10.0, 8.0, 0.0);
  lanelet::Point3d p11(lanelet::utils::getId(), 10.0, 4.0, 0.0);
  lanelet::Point3d p12(lanelet::utils::getId(), -10.0, 4.0, 0.0);
  lanelet::LineString3d multi_boundary(lanelet::utils::getId(), {p9, p10, p11, p12});

  auto dist_multi = utils::calc_signed_lateral_distance_to_boundary(multi_boundary, ego_pose);
  ASSERT_TRUE(dist_multi.has_value());
  EXPECT_DOUBLE_EQ(dist_multi.value(), 4.0);

  // 8. Visualizing the Multi-segment Raycast
  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();

    // Plot the multi-segment boundary
    std::vector<double> bound_x = {p9.x(), p10.x(), p11.x(), p12.x()};
    std::vector<double> bound_y = {p9.y(), p10.y(), p11.y(), p12.y()};
    plt.plot(Args(bound_x, bound_y), Kwargs("color"_a = "red", "label"_a = "U-Shaped Boundary"));

    // Plot Ego Pose
    plt.scatter(
      Args(std::vector<double>{0.0}, std::vector<double>{0.0}),
      Kwargs("color"_a = "black", "s"_a = 100, "marker"_a = "o", "label"_a = "Ego Pose"));

    // Plot the lateral ray up to the closest intersection
    std::vector<double> ray_x = {0.0, 0.0};
    std::vector<double> ray_y = {0.0, dist_multi.value()};
    plt.plot(
      Args(ray_x, ray_y),
      Kwargs("color"_a = "green", "linestyle"_a = "--", "label"_a = "Lateral Ray (Selected)"));

    // Plot the rest of the ray that intersects the further wall
    std::vector<double> ray_miss_x = {0.0, 0.0};
    std::vector<double> ray_miss_y = {dist_multi.value(), 8.0};
    plt.plot(
      Args(ray_miss_x, ray_miss_y),
      Kwargs("color"_a = "gray", "linestyle"_a = ":", "label"_a = "Ignored Further Intersection"));

    plt.axis(Args("equal"));
    plt.legend();
    plt.title(Args("Signed Lateral Raycast to Closest Boundary"));
    save_figure(plt, export_folder);
  });
}

// ==============================================================================
// 3. Extracted Utility Function Tests
// ==============================================================================

TEST(UncrossableBoundaryUtilsTest, TestGetSegment3DFromId)
{
  // 1. Create a dummy LaneletMap
  auto map = std::make_shared<lanelet::LaneletMap>();

  // 2. Add a LineString with 3 points (representing 2 segments)
  lanelet::Point3d p1(lanelet::utils::getId(), 1.0, 2.0, 3.0);
  lanelet::Point3d p2(lanelet::utils::getId(), 4.0, 5.0, 6.0);
  lanelet::Point3d p3(lanelet::utils::getId(), 7.0, 8.0, 9.0);
  lanelet::LineString3d ls(lanelet::utils::getId(), {p1, p2, p3});
  map->add(ls);

  // 3. Test extraction of the first segment (index 0 to 1)
  IdxForRTreeSegment id1{ls.id(), 0, 1};
  auto seg1 = utils::get_segment_3d_from_id(map, id1);
  EXPECT_DOUBLE_EQ(seg1.first.x(), 1.0);
  EXPECT_DOUBLE_EQ(seg1.first.y(), 2.0);
  EXPECT_DOUBLE_EQ(seg1.first.z(), 3.0);
  EXPECT_DOUBLE_EQ(seg1.second.x(), 4.0);
  EXPECT_DOUBLE_EQ(seg1.second.y(), 5.0);
  EXPECT_DOUBLE_EQ(seg1.second.z(), 6.0);

  // 4. Test extraction of the second segment (index 1 to 2)
  IdxForRTreeSegment id2{ls.id(), 1, 2};
  auto seg2 = utils::get_segment_3d_from_id(map, id2);
  EXPECT_DOUBLE_EQ(seg2.first.x(), 4.0);
  EXPECT_DOUBLE_EQ(seg2.second.x(), 7.0);
}

TEST(UncrossableBoundaryUtilsTest, TestIsClosestToBoundarySegment)
{
  Segment2d left_side{{0.0, 2.0}, {2.0, 2.0}};
  Segment2d right_side{{0.0, -2.0}, {2.0, -2.0}};

  // 1. Boundary clearly closer to the left side
  Segment2d bound_left{{0.0, 3.0}, {2.0, 3.0}};
  EXPECT_TRUE(utils::is_closest_to_boundary_segment(bound_left, left_side, right_side));
  EXPECT_FALSE(utils::is_closest_to_boundary_segment(bound_left, right_side, left_side));

  // 2. Boundary clearly closer to the right side
  Segment2d bound_right{{0.0, -3.0}, {2.0, -3.0}};
  EXPECT_FALSE(utils::is_closest_to_boundary_segment(bound_right, left_side, right_side));
  EXPECT_TRUE(utils::is_closest_to_boundary_segment(bound_right, right_side, left_side));

  // 3. Boundary is perfectly equidistant (exactly in the middle at Y = 0)
  // The function uses `<=` so an equidistant boundary should return true for both sides.
  Segment2d bound_center{{0.0, 0.0}, {2.0, 0.0}};
  EXPECT_TRUE(utils::is_closest_to_boundary_segment(bound_center, left_side, right_side));
  EXPECT_TRUE(utils::is_closest_to_boundary_segment(bound_center, right_side, left_side));
}

TEST(UncrossableBoundaryUtilsTest, TestIsSegmentWithinEgoHeight)
{
  const double ego_z = 0.0;
  const double ego_height = 2.5;

  // 1. Fully within height bounds (z from 1.0 to 1.5)
  Segment3d seg_within{{0.0, 0.0, 1.0}, {1.0, 0.0, 1.5}};
  EXPECT_TRUE(utils::is_segment_within_ego_height(seg_within, ego_z, ego_height));

  // 2. Partial overlap: Starts within (z=2.0), ends above vehicle height (z=3.0)
  // Function checks if the *minimum* distance to the base is within the height.
  Segment3d seg_partial{{0.0, 0.0, 2.0}, {1.0, 0.0, 3.0}};
  EXPECT_TRUE(utils::is_segment_within_ego_height(seg_partial, ego_z, ego_height));

  // 3. Fully above vehicle (e.g., an overpass or high sign at z=3.0 to z=4.0)
  Segment3d seg_above{{0.0, 0.0, 3.0}, {1.0, 0.0, 4.0}};
  EXPECT_FALSE(utils::is_segment_within_ego_height(seg_above, ego_z, ego_height));

  // 4. Fully below vehicle (e.g., a road on a lower bridge layer at z=-3.0)
  Segment3d seg_below{{0.0, 0.0, -3.0}, {1.0, 0.0, -4.0}};
  EXPECT_FALSE(utils::is_segment_within_ego_height(seg_below, ego_z, ego_height));
}

TEST(UncrossableBoundaryUtilsTest, TestIsCritical)
{
  Side<std::optional<CriticalPointPair>> projections;

  // 1. Empty sides -> Not critical
  EXPECT_FALSE(utils::is_critical(projections));

  // 2. Contains only safe and approaching points -> Not critical
  CriticalPointPair pair_safe;
  pair_safe.physical_departure_point.departure_type = DepartureType::NONE;
  CriticalPointPair pair_app;
  pair_app.physical_departure_point.departure_type = DepartureType::APPROACHING;

  projections.left = pair_safe;
  projections.right = pair_app;
  EXPECT_FALSE(utils::is_critical(projections));

  // 3. Add a critical point to the left side -> Critical
  CriticalPointPair pair_crit;
  pair_crit.physical_departure_point.departure_type = DepartureType::CRITICAL;
  projections.left = pair_crit;
  EXPECT_TRUE(utils::is_critical(projections));

  // 4. Critical point on the right side -> Critical
  projections.left = std::nullopt;
  projections.right = pair_crit;
  EXPECT_TRUE(utils::is_critical(projections));
}
}  // namespace autoware::boundary_departure_checker
