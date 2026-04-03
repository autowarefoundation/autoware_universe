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

#include <autoware/motion_utils/distance/distance.hpp>

#include <gtest/gtest.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <string>
#include <utility>
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
#endif
}  // namespace

namespace autoware::boundary_departure_checker
{
constexpr const char * export_folder = "test_uncrossable_boundary_checker";
TEST(UncrossableBoundaryTest, TestSegmentToSegmentProjection)
{
  // 1. Setup PyPlot context

  // Define segments
  Segment2d ego_seg{{0.0, 0.0}, {0.0, 2.0}};
  Segment2d boundary_seg{{1.0, 0.0}, {1.0, 2.0}};

  auto result = utils::segment_to_segment_nearest_projection(ego_seg, boundary_seg, 0);

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->lat_dist, 1.0, 1e-6);
  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();
    plot_ego_and_boundary(plt, ego_seg, boundary_seg, result);
    save_figure(plt, export_folder);
  });
}

TEST(UncrossableBoundaryTest, TestIntersectionDetection)
{
  // Ego side segment crossing the boundary
  Segment2d ego_seg{{0.0, 0.0}, {1.0, 2.0}};
  Segment2d boundary_seg{{1.0, 0.0}, {1.0, 2.0}};

  auto result = utils::segment_to_segment_nearest_projection(ego_seg, boundary_seg, 0);

  ASSERT_TRUE(result.has_value());
  // Distance should be 0.0 because they intersect
  EXPECT_DOUBLE_EQ(result->lat_dist, 0.0);

  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();
    plot_ego_and_boundary(plt, ego_seg, boundary_seg, result);
    save_figure(plt, export_folder);
  });
}

TEST(UncrossableBoundaryTest, TestParallelSegments)
{
  // Parallel segments with partial overlap in longitudinal direction
  Segment2d ego_seg{{0.0, 0.0}, {2.0, 0.0}};
  Segment2d boundary_seg{{1.0, 1.0}, {3.0, 1.0}};

  auto result = utils::segment_to_segment_nearest_projection(ego_seg, boundary_seg, 0);

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->lat_dist, 1.0, 1e-6);
  // Should project ego_front (0,0) or bound_start (1,1) correctly
  EXPECT_GE(result->pt_on_ego.x(), 0.0);
  EXPECT_LE(result->pt_on_ego.x(), 2.0);

  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();
    plot_ego_and_boundary(plt, ego_seg, boundary_seg, result);
    save_figure(plt, export_folder);
  });
}

TEST(UncrossableBoundaryTest, TestPerpendicularNonIntersecting)
{
  // Even if the shortest distance is found by projecting from the boundary onto the ego,
  // the result is internally swapped so 'pt_on_ego' always references the vehicle.
  // This maintains a common reference frame for the departure checker, provided
  // the points can be projected perpendicularly between the segments.
  Segment2d ego_seg{{0.0, 0.0}, {1.0, 0.0}};
  Segment2d boundary_seg{{2.0, -1.0}, {2.0, 1.0}};

  auto result = utils::segment_to_segment_nearest_projection(ego_seg, boundary_seg, 0);

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->lat_dist, 1.0, 1e-6);

  EXPECT_DOUBLE_EQ(result->pt_on_ego.x(), 1.0);
  EXPECT_DOUBLE_EQ(result->pt_on_bound.x(), 2.0);
  EXPECT_DOUBLE_EQ(result->pt_on_bound.y(), 0.0);

  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();
    plot_ego_and_boundary(plt, ego_seg, boundary_seg, result);
    save_figure(plt, export_folder);
  });
}

TEST(UncrossableBoundaryTest, TestPointBeyondSegmentEnd)
{
  // Boundary segment is short and far ahead of ego
  Segment2d ego_seg{{0.0, 0.0}, {1.0, 0.0}};
  Segment2d boundary_seg{{3.0, 2.0}, {4.0, 2.0}};

  auto result = utils::segment_to_segment_nearest_projection(ego_seg, boundary_seg, 0);

  ASSERT_FALSE(result.has_value());

  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();
    plot_ego_and_boundary(plt, ego_seg, boundary_seg, result);
    save_figure(plt, export_folder);
  });
}

TEST(UncrossableBoundaryTest, TestCollinearSegments)
{
  // Segments on the same line but separated
  Segment2d ego_seg{{0.0, 0.0}, {1.0, 0.0}};
  Segment2d boundary_seg{{2.0, 0.0}, {3.0, 0.0}};

  auto result = utils::segment_to_segment_nearest_projection(ego_seg, boundary_seg, 0);

  ASSERT_FALSE(result.has_value());

  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();
    plot_ego_and_boundary(plt, ego_seg, boundary_seg, result);
    save_figure(plt, export_folder);
  });
}

TEST(UncrossableBoundaryTest, TestMiddleOfSegmentCrossingForLonDist)
{
  TrajectoryPoints ego_pred_traj;
  for (int i = 0; i < 3; ++i) {
    TrajectoryPoint p;
    constexpr double multiplier = 10.0;
    p.pose.position.x = i * multiplier;
    p.pose.position.y = 0.0;
    p.time_from_start = rclcpp::Duration::from_seconds(i);
    ego_pred_traj.push_back(p);
  }

  // 2. Setup EgoSides (Left and Right segments for each trajectory point)
  // Let's assume the ego vehicle is 4.0m long (2.0m front overhang, 2.0m rear overhang)
  // and 2.0m wide (1.0m left, 1.0m right).
  EgoSides ego_sides(ego_pred_traj.size());
  for (size_t i = 0; i < ego_pred_traj.size(); ++i) {
    double cx = ego_pred_traj[i].pose.position.x;

    // Left segment: Front-Left to Rear-Left
    // For i=1 (cx=10.0), this segment goes from X=12.0 to X=8.0 at Y=1.0
    ego_sides[i].left = Segment2d{{cx + 2.0, 1.0}, {cx - 2.0, 1.0}};

    // Right segment: Front-Right to Rear-Right
    ego_sides[i].right = Segment2d{{cx + 2.0, -1.0}, {cx - 2.0, -1.0}};
  }

  // 3. Setup Boundaries
  // We want the boundary to cross the LEFT segment exactly in the middle during the i=1 iteration.
  // The left segment at i=1 spans from X=12.0 to X=8.0. The exact middle is X=10.0.
  // We place a vertical boundary segment crossing X=10.0.
  BoundarySideWithIdx boundaries;
  SegmentWithIdx bound;
  bound.first = Segment2d{{10.0, 0.0}, {10.0, 2.0}};
  bound.second = IdxForRTreeSegment{1, 0, 1};
  boundaries.left.push_back(bound);

  // 4. Run the function
  auto result =
    utils::get_closest_boundary_segments_from_side(ego_pred_traj, boundaries, ego_sides);

  // 5. Verify the results specifically at the i=1 iteration
  ASSERT_EQ(result.left.size(), 3);
  const auto & proj_at_i = result.left[1];

  // A. The lateral distance should be 0 because the boundary directly intersects the ego left
  // segment
  EXPECT_DOUBLE_EQ(proj_at_i.lat_dist, 0.0);

  // B. The intersection point should be exactly at the middle of the ego's left segment:
  // (10.0, 1.0)
  EXPECT_DOUBLE_EQ(proj_at_i.pt_on_ego.x(), 10.0);
  EXPECT_DOUBLE_EQ(proj_at_i.pt_on_ego.y(), 1.0);

  // C. The lon_offset is the distance from the FRONT of the ego segment (12.0, 1.0) to the
  // intersection (10.0, 1.0) Since it's exactly in the middle of a 4.0m long segment, the offset
  // must be exactly 2.0m.
  EXPECT_DOUBLE_EQ(proj_at_i.lon_offset, 2.0);

  // D. Confirm lon_dist_on_pred_traj
  // Arc length `s` at i=1 should be the distance from P0 to P1 (10.0m).
  // Formula: lon_dist_on_pred_traj = s - lon_offset = 10.0 - 2.0 = 8.0.
  //
  // NOTE: If this specific EXPECT_DOUBLE_EQ fails and returns -2.0 instead of 8.0, it proves
  // that the `if (i > 1)` bug in `src/utils.cpp` inside `get_closest_boundary_segments_from_side`
  // is preventing `s` from accumulating correctly for the early trajectory points.
  EXPECT_DOUBLE_EQ(proj_at_i.lon_dist_on_pred_traj, 8.0);

  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();
    plot_ego_and_boundary(plt, ego_sides[1].left, bound.first, proj_at_i);
    save_figure(plt, export_folder);
  });
}

TEST(UncrossableBoundaryTest, TestRealisticLaneDeparture)
{
  // 1. Setup a 24-7-25 trajectory drifting LEFT out of the lane.
  TrajectoryPoints ego_pred_traj;
  for (int i = 0; i < 5; ++i) {
    TrajectoryPoint p;
    p.pose.position.x = i * 24.0;
    p.pose.position.y = i * 7.0;
    p.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(std::atan2(7.0, 24.0));
    p.time_from_start = rclcpp::Duration::from_seconds(i);
    ego_pred_traj.push_back(p);
  }

  // 2. Setup EgoSides explicitly to match the yaw
  EgoSides ego_sides(ego_pred_traj.size());
  for (size_t i = 0; i < ego_pred_traj.size(); ++i) {
    double cx = ego_pred_traj[i].pose.position.x;
    double cy = ego_pred_traj[i].pose.position.y;

    Point2d fl{cx + 4.1, cy + 3.8};
    Point2d rl{cx - 5.5, cy + 1.0};
    Point2d fr{cx + 5.5, cy - 1.0};
    Point2d rr{cx - 4.1, cy - 3.8};

    ego_sides[i].left = Segment2d{fl, rl};
    ego_sides[i].right = Segment2d{fr, rr};
  }

  // 3. Setup Realistic Road Boundaries (Left AND Right)
  BoundarySideWithIdx boundaries;

  // LEFT BOUNDARY: Exactly at Y = 9.4 (Ego crosses this at i=1)
  SegmentWithIdx left_bound;
  left_bound.first = Segment2d{{-10.0, 9.4}, {120.0, 9.4}};
  left_bound.second = IdxForRTreeSegment{1, 0, 1};
  boundaries.left.push_back(left_bound);

  // RIGHT BOUNDARY: Exactly at Y = -5.0 (Ego moves away from this)
  SegmentWithIdx right_bound;
  right_bound.first = Segment2d{{-10.0, -5.0}, {120.0, -5.0}};
  right_bound.second = IdxForRTreeSegment{2, 0, 1};
  boundaries.right.push_back(right_bound);

  // 4. Run the function
  auto result =
    utils::get_closest_boundary_segments_from_side(ego_pred_traj, boundaries, ego_sides);
  ASSERT_EQ(result.left.size(), 5);
  ASSERT_EQ(result.right.size(), 5);

  // =========================================================================
  // 5. VERIFY LEFT BOUNDARY (POSITIVE -> ZERO -> NEGATIVE)
  // =========================================================================
  EXPECT_NEAR(result.left[0].lat_dist, 5.6, 1e-6);    // i=0: Approaching (Positive)
  EXPECT_NEAR(result.left[1].lat_dist, 0.0, 1e-6);    // i=1: Intersecting (Zero)
  EXPECT_NEAR(result.left[2].lat_dist, -5.6, 1e-6);   // i=2: Crossed (Negative)
  EXPECT_NEAR(result.left[3].lat_dist, -12.6, 1e-6);  // i=3: Farther outside (Negative)
  EXPECT_NEAR(result.left[4].lat_dist, -19.6, 1e-6);  // i=4: Even farther (Negative)

  // =========================================================================
  // 6. VERIFY RIGHT BOUNDARY (ALWAYS POSITIVE, INCREASING DISTANCE)
  // =========================================================================
  // Since the car is drifting left (Y increases), the distance to the right boundary (Y=-5.0)
  // increases. Closest point is always Rear-Right. i=0: RR Y is -3.8. Dist = -3.8 - (-5.0) = +1.2
  EXPECT_NEAR(result.right[0].lat_dist, 1.2, 1e-6);
  // i=1: RR Y is 3.2. Dist = 3.2 - (-5.0) = +8.2
  EXPECT_NEAR(result.right[1].lat_dist, 8.2, 1e-6);
  // i=2: RR Y is 10.2. Dist = +15.2
  EXPECT_NEAR(result.right[2].lat_dist, 15.2, 1e-6);

  // 7. Plot the realistic scenario
  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();

    // Plot Trajectory (dashed gray line)
    std::vector<double> traj_x, traj_y;
    for (const auto & p : ego_pred_traj) {
      traj_x.push_back(p.pose.position.x);
      traj_y.push_back(p.pose.position.y);
    }
    plt.plot(
      Args(traj_x, traj_y),
      Kwargs("color"_a = "gray", "linestyle"_a = "--", "marker"_a = "o", "label"_a = "Trajectory"));

    // Plot Ego Vehicle Boxes
    for (size_t i = 0; i < ego_sides.size(); ++i) {
      auto fl = ego_sides[i].left.first;
      auto rl = ego_sides[i].left.second;
      auto fr = ego_sides[i].right.first;
      auto rr = ego_sides[i].right.second;

      std::vector<double> bx = {fl.x(), rl.x(), rr.x(), fr.x(), fl.x()};
      std::vector<double> by = {fl.y(), rl.y(), rr.y(), fr.y(), fl.y()};

      plt.plot(Args(bx, by), Kwargs("color"_a = "blue", "linewidth"_a = 1.5, "alpha"_a = 0.5));
    }

    // Plot Boundaries
    auto plot_bound =
      [&](const SegmentWithIdx & b, const std::string & color, const std::string & label) {
        std::vector<double> bx = {b.first.first.x(), b.first.second.x()};
        std::vector<double> by = {b.first.first.y(), b.first.second.y()};
        plt.plot(Args(bx, by), Kwargs("color"_a = color, "linewidth"_a = 2.0, "label"_a = label));
      };
    plot_bound(left_bound, "red", "Left Boundary");
    plot_bound(right_bound, "darkred", "Right Boundary");

    // Plot Projections
    auto plot_projections =
      [&](const std::vector<ProjectionToBound> & projections, const std::string & color) {
        for (const auto & proj : projections) {
          std::vector<double> px = {proj.pt_on_ego.x(), proj.pt_on_bound.x()};
          std::vector<double> py = {proj.pt_on_ego.y(), proj.pt_on_bound.y()};
          plt.plot(Args(px, py), Kwargs("color"_a = color, "linestyle"_a = ":"));
          plt.scatter(
            Args(std::vector<double>{proj.pt_on_ego.x()}, std::vector<double>{proj.pt_on_ego.y()}),
            Kwargs("color"_a = "orange", "s"_a = 30, "zorder"_a = 5));
        }
      };
    plot_projections(result.left, "green");
    plot_projections(result.right, "lightgreen");

    plt.legend();
    plt.axis(Args("equal"));
    plt.xlabel(Args("X [m]"));
    plt.ylabel(Args("Y [m]"));
    plt.title(Args("Left/Right Signed Distance Verification"));

    save_figure(plt, export_folder);
  });
}
TEST(UncrossableBoundaryTest, TestRealisticRightLaneDeparture)
{
  // 1. Setup a multi-point predicted trajectory drifting RIGHT out of a lane
  // We use the same 24-7-25 triangle, but with negative Y to drift right.
  // Distance between points is exactly 25.0m.
  TrajectoryPoints ego_pred_traj;
  for (int i = 0; i < 5; ++i) {
    TrajectoryPoint p;
    p.pose.position.x = i * 24.0;
    p.pose.position.y = i * -7.0;  // Drifting right (negative Y)
    p.pose.orientation =
      autoware_utils_geometry::create_quaternion_from_yaw(std::atan2(-7.0, 24.0));
    p.time_from_start = rclcpp::Duration::from_seconds(i);
    ego_pred_traj.push_back(p);
  }

  // 2. Setup EgoSides explicitly to match the yaw
  // Vehicle Length = 10.0m (Front 5.0, Rear 5.0) | Width = 5.0m (Left 2.5, Right 2.5)
  // Rotation Math: cos(theta) = 0.96, sin(theta) = -0.28
  EgoSides ego_sides(ego_pred_traj.size());
  for (size_t i = 0; i < ego_pred_traj.size(); ++i) {
    double cx = ego_pred_traj[i].pose.position.x;
    double cy = ego_pred_traj[i].pose.position.y;

    // Exact mathematical corners after rotation:
    Point2d fl{cx + 5.5, cy + 1.0};  // Front-Left
    Point2d rl{cx - 4.1, cy + 3.8};  // Rear-Left
    Point2d fr{cx + 4.1, cy - 3.8};  // Front-Right
    Point2d rr{cx - 5.5, cy - 1.0};  // Rear-Right

    ego_sides[i].left = Segment2d{fl, rl};
    ego_sides[i].right = Segment2d{fr, rr};
  }

  // 3. Setup Realistic Road Boundaries (Left AND Right)
  BoundarySideWithIdx boundaries;

  // LEFT BOUNDARY: Exactly at Y = 5.0 (Ego moves away from this)
  SegmentWithIdx left_bound;
  left_bound.first = Segment2d{{-10.0, 5.0}, {120.0, 5.0}};
  left_bound.second = IdxForRTreeSegment{1, 0, 1};
  boundaries.left.push_back(left_bound);

  // RIGHT BOUNDARY: Exactly at Y = -9.4 (Ego crosses this at i=1)
  // At i=1, the midpoint of the right side segment is exactly (23.3, -9.4)
  SegmentWithIdx right_bound;
  right_bound.first = Segment2d{{-10.0, -9.4}, {120.0, -9.4}};
  right_bound.second = IdxForRTreeSegment{2, 0, 1};
  boundaries.right.push_back(right_bound);

  // 4. Run the function
  auto result =
    utils::get_closest_boundary_segments_from_side(ego_pred_traj, boundaries, ego_sides);
  ASSERT_EQ(result.left.size(), 5);
  ASSERT_EQ(result.right.size(), 5);

  // =========================================================================
  // 5. VERIFY RIGHT BOUNDARY (POSITIVE -> ZERO -> NEGATIVE)
  // =========================================================================
  // i=0: Approaching boundary. Front-Right corner is closest (Y=-3.8).
  // Distance to boundary (Y=-9.4) is exactly 5.6m.
  EXPECT_NEAR(result.right[0].lat_dist, 5.6, 1e-6);

  // i=1: Intersecting boundary. The right segment midpoint crosses Y=-9.4 exactly.
  EXPECT_NEAR(result.right[1].lat_dist, 0.0, 1e-6);
  EXPECT_DOUBLE_EQ(result.right[1].pt_on_ego.x(), 23.3);
  EXPECT_DOUBLE_EQ(result.right[1].pt_on_ego.y(), -9.4);
  EXPECT_DOUBLE_EQ(result.right[1].lon_offset, 5.0);  // Exactly 5.0m from the Front-Right corner
  EXPECT_DOUBLE_EQ(result.right[1].lon_dist_on_pred_traj, 20.0);  // s(25.0) - offset(5.0) = 20.0

  // i=2: Completely crossed boundary. Rear-Right corner is now closest (Y=-15.0).
  // Outside the boundary, so lateral distance is negative. |-15.0 - (-9.4)| = 5.6.
  EXPECT_NEAR(result.right[2].lat_dist, -5.6, 1e-6);

  // i=3: Drifting further outside the right lane.
  // Rear-Right Y is -22.0. Dist = 22.0 - 9.4 = 12.6.
  EXPECT_NEAR(result.right[3].lat_dist, -12.6, 1e-6);

  // i=4: Even further out.
  EXPECT_NEAR(result.right[4].lat_dist, -19.6, 1e-6);

  // =========================================================================
  // 6. VERIFY LEFT BOUNDARY (ALWAYS POSITIVE, INCREASING DISTANCE)
  // =========================================================================
  // Vehicle is drifting right (negative Y), so distance to left boundary (Y=5.0) strictly
  // increases. Closest point is always Rear-Left. i=0: RL Y is 3.8. Dist = 5.0 - 3.8 = 1.2
  EXPECT_NEAR(result.left[0].lat_dist, 1.2, 1e-6);
  // i=1: RL Y is -3.2. Dist = 5.0 - (-3.2) = 8.2
  EXPECT_NEAR(result.left[1].lat_dist, 8.2, 1e-6);
  // i=2: RL Y is -10.2. Dist = 5.0 - (-10.2) = 15.2
  EXPECT_NEAR(result.left[2].lat_dist, 15.2, 1e-6);

  // 7. Plot the realistic scenario
  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();

    // Plot Trajectory (dashed gray line)
    std::vector<double> traj_x, traj_y;
    for (const auto & p : ego_pred_traj) {
      traj_x.push_back(p.pose.position.x);
      traj_y.push_back(p.pose.position.y);
    }
    plt.plot(
      Args(traj_x, traj_y),
      Kwargs("color"_a = "gray", "linestyle"_a = "--", "marker"_a = "o", "label"_a = "Trajectory"));

    // Plot Ego Vehicle Boxes
    for (size_t i = 0; i < ego_sides.size(); ++i) {
      auto fl = ego_sides[i].left.first;
      auto rl = ego_sides[i].left.second;
      auto fr = ego_sides[i].right.first;
      auto rr = ego_sides[i].right.second;

      std::vector<double> bx = {fl.x(), rl.x(), rr.x(), fr.x(), fl.x()};
      std::vector<double> by = {fl.y(), rl.y(), rr.y(), fr.y(), fl.y()};

      if (i == 0) {
        plt.plot(
          Args(bx, by),
          Kwargs(
            "color"_a = "blue", "linewidth"_a = 1.5, "label"_a = "Ego Vehicle", "alpha"_a = 0.5));
      } else {
        plt.plot(Args(bx, by), Kwargs("color"_a = "blue", "linewidth"_a = 1.5, "alpha"_a = 0.5));
      }
    }

    // Plot Boundaries
    auto plot_bound =
      [&](const SegmentWithIdx & b, const std::string & color, const std::string & label) {
        std::vector<double> bx = {b.first.first.x(), b.first.second.x()};
        std::vector<double> by = {b.first.first.y(), b.first.second.y()};
        plt.plot(Args(bx, by), Kwargs("color"_a = color, "linewidth"_a = 2.0, "label"_a = label));
      };
    plot_bound(left_bound, "darkred", "Left Boundary");
    plot_bound(right_bound, "red", "Right Boundary");

    // Plot Projections
    auto plot_projections = [&](
                              const std::vector<ProjectionToBound> & projections,
                              const std::string & color, const std::string & label) {
      bool added_label = false;
      for (const auto & proj : projections) {
        std::vector<double> px = {proj.pt_on_ego.x(), proj.pt_on_bound.x()};
        std::vector<double> py = {proj.pt_on_ego.y(), proj.pt_on_bound.y()};

        if (!added_label) {
          plt.plot(Args(px, py), Kwargs("color"_a = color, "linestyle"_a = ":", "label"_a = label));
          added_label = true;
        } else {
          plt.plot(Args(px, py), Kwargs("color"_a = color, "linestyle"_a = ":"));
        }

        plt.scatter(
          Args(std::vector<double>{proj.pt_on_ego.x()}, std::vector<double>{proj.pt_on_ego.y()}),
          Kwargs("color"_a = "orange", "s"_a = 30, "zorder"_a = 5));
      }
    };
    plot_projections(result.left, "lightgreen", "Left Projections");
    plot_projections(result.right, "green", "Right Projections");

    plt.legend();
    plt.axis(Args("equal"));
    plt.xlabel(Args("X [m]"));
    plt.ylabel(Args("Y [m]"));
    plt.title(Args("Realistic Right Lane Departure with Signed Lateral Distance"));

    save_figure(plt, export_folder);
  });
}
TEST(UncrossableBoundaryUtilsTest, TestCalcJudgeLineDist)
{
  constexpr double acceleration = 0.0;
  constexpr double max_stop_accel = -4.0;
  constexpr double max_stop_jerk = -10.0;
  constexpr double delay_time = 1.0;

  constexpr double v_test = 10.0;
  const auto dist_opt = motion_utils::calculate_stop_distance(
    v_test, acceleration, max_stop_accel, max_stop_jerk, delay_time);
  ASSERT_TRUE(dist_opt.has_value());
  const double dist = dist_opt.value();
  EXPECT_GT(dist, 22.5);

  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();

    std::vector<double> velocities;
    std::vector<double> distances;
    for (double v = 0.0; v <= 20.0; v += 1.0) {
      velocities.push_back(v);

      if (
        const auto dist_it_opt = motion_utils::calculate_stop_distance(
          v, acceleration, max_stop_accel, max_stop_jerk, delay_time)) {
        distances.push_back(*dist_it_opt);
      }
    }

    plt.plot(Args(velocities, distances), Kwargs("marker"_a = "o"));
    plt.xlabel(Args("Velocity [m/s]"));
    plt.ylabel(Args("Judge Line Distance [m]"));
    plt.title(Args("Braking Distance with Jerk Limit"));

    std::vector<double> line_x_h = {v_test, v_test};
    std::vector<double> line_y_h = {0.0, dist};
    plt.plot(
      Args(line_x_h, line_y_h), Kwargs("color"_a = "gray", "linestyle"_a = "--", "alpha"_a = 0.5));

    std::vector<double> line_x_v = {0.0, v_test};
    std::vector<double> line_y_v = {dist, dist};

    plt.plot(
      Args(line_x_v, line_y_v), Kwargs("color"_a = "gray", "linestyle"_a = "--", "alpha"_a = 0.5));
    save_figure(plt, export_folder);
  });
}

TEST(UncrossableBoundaryUtilsTest, TestPointToSegmentProjection)
{
  Point2d p{0.5, 1.0};
  Segment2d segment{{0.0, 0.0}, {1.0, 0.0}};
  auto result = utils::point_to_segment_projection(p, segment);

  ASSERT_TRUE(result.has_value());
  auto [proj, dist] = *result;
  EXPECT_DOUBLE_EQ(dist, 1.0);

  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();

    // Plot Segment
    plt.plot(
      Args(
        std::vector<double>{segment.first.x(), segment.second.x()},
        std::vector<double>{segment.first.y(), segment.second.y()}),
      Kwargs("color"_a = "red", "label"_a = "Boundary Segment"));

    // Plot Point and its projection
    plt.scatter(
      Args(std::vector<double>{p.x()}, std::vector<double>{p.y()}),
      Kwargs("label"_a = "Ego Point"));
    plt.plot(
      Args(std::vector<double>{p.x(), proj.x()}, std::vector<double>{p.y(), proj.y()}),
      Kwargs("color"_a = "green", "linestyle"_a = "--", "label"_a = "Lateral Projection"));

    plt.axis(Args("equal"));
    plt.legend();
    save_figure(plt, export_folder);
  });
}

TEST(UncrossableBoundaryUtilsTest, TestTrimPredPath)
{
  TrajectoryPoints path;
  std::vector<double> x_orig;
  std::vector<double> y_orig;
  for (int i = 0; i < 10; ++i) {
    TrajectoryPoint p;
    p.pose.position.x = static_cast<double>(i);
    p.pose.position.y = 0.0;
    p.time_from_start = rclcpp::Duration::from_seconds(static_cast<double>(i));
    path.push_back(p);

    x_orig.push_back(p.pose.position.x);
    y_orig.push_back(p.pose.position.y);
  }

  constexpr double cutoff_time_s = 4.5;
  auto trimmed = utils::trim_pred_path(path, cutoff_time_s);

  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();

    std::vector<double> x_trim;
    std::vector<double> y_trim;
    for (const auto & p : trimmed) {
      x_trim.push_back(p.pose.position.x);
      y_trim.push_back(0.0);
    }

    plt.plot(
      Args(x_orig, y_orig),
      Kwargs("color"_a = "green", "label"_a = "Full Path", "alpha"_a = 0.3, "linestyle"_a = "--"));

    plt.plot(
      Args(x_trim, y_trim),
      Kwargs("color"_a = "blue", "label"_a = "Trimmed Path", "marker"_a = "o"));

    std::vector<double> cutoff_x = {cutoff_time_s, cutoff_time_s};
    std::vector<double> cutoff_y = {-0.2, 0.2};
    plt.plot(
      Args(cutoff_x, cutoff_y),
      Kwargs("color"_a = "red", "linestyle"_a = ":", "label"_a = "Cutoff Threshold"));

    plt.xlabel(Args("X Distance [m]"));
    plt.title(Args("Trajectory Trimming (Time-based Cutoff)"));
    plt.legend();

    save_figure(plt, export_folder);
  });

  EXPECT_EQ(trimmed.size(), 6);
}

TEST(UncrossableBoundaryUtilsTest, TestMarginFromCovariance)
{
  const auto vehicle_info = autoware::vehicle_info_utils::createVehicleInfo(
    0.383, 0.235, 2.79, 1.64, 1.0, 1.1, 0.128, 0.128, 2.5, 0.70);

  geometry_msgs::msg::PoseWithCovariance cov_msg;
  cov_msg.covariance[0] = 0.5;  // Map X variance
  cov_msg.covariance[7] = 0.2;  // Map Y variance
  const double yaw = M_PI / 4.0;
  cov_msg.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw);

  const auto base_fp = vehicle_info.createFootprint(0.0, 0.0);
  const auto margin = utils::calc_margin_from_covariance(cov_msg, 1.0);
  const auto expanded_fp = vehicle_info.createFootprint(margin.lat_m, margin.lon_m);

  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();
    auto [fig, axes] = plt.subplots(1, 2);

    auto get_coords = [](const LinearRing2d & ring) {
      std::vector<double> x;
      std::vector<double> y;
      for (const auto & p : ring) {
        x.push_back(p.x());
        y.push_back(p.y());
      }
      return std::make_pair(x, y);
    };

    auto [bx, by] = get_coords(base_fp);
    auto [ex, ey] = get_coords(expanded_fp);

    axes[0].plot(Args(bx, by), Kwargs("color"_a = "gray", "label"_a = "Base Vehicle"));
    axes[0].set_title(Args("Step 1: Base Footprint"));
    axes[0].set_aspect(Args("equal"));

    axes[1].plot(Args(bx, by), Kwargs("color"_a = "gray", "linestyle"_a = "--", "alpha"_a = 0.5));
    axes[1].plot(Args(ex, ey), Kwargs("color"_a = "blue", "label"_a = "Expanded (Margin)"));
    axes[1].set_title(Args("Step 2: Expanded Footprint"));
    axes[1].set_aspect(Args("equal"));

    fig.tight_layout();
    save_figure(plt, export_folder);
  });

  EXPECT_GT(margin.lon_m, 0.0);
  EXPECT_GT(margin.lat_m, 0.0);
}

TEST(UncrossableBoundaryUtilsTest, TestIsUncrossableType)
{
  lanelet::LineString3d ls(lanelet::utils::getId());
  ls.attributes()[lanelet::AttributeName::Type] = "road_border";
  std::vector<std::string> types = {"road_border", "curb"};

  EXPECT_TRUE(utils::is_uncrossable_type(types, ls));

  ls.attributes()[lanelet::AttributeName::Type] = "lane_divider";

  EXPECT_FALSE(utils::is_uncrossable_type(types, ls));
}
}  // namespace autoware::boundary_departure_checker
