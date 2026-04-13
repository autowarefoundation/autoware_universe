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

// Evaluates lateral distance for parallel segments with longitudinal overlap.
TEST(UncrossableBoundaryTest, TestParallelSegments)
{
  // Arrange:
  Segment2d ego_seg{{0.0, 0.0}, {2.0, 0.0}};       // horizontal at y=0, x=[0,2]
  Segment2d boundary_seg{{1.0, 1.0}, {3.0, 1.0}};  // horizontal at y=1, x=[1,3]

  // Act:
  auto result = utils::calc_nearest_projection(ego_seg, boundary_seg, 0);

  // Assert:
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->lat_dist, 1.0, 1e-6);
  EXPECT_GE(result->pt_on_ego.x(), 0.0);
  EXPECT_LE(result->pt_on_ego.x(), 2.0);

  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();
    plot_ego_and_boundary(plt, ego_seg, boundary_seg, result);
    save_figure(plt, export_folder);
  });
}

// Evaluates projection when segments are perpendicular and do not intersect.
TEST(UncrossableBoundaryTest, TestPerpendicularNonIntersecting)
{
  // Arrange:
  Segment2d ego_seg{{0.0, 0.0}, {1.0, 0.0}};       // horizontal x=[0,1]
  Segment2d boundary_seg{{2.0, -1.0}, {2.0, 1.0}};  // vertical at x=2

  // Act:
  auto result = utils::calc_nearest_projection(ego_seg, boundary_seg, 0);

  // Assert:
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

// Evaluates that collinear segments with a gap return no projection.
TEST(UncrossableBoundaryTest, TestCollinearSegments)
{
  // Arrange:
  Segment2d ego_seg{{0.0, 0.0}, {1.0, 0.0}};       // horizontal x=[0,1]
  Segment2d boundary_seg{{2.0, 0.0}, {3.0, 0.0}};  // horizontal x=[2,3]

  // Act:
  auto result = utils::calc_nearest_projection(ego_seg, boundary_seg, 0);

  // Assert:
  ASSERT_FALSE(result.has_value());

  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();
    plot_ego_and_boundary(plt, ego_seg, boundary_seg, result);
    save_figure(plt, export_folder);
  });
}

// Evaluates longitudinal distance calculation when a boundary crosses the ego side midpoint.
TEST(UncrossableBoundaryTest, TestMiddleOfSegmentCrossingForLonDist)
{
  // Arrange:
  TrajectoryPoints ego_pred_traj;
  for (int i = 0; i < 3; ++i) {
    TrajectoryPoint p;
    constexpr double multiplier = 10.0;
    p.pose.position.x = i * multiplier;
    p.pose.position.y = 0.0;
    p.time_from_start = rclcpp::Duration::from_seconds(i);
    ego_pred_traj.push_back(p);
  }

  FootprintSideSegmentsArray ego_sides(ego_pred_traj.size());
  for (size_t i = 0; i < ego_pred_traj.size(); ++i) {
    double cx = ego_pred_traj[i].pose.position.x;
    ego_sides[i].left = Segment2d{{cx + 2.0, 1.0}, {cx - 2.0, 1.0}};
    ego_sides[i].right = Segment2d{{cx + 2.0, -1.0}, {cx - 2.0, -1.0}};
  }

  BoundarySegmentsBySide boundaries;
  SegmentWithIdx bound;
  bound.first = Segment2d{{10.0, 0.0}, {10.0, 2.0}};  // crosses left side at i=1
  bound.second = IdxForRTreeSegment{1, 0, 1};
  boundaries.left.push_back(bound);

  // Act:
  auto result =
    utils::get_closest_boundary_segments_from_side(ego_pred_traj, boundaries, ego_sides);

  // Assert:
  ASSERT_EQ(result.left.size(), 3);
  const auto & proj_at_i = result.left[1];
  EXPECT_DOUBLE_EQ(proj_at_i.lat_dist, 0.0);
  EXPECT_DOUBLE_EQ(proj_at_i.pt_on_ego.x(), 10.0);
  EXPECT_DOUBLE_EQ(proj_at_i.pt_on_ego.y(), 1.0);
  EXPECT_DOUBLE_EQ(proj_at_i.ego_front_to_proj_offset_m, 2.0);
  EXPECT_DOUBLE_EQ(proj_at_i.dist_along_trajectory_m, 8.0);  // s(10) - offset(2) = 8

  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();
    plot_ego_and_boundary(plt, ego_sides[1].left, bound.first, proj_at_i);
    save_figure(plt, export_folder);
  });
}

// Evaluates realistic lane departure scenario with drifting to the LEFT.
TEST(UncrossableBoundaryTest, TestRealisticLaneDeparture)
{
  // Arrange:
  TrajectoryPoints ego_pred_traj;
  for (int i = 0; i < 5; ++i) {
    TrajectoryPoint p;
    p.pose.position.x = i * 24.0;
    p.pose.position.y = i * 7.0;
    p.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(std::atan2(7.0, 24.0));
    p.time_from_start = rclcpp::Duration::from_seconds(i);
    ego_pred_traj.push_back(p);
  }

  FootprintSideSegmentsArray ego_sides(ego_pred_traj.size());
  for (size_t i = 0; i < ego_pred_traj.size(); ++i) {
    double cx = ego_pred_traj[i].pose.position.x;
    double cy = ego_pred_traj[i].pose.position.y;
    ego_sides[i].left = Segment2d{{cx + 4.1, cy + 3.8}, {cx - 5.5, cy + 1.0}};
    ego_sides[i].right = Segment2d{{cx + 5.5, cy - 1.0}, {cx - 4.1, cy - 3.8}};
  }

  BoundarySegmentsBySide boundaries;
  SegmentWithIdx left_bound;
  left_bound.first = Segment2d{{-10.0, 9.4}, {120.0, 9.4}};  // Crossed at i=1
  left_bound.second = IdxForRTreeSegment{1, 0, 1};
  boundaries.left.push_back(left_bound);

  SegmentWithIdx right_bound;
  right_bound.first = Segment2d{{-10.0, -5.0}, {120.0, -5.0}};  // Moving away
  right_bound.second = IdxForRTreeSegment{2, 0, 1};
  boundaries.right.push_back(right_bound);

  // Act:
  auto result =
    utils::get_closest_boundary_segments_from_side(ego_pred_traj, boundaries, ego_sides);

  // Assert:
  ASSERT_EQ(result.left.size(), 5);
  ASSERT_EQ(result.right.size(), 5);
  EXPECT_NEAR(result.left[0].lat_dist, 5.6, 1e-6);    // approaching
  EXPECT_NEAR(result.left[1].lat_dist, 0.0, 1e-6);    // intersecting
  EXPECT_NEAR(result.left[2].lat_dist, -5.6, 1e-6);   // crossed
  EXPECT_NEAR(result.right[0].lat_dist, 1.2, 1e-6);   // RR corner at -3.8, bound at -5.0
  EXPECT_NEAR(result.right[1].lat_dist, 8.2, 1e-6);

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
    auto plot_projs = [&](const std::vector<ProjectionToBound> & projs, const std::string & color) {
      for (const auto & proj : projs) {
        std::vector<double> px = {proj.pt_on_ego.x(), proj.pt_on_bound.x()};
        std::vector<double> py = {proj.pt_on_ego.y(), proj.pt_on_bound.y()};
        plt.plot(Args(px, py), Kwargs("color"_a = color, "linestyle"_a = ":"));
        plt.scatter(
          Args(std::vector<double>{proj.pt_on_ego.x()}, std::vector<double>{proj.pt_on_ego.y()}),
          Kwargs("color"_a = "orange", "s"_a = 30, "zorder"_a = 5));
      }
    };
    plot_projs(result.left, "green");
    plot_projs(result.right, "lightgreen");

    plt.legend();
    plt.axis(Args("equal"));
    plt.xlabel(Args("X [m]"));
    plt.ylabel(Args("Y [m]"));
    plt.title(Args("Left/Right Signed Distance Verification"));

    save_figure(plt, export_folder);
  });
}

// Evaluates realistic lane departure scenario with drifting to the RIGHT.
TEST(UncrossableBoundaryTest, TestRealisticRightLaneDeparture)
{
  // Arrange:
  TrajectoryPoints ego_pred_traj;
  for (int i = 0; i < 5; ++i) {
    TrajectoryPoint p;
    p.pose.position.x = i * 24.0;
    p.pose.position.y = i * -7.0;  // Drifting right
    p.pose.orientation =
      autoware_utils_geometry::create_quaternion_from_yaw(std::atan2(-7.0, 24.0));
    p.time_from_start = rclcpp::Duration::from_seconds(i);
    ego_pred_traj.push_back(p);
  }

  // yaw rotation: cos(theta)=0.96, sin(theta)=-0.28
  FootprintSideSegmentsArray ego_sides(ego_pred_traj.size());
  for (size_t i = 0; i < ego_pred_traj.size(); ++i) {
    double cx = ego_pred_traj[i].pose.position.x;
    double cy = ego_pred_traj[i].pose.position.y;
    ego_sides[i].left = Segment2d{{cx + 5.5, cy + 1.0}, {cx - 4.1, cy + 3.8}};
    ego_sides[i].right = Segment2d{{cx + 4.1, cy - 3.8}, {cx - 5.5, cy - 1.0}};
  }

  BoundarySegmentsBySide boundaries;
  SegmentWithIdx left_bound;
  left_bound.first = Segment2d{{-10.0, 5.0}, {120.0, 5.0}};
  left_bound.second = IdxForRTreeSegment{1, 0, 1};
  boundaries.left.push_back(left_bound);

  SegmentWithIdx right_bound;
  right_bound.first = Segment2d{{-10.0, -9.4}, {120.0, -9.4}};  // Crossed at i=1
  right_bound.second = IdxForRTreeSegment{2, 0, 1};
  boundaries.right.push_back(right_bound);

  // Act:
  auto result =
    utils::get_closest_boundary_segments_from_side(ego_pred_traj, boundaries, ego_sides);

  // Assert:
  ASSERT_EQ(result.left.size(), 5);
  ASSERT_EQ(result.right.size(), 5);

  // RIGHT BOUNDARY verification
  EXPECT_NEAR(result.right[0].lat_dist, 5.6, 1e-6);   // approaching
  EXPECT_NEAR(result.right[1].lat_dist, 0.0, 1e-6);   // intersecting
  EXPECT_DOUBLE_EQ(result.right[1].pt_on_ego.x(), 23.3);
  EXPECT_DOUBLE_EQ(result.right[1].pt_on_ego.y(), -9.4);
  EXPECT_DOUBLE_EQ(result.right[1].ego_front_to_proj_offset_m, 5.0);
  EXPECT_DOUBLE_EQ(result.right[1].dist_along_trajectory_m, 20.0);
  EXPECT_NEAR(result.right[2].lat_dist, -5.6, 1e-6);   // crossed
  EXPECT_NEAR(result.right[3].lat_dist, -12.6, 1e-6);
  EXPECT_NEAR(result.right[4].lat_dist, -19.6, 1e-6);

  // LEFT BOUNDARY verification
  EXPECT_NEAR(result.left[0].lat_dist, 1.2, 1e-6);
  EXPECT_NEAR(result.left[1].lat_dist, 8.2, 1e-6);
  EXPECT_NEAR(result.left[2].lat_dist, 15.2, 1e-6);

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
    auto plot_projs = [&](
                        const std::vector<ProjectionToBound> & projs, const std::string & color,
                        const std::string & label) {
      bool added_label = false;
      for (const auto & proj : projs) {
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
    plot_projs(result.left, "lightgreen", "Left Projections");
    plot_projs(result.right, "green", "Right Projections");

    plt.legend();
    plt.axis(Args("equal"));
    plt.xlabel(Args("X [m]"));
    plt.ylabel(Args("Y [m]"));
    plt.title(Args("Realistic Right Lane Departure with Signed Lateral Distance"));

    save_figure(plt, export_folder);
  });
}

// Evaluates braking distance calculation including jerk limits.
TEST(UncrossableBoundaryUtilsTest, TestCalcJudgeLineDist)
{
  // Arrange:
  constexpr double acceleration = 0.0;
  constexpr double max_stop_accel = -4.0;
  constexpr double max_stop_jerk = -10.0;
  constexpr double delay_time = 1.0;
  constexpr double v_test = 10.0;

  // Act:
  const auto dist_opt = motion_utils::calculate_stop_distance(
    v_test, acceleration, max_stop_accel, max_stop_jerk, delay_time);

  // Assert:
  ASSERT_TRUE(dist_opt.has_value());
  const double dist = dist_opt.value();
  EXPECT_GT(dist, 22.5);  // expected distance for 10m/s with 1s delay and -4m/s^2

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

// Evaluates point-to-segment perpendicular projection.
TEST(UncrossableBoundaryUtilsTest, TestPointToSegmentProjection)
{
  // Arrange:
  Point2d p{0.5, 1.0};
  Segment2d segment{{0.0, 0.0}, {1.0, 0.0}};

  // Act:
  auto result = utils::point_to_segment_projection(p, segment);

  // Assert:
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

// Evaluates uncrossable boundary type detection based on attribute strings.
TEST(UncrossableBoundaryUtilsTest, TestIsUncrossableType)
{
  // Arrange:
  lanelet::LineString3d ls(lanelet::utils::getId());
  ls.attributes()[lanelet::AttributeName::Type] = "road_border";
  std::vector<std::string> types = {"road_border", "curb"};

  // Act & Assert:
  EXPECT_TRUE(utils::is_uncrossable_type(types, ls));

  ls.attributes()[lanelet::AttributeName::Type] = "lane_divider";
  EXPECT_FALSE(utils::is_uncrossable_type(types, ls));
}

// Evaluates behavior with empty projection list.
TEST(UncrossableBoundaryUtilsTest, TestEvaluateProjectionsSeverityEmpty)
{
  // Arrange:
  Side<ProjectionsToBound> input;
  UncrossableBoundaryDepartureParam param;

  // Act:
  auto result = input.transform_each_side([&](const auto & side_value) {
    return utils::filter_and_assign_departure_types(side_value, param, 10.0);
  });

  // Assert:
  EXPECT_TRUE(result.left.empty());
  EXPECT_TRUE(result.right.empty());
}

// Evaluates behavior when all projections are laterally safe.
TEST(UncrossableBoundaryUtilsTest, TestEvaluateProjectionsSeverityNone)
{
  // Arrange:
  Side<ProjectionsToBound> input;
  UncrossableBoundaryDepartureParam param;
  param.lateral_margin_m = 0.5;

  ProjectionToBound safe_pt(0);
  safe_pt.lat_dist = 2.0;               // safely away (> 0.5)
  safe_pt.dist_along_trajectory_m = 5.0;
  safe_pt.time_from_start = 1.0;
  input.left.push_back(safe_pt);

  // Act:
  auto transformed = input.transform_each_side([&](const auto & side_value) {
    return utils::filter_and_assign_departure_types(side_value, param, 10.0);
  });
  auto result = transformed.transform_each_side([&](const auto & side_value) {
    return utils::apply_backward_buffer_and_filter(side_value, param);
  });

  // Assert:
  EXPECT_FALSE(result.left.has_value());
}

// Evaluates assignment of departure types (NONE, APPROACHING, CRITICAL) based on metrics.
TEST(UncrossableBoundaryUtilsTest, TestEvaluateProjectionsSeverityApproaching)
{
  // Arrange:
  Side<ProjectionsToBound> input;
  UncrossableBoundaryDepartureParam param;
  param.lateral_margin_m = 0.5;
  param.time_to_departure_cutoff_s = 2.0;
  double min_braking_dist = 10.0;

  ProjectionToBound app_pt(0);
  app_pt.lat_dist = 0.1;               // laterally close (<= 0.5)
  app_pt.dist_along_trajectory_m = 15.0;  // longitudinally far (> 10.0)
  app_pt.ego_front_to_proj_offset_m = 0.0;
  app_pt.time_from_start = 3.0;        // temporally far (> 2.0)
  input.left.push_back(app_pt);

  // Act:
  auto result = input.transform_each_side([&](const auto & side_value) {
    return utils::filter_and_assign_departure_types(side_value, param, min_braking_dist);
  });

  // Assert:
  ASSERT_EQ(result.left.size(), 1);
  EXPECT_TRUE(result.left.front().is_approaching());
}

// Evaluates the backward buffering logic to extend critical state safely.
TEST(UncrossableBoundaryUtilsTest, TestEvaluateProjectionsSeverityBackwardBuffer)
{
  // Arrange:
  Side<ProjectionsToBound> input;
  UncrossableBoundaryDepartureParam param;
  param.lateral_margin_m = 0.5;
  param.time_to_departure_cutoff_s = 2.0;
  param.longitudinal_margin_m = 1.0;  // 1.0m backward buffer
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

  input.left.push_back(create_pt(0, 12.0, 2.4));  // APPROACHING
  input.left.push_back(create_pt(1, 13.0, 2.6));  // APPROACHING
  input.left.push_back(create_pt(2, 14.0, 2.8));  // APPROACHING
  input.left.push_back(create_pt(3, 15.0, 1.9));  // CRITICAL

  // Act:
  auto result = input.transform_each_side([&](const auto & side_value) {
    const auto min_to_bounds =
      utils::filter_and_assign_departure_types(side_value, param, min_braking_dist);
    return utils::apply_backward_buffer_and_filter(min_to_bounds, param);
  });

  // Assert:
  ASSERT_TRUE(result.left.has_value());
  const auto & crit_pair = result.left.value();
  EXPECT_DOUBLE_EQ(crit_pair.physical_departure_point.dist_along_trajectory_m, 15.0);
  EXPECT_EQ(crit_pair.safety_buffer_start.pose_index, 2);
  EXPECT_DOUBLE_EQ(crit_pair.safety_buffer_start.dist_along_trajectory_m, 14.0);

  // Assert: (Plotting)
  plot_backward_buffer_results(input, crit_pair, export_folder);
}

// Evaluates R-tree construction for uncrossable boundaries.
TEST(UncrossableBoundaryUtilsTest, TestBuildUncrossableBoundariesRTree)
{
  // Arrange:
  lanelet::LaneletMap map;
  lanelet::Point3d p1(lanelet::utils::getId(), 0.0, 0.0, 0.0);
  lanelet::Point3d p2(lanelet::utils::getId(), 1.0, 0.0, 0.0);
  lanelet::Point3d p3(lanelet::utils::getId(), 2.0, 0.0, 0.0);
  lanelet::LineString3d ls1(lanelet::utils::getId(), {p1, p2, p3});
  ls1.attributes()[lanelet::AttributeName::Type] = "road_border";
  map.add(ls1);

  lanelet::Point3d p4(lanelet::utils::getId(), 0.0, 1.0, 0.0);
  lanelet::Point3d p5(lanelet::utils::getId(), 1.0, 1.0, 0.0);
  lanelet::LineString3d ls2(lanelet::utils::getId(), {p4, p5});
  ls2.attributes()[lanelet::AttributeName::Type] = "lane_divider";
  map.add(ls2);

  std::vector<std::string> types_to_detect = {"road_border"};

  // Act:
  auto rtree = utils::build_uncrossable_boundaries_rtree(map, types_to_detect);

  // Assert:
  EXPECT_EQ(rtree.size(), 2);  // 2 segments from ls1
  std::vector<SegmentWithIdx> results;
  rtree.query(bgi::nearest(lanelet::BasicPoint2d(0.5, 0.0), 1), std::back_inserter(results));
  ASSERT_EQ(results.size(), 1);
  EXPECT_EQ(results.front().second.linestring_id, ls1.id());
}
}  // namespace autoware::boundary_departure_checker
Id(), {p4, p5});
  ls2.attributes()[lanelet::AttributeName::Type] = "lane_divider";
  map.add(ls2);

  std::vector<std::string> types_to_detect = {"road_border"};

  // Act:
  auto rtree = utils::build_uncrossable_boundaries_rtree(map, types_to_detect);

  // Assert:
  EXPECT_EQ(rtree.size(), 2);  // 2 segments from ls1
  std::vector<SegmentWithIdx> results;
  rtree.query(bgi::nearest(lanelet::BasicPoint2d(0.5, 0.0), 1), std::back_inserter(results));
  ASSERT_EQ(results.size(), 1);
  EXPECT_EQ(results.front().second.linestring_id, ls1.id());
}
}  // namespace autoware::boundary_departure_checker
{p4, p5});
  ls2.attributes()[lanelet::AttributeName::Type] = "lane_divider";
  map.add(ls2);

  std::vector<std::string> types_to_detect = {"road_border"};

  // Act:
  auto rtree = utils::build_uncrossable_boundaries_rtree(map, types_to_detect);

  // Assert:
  EXPECT_EQ(rtree.size(), 2);  // 2 segments from ls1
  std::vector<SegmentWithIdx> results;
  rtree.query(bgi::nearest(lanelet::BasicPoint2d(0.5, 0.0), 1), std::back_inserter(results));
  ASSERT_EQ(results.size(), 1);
  EXPECT_EQ(results.front().second.linestring_id, ls1.id());
}
}  // namespace autoware::boundary_departure_checker
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

// Evaluates uncrossable boundary type detection based on attribute strings.
TEST(UncrossableBoundaryUtilsTest, TestIsUncrossableType)
{
  // Arrange:
  lanelet::LineString3d ls(lanelet::utils::getId());
  ls.attributes()[lanelet::AttributeName::Type] = "road_border";
  std::vector<std::string> types = {"road_border", "curb"};

  // Act & Assert:
  EXPECT_TRUE(utils::is_uncrossable_type(types, ls));

  ls.attributes()[lanelet::AttributeName::Type] = "lane_divider";
  EXPECT_FALSE(utils::is_uncrossable_type(types, ls));
}

// Evaluates behavior with empty projection list.
TEST(UncrossableBoundaryUtilsTest, TestEvaluateProjectionsSeverityEmpty)
{
  // Arrange:
  Side<ProjectionsToBound> input;
  UncrossableBoundaryDepartureParam param;

  // Act:
  auto result = input.transform_each_side([&](const auto & side_value) {
    return utils::filter_and_assign_departure_types(side_value, param, 10.0);
  });

  // Assert:
  EXPECT_TRUE(result.left.empty());
  EXPECT_TRUE(result.right.empty());
}

// Evaluates behavior when all projections are laterally safe.
TEST(UncrossableBoundaryUtilsTest, TestEvaluateProjectionsSeverityNone)
{
  // Arrange:
  Side<ProjectionsToBound> input;
  UncrossableBoundaryDepartureParam param;
  param.lateral_margin_m = 0.5;

  ProjectionToBound safe_pt(0);
  safe_pt.lat_dist = 2.0;               // safely away (> 0.5)
  safe_pt.dist_along_trajectory_m = 5.0;
  safe_pt.time_from_start = 1.0;
  input.left.push_back(safe_pt);

  // Act:
  auto transformed = input.transform_each_side([&](const auto & side_value) {
    return utils::filter_and_assign_departure_types(side_value, param, 10.0);
  });
  auto result = transformed.transform_each_side([&](const auto & side_value) {
    return utils::apply_backward_buffer_and_filter(side_value, param);
  });

  // Assert:
  EXPECT_FALSE(result.left.has_value());
}

// Evaluates assignment of departure types (NONE, APPROACHING, CRITICAL) based on metrics.
TEST(UncrossableBoundaryUtilsTest, TestEvaluateProjectionsSeverityApproaching)
{
  // Arrange:
  Side<ProjectionsToBound> input;
  UncrossableBoundaryDepartureParam param;
  param.lateral_margin_m = 0.5;
  param.time_to_departure_cutoff_s = 2.0;
  double min_braking_dist = 10.0;

  ProjectionToBound app_pt(0);
  app_pt.lat_dist = 0.1;               // laterally close (<= 0.5)
  app_pt.dist_along_trajectory_m = 15.0;  // longitudinally far (> 10.0)
  app_pt.ego_front_to_proj_offset_m = 0.0;
  app_pt.time_from_start = 3.0;        // temporally far (> 2.0)
  input.left.push_back(app_pt);

  // Act:
  auto result = input.transform_each_side([&](const auto & side_value) {
    return utils::filter_and_assign_departure_types(side_value, param, min_braking_dist);
  });

  // Assert:
  ASSERT_EQ(result.left.size(), 1);
  EXPECT_TRUE(result.left.front().is_approaching());
}

// Evaluates the backward buffering logic to extend critical state safely.
TEST(UncrossableBoundaryUtilsTest, TestEvaluateProjectionsSeverityBackwardBuffer)
{
  // Arrange:
  Side<ProjectionsToBound> input;
  UncrossableBoundaryDepartureParam param;
  param.lateral_margin_m = 0.5;
  param.time_to_departure_cutoff_s = 2.0;
  param.longitudinal_margin_m = 1.0;  // 1.0m backward buffer
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

  input.left.push_back(create_pt(0, 12.0, 2.4));  // APPROACHING
  input.left.push_back(create_pt(1, 13.0, 2.6));  // APPROACHING
  input.left.push_back(create_pt(2, 14.0, 2.8));  // APPROACHING
  input.left.push_back(create_pt(3, 15.0, 1.9));  // CRITICAL

  // Act:
  auto result = input.transform_each_side([&](const auto & side_value) {
    const auto min_to_bounds =
      utils::filter_and_assign_departure_types(side_value, param, min_braking_dist);
    return utils::apply_backward_buffer_and_filter(min_to_bounds, param);
  });

  // Assert:
  ASSERT_TRUE(result.left.has_value());
  const auto & crit_pair = result.left.value();
  EXPECT_DOUBLE_EQ(crit_pair.physical_departure_point.dist_along_trajectory_m, 15.0);
  EXPECT_EQ(crit_pair.safety_buffer_start.pose_index, 2);
  EXPECT_DOUBLE_EQ(crit_pair.safety_buffer_start.dist_along_trajectory_m, 14.0);

  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();

    std::vector<double> cand_x, cand_y;
    for (const auto & cand : input.left) {
      cand_x.push_back(cand.dist_along_trajectory_m);
      cand_y.push_back(cand.lat_dist);
    }
    plt.scatter(
      Args(cand_x, cand_y), Kwargs(
                              "color"_a = "gray", "marker"_a = "x", "s"_a = 60,
                              "label"_a = "All Candidates", "alpha"_a = 0.5));

    std::vector<double> crit_x, crit_y;
    crit_x.push_back(crit_pair.physical_departure_point.dist_along_trajectory_m);
    crit_y.push_back(crit_pair.physical_departure_point.lat_dist);
    crit_x.push_back(crit_pair.safety_buffer_start.dist_along_trajectory_m);
    crit_y.push_back(crit_pair.safety_buffer_start.lat_dist);

    plt.scatter(Args(crit_x, crit_y), Kwargs("color"_a = "red", "label"_a = "Critical (Buffered)"));

    plt.axvline(
      Args(15.0), Kwargs("color"_a = "black", "linestyle"_a = ":", "label"_a = "Crash Point"));
    plt.axvline(
      Args(14.0),
      Kwargs("color"_a = "purple", "linestyle"_a = "--", "label"_a = "Buffer Limit (1.0m)"));

    plt.xlabel(Args("Longitudinal Distance [m]"));
    plt.ylabel(Args("Lateral Distance [m]"));
    plt.title(Args("Evaluate Projections Severity: Backward Buffer"));
    plt.legend();

    save_figure(plt, export_folder);
  });
}

// Evaluates R-tree construction for uncrossable boundaries.
TEST(UncrossableBoundaryUtilsTest, TestBuildUncrossableBoundariesRTree)
{
  // Arrange:
  lanelet::LaneletMap map;
  lanelet::Point3d p1(lanelet::utils::getId(), 0.0, 0.0, 0.0);
  lanelet::Point3d p2(lanelet::utils::getId(), 1.0, 0.0, 0.0);
  lanelet::Point3d p3(lanelet::utils::getId(), 2.0, 0.0, 0.0);
  lanelet::LineString3d ls1(lanelet::utils::getId(), {p1, p2, p3});
  ls1.attributes()[lanelet::AttributeName::Type] = "road_border";
  map.add(ls1);

  lanelet::Point3d p4(lanelet::utils::getId(), 0.0, 1.0, 0.0);
  lanelet::Point3d p5(lanelet::utils::getId(), 1.0, 1.0, 0.0);
  lanelet::LineString3d ls2(lanelet::utils::getId(), {p4, p5});
  ls2.attributes()[lanelet::AttributeName::Type] = "lane_divider";
  map.add(ls2);

  std::vector<std::string> types_to_detect = {"road_border"};

  // Act:
  auto rtree = utils::build_uncrossable_boundaries_rtree(map, types_to_detect);

  // Assert:
  EXPECT_EQ(rtree.size(), 2);  // 2 segments from ls1
  std::vector<SegmentWithIdx> results;
  rtree.query(bgi::nearest(lanelet::BasicPoint2d(0.5, 0.0), 1), std::back_inserter(results));
  ASSERT_EQ(results.size(), 1);
  EXPECT_EQ(results.front().second.linestring_id, ls1.id());
}
}  // namespace autoware::boundary_departure_checker
