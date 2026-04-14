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
    autoware::boundary_departure_checker::save_figure(plt, "test_uncrossable_boundary_checker");
  });
#endif
}

void plot_realistic_departure(
  [[maybe_unused]] const autoware::boundary_departure_checker::TrajectoryPoints & ego_pred_traj,
  [[maybe_unused]] const autoware::boundary_departure_checker::FootprintSideSegmentsArray &
    ego_sides,
  [[maybe_unused]] const autoware::boundary_departure_checker::SegmentWithIdx & left_bound,
  [[maybe_unused]] const autoware::boundary_departure_checker::SegmentWithIdx & right_bound,
  [[maybe_unused]] const autoware::boundary_departure_checker::Side<
    std::vector<ProjectionToBound>> & result,
  [[maybe_unused]] const std::string & title)
{
#ifdef EXPORT_TEST_PLOT_FIGURE
  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();

    std::vector<double> traj_x, traj_y;
    for (const auto & p : ego_pred_traj) {
      traj_x.push_back(p.pose.position.x);
      traj_y.push_back(p.pose.position.y);
    }
    plt.plot(
      Args(traj_x, traj_y),
      Kwargs("color"_a = "gray", "linestyle"_a = "--", "marker"_a = "o", "label"_a = "Trajectory"));

    for (size_t i = 0; i < ego_sides.size(); ++i) {
      auto fl = ego_sides[i].left.first;
      auto rl = ego_sides[i].left.second;
      auto fr = ego_sides[i].right.first;
      auto rr = ego_sides[i].right.second;
      std::vector<double> bx = {fl.x(), rl.x(), rr.x(), fr.x(), fl.x()};
      std::vector<double> by = {fl.y(), rl.y(), rr.y(), fr.y(), fl.y()};
      plt.plot(Args(bx, by), Kwargs("color"_a = "blue", "linewidth"_a = 1.5, "alpha"_a = 0.5));
    }

    auto plot_bound = [&](const auto & b, const std::string & color, const std::string & label) {
      std::vector<double> bx = {b.first.first.x(), b.first.second.x()};
      std::vector<double> by = {b.first.first.y(), b.first.second.y()};
      plt.plot(Args(bx, by), Kwargs("color"_a = color, "linewidth"_a = 2.0, "label"_a = label));
    };
    plot_bound(left_bound, "red", "Left Boundary");
    plot_bound(right_bound, "darkred", "Right Boundary");

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
    plt.title(Args(title));
    autoware::boundary_departure_checker::save_figure(plt, "test_uncrossable_boundary_checker");
  });
#endif
}

void plot_braking_distance(
  [[maybe_unused]] const double acceleration, [[maybe_unused]] const double max_stop_accel,
  [[maybe_unused]] const double max_stop_jerk, [[maybe_unused]] const double delay_time,
  [[maybe_unused]] const double v_test, [[maybe_unused]] const double dist)
{
#ifdef EXPORT_TEST_PLOT_FIGURE
  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();
    std::vector<double> velocities, distances;
    for (double v = 0.0; v <= 20.0; v += 1.0) {
      velocities.push_back(v);
      if (
        const auto d_opt = autoware::motion_utils::calculate_stop_distance(
          v, acceleration, max_stop_accel, max_stop_jerk, delay_time))
        distances.push_back(*d_opt);
    }
    plt.plot(Args(velocities, distances), Kwargs("marker"_a = "o"));
    plt.xlabel(Args("Velocity [m/s]"));
    plt.ylabel(Args("Judge Line Distance [m]"));
    plt.title(Args("Braking Distance with Jerk Limit"));
    plt.plot(
      Args(std::vector<double>{v_test, v_test}, std::vector<double>{0.0, dist}),
      Kwargs("color"_a = "gray", "linestyle"_a = "--", "alpha"_a = 0.5));
    plt.plot(
      Args(std::vector<double>{0.0, v_test}, std::vector<double>{dist, dist}),
      Kwargs("color"_a = "gray", "linestyle"_a = "--", "alpha"_a = 0.5));
    autoware::boundary_departure_checker::save_figure(plt, "test_uncrossable_boundary_checker");
  });
#endif
}

void plot_point_projection(
  [[maybe_unused]] const autoware_utils_geometry::Point2d & p,
  [[maybe_unused]] const Segment2d & segment,
  [[maybe_unused]] const autoware_utils_geometry::Point2d & proj)
{
#ifdef EXPORT_TEST_PLOT_FIGURE
  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();
    plt.plot(
      Args(
        std::vector<double>{segment.first.x(), segment.second.x()},
        std::vector<double>{segment.first.y(), segment.second.y()}),
      Kwargs("color"_a = "red", "label"_a = "Boundary Segment"));
    plt.scatter(
      Args(std::vector<double>{p.x()}, std::vector<double>{p.y()}),
      Kwargs("label"_a = "Ego Point"));
    plt.plot(
      Args(std::vector<double>{p.x(), proj.x()}, std::vector<double>{p.y(), proj.y()}),
      Kwargs("color"_a = "green", "linestyle"_a = "--", "label"_a = "Lateral Projection"));
    plt.axis(Args("equal"));
    plt.legend();
    autoware::boundary_departure_checker::save_figure(plt, "test_uncrossable_boundary_checker");
  });
#endif
}

void plot_backward_buffer(
  [[maybe_unused]] const autoware::boundary_departure_checker::ProjectionsToBound & input_left,
  [[maybe_unused]] const autoware::boundary_departure_checker::CriticalPointPair & crit_pair)
{
#ifdef EXPORT_TEST_PLOT_FIGURE
  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();
    std::vector<double> cand_x, cand_y;
    for (const auto & cand : input_left) {
      cand_x.push_back(cand.dist_along_trajectory_m);
      cand_y.push_back(cand.lat_dist);
    }
    plt.scatter(
      Args(cand_x, cand_y), Kwargs(
                              "color"_a = "gray", "marker"_a = "x", "s"_a = 60,
                              "label"_a = "All Candidates", "alpha"_a = 0.5));
    std::vector<double> crit_x = {
      crit_pair.physical_departure_point.dist_along_trajectory_m,
      crit_pair.safety_buffer_start.dist_along_trajectory_m};
    std::vector<double> crit_y = {
      crit_pair.physical_departure_point.lat_dist, crit_pair.safety_buffer_start.lat_dist};
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
    autoware::boundary_departure_checker::save_figure(plt, "test_uncrossable_boundary_checker");
  });
#endif
}
}  // namespace

namespace autoware::boundary_departure_checker
{
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
  plot_ego_and_boundary(ego_seg, boundary_seg, result);
}

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
  plot_ego_and_boundary(ego_seg, boundary_seg, result);
}

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
  plot_ego_and_boundary(ego_seg, boundary_seg, result);
}

TEST(UncrossableBoundaryTest, TestMiddleOfSegmentCrossingForLonDist)
{
  // Verifies accurate longitudinal distance calculation when boundary crosses in the middle of an
  // ego segment.

  // Arrange:
  TrajectoryPoints ego_pred_traj;
  for (int i = 0; i < 3; ++i) {
    TrajectoryPoint p;
    p.pose.position.x = i * 10.0;
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
  bound.first = Segment2d{{10.0, 0.0}, {10.0, 2.0}};
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
  EXPECT_DOUBLE_EQ(proj_at_i.ego_front_to_proj_offset_m, 2.0);
  EXPECT_DOUBLE_EQ(proj_at_i.dist_along_trajectory_m, 8.0);
  plot_ego_and_boundary(ego_sides[1].left, bound.first, proj_at_i);
}

TEST(UncrossableBoundaryTest, TestRealisticLaneDeparture)
{
  // Verifies signed lateral distance transitions during a realistic left-side departure.

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
  left_bound.first = Segment2d{{-10.0, 9.4}, {120.0, 9.4}};
  left_bound.second = IdxForRTreeSegment{1, 0, 1};
  boundaries.left.push_back(left_bound);

  SegmentWithIdx right_bound;
  right_bound.first = Segment2d{{-10.0, -5.0}, {120.0, -5.0}};
  right_bound.second = IdxForRTreeSegment{2, 0, 1};
  boundaries.right.push_back(right_bound);

  // Act:
  auto result =
    utils::get_closest_boundary_segments_from_side(ego_pred_traj, boundaries, ego_sides);

  // Assert:
  EXPECT_NEAR(result.left[0].lat_dist, 5.6, 1e-6);
  EXPECT_NEAR(result.left[1].lat_dist, 0.0, 1e-6);
  EXPECT_NEAR(result.left[2].lat_dist, -5.6, 1e-6);
  plot_realistic_departure(
    ego_pred_traj, ego_sides, left_bound, right_bound, result, "Left Signed Distance Verification");
}

TEST(UncrossableBoundaryTest, TestRealisticRightLaneDeparture)
{
  // Verifies signed lateral distance transitions during a realistic right-side departure.

  // Arrange:
  TrajectoryPoints ego_pred_traj;
  for (int i = 0; i < 5; ++i) {
    TrajectoryPoint p;
    p.pose.position.x = i * 24.0;
    p.pose.position.y = i * -7.0;
    p.pose.orientation =
      autoware_utils_geometry::create_quaternion_from_yaw(std::atan2(-7.0, 24.0));
    p.time_from_start = rclcpp::Duration::from_seconds(i);
    ego_pred_traj.push_back(p);
  }

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
  right_bound.first = Segment2d{{-10.0, -9.4}, {120.0, -9.4}};
  right_bound.second = IdxForRTreeSegment{2, 0, 1};
  boundaries.right.push_back(right_bound);

  // Act:
  auto result =
    utils::get_closest_boundary_segments_from_side(ego_pred_traj, boundaries, ego_sides);

  // Assert:
  EXPECT_NEAR(result.right[0].lat_dist, 5.6, 1e-6);
  EXPECT_NEAR(result.right[1].lat_dist, 0.0, 1e-6);
  EXPECT_NEAR(result.right[2].lat_dist, -5.6, 1e-6);
  plot_realistic_departure(
    ego_pred_traj, ego_sides, left_bound, right_bound, result,
    "Right Signed Distance Verification");
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
  plot_braking_distance(acceleration, max_stop_accel, max_stop_jerk, delay_time, v_test, *dist_opt);
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
  plot_point_projection(p, segment, result->first);
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
  plot_backward_buffer(input.left, *result.left);
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
