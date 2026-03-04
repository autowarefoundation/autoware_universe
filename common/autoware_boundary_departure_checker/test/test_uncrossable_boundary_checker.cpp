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

TEST(UncrossableBoundaryUtilsTest, TestCalcJudgeLineDist)
{
  constexpr double acceleration = 0.0;
  constexpr double max_stop_accel = -4.0;
  constexpr double max_stop_jerk = -10.0;
  constexpr double delay_time = 1.0;

  constexpr double v_test = 10.0;
  const double dist = utils::calc_judge_line_dist_with_jerk_limit(
    v_test, acceleration, max_stop_accel, max_stop_jerk, delay_time);
  EXPECT_GT(dist, 22.5);

  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();

    std::vector<double> velocities;
    std::vector<double> distances;
    for (double v = 0.0; v <= 20.0; v += 1.0) {
      velocities.push_back(v);
      distances.push_back(
        utils::calc_judge_line_dist_with_jerk_limit(
          v, acceleration, max_stop_accel, max_stop_jerk, delay_time));
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
