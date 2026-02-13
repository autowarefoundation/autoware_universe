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
#include <vector>

namespace
{
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
  const ProjectionToBound & projection)
{
  plot_separate_segment(plt, ego_seg, "blue", "Ego Side");
  plot_separate_segment(plt, boundary_seg, "red", "Boundary");
  plot_projection_line(plt, projection);
  plt.legend();
  plt.axis(Args("equal"));
}
}  // namespace

namespace autoware::boundary_departure_checker
{
constexpr const char * export_folder = "test_uncrossable_boundary_checker";

static pybind11::scoped_interpreter guard{};

TEST(UncrossableBoundaryTest, TestSegmentToSegmentProjection)
{
  // 1. Setup PyPlot context

  // Define segments
  Segment2d ego_seg{{0.0, 0.0}, {0.0, 2.0}};
  Segment2d boundary_seg{{1.0, 0.0}, {1.0, 2.0}};

  auto result = utils::segment_to_segment_nearest_projection(ego_seg, boundary_seg, 0);

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->lat_dist, 1.0, 1e-6);

  auto plt = autoware::pyplot::import();
  plot_ego_and_boundary(plt, ego_seg, boundary_seg, *result);
  save_figure("test_segment_to_segment_projection.png", export_folder);
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

  auto plt = autoware::pyplot::import();
  plot_ego_and_boundary(plt, ego_seg, boundary_seg, *result);
  save_figure("test_intersection_detection.png", export_folder);
}

}  // namespace autoware::boundary_departure_checker
