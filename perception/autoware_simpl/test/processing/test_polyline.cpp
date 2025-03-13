// Copyright 2025 TIER IV, Inc.
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

#include "autoware/simpl/archetype/agent.hpp"
#include "autoware/simpl/archetype/datatype.hpp"
#include "autoware/simpl/archetype/map.hpp"
#include "autoware/simpl/processing/polyline.hpp"

#include <gtest/gtest.h>

#include <cstddef>

namespace autoware::simpl
{
namespace
{
archetype::AgentState construct_state()
{
  return {1.0, 1.0, 1.0, 0.5 * M_PI, 1.0, 1.0, archetype::AgentLabel::VEHICLE, true};
}

archetype::MapPoint construct_point()
{
  return {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, archetype::MapLabel::ROADWAY};
}

archetype::MapPoints construct_points()
{
  archetype::MapPoints points;
  for (size_t n = 0; n < 5; ++n) {
    points.emplace_back(construct_point());
  }
  return points;
}
}  // namespace

TEST(TestPolyline, testCreatePolylines)
{
  const auto map_points = construct_points();
  const auto current_ego = construct_state();
  constexpr size_t max_num_point = 2;
  constexpr double break_distance = 1.0;
  const auto result =
    processing::create_polylines(map_points, current_ego, max_num_point, break_distance);

  EXPECT_EQ(result.size(), 3);

  constexpr double abs_error = 1e-6;
  for (size_t n = 0; n < result.size(); ++n) {
    const auto & polyline = result.at(n);

    // first 2 polylines contain 3 points, but the last one contains 1 point
    if (n < 2) {
      EXPECT_EQ(polyline.size(), max_num_point);
    } else {
      EXPECT_EQ(polyline.size(), 1);
    }

    for (const auto & point : polyline) {
      EXPECT_NEAR(point.x, 0.0, abs_error);
      EXPECT_NEAR(point.y, 0.0, abs_error);
      EXPECT_NEAR(point.z, 1.0, abs_error);
      EXPECT_NEAR(point.dx, -1.0, abs_error);
      EXPECT_NEAR(point.dy, 1.0, abs_error);
      EXPECT_NEAR(point.dz, 1.0, abs_error);
      EXPECT_EQ(point.label, archetype::MapLabel::ROADWAY);
    }
  }
}

TEST(TestPolyline, testFindCenter)
{
  const archetype::MapPoints map_points{
    archetype::MapPoint(0.0, 1.0, 2.0, 0.0, 0.0, 0.0, archetype::MapLabel::ROADWAY),
    archetype::MapPoint(1.0, 2.0, 3.0, 1.0, 1.0, 1.0, archetype::MapLabel::ROADWAY),
    archetype::MapPoint(2.0, 3.0, 4.0, 1.0, 1.0, 1.0, archetype::MapLabel::ROADWAY),
    archetype::MapPoint(3.0, 4.0, 5.0, 1.0, 1.0, 1.0, archetype::MapLabel::ROADWAY),
    archetype::MapPoint(4.0, 5.0, 6.0, 1.0, 1.0, 1.0, archetype::MapLabel::ROADWAY),
    archetype::MapPoint(5.0, 6.0, 7.0, 1.0, 1.0, 1.0, archetype::MapLabel::ROADWAY)};

  const auto center = processing::find_center(map_points);

  constexpr double abs_error = 1e-6;
  EXPECT_NEAR(center.x, 2.5, abs_error);
  EXPECT_NEAR(center.y, 3.5, abs_error);
  EXPECT_NEAR(center.z, 4.5, abs_error);
  // dx/dy/dz are always 0.0
  EXPECT_NEAR(center.dx, 0.0, abs_error);
  EXPECT_NEAR(center.dy, 0.0, abs_error);
  EXPECT_NEAR(center.dz, 0.0, abs_error);
  EXPECT_EQ(center.label, archetype::MapLabel::ROADWAY);
}
}  // namespace autoware::simpl
