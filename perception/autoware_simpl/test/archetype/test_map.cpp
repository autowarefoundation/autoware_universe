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

#include "autoware/simpl/archetype/map.hpp"

#include <gtest/gtest.h>

namespace autoware::simpl
{
namespace
{
archetype::MapPoint construct_point()
{
  return {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, archetype::MapLabel::ROADWAY};
}

archetype::MapPoint construct_other()
{
  return {2.0, 2.0, 2.0, 1.0, 1.0, 1.0, archetype::MapLabel::ROADWAY};
}
}  // namespace

TEST(TestMap, testMapPoint)
{
  const auto point = construct_point();

  // check elements
  EXPECT_DOUBLE_EQ(point.x, 1.0);
  EXPECT_DOUBLE_EQ(point.y, 1.0);
  EXPECT_DOUBLE_EQ(point.z, 1.0);
  EXPECT_DOUBLE_EQ(point.dx, 1.0);
  EXPECT_DOUBLE_EQ(point.dy, 1.0);
  EXPECT_DOUBLE_EQ(point.dz, 1.0);
  EXPECT_EQ(point.label, archetype::MapLabel::ROADWAY);

  // check member functions
  EXPECT_EQ(point.num_attribute(), 7);

  constexpr double abs_error = 1e-3;
  EXPECT_NEAR(point.distance(), 1.73202, abs_error);

  const auto other = construct_other();
  EXPECT_NEAR(point.distance_from(other), 1.73202, abs_error);

  const auto [diff_x, diff_y, diff_z] = point.diff(other);
  EXPECT_NEAR(diff_x, -1.0, abs_error);
  EXPECT_NEAR(diff_y, -1.0, abs_error);
  EXPECT_NEAR(diff_z, -1.0, abs_error);

  const auto result = point.interpolate(other, 0.5);
  EXPECT_NEAR(result.x, 1.5, abs_error);
  EXPECT_NEAR(result.y, 1.5, abs_error);
  EXPECT_NEAR(result.z, 1.5, abs_error);
  // dx/dy/dz are always 0.0
  EXPECT_NEAR(result.dx, 0.0, abs_error);
  EXPECT_NEAR(result.dy, 0.0, abs_error);
  EXPECT_NEAR(result.dz, 0.0, abs_error);
  EXPECT_EQ(result.label, archetype::MapLabel::ROADWAY);
}
}  // namespace autoware::simpl
