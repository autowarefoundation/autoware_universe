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
#include "autoware/simpl/archetype/map.hpp"
#include "autoware/simpl/processing/geometry.hpp"

#include <gtest/gtest.h>

#include <cmath>

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
}  // namespace

TEST(TestGeometry, testAgentTransform2dWithState)
{
  const auto from = construct_state();
  const auto to = construct_state();
  const auto result = processing::transform2d(from, to);

  constexpr double abs_error = 1e-6;
  EXPECT_NEAR(result.x, 0.0, abs_error);
  EXPECT_NEAR(result.y, 0.0, abs_error);
  EXPECT_NEAR(result.z, 1.0, abs_error);
  EXPECT_NEAR(result.yaw, 0.0, abs_error);
  EXPECT_NEAR(result.vx, -1.0, abs_error);
  EXPECT_NEAR(result.vy, 1.0, abs_error);
  EXPECT_EQ(result.label, archetype::AgentLabel::VEHICLE);
  EXPECT_TRUE(result.is_valid);
}

TEST(TestGeometry, testAgentTransform2dWithElement)
{
  const double from_x = 1.0;
  const double from_y = 1.0;
  const double from_vx = 1.0;
  const double from_vy = 1.0;
  const auto to = construct_state();
  const auto [result_x, result_y, result_vx, result_vy] =
    processing::transform2d(from_x, from_y, from_vx, from_vy, to);

  constexpr double abs_error = 1e-6;
  EXPECT_NEAR(result_x, 0.0, abs_error);
  EXPECT_NEAR(result_y, 2.0, abs_error);
  EXPECT_NEAR(result_vx, -1.0, abs_error);
  EXPECT_NEAR(result_vy, 1.0, abs_error);
}

TEST(TestGeometry, testMapTransform2dWithPoint)
{
  const auto from = construct_point();
  const auto to = construct_state();
  const auto result = processing::transform2d(from, to);

  constexpr double abs_error = 1e-6;
  EXPECT_NEAR(result.x, 0.0, abs_error);
  EXPECT_NEAR(result.y, 0.0, abs_error);
  EXPECT_NEAR(result.z, 1.0, abs_error);
  EXPECT_NEAR(result.dx, -1.0, abs_error);
  EXPECT_NEAR(result.dy, 1.0, abs_error);
  EXPECT_NEAR(result.dz, 1.0, abs_error);
  EXPECT_EQ(result.label, archetype::MapLabel::ROADWAY);
}

TEST(TestGeometry, testMapTransform2dWithElements)
{
  const auto from = construct_point();
  const double to_x = 1.0;
  const double to_y = 1.0;
  const double to_yaw = 0.5 * M_PI;
  const auto result = processing::transform2d(from, to_x, to_y, to_yaw);

  constexpr double abs_error = 1e-6;
  EXPECT_NEAR(result.x, 0.0, abs_error);
  EXPECT_NEAR(result.y, 0.0, abs_error);
  EXPECT_NEAR(result.z, 1.0, abs_error);
  EXPECT_NEAR(result.dx, -1.0, abs_error);
  EXPECT_NEAR(result.dy, 1.0, abs_error);
  EXPECT_NEAR(result.dz, 1.0, abs_error);
  EXPECT_EQ(result.label, archetype::MapLabel::ROADWAY);
}
}  // namespace autoware::simpl
