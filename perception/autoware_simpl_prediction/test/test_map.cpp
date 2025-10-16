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

#include "autoware/simpl_prediction/archetype/agent.hpp"
#include "autoware/simpl_prediction/archetype/map.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace autoware::simpl_prediction::test
{
namespace
{
bool is_close(double a, double b, double tol = 1e-6)
{
  return std::abs(a - b) < tol;
}
}  // namespace

using autoware::simpl_prediction::archetype::AgentState;
using autoware::simpl_prediction::archetype::MapLabel;
using autoware::simpl_prediction::archetype::MapPoint;

TEST(TestMapPoint, DefaultConstructor)
{
  MapPoint p;
  EXPECT_DOUBLE_EQ(p.x, 0.0);
  EXPECT_DOUBLE_EQ(p.y, 0.0);
  EXPECT_DOUBLE_EQ(p.z, 0.0);
  EXPECT_EQ(p.label, MapLabel::UNKNOWN);
}

TEST(TestMapPoint, ParameterizedConstructor)
{
  MapPoint p(1.0, 2.0, 3.0, MapLabel::ROADWAY);
  EXPECT_DOUBLE_EQ(p.x, 1.0);
  EXPECT_DOUBLE_EQ(p.y, 2.0);
  EXPECT_DOUBLE_EQ(p.z, 3.0);
  EXPECT_EQ(p.label, MapLabel::ROADWAY);
}

TEST(TestMapPoint, DistanceToOrigin)
{
  MapPoint p(3.0, 4.0, 0.0, MapLabel::ROADWAY);
  EXPECT_DOUBLE_EQ(p.distance(), 5.0);
}

TEST(TestMapPoint, DistanceFromMapPoint)
{
  MapPoint a(1.0, 2.0, 0.0, MapLabel::ROADWAY);
  MapPoint b(4.0, 6.0, 0.0, MapLabel::ROADWAY);
  EXPECT_DOUBLE_EQ(a.distance_from(b), 5.0);
}

TEST(TestMapPoint, DistanceFromAgentState)
{
  MapPoint p(1.0, 2.0, 0.0, MapLabel::ROADWAY);
  AgentState s(4.0, 6.0, 0.0, 0.0, 0.0, 0.0, true);
  EXPECT_DOUBLE_EQ(p.distance_from(s), 5.0);
}

TEST(TestMapPoint, DiffUnnormalized)
{
  MapPoint a(5.0, 1.0, 0.0, MapLabel::DASHED);
  MapPoint b(2.0, 5.0, 0.0, MapLabel::DASHED);
  auto [dx, dy] = a.diff(b, false);
  EXPECT_DOUBLE_EQ(dx, 3.0);
  EXPECT_DOUBLE_EQ(dy, -4.0);
}

TEST(TestMapPoint, DiffNormalized)
{
  MapPoint a(5.0, 1.0, 0.0, MapLabel::DASHED);
  MapPoint b(2.0, 5.0, 0.0, MapLabel::DASHED);
  auto [nx, ny] = a.diff(b, true);
  EXPECT_TRUE(is_close(nx, 0.6));
  EXPECT_TRUE(is_close(ny, -0.8));
}

TEST(TestMapPoint, LinearInterpolation)
{
  MapPoint a(0.0, 0.0, 0.0, MapLabel::SOLID);
  MapPoint b(10.0, 10.0, 10.0, MapLabel::SOLID);

  MapPoint mid = a.lerp(b, 0.5);
  EXPECT_DOUBLE_EQ(mid.x, 5.0);
  EXPECT_DOUBLE_EQ(mid.y, 5.0);
  EXPECT_DOUBLE_EQ(mid.z, 5.0);
  EXPECT_EQ(mid.label, MapLabel::SOLID);
}

}  // namespace autoware::simpl_prediction::test
