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

#include "autoware/simpl_prediction/archetype/polyline.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

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
using autoware::simpl_prediction::archetype::Polyline;

TEST(TestPolyline, ConstructAndAccess)
{
  std::vector<MapPoint> pts = {
    {0.0, 0.0, 0.0, MapLabel::ROADWAY}, {1.0, 0.0, 0.0, MapLabel::ROADWAY}};
  Polyline polyline(0, pts);

  EXPECT_EQ(polyline.size(), 2u);
  EXPECT_EQ(polyline.front().x, 0.0);
  EXPECT_EQ(polyline.back().x, 1.0);
}

TEST(TestPolyline, EmptyPolyline)
{
  Polyline empty;
  EXPECT_TRUE(empty.empty());
  EXPECT_EQ(empty.size(), 0u);
}

TEST(TestPolyline, CenterCalculation)
{
  std::vector<MapPoint> pts = {
    {0.0, 0.0, 0.0, MapLabel::SOLID},
    {1.0, 0.0, 0.0, MapLabel::SOLID},
    {2.0, 0.0, 0.0, MapLabel::SOLID}};
  Polyline polyline(0, pts);
  auto center = polyline.center();

  EXPECT_TRUE(is_close(center.x, 1.0));
  EXPECT_TRUE(is_close(center.y, 0.0));
}

TEST(TestPolyline, DistanceFromAgentState)
{
  std::vector<MapPoint> pts = {
    {1.0, 1.0, 0.0, MapLabel::UNKNOWN}, {2.0, 2.0, 0.0, MapLabel::UNKNOWN}};
  Polyline polyline(0, pts);

  AgentState agent(0.0, 0.0, 0.0, 0.0, 0, 0, true);
  EXPECT_TRUE(polyline.distance_from(agent) > 0.0);
}

TEST(TestPolyline, TransformToAgentFrame)
{
  std::vector<MapPoint> pts = {
    {1.0, 0.0, 0.0, MapLabel::ROADWAY}, {2.0, 0.0, 0.0, MapLabel::ROADWAY}};
  Polyline polyline(0, pts);

  AgentState state(1.0, 0.0, 0.0, M_PI / 2, 0, 0, true);
  auto transformed = polyline.transform(state);

  EXPECT_EQ(transformed.size(), 2u);
  EXPECT_TRUE(is_close(transformed[0].x, 0.0));
  EXPECT_TRUE(is_close(transformed[0].y, 0.0));
  EXPECT_TRUE(is_close(transformed[1].x, 0.0));
  EXPECT_TRUE(is_close(transformed[1].y, -1.0));
}

TEST(TestPolyline, TrimNeighborsWithinRange)
{
  AgentState ref_state(0.0, 0.0, 0.0, 0.0, 0, 0, true);

  Polyline p1(0, {{1.0, 0.0, 0.0, MapLabel::ROADWAY}});
  Polyline p2(1, {{10.0, 0.0, 0.0, MapLabel::ROADWAY}});

  std::vector<Polyline> input = {p1, p2};
  auto result = trim_neighbors(input, ref_state, 5.0);

  ASSERT_EQ(result.size(), 1u);
  EXPECT_TRUE(is_close(result[0].center().x, 1.0));
}
}  // namespace autoware::simpl_prediction::test
