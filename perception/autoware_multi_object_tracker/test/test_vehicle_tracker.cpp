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

#include "autoware/multi_object_tracker/tracker/update/vehicle_update_strategy.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace autoware::multi_object_tracker
{
namespace
{
// Matches the constants defined inside correctWheelAnchorLateral.
constexpr double kAlpha = 0.2;  // balance_alpha
constexpr double kBeta = 0.3;   // corner_residual_beta

// correctWheelAnchorLateral is pure scalar math: given the signed lateral offset of the observed
// edge center from the tracker center, it returns the lateral move and added variance. The
// corrected lateral coordinate is `lateral_offset + lateral_move`.
struct LateralResult
{
  double lateral;  // corrected lateral coordinate (= lateral_offset + lateral_move)
  double var_lat;
};

LateralResult run(double tracker_width, double polygon_width, double lateral_offset)
{
  double var_lat = 0.0;
  const double lateral_move =
    correctWheelAnchorLateral(lateral_offset, tracker_width, polygon_width, var_lat);
  return {lateral_offset + lateral_move, var_lat};
}
}  // namespace

// Equal widths: no slack, no gap -> anchor untouched, no added variance.
TEST(CorrectWheelAnchorLateral, EqualWidthIsNoOp)
{
  const auto r = run(2.0, 2.0, 0.5);
  EXPECT_DOUBLE_EQ(r.lateral, 0.5);  // anchor unchanged
  EXPECT_DOUBLE_EQ(r.var_lat, 0.0);
}

// Polygon narrower than the tracker (partial view): anchor kept, worst-case lateral offset added
// as variance, var = ((w_t - w_p) / 2)^2.
TEST(CorrectWheelAnchorLateral, NarrowPolygonAddsVarianceOnly)
{
  const auto r = run(2.0, 1.0, 0.5);
  EXPECT_DOUBLE_EQ(r.lateral, 0.5);   // anchor unchanged
  EXPECT_DOUBLE_EQ(r.var_lat, 0.25);  // (0.5)^2
}

// Wide polygon, observed center on the body axis: nothing to pull, full slack uncertainty.
TEST(CorrectWheelAnchorLateral, WideCenteredHoldsAnchorMaxVariance)
{
  const double slack = 1.0;  // (4 - 2) / 2
  const auto r = run(2.0, 4.0, 0.0);
  EXPECT_DOUBLE_EQ(r.lateral, 0.0);
  EXPECT_DOUBLE_EQ(r.var_lat, slack * slack);  // std = slack
}

// Wide polygon, observed center inside the slack: anchor held near the tracker (slope alpha), so it
// is pulled back toward the body axis rather than snapped to the observed center.
TEST(CorrectWheelAnchorLateral, WideContainedPullsTowardTracker)
{
  const double slack = 1.0;
  const double d = 0.5;  // < slack -> contained
  const auto r = run(2.0, 4.0, d);
  EXPECT_DOUBLE_EQ(r.lateral, kAlpha * d);  // 0.1, pulled in from 0.5
  const double t = d / slack;
  const double std_lat = slack * (1.0 - (1.0 - kBeta) * t);
  EXPECT_DOUBLE_EQ(r.var_lat, std_lat * std_lat);
}

// Wide polygon, observed center beyond the slack: a corner is exposed, anchor follows it (unit
// slope) and the added variance shrinks toward beta * slack.
TEST(CorrectWheelAnchorLateral, WideUncontainedFollowsCorner)
{
  const double slack = 1.0;
  const double d = 2.0;  // > slack -> corner exposed
  const auto r = run(2.0, 4.0, d);
  EXPECT_DOUBLE_EQ(r.lateral, (d - slack) + kAlpha * slack);       // 1.2
  EXPECT_DOUBLE_EQ(r.var_lat, (kBeta * slack) * (kBeta * slack));  // std = beta * slack
}

// The dead-zone is continuous at |d| = slack: both branches agree on anchor and variance.
TEST(CorrectWheelAnchorLateral, ContinuousAtBoundary)
{
  const double slack = 1.0;
  const double eps = 1e-6;
  const auto inside = run(2.0, 4.0, slack - eps);
  const auto outside = run(2.0, 4.0, slack + eps);
  EXPECT_NEAR(inside.lateral, outside.lateral, 1e-4);
  EXPECT_NEAR(inside.var_lat, outside.var_lat, 1e-4);
}

// Sign symmetry: a negative lateral offset mirrors the positive case.
TEST(CorrectWheelAnchorLateral, SignSymmetry)
{
  const auto pos = run(2.0, 4.0, 1.5);
  const auto neg = run(2.0, 4.0, -1.5);
  EXPECT_DOUBLE_EQ(pos.lateral, -neg.lateral);
  EXPECT_DOUBLE_EQ(pos.var_lat, neg.var_lat);
}

}  // namespace autoware::multi_object_tracker
