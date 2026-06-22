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
// Mirrors the zone constants defined inside correctWheelAnchorLateral.
constexpr double kDeadFrac  = 0.20;
constexpr double kInnerFrac = 0.50;
constexpr double kOuterFrac = 0.40;

struct LateralResult
{
  double lateral;  // corrected lateral coordinate (= lateral_offset + lateral_move)
  double var_lat;
};

LateralResult run(double tracker_width, double polygon_width, double lateral_offset)
{
  const double tracker_half = tracker_width * 0.5;
  const double polygon_half = polygon_width * 0.5;
  double var_lat = 0.0;
  const double lateral_move = correctWheelAnchorLateral(
    +tracker_half, -tracker_half,
    lateral_offset + polygon_half, lateral_offset - polygon_half,
    var_lat);
  return {lateral_offset + lateral_move, var_lat};
}
}  // namespace

// Equal widths, centered: all excesses are zero -> zone 1, no correction, no variance.
TEST(CorrectWheelAnchorLateral, EqualWidthCenteredIsNoOp)
{
  const auto r = run(2.0, 2.0, 0.0);
  EXPECT_DOUBLE_EQ(r.lateral, 0.0);
  EXPECT_DOUBLE_EQ(r.var_lat, 0.0);
}

// Zone 1: polygon slightly wider than tracker (excess < dead), centered.
// No correction; uncorrected residuals (0.1 each side) contribute to var.
TEST(CorrectWheelAnchorLateral, Zone1DeadZoneNoCorrection)
{
  // tracker_width=2 -> H=1, dead=0.2. polygon_width=2.2 -> excess=0.1 on both sides.
  const double exc = 0.1;  // < dead (=0.2*H=0.2)
  const auto r = run(2.0, 2.2, 0.0);
  EXPECT_DOUBLE_EQ(r.lateral, 0.0);
  EXPECT_NEAR(r.var_lat, exc * exc + exc * exc, 1e-9);
}

// Zone 2 symmetric: polygon much narrower, centered -> corrections cancel, no net move.
TEST(CorrectWheelAnchorLateral, Zone2SymmetricNoMove)
{
  // excess_L = excess_R = 0.25 - 1.0 = -0.75 -> both fully matched (t=1)
  const auto r = run(2.0, 0.5, 0.0);
  EXPECT_DOUBLE_EQ(r.lateral, 0.0);
  EXPECT_DOUBLE_EQ(r.var_lat, 0.0);  // t=1 -> residual = 0
}

// Zone 2 asymmetric: narrow polygon offset left, width-based weight pulls anchor toward center.
TEST(CorrectWheelAnchorLateral, Zone2WidthBasedPullsTowardCenter)
{
  // tracker=2 (H=1), polygon=1.4 (half=0.7), offset=0.4
  // half_width_miss = (2-1.4)/2 = 0.3; dead=0.2, inner=0.5
  // t_inner = clamp((0.3-0.2)/(0.5-0.2), 0,1) = 1/3
  // lateral_move = -0.4 * (1/3) = -2/15
  // resid = 0.3 * (2/3) = 0.2 -> var_lat = 2*0.04 = 0.08
  const double dead         = kDeadFrac * 1.0;
  const double inner        = kInnerFrac * 1.0;
  const double half_wm      = 0.3;
  const double lat_off      = 0.4;
  const double t_inner      = std::clamp((half_wm - dead) / (inner - dead), 0.0, 1.0);
  const double exp_move     = -lat_off * t_inner;
  const double resid        = half_wm * (1.0 - t_inner);
  const double exp_var      = 2.0 * resid * resid;
  const auto r = run(2.0, 1.4, lat_off);
  EXPECT_NEAR(r.lateral, lat_off + exp_move, 1e-9);
  EXPECT_NEAR(r.var_lat, exp_var, 1e-9);
}

// Zone 3 symmetric: wide polygon centered -> both sides cancel, no net move, no residual.
TEST(CorrectWheelAnchorLateral, Zone3SymmetricNoMove)
{
  // tracker=2 H=1, polygon=3.6 -> excess = 0.8 on both sides (>> outer=0.4), t=1
  const auto r = run(2.0, 3.6, 0.0);
  EXPECT_DOUBLE_EQ(r.lateral, 0.0);
  EXPECT_DOUBLE_EQ(r.var_lat, 0.0);
}

// Zone 3 asymmetric: wide polygon, anchor offset left -> net move right (toward tracker center).
TEST(CorrectWheelAnchorLateral, Zone3AsymmetricPullsTowardCenter)
{
  // tracker=2 H=1, polygon=3.6, offset=0.3
  // excess_L = 0.3+1.8-1 = 1.1 -> t_L=1.0, excess_R = -1-(0.3-1.8) = 0.5 -> t_R=1.0
  const auto r = run(2.0, 3.6, 0.3);
  EXPECT_NEAR(r.lateral, 0.3 + (-1.1 * 1.0 + 0.5 * 1.0), 1e-9);  // = -0.3
  EXPECT_DOUBLE_EQ(r.var_lat, 0.0);
}

// Zone 3 partial: one side in zone 3 (partially), other in zone 1.
TEST(CorrectWheelAnchorLateral, Zone3PartialOneSideCorrected)
{
  // tracker=2 H=1, polygon=2.6 (half=1.3), offset=0.3
  // excess_L = 0.3+1.3-1 = 0.6 -> zone 3, t_L = clamp((0.6-0.2)/0.2, 0,1) = 1.0
  // excess_R = -1-(0.3-1.3) = 0.0 -> zone 1, t_R = 0
  const double H       = 1.0;
  const double dead    = kDeadFrac * H;
  const double outer   = kOuterFrac * H;
  const double exc_L   = 0.6;
  const double exc_R   = 0.0;
  const double t_L     = std::clamp((exc_L - dead) / (outer - dead), 0.0, 1.0);
  const double t_R     = 0.0;
  const double exp_move = -exc_L * t_L + exc_R * t_R;
  const auto r = run(2.0, 2.6, 0.3);
  EXPECT_NEAR(r.lateral, 0.3 + exp_move, 1e-9);
  EXPECT_NEAR(r.var_lat, 0.0, 1e-9);  // both residuals are 0
}

// Continuity at zone 1/2 boundary: just inside vs just outside dead zone -> nearly identical.
TEST(CorrectWheelAnchorLateral, ContinuousAtZone1Zone2Boundary)
{
  constexpr double eps = 1e-6;
  // tracker=2 H=1, polygon_width chosen so excess_L = -(dead ± eps), offset=0 keeps both sides equal
  // dead = 0.2, excess = polygon_half - 1 => polygon_half = 1 + excess => polygon_width = 2*(1+excess)
  const double dead = kDeadFrac * 1.0;
  const double w_in  = 2.0 * (1.0 - dead + eps);  // excess just inside dead zone
  const double w_out = 2.0 * (1.0 - dead - eps);  // excess just below dead zone (zone 2)
  const auto inside  = run(2.0, w_in,  0.0);
  const auto outside = run(2.0, w_out, 0.0);
  EXPECT_NEAR(inside.lateral,  outside.lateral,  1e-4);
  EXPECT_NEAR(inside.var_lat,  outside.var_lat,  1e-4);
}

// Continuity at zone 1/3 boundary: just inside vs just outside dead zone (wide side).
TEST(CorrectWheelAnchorLateral, ContinuousAtZone1Zone3Boundary)
{
  constexpr double eps = 1e-6;
  const double dead  = kDeadFrac * 1.0;
  const double w_in  = 2.0 * (1.0 + dead - eps);  // excess just below dead (zone 1)
  const double w_out = 2.0 * (1.0 + dead + eps);  // excess just above dead (zone 3)
  const auto inside  = run(2.0, w_in,  0.0);
  const auto outside = run(2.0, w_out, 0.0);
  EXPECT_NEAR(inside.lateral,  outside.lateral,  1e-4);
  EXPECT_NEAR(inside.var_lat,  outside.var_lat,  1e-4);
}

// Sign symmetry: flipping lateral_offset mirrors corrected lateral and preserves variance.
TEST(CorrectWheelAnchorLateral, SignSymmetry)
{
  const double d = 0.3;
  const auto pos = run(2.0, 3.6, +d);
  const auto neg = run(2.0, 3.6, -d);
  EXPECT_NEAR(pos.lateral, -neg.lateral, 1e-9);
  EXPECT_NEAR(pos.var_lat,  neg.var_lat, 1e-9);
}

}  // namespace autoware::multi_object_tracker
