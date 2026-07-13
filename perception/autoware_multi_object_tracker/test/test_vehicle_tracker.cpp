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

#include "autoware/multi_object_tracker/object_model/object_model.hpp"
#include "autoware/multi_object_tracker/tracker/motion_model/bicycle_motion_model.hpp"
#include "autoware/multi_object_tracker/tracker/update/vehicle_update_strategy.hpp"

#include <autoware_utils_geometry/msg/covariance.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>

namespace autoware::multi_object_tracker
{
namespace
{
// correctWheelAnchorLateral is pure scalar math. The measurement polygon is centered on the
// observed edge-center anchor (lateral_offset) with half-width polygon_half; the tracker is
// centered at 0 with half-width tracker_half.
//
// Aligning the fixed-width tracker box to each polygon edge gives two candidate vehicle centers;
// their segment is the lateral "dead-zone" (width |polygon_width - tracker_width|). The corrected
// lateral coordinate is the tracker center (0) projected into that dead-zone, and the added lateral
// variance is the dead-zone half-width squared.
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
  const double lateral_move =
    correctWheelAnchorLateral(lateral_offset, tracker_half, polygon_half, var_lat);
  return {lateral_offset + lateral_move, var_lat};
}

// Variance is the dead-zone half-width squared, independent of the offset.
double expectedVar(double tracker_width, double polygon_width)
{
  const double half_dead_zone = 0.5 * std::abs(polygon_width - tracker_width);
  return half_dead_zone * half_dead_zone;
}
}  // namespace

// Equal widths: zero dead-zone, the two candidate centers coincide -> anchor untouched, no
// variance.
TEST(CorrectWheelAnchorLateral, EqualWidthIsNoOp)
{
  const auto r = run(2.0, 2.0, 0.5);
  EXPECT_DOUBLE_EQ(r.lateral, 0.5);  // anchor unchanged
  EXPECT_DOUBLE_EQ(r.var_lat, 0.0);
}

// Row 1 - polygon smaller than the tracker and fully inside it (both overhangs negative): the
// tracker center sits inside the dead-zone, so the anchor snaps to the tracker lateral center.
TEST(CorrectWheelAnchorLateral, SmallPolygonWithinSnapsToCenter)
{
  const auto r = run(2.0, 1.0, 0.3);  // polygon [-0.2, 0.8] inside tracker [-1, 1]
  EXPECT_DOUBLE_EQ(r.lateral, 0.0);
  EXPECT_DOUBLE_EQ(r.var_lat, expectedVar(2.0, 1.0));  // 0.25
}

// Row 2 - polygon smaller than the tracker but one edge protrudes (one overhang positive): the
// protruding polygon edge is taken as a real vehicle edge and the box slides by that overhang.
TEST(CorrectWheelAnchorLateral, SmallPolygonProtrudingFollowsEdge)
{
  const auto r = run(2.0, 1.0, 0.8);  // polygon [0.3, 1.3], left edge protrudes past tracker 1.0
  EXPECT_DOUBLE_EQ(r.lateral, 0.3);   // tracker_center + overhang_left (1.3 - 1.0)
  EXPECT_DOUBLE_EQ(r.var_lat, expectedVar(2.0, 1.0));  // 0.25
}

// Row 3 - polygon larger than the tracker, tracker fully inside it (both overhangs positive): the
// tracker center is inside the dead-zone, so the anchor is held at the tracker center.
TEST(CorrectWheelAnchorLateral, WidePolygonStraddlingHoldsCenter)
{
  const auto r = run(2.0, 4.0, 0.5);  // polygon [-1.5, 2.5] straddles tracker [-1, 1]
  EXPECT_DOUBLE_EQ(r.lateral, 0.0);
  EXPECT_DOUBLE_EQ(r.var_lat, expectedVar(2.0, 4.0));  // 1.0
}

// Row 4 - polygon larger than the tracker but one edge recedes inside it (one overhang negative):
// the recessed polygon edge is the real vehicle edge; the anchor snaps to that edge-aligned center.
TEST(CorrectWheelAnchorLateral, WidePolygonRecedingFollowsEdge)
{
  const auto r = run(2.0, 4.0, 2.0);  // polygon [0, 4], right edge 0 recedes inside tracker
  EXPECT_DOUBLE_EQ(r.lateral, 1.0);   // tracker right edge aligned to polygon_right (0 + 1)
  EXPECT_DOUBLE_EQ(r.var_lat, expectedVar(2.0, 4.0));  // 1.0
}

// The projection is continuous as the tracker center crosses the dead-zone boundary.
TEST(CorrectWheelAnchorLateral, ContinuousAtBoundary)
{
  const double eps = 1e-6;
  const auto inside = run(2.0, 4.0, 1.0 - eps);
  const auto outside = run(2.0, 4.0, 1.0 + eps);
  EXPECT_NEAR(inside.lateral, outside.lateral, 1e-4);
  EXPECT_NEAR(inside.var_lat, outside.var_lat, 1e-4);
}

// Sign symmetry: a negative lateral offset mirrors the positive case.
TEST(CorrectWheelAnchorLateral, SignSymmetry)
{
  const auto pos = run(2.0, 4.0, 2.0);
  const auto neg = run(2.0, 4.0, -2.0);
  EXPECT_DOUBLE_EQ(pos.lateral, -neg.lateral);
  EXPECT_DOUBLE_EQ(pos.var_lat, neg.var_lat);
}

// ---------------------------------------------------------------------------------------------
// BicycleMotionModel::blendAxleCovariance — front/rear covariance blend.
// ---------------------------------------------------------------------------------------------
namespace
{
using autoware_utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;

constexpr double kPredictSeconds = 3.0;  // let the front axle accrue heading/length process noise
constexpr double kVehicleLength = 4.5;
constexpr double kLateralShift = 0.6;  // [m] common-mode lateral measurement bias (yaw unchanged)

rclcpp::Time startTime()
{
  return rclcpp::Time(0, 0, RCL_ROS_TIME);
}
rclcpp::Time evolvedTime()
{
  return startTime() + rclcpp::Duration::from_seconds(kPredictSeconds);
}

// A straight-moving normal vehicle whose covariance has evolved for kPredictSeconds. Process noise
// hits the front axle point only, so the front/rear position covariance ends up asymmetric — the
// condition that lets a common lateral bias lever into yaw.
BicycleMotionModel makeEvolvedVehicle()
{
  BicycleMotionModel model;
  model.setMotionParams(
    object_model::normal_vehicle.process_noise, object_model::normal_vehicle.bicycle_state,
    object_model::normal_vehicle.process_limit);

  std::array<double, 36> pose_cov{};
  pose_cov[XYZRPY_COV_IDX::X_X] = 0.25;
  pose_cov[XYZRPY_COV_IDX::Y_Y] = 0.25;
  pose_cov[XYZRPY_COV_IDX::YAW_YAW] = 0.01;

  model.initialize(
    startTime(), 0.0, 0.0, 0.0, pose_cov, 3.0 /*vel_long*/, 1.0 /*vel_long_cov*/, 0.0 /*vel_lat*/,
    0.01 /*vel_lat_cov*/, kVehicleLength);
  model.predictState(evolvedTime());
  return model;
}

double exportedYawVariance(const BicycleMotionModel & model)
{
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Twist twist;
  std::array<double, 36> pose_cov{};
  std::array<double, 36> twist_cov{};
  model.getPredictedState(evolvedTime(), pose, pose_cov, twist, twist_cov);
  return pose_cov[XYZRPY_COV_IDX::YAW_YAW];
}

// A tight, common-mode lateral measurement: the body center shifted by +kLateralShift in y with an
// UNCHANGED yaw of 0 — exactly the signature of a localization heading bias at range.
double yawAfterCommonLateralBias(BicycleMotionModel & model)
{
  std::array<double, 36> meas_cov{};
  meas_cov[XYZRPY_COV_IDX::X_X] = 0.1;
  meas_cov[XYZRPY_COV_IDX::Y_Y] = 0.1;
  model.updateStatePoseHead(0.0, kLateralShift, 0.0, meas_cov, kVehicleLength);
  return model.getYawState();
}
}  // namespace

// blend_ratio = 0 is a no-op: neither the heading state nor the exported yaw variance changes.
TEST(BlendAxleCovariance, NoOpAtZero)
{
  BicycleMotionModel model = makeEvolvedVehicle();
  const double yaw_before = model.getYawState();
  const double yaw_var_before = exportedYawVariance(model);

  EXPECT_TRUE(model.blendAxleCovariance(0.0));

  EXPECT_DOUBLE_EQ(model.getYawState(), yaw_before);
  EXPECT_DOUBLE_EQ(exportedYawVariance(model), yaw_var_before);
}

// The persymmetric average preserves Cov(difference), so the exported yaw variance is unchanged
// even at full blend — the blend removes coupling, it does not loosen (or tighten) heading.
TEST(BlendAxleCovariance, PreservesYawVariance)
{
  BicycleMotionModel model = makeEvolvedVehicle();
  const double yaw_var_before = exportedYawVariance(model);

  EXPECT_TRUE(model.blendAxleCovariance(1.0));

  EXPECT_NEAR(exportedYawVariance(model), yaw_var_before, 1e-9);
}

// The core behavior: a common lateral bias rotates the box when the front/rear covariance is
// asymmetric (no blend), but only translates it after a full blend.
TEST(BlendAxleCovariance, CommonLateralBiasDoesNotRotate)
{
  BicycleMotionModel unblended = makeEvolvedVehicle();
  BicycleMotionModel blended = makeEvolvedVehicle();

  const double yaw_pre = unblended.getYawState();
  ASSERT_NEAR(yaw_pre, 0.0, 1e-9);  // straight-line motion keeps the pre-update heading at zero

  EXPECT_TRUE(blended.blendAxleCovariance(1.0));

  const double yaw_unblended = yawAfterCommonLateralBias(unblended);
  const double yaw_blended = yawAfterCommonLateralBias(blended);

  // Without the blend the asymmetry levers the common bias into a clear spurious rotation.
  EXPECT_GT(std::abs(yaw_unblended - yaw_pre), 5e-3);
  // With a full blend the same bias leaves the heading essentially untouched (pure translation).
  EXPECT_NEAR(yaw_blended, yaw_pre, 1e-6);
}

}  // namespace autoware::multi_object_tracker
