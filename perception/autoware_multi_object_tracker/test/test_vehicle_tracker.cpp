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

#include <Eigen/Eigenvalues>
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
  BicycleMotionModel no_blend = makeEvolvedVehicle();
  BicycleMotionModel blended = makeEvolvedVehicle();

  const double yaw_pre = no_blend.getYawState();
  ASSERT_NEAR(yaw_pre, 0.0, 1e-9);  // straight-line motion keeps the pre-update heading at zero

  EXPECT_TRUE(blended.blendAxleCovariance(1.0));

  const double yaw_no_blend = yawAfterCommonLateralBias(no_blend);
  const double yaw_blended = yawAfterCommonLateralBias(blended);

  // Without the blend the asymmetry levers the common bias into a clear spurious rotation.
  EXPECT_GT(std::abs(yaw_no_blend - yaw_pre), 5e-3);
  // With a full blend the same bias leaves the heading essentially untouched (pure translation).
  EXPECT_NEAR(yaw_blended, yaw_pre, 1e-6);
}

// ---------------------------------------------------------------------------------------------
// BicycleMotionModel process noise — common/differential front/rear structure.
// A single predict step from a zero covariance isolates the process noise: P_next = A*0*A^T + Q =
// Q, so the front/rear position blocks of the predicted covariance ARE the process-noise blocks.
// ---------------------------------------------------------------------------------------------
namespace
{
constexpr double kQStepSeconds = 0.1;  // single sub-step (< dt_max = 0.11)

// Predict one step from P = 0 and return the full state covariance (= process noise Q of that
// step).
BicycleMotionModel::StateMat processNoiseFromZero(const double yaw)
{
  BicycleMotionModel model;
  model.setMotionParams(
    object_model::normal_vehicle.process_noise, object_model::normal_vehicle.bicycle_state,
    object_model::normal_vehicle.process_limit);

  const std::array<double, 36> zero_cov{};  // zero pose + velocity covariance -> P = 0
  model.initialize(
    startTime(), 0.0, 0.0, yaw, zero_cov, 5.0 /*vel_long*/, 0.0 /*vel_long_cov*/, 0.0 /*vel_lat*/,
    0.0 /*vel_lat_cov*/, kVehicleLength);

  BicycleMotionModel::StateVec x;
  BicycleMotionModel::StateMat p;
  // Base overload returning the raw 6x6 covariance (hidden by the derived pose/twist overload).
  model.MotionModel<6>::getPredictedState(
    startTime() + rclcpp::Duration::from_seconds(kQStepSeconds), x, p);
  return p;
}

// 2x2 position block for axle points anchored at state indices (a, a+1) x (b, b+1).
Eigen::Matrix2d axleBlock(const BicycleMotionModel::StateMat & p, const int a, const int b)
{
  Eigen::Matrix2d block;
  block << p(a, b), p(a, b + 1), p(a + 1, b), p(a + 1, b + 1);
  return block;
}
}  // namespace

// The front/rear cross-covariance equals the common (rear) block, so whole-body translation noise
// stays out of the difference vector d = p2 - p1: Cov(d) = Cov(front) - Cov(rear) = D (differential
// heading/length noise only) instead of the old (2 - 2*kappa)*C + D leak.
TEST(ProcessNoiseAxleStructure, CrossBlockEqualsCommonBlock)
{
  for (const double yaw : {0.0, 0.7, -1.3}) {
    const auto p = processNoiseFromZero(yaw);
    const Eigen::Matrix2d rear = axleBlock(p, BicycleMotionModel::X1, BicycleMotionModel::X1);
    const Eigen::Matrix2d front = axleBlock(p, BicycleMotionModel::X2, BicycleMotionModel::X2);
    const Eigen::Matrix2d cross = axleBlock(p, BicycleMotionModel::X1, BicycleMotionModel::X2);

    // Common-mode translation is fully shared: cross block == rear (common) block C.
    EXPECT_TRUE(cross.isApprox(rear, 1e-12)) << "yaw=" << yaw << "\ncross:\n"
                                             << cross << "\nrear:\n"
                                             << rear;

    // Cov(d) = Cov(rear) + Cov(front) - cross - cross^T, which must equal the pure differential D.
    const Eigen::Matrix2d cov_d = rear + front - cross - cross.transpose();
    const Eigen::Matrix2d differential = front - rear;
    EXPECT_TRUE(cov_d.isApprox(differential, 1e-12)) << "yaw=" << yaw << "\nCov(d):\n"
                                                     << cov_d << "\nD:\n"
                                                     << differential;

    // The differential is genuinely non-zero, so real heading/length noise still grows Cov(d).
    EXPECT_GT(differential.trace(), 0.0) << "yaw=" << yaw;
  }
}

// The common/differential split is a sum of PSD blocks ([[C, C],[C, C+D]] has Schur complement
// D >= 0), so the predicted covariance stays positive semi-definite.
TEST(ProcessNoiseAxleStructure, PositiveSemiDefinite)
{
  for (const double yaw : {0.0, 0.7, -1.3}) {
    const auto p = processNoiseFromZero(yaw);
    Eigen::SelfAdjointEigenSolver<BicycleMotionModel::StateMat> solver(p);
    ASSERT_EQ(solver.info(), Eigen::Success) << "yaw=" << yaw;
    EXPECT_GE(solver.eigenvalues().minCoeff(), -1e-9) << "yaw=" << yaw;
  }
}

// ---------------------------------------------------------------------------------------------
// BicycleMotionModel::getPredictedState — exported pose covariance conversion.
// The exported (x, y, yaw) block is the axle covariance propagated through the single Jacobian G of
// [center, yaw] = f(X1, Y1, X2, Y2). It must use the center blend for position (not one axle
// point), carry the position<->yaw cross-covariance, and stay symmetric and PSD.
// ---------------------------------------------------------------------------------------------
namespace
{
std::array<double, 36> exportedPoseCov(const BicycleMotionModel & model)
{
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Twist twist;
  std::array<double, 36> pose_cov{};
  std::array<double, 36> twist_cov{};
  model.getPredictedState(evolvedTime(), pose, pose_cov, twist, twist_cov);
  return pose_cov;
}

// Exported 3x3 (x, y, yaw) block as an Eigen matrix.
Eigen::Matrix3d exportedPoseBlock(const BicycleMotionModel & model)
{
  const auto c = exportedPoseCov(model);
  Eigen::Matrix3d block;
  block << c[XYZRPY_COV_IDX::X_X], c[XYZRPY_COV_IDX::X_Y], c[XYZRPY_COV_IDX::X_YAW],
    c[XYZRPY_COV_IDX::Y_X], c[XYZRPY_COV_IDX::Y_Y], c[XYZRPY_COV_IDX::Y_YAW],
    c[XYZRPY_COV_IDX::YAW_X], c[XYZRPY_COV_IDX::YAW_Y], c[XYZRPY_COV_IDX::YAW_YAW];
  return block;
}
}  // namespace

// End-to-end: the exported block equals G * P_sub * G^T rebuilt from the raw axle covariance, so
// position rides the center blend and yaw the difference vector, from one consistent Jacobian.
TEST(ExportedPoseCovariance, MatchesJacobianPropagation)
{
  const BicycleMotionModel model = makeEvolvedVehicle();

  BicycleMotionModel::StateVec x;
  BicycleMotionModel::StateMat p;
  model.MotionModel<6>::getPredictedState(evolvedTime(), x, p);

  const double dx = x(BicycleMotionModel::X2) - x(BicycleMotionModel::X1);
  const double dy = x(BicycleMotionModel::Y2) - x(BicycleMotionModel::Y1);
  const double wheel_base = std::hypot(dx, dy);
  const double sin_yaw = dy / wheel_base;
  const double cos_yaw = dx / wheel_base;

  // normal_vehicle axle ratios (wheel_pos_ratio_front / _rear)
  const auto & bicycle_state = object_model::normal_vehicle.bicycle_state;
  const double lf_ratio = bicycle_state.wheel_pos_ratio_front;
  const double lr_ratio = bicycle_state.wheel_pos_ratio_rear;
  const double inv_wheel_base_ratio = 1.0 / (lf_ratio + lr_ratio);
  const double w_rear = lf_ratio * inv_wheel_base_ratio;
  const double w_front = lr_ratio * inv_wheel_base_ratio;

  Eigen::Matrix<double, 3, 4> g;
  g << w_rear, 0.0, w_front, 0.0, 0.0, w_rear, 0.0, w_front, sin_yaw / wheel_base,
    -cos_yaw / wheel_base, -sin_yaw / wheel_base, cos_yaw / wheel_base;
  const Eigen::Matrix3d expected = g * p.topLeftCorner<4, 4>() * g.transpose();

  EXPECT_TRUE(exportedPoseBlock(model).isApprox(expected, 1e-12))
    << "exported:\n"
    << exportedPoseBlock(model) << "\nexpected:\n"
    << expected;
}

// Position <-> yaw is genuinely correlated once the front/rear covariance is asymmetric (it was
// hard-coded to zero before), and the exported block is symmetric. For straight motion at yaw = 0
// the coupling lives in the lateral (y) <-> yaw term.
TEST(ExportedPoseCovariance, PositionYawCorrelated)
{
  const auto c = exportedPoseCov(makeEvolvedVehicle());

  EXPECT_GT(std::abs(c[XYZRPY_COV_IDX::X_YAW]) + std::abs(c[XYZRPY_COV_IDX::Y_YAW]), 1e-6);

  EXPECT_DOUBLE_EQ(c[XYZRPY_COV_IDX::X_YAW], c[XYZRPY_COV_IDX::YAW_X]);
  EXPECT_DOUBLE_EQ(c[XYZRPY_COV_IDX::Y_YAW], c[XYZRPY_COV_IDX::YAW_Y]);
  EXPECT_DOUBLE_EQ(c[XYZRPY_COV_IDX::X_Y], c[XYZRPY_COV_IDX::Y_X]);
}

// The assembled (x, y, yaw) block is positive semi-definite: G * P_sub * G^T with P_sub PSD.
TEST(ExportedPoseCovariance, PositiveSemiDefinite)
{
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(exportedPoseBlock(makeEvolvedVehicle()));
  ASSERT_EQ(solver.info(), Eigen::Success);
  EXPECT_GE(solver.eigenvalues().minCoeff(), -1e-12);
}

}  // namespace autoware::multi_object_tracker
