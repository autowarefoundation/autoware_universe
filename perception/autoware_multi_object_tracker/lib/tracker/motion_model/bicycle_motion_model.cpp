// Copyright 2024 Tier IV, Inc.
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
//
//
// Author: v1.0 Taekjin Lee
//
#define EIGEN_MPL2_ONLY

#include "autoware/multi_object_tracker/tracker/motion_model/bicycle_motion_model.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <autoware_utils/ros/msg_covariance.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>

namespace autoware::multi_object_tracker
{

// cspell: ignore CTRV
// Bicycle CTRV motion model
// CTRV : Constant Turn Rate and constant Velocity
using autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;

BicycleMotionModel::BicycleMotionModel() : logger_(rclcpp::get_logger("BicycleMotionModel"))
{
  // set prediction parameters
  constexpr double dt_max = 0.11;  // [s] maximum time interval for prediction
  setMaxDeltaTime(dt_max);
}

void BicycleMotionModel::setMotionParams(
  const double & q_stddev_acc_long, const double & q_stddev_acc_lat,
  const double & q_stddev_yaw_rate_min, const double & q_stddev_yaw_rate_max,
  const double & q_stddev_slip_rate_min, const double & q_stddev_slip_rate_max,
  const double & q_max_slip_angle, const double & lf_ratio, const double & lf_min,
  const double & lr_ratio, const double & lr_min)
{
  // set process noise covariance parameters
  motion_params_.q_stddev_acc_long = q_stddev_acc_long;
  motion_params_.q_stddev_acc_lat = q_stddev_acc_lat;
  motion_params_.q_cov_acc_long = q_stddev_acc_long * q_stddev_acc_long;
  motion_params_.q_cov_acc_lat = q_stddev_acc_lat * q_stddev_acc_lat;
  motion_params_.q_stddev_yaw_rate_min = q_stddev_yaw_rate_min;
  motion_params_.q_stddev_yaw_rate_max = q_stddev_yaw_rate_max;
  motion_params_.q_cov_slip_rate_min = q_stddev_slip_rate_min * q_stddev_slip_rate_min;
  motion_params_.q_cov_slip_rate_max = q_stddev_slip_rate_max * q_stddev_slip_rate_max;
  motion_params_.q_max_slip_angle = q_max_slip_angle;

  constexpr double minimum_wheel_pos = 0.01;  // minimum of 0.01m
  if (lf_min < minimum_wheel_pos || lr_min < minimum_wheel_pos) {
    RCLCPP_WARN(
      logger_,
      "BicycleMotionModel::setMotionParams: minimum wheel position should be greater than 0.01m.");
  }
  motion_params_.lf_min = std::max(minimum_wheel_pos, lf_min);
  motion_params_.lr_min = std::max(minimum_wheel_pos, lr_min);
  motion_params_.lf_ratio = lf_ratio;
  motion_params_.lr_ratio = lr_ratio;
}

void BicycleMotionModel::setMotionLimits(const double & max_vel, const double & max_slip)
{
  // set motion limitations
  motion_params_.max_vel = max_vel;
  motion_params_.max_slip = max_slip;
}

bool BicycleMotionModel::initialize(
  const rclcpp::Time & time, const double & x, const double & y, const double & yaw,
  const std::array<double, 36> & pose_cov, const double & vel_x, const double & vel_x_cov,
  const double & vel_y, const double & vel_y_cov, const double & length)
{
  double lr = length * motion_params_.lr_ratio;
  double lf = length * motion_params_.lf_ratio;
  lr = std::max(lr, motion_params_.lr_min);
  lf = std::max(lf, motion_params_.lf_min);
  const double x1 = x - lr * std::cos(yaw);
  const double y1 = y - lr * std::sin(yaw);
  const double x2 = x + lf * std::cos(yaw);
  const double y2 = y + lf * std::sin(yaw);

  // initialize state vector X
  StateVec X;
  X << x1, y1, x2, y2, vel_x, vel_y * 2.0;

  // initialize covariance matrix P
  StateMat P;
  P.setZero();
  P(IDX::X1, IDX::X1) = pose_cov[XYZRPY_COV_IDX::X_X];
  P(IDX::X1, IDX::Y1) = pose_cov[XYZRPY_COV_IDX::X_Y];
  P(IDX::Y1, IDX::X1) = pose_cov[XYZRPY_COV_IDX::Y_X];
  P(IDX::Y1, IDX::Y1) = pose_cov[XYZRPY_COV_IDX::Y_Y];
  P(IDX::X2, IDX::X2) = pose_cov[XYZRPY_COV_IDX::X_X];
  P(IDX::X2, IDX::Y2) = pose_cov[XYZRPY_COV_IDX::X_Y];
  P(IDX::Y2, IDX::X2) = pose_cov[XYZRPY_COV_IDX::Y_X];
  P(IDX::Y2, IDX::Y2) = pose_cov[XYZRPY_COV_IDX::Y_Y];
  P(IDX::VX, IDX::VX) = vel_x_cov;
  P(IDX::VY, IDX::VY) = vel_y_cov;

  return MotionModel::initialize(time, X, P);
}

double BicycleMotionModel::getYawState() const
{
  // get yaw angle from the state
  return std::atan2(getStateElement(IDX::Y2) - getStateElement(IDX::Y1),
                    getStateElement(IDX::X2) - getStateElement(IDX::X1));
}

double BicycleMotionModel::getLength() const
{
  // get length of the vehicle from the state
  const double wheel_base = std::hypot(getStateElement(IDX::X2) - getStateElement(IDX::X1),
                    getStateElement(IDX::Y2) - getStateElement(IDX::Y1));
  return wheel_base / (motion_params_.lf_ratio + motion_params_.lr_ratio);
}

bool BicycleMotionModel::updateStatePose(
  const double & x, const double & y, const std::array<double, 36> & pose_cov, const double & length)
{
  // yaw angle is not provided, so we use the current yaw state
  const double yaw = getYawState();
  return updateStatePoseHead(x, y, yaw, pose_cov, length);
}

bool BicycleMotionModel::updateStatePoseHead(
  const double & x, const double & y, const double & yaw, const std::array<double, 36> & pose_cov, const double & length)
{
  // check if the state is initialized
  if (!checkInitialized()) return false;

  // convert the state to the bicycle model state
  double lr = length * motion_params_.lr_ratio;
  double lf = length * motion_params_.lf_ratio;
  lr = std::max(lr, motion_params_.lr_min);
  lf = std::max(lf, motion_params_.lf_min);
  const double x1 = x - lr * std::cos(yaw);
  const double y1 = y - lr * std::sin(yaw);
  const double x2 = x + lf * std::cos(yaw);
  const double y2 = y + lf * std::sin(yaw);

  // update state
  constexpr int DIM_Y = 4;
  Eigen::Matrix<double, DIM_Y, 1> Y;
  Y << x1, y1, x2, y2;

  Eigen::Matrix<double, DIM_Y, DIM> C = Eigen::Matrix<double, DIM_Y, DIM>::Zero();
  C(0, IDX::X1) = 1.0;
  C(1, IDX::Y1) = 1.0;
  C(2, IDX::X2) = 1.0;
  C(3, IDX::Y2) = 1.0;

  Eigen::Matrix<double, DIM_Y, DIM_Y> R = Eigen::Matrix<double, DIM_Y, DIM_Y>::Zero();
  R(0, 0) = pose_cov[XYZRPY_COV_IDX::X_X];
  R(0, 1) = pose_cov[XYZRPY_COV_IDX::X_Y];
  R(1, 0) = pose_cov[XYZRPY_COV_IDX::Y_X];
  R(1, 1) = pose_cov[XYZRPY_COV_IDX::Y_Y];
  R(2, 2) = pose_cov[XYZRPY_COV_IDX::X_X];
  R(2, 3) = pose_cov[XYZRPY_COV_IDX::X_Y];
  R(3, 2) = pose_cov[XYZRPY_COV_IDX::Y_X];
  R(3, 3) = pose_cov[XYZRPY_COV_IDX::Y_Y];

  // todo: implement yaw covariance

  return ekf_.update(Y, C, R);
}

bool BicycleMotionModel::updateStatePoseVel(
  const double & x, const double & y, const std::array<double, 36> & pose_cov, const double & vel_x, const double & vel_y, 
  const std::array<double, 36> & twist_cov, const double & length)
{
  // check if the state is initialized
  if (!checkInitialized()) return false;

  // convert the state to the bicycle model state
  const double yaw = getYawState();
  double lr = length * motion_params_.lr_ratio;
  double lf = length * motion_params_.lf_ratio;
  lr = std::max(lr, motion_params_.lr_min);
  lf = std::max(lf, motion_params_.lf_min);
  const double x1 = x - lr * std::cos(yaw);
  const double y1 = y - lr * std::sin(yaw);
  const double x2 = x + lf * std::cos(yaw);
  const double y2 = y + lf * std::sin(yaw);

  // update state, with velocity
  constexpr int DIM_Y = 6;
  Eigen::Matrix<double, DIM_Y, 1> Y;
  Y << x1, y1, x2, y2, vel_x, vel_y * 2.0;

  Eigen::Matrix<double, DIM_Y, DIM> C;
  C.setZero();
  C(0, IDX::X1) = 1.0;
  C(1, IDX::Y1) = 1.0;
  C(2, IDX::X2) = 1.0;
  C(3, IDX::Y2) = 1.0;
  C(4, IDX::VX) = 1.0;
  C(5, IDX::VY) = 1.0;

  Eigen::Matrix<double, DIM_Y, DIM_Y> R;
  R.setZero();
  R(0, 0) = pose_cov[XYZRPY_COV_IDX::X_X];
  R(0, 1) = pose_cov[XYZRPY_COV_IDX::X_Y];
  R(1, 0) = pose_cov[XYZRPY_COV_IDX::Y_X];
  R(1, 1) = pose_cov[XYZRPY_COV_IDX::Y_Y];
  R(2, 2) = pose_cov[XYZRPY_COV_IDX::X_X];
  R(2, 3) = pose_cov[XYZRPY_COV_IDX::X_Y];
  R(3, 2) = pose_cov[XYZRPY_COV_IDX::Y_X];
  R(3, 3) = pose_cov[XYZRPY_COV_IDX::Y_Y];
  R(4, 4) = twist_cov[XYZRPY_COV_IDX::X_X];
  R(5, 5) = twist_cov[XYZRPY_COV_IDX::Y_Y];

  return ekf_.update<DIM_Y>(Y, C, R);
}

bool BicycleMotionModel::updateStatePoseHeadVel(
  const double & x, const double & y, const double & yaw, const std::array<double, 36> & pose_cov,
  const double & vel_x, const double & vel_y, const std::array<double, 36> & twist_cov, const double & length)
{
  // check if the state is initialized
  if (!checkInitialized()) return false;

  // convert the state to the bicycle model state
  double lr = length * motion_params_.lr_ratio;
  double lf = length * motion_params_.lf_ratio;
  lr = std::max(lr, motion_params_.lr_min);
  lf = std::max(lf, motion_params_.lf_min);
  const double x1 = x - lr * std::cos(yaw);
  const double y1 = y - lr * std::sin(yaw);
  const double x2 = x + lf * std::cos(yaw);
  const double y2 = y + lf * std::sin(yaw);

  const double yaw_track = getYawState();
  const double yaw_delta = yaw - yaw_track;
  const double vel_long = vel_x * std::cos(yaw_delta) - vel_y * std::sin(yaw_delta);
  const double vel_lat = vel_x * std::sin(yaw_delta) + vel_y * std::cos(yaw_delta);

  // update state
  constexpr int DIM_Y = 6;
  Eigen::Matrix<double, DIM_Y, 1> Y;
  Y << x1, y1, x2, y2, vel_long, vel_lat * 2.0;

  Eigen::Matrix<double, DIM_Y, DIM> C = Eigen::Matrix<double, DIM_Y, DIM>::Zero();
  C(0, IDX::X1) = 1.0;
  C(1, IDX::Y1) = 1.0;
  C(2, IDX::X2) = 1.0;
  C(3, IDX::Y2) = 1.0;
  C(4, IDX::VX) = 1.0;
  C(5, IDX::VY) = 1.0;

  Eigen::Matrix<double, DIM_Y, DIM_Y> R = Eigen::Matrix<double, DIM_Y, DIM_Y>::Zero();
  R(0, 0) = pose_cov[XYZRPY_COV_IDX::X_X];
  R(0, 1) = pose_cov[XYZRPY_COV_IDX::X_Y];
  R(1, 0) = pose_cov[XYZRPY_COV_IDX::Y_X];
  R(1, 1) = pose_cov[XYZRPY_COV_IDX::Y_Y];
  R(2, 2) = pose_cov[XYZRPY_COV_IDX::X_X];
  R(2, 3) = pose_cov[XYZRPY_COV_IDX::X_Y];
  R(3, 2) = pose_cov[XYZRPY_COV_IDX::Y_X];
  R(3, 3) = pose_cov[XYZRPY_COV_IDX::Y_Y];
  R(4, 4) = twist_cov[XYZRPY_COV_IDX::X_X];
  R(5, 5) = twist_cov[XYZRPY_COV_IDX::Y_Y];

  return ekf_.update(Y, C, R);
}

bool BicycleMotionModel::limitStates()
{
  StateVec X_t;
  StateMat P_t;
  ekf_.getX(X_t);
  ekf_.getP(P_t);

  // maximum reverse velocity
  if (motion_params_.max_reverse_vel < 0 && X_t(IDX::VX) < motion_params_.max_reverse_vel) {
    // rotate the object orientation by 180 degrees
    // replace X1 and Y1 with X2 and Y2
    const double x1 = X_t(IDX::X1);
    const double y1 = X_t(IDX::Y1);
    X_t(IDX::X1) = X_t(IDX::X2);
    X_t(IDX::Y1) = X_t(IDX::Y2);
    X_t(IDX::X2) = x1;
    X_t(IDX::Y2) = y1;
    // reverse the velocity
    X_t(IDX::VX) = -X_t(IDX::VX);
    // rotation velocity does not change
  }
  // maximum velocity
  if (!(-motion_params_.max_vel <= X_t(IDX::VX) && X_t(IDX::VX) <= motion_params_.max_vel)) {
    X_t(IDX::VX) = X_t(IDX::VX) < 0 ? -motion_params_.max_vel : motion_params_.max_vel;
  }

  // // maximum slip angle
  // const double slip_angle = std::atan2(X_t(IDX::VY), X_t(IDX::VX));
  // if (!(-motion_params_.max_slip <= slip_angle && slip_angle <= motion_params_.max_slip)) {
  //   X_t(IDX::VY) = X_t(IDX::VY) < 0 ? -motion_params_.max_slip * X_t(IDX::VX) : motion_params_.max_slip * X_t(IDX::VX);
  // }

  // overwrite state
  ekf_.init(X_t, P_t);
  return true;
}

bool BicycleMotionModel::adjustPosition(const double & delta_x, const double & delta_y)
{
  // check if the state is initialized
  if (!checkInitialized()) return false;

  // adjust position
  StateVec X_t;
  StateMat P_t;
  ekf_.getX(X_t);
  ekf_.getP(P_t);
  X_t(IDX::X1) += delta_x;
  X_t(IDX::Y1) += delta_y;
  X_t(IDX::X2) += delta_x;
  X_t(IDX::Y2) += delta_y;
  ekf_.init(X_t, P_t);

  return true;
}

bool BicycleMotionModel::predictStateStep(const double dt, KalmanFilter & ekf) const
{
  /*  Motion model: static bicycle model (constant slip angle, constant velocity)
   *
   * wheel_base = sqrt((x2 - x1)^2 + (y2 - y1)^2)
   * yaw = atan2(y2 - y1, x2 - x1)
   * sin_theta = (y2 - y1) / wheel_base
   * cos_theta = (x2 - x1) / wheel_base
   * x1_{k+1}   = x1_k + vel_x_k*(x2_k - x1_k)/wheel_base * dt
   * y1_{k+1}   = y1_k + vel_x_k*(y2_k - y1_k)/wheel_base * dt
   * x2_{k+1}   = x2_k + vel_x_k*(x2_k - x1_k)/wheel_base * dt - vel_y_k*(y2_k - y1_k)/wheel_base * dt
   * y2_{k+1}   = y2_k + vel_x_k*(y2_k - y1_k)/wheel_base * dt + vel_y_k*(x2_k - x1_k)/wheel_base * dt
   * vel_x_{k+1} = vel_x_k
   * vel_y_{k+1} = vel_y_k * exp(-dt / 2.0)  // slip angle decays exponentially with a half-life of 2 seconds
   */

  /*  Jacobian Matrix
   *
   * A_x1 = [1 - vel_x_k / wheel_base * dt, 0, vel_x_k / wheel_base * dt, 0, (x2_k - x1_k)/wheel_base * dt, 0]
   * A_y1 = [0, 1 - vel_x_k / wheel_base * dt, 0, vel_x_k / wheel_base * dt, (y2_k - y1_k)/wheel_base * dt, 0]
   * A_x2 = [- vel_x_k / wheel_base * dt, vel_y_k / wheel_base * dt, 1 + vel_x_k / wheel_base * dt,     - vel_y_k / wheel_base * dt, (x2_k - x1_k)/wheel_base * dt, - (y2_k - y1_k)/wheel_base * dt]
   * A_y2 = [- vel_y_k / wheel_base * dt, vel_x_k / wheel_base * dt,     vel_y_k / wheel_base * dt,   1 + vel_x_k / wheel_base * dt, (y2_k - y1_k)/wheel_base * dt,   (x2_k - x1_k)/wheel_base * dt]
   * A_vx = [0, 0, 0, 0, 1, 0]
   * A_vy = [0, 0, 0, 0, 0, exp(-dt / 2.0)]
   */

  // Current state vector X t
  StateVec X_t;
  ekf.getX(X_t);

  const double x1 = X_t(IDX::X1);
  const double y1 = X_t(IDX::Y1);
  const double x2 = X_t(IDX::X2);
  const double y2 = X_t(IDX::Y2);
  const double vel_x = X_t(IDX::VX);
  const double vel_y = X_t(IDX::VY);

  const double yaw = std::atan2(y2 - y1, x2 - x1);
  const double wheel_base = std::hypot(x2 - x1, y2 - y1);
  const double sin_yaw = (y2 - y1) / wheel_base;
  const double cos_yaw = (x2 - x1) / wheel_base;
  const double wheel_base_inv_dt = dt / wheel_base;
  const double sin_yaw_dt = sin_yaw * dt;
  const double cos_yaw_dt = cos_yaw * dt;

  // Predict state vector X t+1
  StateVec X_next_t;
  X_next_t(IDX::X1) = x1 + vel_x * cos_yaw_dt;
  X_next_t(IDX::Y1) = y1 + vel_x * sin_yaw_dt;
  X_next_t(IDX::X2) = x2 + vel_x * cos_yaw_dt - vel_y * sin_yaw_dt;
  X_next_t(IDX::Y2) = y2 + vel_x * sin_yaw_dt + vel_y * cos_yaw_dt;
  X_next_t(IDX::VX) = vel_x;  // velocity does not change
  // Apply exponential decay to slip angle over time, with a half-life of 2 seconds
  constexpr double gamma = 0.69314718056;  // natural logarithm of 2
  const double decay_rate = std::exp(-dt * gamma / 2.0);
  X_next_t(IDX::VY) = vel_y * decay_rate;  // slip angle decays exponentially


  // State transition matrix A
  ProcessMat A;
  A.setZero();

  A(IDX::X1, IDX::X1) = 1.0 - vel_x * wheel_base_inv_dt;
  A(IDX::X1, IDX::X2) = vel_x * wheel_base_inv_dt;
  A(IDX::X1, IDX::VX) = cos_yaw_dt;

  A(IDX::Y1, IDX::Y1) = 1.0 - vel_x * wheel_base_inv_dt;
  A(IDX::Y1, IDX::Y2) = vel_x * wheel_base_inv_dt;
  A(IDX::Y1, IDX::VX) = sin_yaw_dt;

  A(IDX::X2, IDX::X1) = -vel_x * wheel_base_inv_dt;
  A(IDX::X2, IDX::Y1) = vel_y * wheel_base_inv_dt;
  A(IDX::X2, IDX::X2) = 1.0 + vel_x * wheel_base_inv_dt;
  A(IDX::X2, IDX::Y2) = -vel_y * wheel_base_inv_dt;
  A(IDX::X2, IDX::VX) = cos_yaw_dt;
  A(IDX::X2, IDX::VY) = -sin_yaw_dt;

  A(IDX::Y2, IDX::X1) = -vel_y * wheel_base_inv_dt;
  A(IDX::Y2, IDX::Y1) = vel_x * wheel_base_inv_dt;
  A(IDX::Y2, IDX::X2) = vel_y * wheel_base_inv_dt;
  A(IDX::Y2, IDX::Y2) = 1.0 + vel_x * wheel_base_inv_dt;
  A(IDX::Y2, IDX::VX) = sin_yaw_dt;
  A(IDX::Y2, IDX::VY) = cos_yaw_dt;

  A(IDX::VX, IDX::VX) = 1.0;  // velocity does not change
  A(IDX::VY, IDX::VY) = decay_rate;

  // // Process noise covariance Q
  // double q_stddev_yaw_rate = motion_params_.q_stddev_yaw_rate_min;
  // if (vel > 0.01) {
  //   /* uncertainty of the yaw rate is limited by the following:
  //    *  - centripetal acceleration a_lat : d(yaw)/dt = w = a_lat/v
  //    *  - or maximum slip angle slip_max : w = v*sin(slip_max)/l_r
  //    */
  //   q_stddev_yaw_rate = std::min(
  //     motion_params_.q_stddev_acc_lat / vel,
  //     vel * std::sin(motion_params_.q_max_slip_angle) / lr_);  // [rad/s]
  //   q_stddev_yaw_rate = std::clamp(
  //     q_stddev_yaw_rate, motion_params_.q_stddev_yaw_rate_min,
  //     motion_params_.q_stddev_yaw_rate_max);
  // }
  // double q_cov_slip_rate{0.0};
  // if (vel <= 0.01) {
  //   q_cov_slip_rate = motion_params_.q_cov_slip_rate_min;
  // } else {
  //   /* The slip angle rate uncertainty is modeled as follows:
  //    * d(slip)/dt ~ - sin(slip)/v * d(v)/dt + l_r/v * d(w)/dt
  //    * where sin(slip) = w * l_r / v
  //    *
  //    * d(w)/dt is assumed to be proportional to w (more uncertain when slip is large)
  //    * d(v)/dt and d(w)/t are considered to be uncorrelated
  //    */
  //   q_cov_slip_rate =
  //     std::pow(motion_params_.q_stddev_acc_lat * sin_slip / vel, 2) + std::pow(sin_slip * 1.5, 2);
  //   q_cov_slip_rate = std::min(q_cov_slip_rate, motion_params_.q_cov_slip_rate_max);
  //   q_cov_slip_rate = std::max(q_cov_slip_rate, motion_params_.q_cov_slip_rate_min);
  // }
  // const double dt2 = dt * dt;
  // const double dt4 = dt2 * dt2;
  // const double q_cov_x = 0.25 * motion_params_.q_cov_acc_long * dt4;
  // const double q_cov_y = 0.25 * motion_params_.q_cov_acc_lat * dt4;
  // const double q_cov_yaw = q_stddev_yaw_rate * q_stddev_yaw_rate * dt2;
  // const double q_cov_vel = motion_params_.q_cov_acc_long * dt2;
  // const double q_cov_slip = q_cov_slip_rate * dt2;

  // StateMat Q;
  // Q.setZero();
  // // Rotate the covariance matrix according to the vehicle yaw
  // // because q_cov_x and y are in the vehicle coordinate system.
  // Q(IDX::X, IDX::X) = (q_cov_x * cos_yaw * cos_yaw + q_cov_y * sin_yaw * sin_yaw);
  // Q(IDX::X, IDX::Y) = (0.5f * (q_cov_x - q_cov_y) * sin_2yaw);
  // Q(IDX::Y, IDX::Y) = (q_cov_x * sin_yaw * sin_yaw + q_cov_y * cos_yaw * cos_yaw);
  // Q(IDX::Y, IDX::X) = Q(IDX::X, IDX::Y);
  // Q(IDX::YAW, IDX::YAW) = q_cov_yaw;
  // Q(IDX::VEL, IDX::VEL) = q_cov_vel;
  // Q(IDX::SLIP, IDX::SLIP) = q_cov_slip;


  // Process noise covariance Q
  double q_cov_slip_rate = motion_params_.q_cov_slip_rate_min;
  constexpr double q_cov_length = 9.0;  // length uncertainty
  const double & q_stddev_yaw_rate = motion_params_.q_stddev_yaw_rate_min;
  const double q_stddev_head = q_stddev_yaw_rate * wheel_base * dt; // yaw uncertainty
  
  const double dt2 = dt * dt;
  const double dt4 = dt2 * dt2;

  const double q_cov_vel_x = motion_params_.q_cov_acc_long * dt2;
  const double q_cov_vel_y = q_cov_slip_rate * dt2 * 4.0;
  const double q_cov_x = 0.25 * motion_params_.q_cov_acc_long * dt4;
  const double q_cov_y = 0.25 * motion_params_.q_cov_acc_lat * dt4;
  const double q_cov_x2 = 0.25 * motion_params_.q_cov_acc_long * dt4 + q_cov_length * dt2;
  const double q_cov_y2 = 0.25 * motion_params_.q_cov_acc_lat * dt4 + q_stddev_head * q_stddev_head + 9.0 * dt2; 

  StateMat Q;
  Q.setZero();
  const double sin_2yaw = std::sin(2.0 * yaw);
  Q(IDX::X1, IDX::X1) = (q_cov_x * cos_yaw * cos_yaw + q_cov_y * sin_yaw * sin_yaw);
  Q(IDX::X1, IDX::Y1) = (0.5f * (q_cov_x - q_cov_y) * sin_2yaw);
  Q(IDX::Y1, IDX::X1) = Q(IDX::X1, IDX::Y1);
  Q(IDX::Y1, IDX::Y1) = (q_cov_x * sin_yaw * sin_yaw + q_cov_y * cos_yaw * cos_yaw);

  Q(IDX::X2, IDX::X2) = (q_cov_x2 * cos_yaw * cos_yaw + q_cov_y2 * sin_yaw * sin_yaw);
  Q(IDX::X2, IDX::Y2) = (0.5f * (q_cov_x2 - q_cov_y2) * sin_2yaw);
  Q(IDX::Y2, IDX::X2) = Q(IDX::X2, IDX::Y2);
  Q(IDX::Y2, IDX::Y2) = (q_cov_x2 * sin_yaw * sin_yaw + q_cov_y2 * cos_yaw * cos_yaw);
  // todo: add yaw process noise

  Q(IDX::VX, IDX::VX) = q_cov_vel_x;
  Q(IDX::VY, IDX::VY) = q_cov_vel_y;


  // control-input model B and control-input u are not used
  // Eigen::MatrixXd B = Eigen::MatrixXd::Zero(DIM, DIM);
  // Eigen::MatrixXd u = Eigen::MatrixXd::Zero(DIM, 1);

  // predict state
  return ekf.predict(X_next_t, A, Q);
}

bool BicycleMotionModel::getPredictedState(
  const rclcpp::Time & time, geometry_msgs::msg::Pose & pose, std::array<double, 36> & pose_cov,
  geometry_msgs::msg::Twist & twist, std::array<double, 36> & twist_cov) const
{
  // get predicted state
  StateVec X;
  StateMat P;
  if (!MotionModel::getPredictedState(time, X, P)) {
    return false;
  }
  const double yaw = std::atan2(X(IDX::Y2) - X(IDX::Y1), X(IDX::X2) - X(IDX::X1));
  const double wheel_base = std::hypot(X(IDX::X2) - X(IDX::X1), X(IDX::Y2) - X(IDX::Y1));

  // set position
  pose.position.x = 0.5 * (X(IDX::X1) + X(IDX::X2));
  pose.position.y = 0.5 * (X(IDX::Y1) + X(IDX::Y2));
  // do not change z

  // set orientation
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();

  // set twist
  twist.linear.x = X(IDX::VX);
  twist.linear.y = X(IDX::VY) * 0.5;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = X(IDX::VY) / wheel_base;

  // set pose covariance
  constexpr double zz_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  constexpr double rr_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  constexpr double pp_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  pose_cov[XYZRPY_COV_IDX::X_X] = (P(IDX::X1, IDX::X1) + P(IDX::X2, IDX::X2)) * 0.25;
  pose_cov[XYZRPY_COV_IDX::X_Y] = (P(IDX::X1, IDX::Y1) + P(IDX::X2, IDX::Y2)) * 0.25;
  pose_cov[XYZRPY_COV_IDX::Y_X] = (P(IDX::Y1, IDX::X1) + P(IDX::Y2, IDX::X2)) * 0.25;
  pose_cov[XYZRPY_COV_IDX::Y_Y] = (P(IDX::Y1, IDX::Y1) + P(IDX::Y2, IDX::Y2)) * 0.25;
  pose_cov[XYZRPY_COV_IDX::YAW_YAW] = P(IDX::X2, IDX::X2) * cos(yaw) + P(IDX::Y2, IDX::Y2) * sin(yaw);
  pose_cov[XYZRPY_COV_IDX::Z_Z] = zz_cov;
  pose_cov[XYZRPY_COV_IDX::ROLL_ROLL] = rr_cov;
  pose_cov[XYZRPY_COV_IDX::PITCH_PITCH] = pp_cov;

  // set twist covariance
  // Eigen::Matrix<double, 3, 2> cov_jacob;
  // cov_jacob << std::cos(X(IDX::SLIP)), -X(IDX::VX) * std::sin(X(IDX::SLIP)),
  //   std::sin(X(IDX::SLIP)), X(IDX::VX) * std::cos(X(IDX::SLIP)), std::sin(X(IDX::SLIP)) / lr_,
  //   X(IDX::VX) * std::cos(X(IDX::SLIP)) / lr_;
  // Eigen::Matrix2d cov_twist;
  // cov_twist << P(IDX::VEL, IDX::VEL), P(IDX::VEL, IDX::SLIP), P(IDX::SLIP, IDX::VEL),
  //   P(IDX::SLIP, IDX::SLIP);
  // Eigen::Matrix3d twist_cov_mat = cov_jacob * cov_twist * cov_jacob.transpose();
  // constexpr double vz_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  // constexpr double wx_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  // constexpr double wy_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  // twist_cov[XYZRPY_COV_IDX::X_X] = twist_cov_mat(0, 0);
  // twist_cov[XYZRPY_COV_IDX::X_Y] = twist_cov_mat(0, 1);
  // twist_cov[XYZRPY_COV_IDX::X_YAW] = twist_cov_mat(0, 2);
  // twist_cov[XYZRPY_COV_IDX::Y_X] = twist_cov_mat(1, 0);
  // twist_cov[XYZRPY_COV_IDX::Y_Y] = twist_cov_mat(1, 1);
  // twist_cov[XYZRPY_COV_IDX::Y_YAW] = twist_cov_mat(1, 2);
  // twist_cov[XYZRPY_COV_IDX::YAW_X] = twist_cov_mat(2, 0);
  // twist_cov[XYZRPY_COV_IDX::YAW_Y] = twist_cov_mat(2, 1);
  // twist_cov[XYZRPY_COV_IDX::YAW_YAW] = twist_cov_mat(2, 2);
  // twist_cov[XYZRPY_COV_IDX::Z_Z] = vz_cov;
  // twist_cov[XYZRPY_COV_IDX::ROLL_ROLL] = wx_cov;
  // twist_cov[XYZRPY_COV_IDX::PITCH_PITCH] = wy_cov;

  constexpr double vel_cov = 0.1 * 0.1;
  twist_cov[XYZRPY_COV_IDX::X_X] = P(IDX::VX, IDX::VX);
  twist_cov[XYZRPY_COV_IDX::Y_Y] = P(IDX::VY, IDX::VY);
  twist_cov[XYZRPY_COV_IDX::YAW_YAW] = P(IDX::VY, IDX::VY) / (wheel_base * wheel_base) * 0.25;
  twist_cov[XYZRPY_COV_IDX::Z_Z] = vel_cov;
  twist_cov[XYZRPY_COV_IDX::ROLL_ROLL] = vel_cov;
  twist_cov[XYZRPY_COV_IDX::PITCH_PITCH] = vel_cov;

  return true;
}

}  // namespace autoware::multi_object_tracker
