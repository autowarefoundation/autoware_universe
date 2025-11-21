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

#define EIGEN_MPL2_ONLY

#include "autoware/multi_object_tracker/tracker/motion_model/bicycle_motion_model.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <autoware_utils_geometry/msg/covariance.hpp>
#include <autoware_utils_math/normalization.hpp>
#include <autoware_utils_math/unit_conversion.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <algorithm>

namespace autoware::multi_object_tracker
{

// cspell: ignore CTRV
// Bicycle CTRV motion model
// CTRV : Constant Turn Rate and constant Velocity
using autoware_utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;

BicycleMotionModel::BicycleMotionModel() : logger_(rclcpp::get_logger("BicycleMotionModel"))
{
  // set prediction parameters
  constexpr double dt_max = 0.11;  // [s] maximum time interval for prediction
  setMaxDeltaTime(dt_max);
}

void BicycleMotionModel::setMotionParams(
  object_model::MotionProcessNoise process_noise, object_model::BicycleModelState bicycle_state,
  object_model::MotionProcessLimit process_limit)
{
  // set process noise covariance parameters
  motion_params_.q_stddev_acc_long = process_noise.acc_long;
  motion_params_.q_stddev_acc_lat = process_noise.acc_lat;
  motion_params_.q_cov_acc_long = process_noise.acc_long * process_noise.acc_long;
  motion_params_.q_cov_acc_lat = process_noise.acc_lat * process_noise.acc_lat;
  motion_params_.q_stddev_yaw_rate_min = process_noise.yaw_rate_min;
  motion_params_.q_stddev_yaw_rate_max = process_noise.yaw_rate_max;
  motion_params_.q_cov_slip_rate_min =
    bicycle_state.slip_rate_stddev_min * bicycle_state.slip_rate_stddev_min;
  motion_params_.q_cov_slip_rate_max =
    bicycle_state.slip_rate_stddev_max * bicycle_state.slip_rate_stddev_max;
  motion_params_.q_max_slip_angle = bicycle_state.slip_angle_max;

  // set wheel position parameters
  motion_params_.lf_min = bicycle_state.wheel_pos_front_min;
  motion_params_.lr_min = bicycle_state.wheel_pos_rear_min;
  motion_params_.lf_ratio = bicycle_state.wheel_pos_ratio_front;
  motion_params_.lr_ratio = bicycle_state.wheel_pos_ratio_rear;
  motion_params_.wheel_base_ratio_inv =
    1.0 / (bicycle_state.wheel_pos_ratio_front + bicycle_state.wheel_pos_ratio_rear);

  motion_params_.wheel_pos_ratio =
    (motion_params_.lf_ratio + motion_params_.lr_ratio) / motion_params_.lr_ratio;
  motion_params_.wheel_gamma_front =
    (0.5 - motion_params_.lf_ratio) / (motion_params_.lf_ratio + motion_params_.lr_ratio);
  motion_params_.wheel_gamma_rear =
    (0.5 - motion_params_.lr_ratio) / (motion_params_.lf_ratio + motion_params_.lr_ratio);

  // set bicycle model parameters
  motion_params_.q_cov_length = bicycle_state.length_uncertainty * bicycle_state.length_uncertainty;

  motion_params_.max_vel = process_limit.vel_long_max;
}

bool BicycleMotionModel::initialize(
  const rclcpp::Time & time, const double & x, const double & y, const double & yaw,
  const std::array<double, 36> & pose_cov, const double & vel_long, const double & vel_long_cov,
  const double & vel_lat, const double & vel_lat_cov, const double & length)
{
  const double lr = length * motion_params_.lr_ratio;
  const double lf = length * motion_params_.lf_ratio;
  const double x1 = x - lr * std::cos(yaw);
  const double y1 = y - lr * std::sin(yaw);
  const double x2 = x + lf * std::cos(yaw);
  const double y2 = y + lf * std::sin(yaw);

  // initialize state vector X
  StateVec X;
  X << x1, y1, x2, y2, vel_long, vel_lat * motion_params_.wheel_pos_ratio;

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
  P(IDX::U, IDX::U) = vel_long_cov;
  P(IDX::V, IDX::V) = vel_lat_cov * motion_params_.wheel_pos_ratio;

  return MotionModel::initialize(time, X, P);
}

double BicycleMotionModel::getYawState() const
{
  // get yaw angle from the state
  return std::atan2(
    getStateElement(IDX::Y2) - getStateElement(IDX::Y1),
    getStateElement(IDX::X2) - getStateElement(IDX::X1));
}

double BicycleMotionModel::getLength() const
{
  // get length of the vehicle from the state
  const double wheel_base = std::hypot(
    getStateElement(IDX::X2) - getStateElement(IDX::X1),
    getStateElement(IDX::Y2) - getStateElement(IDX::Y1));
  return wheel_base / (motion_params_.lf_ratio + motion_params_.lr_ratio);
}

bool BicycleMotionModel::updateStatePose(
  const double & x, const double & y, const std::array<double, 36> & pose_cov,
  const double & length)
{
  // yaw angle is not provided, so we use the current yaw state
  const double yaw = getYawState();
  return updateStatePoseHead(x, y, yaw, pose_cov, length);
}

bool BicycleMotionModel::updateStatePoseHead(
  const double & x, const double & y, const double & yaw, const std::array<double, 36> & pose_cov,
  const double & length)
{
  // check if the state is initialized
  if (!checkInitialized()) return false;

  // convert the state to the bicycle model state
  const double lr = length * motion_params_.lr_ratio;
  const double lf = length * motion_params_.lf_ratio;
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double x1 = x - lr * cos_yaw;
  const double y1 = y - lr * sin_yaw;
  const double x2 = x + lf * cos_yaw;
  const double y2 = y + lf * sin_yaw;

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

  return ekf_.update(Y, C, R);
}

bool BicycleMotionModel::updateStatePoseVel(
  const double & x, const double & y, const std::array<double, 36> & pose_cov, const double & yaw,
  const double & vel_long, const double & vel_lat, const std::array<double, 36> & twist_cov,
  const double & length)
{
  // check if the state is initialized
  if (!checkInitialized()) return false;

  // using given yaw to fix the velocity direction
  // input yaw is velocity direction
  const double ground_vel_x = vel_long * std::cos(yaw) - vel_lat * std::sin(yaw);
  const double ground_vel_y = vel_long * std::sin(yaw) + vel_lat * std::cos(yaw);
  const double vel_angle = std::atan2(ground_vel_y, ground_vel_x);
  const double vel_long_in = std::hypot(ground_vel_x, ground_vel_y);
  constexpr double vel_lat_in = 0.0;  // lateral velocity is not reliable in this case

  return updateStatePoseHeadVel(
    x, y, vel_angle, pose_cov, vel_long_in, vel_lat_in, twist_cov, length);
}

bool BicycleMotionModel::updateStatePoseHeadVel(
  const double & x, const double & y, const double & yaw, const std::array<double, 36> & pose_cov,
  const double & vel_long, const double & vel_lat, const std::array<double, 36> & twist_cov,
  const double & length)
{
  // check if the state is initialized
  if (!checkInitialized()) return false;

  // convert the state to the bicycle model state
  const double lr = length * motion_params_.lr_ratio;
  const double lf = length * motion_params_.lf_ratio;
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double x1 = x - lr * cos_yaw;
  const double y1 = y - lr * sin_yaw;
  const double x2 = x + lf * cos_yaw;
  const double y2 = y + lf * sin_yaw;

  // update state
  constexpr int DIM_Y = 6;
  Eigen::Matrix<double, DIM_Y, 1> Y;
  Y << x1, y1, x2, y2, vel_long, vel_lat * motion_params_.wheel_pos_ratio;

  Eigen::Matrix<double, DIM_Y, DIM> C = Eigen::Matrix<double, DIM_Y, DIM>::Zero();
  C(0, IDX::X1) = 1.0;
  C(1, IDX::Y1) = 1.0;
  C(2, IDX::X2) = 1.0;
  C(3, IDX::Y2) = 1.0;
  C(4, IDX::U) = 1.0;
  C(5, IDX::V) = 1.0;

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
  R(5, 5) = twist_cov[XYZRPY_COV_IDX::Y_Y] * motion_params_.wheel_pos_ratio;

  return ekf_.update(Y, C, R);
}

bool BicycleMotionModel::updateStatePoseRear(
  const double & xr, const double & yr, const std::array<double, 36> & pose_cov)
{
  // check if the state is initialized
  if (!checkInitialized()) return false;

  // get the current state to extract renovate deviation
  StateVec X_t;
  StateMat P_t;
  ekf_.getX(X_t);
  ekf_.getP(P_t);

  const double yaw = getYawState();
  const double wheel_base = std::hypot(X_t(IDX::X2) - X_t(IDX::X1), X_t(IDX::Y2) - X_t(IDX::Y1));
  const double x1 = xr + wheel_base * motion_params_.wheel_gamma_rear * std::cos(yaw);
  const double y1 = yr + wheel_base * motion_params_.wheel_gamma_rear * std::sin(yaw);
  const double delta_x = x1 - X_t(IDX::X1);
  const double delta_y = y1 - X_t(IDX::Y1);

  // update state
  constexpr int DIM_Y = 4;
  Eigen::Matrix<double, DIM_Y, 1> Y;
  Y << x1, y1, X_t(IDX::X2) + delta_x, X_t(IDX::Y2) + delta_y;

  Eigen::Matrix<double, DIM_Y, DIM> C = Eigen::Matrix<double, DIM_Y, DIM>::Zero();
  C(0, IDX::X1) = 1.0;
  C(1, IDX::Y1) = 1.0;
  C(2, IDX::X2) = 1.0;
  C(3, IDX::Y2) = 1.0;

  constexpr double uncertainty_multiplier = 4.0;  // additional uncertainty for unmeasured position
  Eigen::Matrix<double, DIM_Y, DIM_Y> R = Eigen::Matrix<double, DIM_Y, DIM_Y>::Zero();
  R(0, 0) = pose_cov[XYZRPY_COV_IDX::X_X];
  R(0, 1) = pose_cov[XYZRPY_COV_IDX::X_Y];
  R(1, 0) = pose_cov[XYZRPY_COV_IDX::Y_X];
  R(1, 1) = pose_cov[XYZRPY_COV_IDX::Y_Y];
  R(2, 2) = pose_cov[XYZRPY_COV_IDX::X_X] * uncertainty_multiplier;
  R(2, 3) = pose_cov[XYZRPY_COV_IDX::X_Y] * uncertainty_multiplier;
  R(3, 2) = pose_cov[XYZRPY_COV_IDX::Y_X] * uncertainty_multiplier;
  R(3, 3) = pose_cov[XYZRPY_COV_IDX::Y_Y] * uncertainty_multiplier;

  return ekf_.update(Y, C, R);
}

bool BicycleMotionModel::updateStatePoseFront(
  const double & xf, const double & yf, const std::array<double, 36> & pose_cov)
{
  // check if the state is initialized
  if (!checkInitialized()) return false;

  // get the current state to extract renovate deviation
  StateVec X_t;
  StateMat P_t;
  ekf_.getX(X_t);
  ekf_.getP(P_t);

  const double yaw = getYawState();
  const double wheel_base = std::hypot(X_t(IDX::X2) - X_t(IDX::X1), X_t(IDX::Y2) - X_t(IDX::Y1));
  const double x2 = xf - wheel_base * motion_params_.wheel_gamma_front * std::cos(yaw);
  const double y2 = yf - wheel_base * motion_params_.wheel_gamma_front * std::sin(yaw);
  const double delta_x = x2 - X_t(IDX::X2);
  const double delta_y = y2 - X_t(IDX::Y2);

  // update state
  constexpr int DIM_Y = 4;
  Eigen::Matrix<double, DIM_Y, 1> Y;
  Y << X_t(IDX::X1) + delta_x, X_t(IDX::Y1) + delta_y, x2, y2;

  Eigen::Matrix<double, DIM_Y, DIM> C = Eigen::Matrix<double, DIM_Y, DIM>::Zero();
  C(0, IDX::X1) = 1.0;
  C(1, IDX::Y1) = 1.0;
  C(2, IDX::X2) = 1.0;
  C(3, IDX::Y2) = 1.0;

  constexpr double uncertainty_multiplier = 9.0;  // additional uncertainty for unmeasured position
  Eigen::Matrix<double, DIM_Y, DIM_Y> R = Eigen::Matrix<double, DIM_Y, DIM_Y>::Zero();
  R(0, 0) = pose_cov[XYZRPY_COV_IDX::X_X] * uncertainty_multiplier;
  R(0, 1) = pose_cov[XYZRPY_COV_IDX::X_Y] * uncertainty_multiplier;
  R(1, 0) = pose_cov[XYZRPY_COV_IDX::Y_X] * uncertainty_multiplier;
  R(1, 1) = pose_cov[XYZRPY_COV_IDX::Y_Y] * uncertainty_multiplier;
  R(2, 2) = pose_cov[XYZRPY_COV_IDX::X_X];
  R(2, 3) = pose_cov[XYZRPY_COV_IDX::X_Y];
  R(3, 2) = pose_cov[XYZRPY_COV_IDX::Y_X];
  R(3, 3) = pose_cov[XYZRPY_COV_IDX::Y_Y];

  return ekf_.update(Y, C, R);
}

bool BicycleMotionModel::limitStates()
{
  StateVec X_t;
  StateMat P_t;
  ekf_.getX(X_t);
  ekf_.getP(P_t);

  // maximum reverse velocity
  if (motion_params_.max_reverse_vel < 0 && X_t(IDX::U) < motion_params_.max_reverse_vel) {
    // rotate the object orientation by 180 degrees
    // replace X1 and Y1 with X2 and Y2
    const double x_center =
      (X_t(IDX::X1) * motion_params_.lf_ratio + X_t(IDX::X2) * motion_params_.lr_ratio) *
      motion_params_.wheel_base_ratio_inv;
    const double y_center =
      (X_t(IDX::Y1) * motion_params_.lf_ratio + X_t(IDX::Y2) * motion_params_.lr_ratio) *
      motion_params_.wheel_base_ratio_inv;
    const double x1_rel = X_t(IDX::X1) - x_center;
    const double y1_rel = X_t(IDX::Y1) - y_center;
    const double x2_rel = X_t(IDX::X2) - x_center;
    const double y2_rel = X_t(IDX::Y2) - y_center;

    X_t(IDX::X1) = x_center + x2_rel;
    X_t(IDX::Y1) = y_center + y2_rel;
    X_t(IDX::X2) = x_center + x1_rel;
    X_t(IDX::Y2) = y_center + y1_rel;

    // reverse the velocity
    X_t(IDX::U) = -X_t(IDX::U);
    // rotation velocity does not change

    // replace covariance
    // Swap rows and columns
    P_t.row(IDX::X1).swap(P_t.row(IDX::X2));
    P_t.row(IDX::Y1).swap(P_t.row(IDX::Y2));
    P_t.col(IDX::X1).swap(P_t.col(IDX::X2));
    P_t.col(IDX::Y1).swap(P_t.col(IDX::Y2));
  }
  // maximum velocity
  if (!(-motion_params_.max_vel <= X_t(IDX::U) && X_t(IDX::U) <= motion_params_.max_vel)) {
    X_t(IDX::U) = X_t(IDX::U) < 0 ? -motion_params_.max_vel : motion_params_.max_vel;
  }

  // maximum lateral velocity by lateral acceleration limitations
  // a_max = vel_long^2 * vel_lat / wheel_base
  // vel_lat_limit = a_max * wheel_base / vel_long^2
  {
    const double wheel_base = std::hypot(X_t(IDX::X2) - X_t(IDX::X1), X_t(IDX::Y2) - X_t(IDX::Y1));
    constexpr double acc_lat_max = 9.81 * 0.5;  // [m/s^2] maximum lateral acceleration (0.5g);
    const double vel_lat_limit = acc_lat_max * wheel_base / (X_t(IDX::U) * X_t(IDX::U));
    const double vel_lat_limit_adjusted = vel_lat_limit * motion_params_.wheel_pos_ratio;
    if (std::abs(X_t(IDX::V)) > vel_lat_limit_adjusted) {
      // limit lateral velocity
      X_t(IDX::V) = X_t(IDX::V) < 0 ? -vel_lat_limit_adjusted : vel_lat_limit_adjusted;
    }
  }

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
  /*  Motion model: static bicycle model (constant turn rate, constant velocity)
   *
   * wheel_base = sqrt((x2 - x1)^2 + (y2 - y1)^2)
   * yaw = atan2(y2 - y1, x2 - x1)
   * sin_theta = (y2 - y1) / wheel_base
   * cos_theta = (x2 - x1) / wheel_base
   * x1_{k+1}   = x1_k + vel_long_k*(x2_k - x1_k)/wheel_base * dt
   * y1_{k+1}   = y1_k + vel_long_k*(y2_k - y1_k)/wheel_base * dt
   * x2_{k+1}   = x2_k + vel_long_k*(x2_k - x1_k)/wheel_base * dt - vel_lat_k*(y2_k -
   *              y1_k)/wheel_base * dt
   * y2_{k+1}   = y2_k + vel_long_k*(y2_k - y1_k)/wheel_base * dt +
   *              vel_lat_k*(x2_k - x1_k)/wheel_base * dt
   * vel_long_{k+1} = vel_long_k
   * vel_lat_{k+1} = vel_lat_k * exp(-dt / 2.0)  // lateral velocity decays exponentially
   *                                                with a half-life of 2 seconds
   */

  /*  Jacobian Matrix
   *
   * A_x1 = [1 - vel_long_k / wheel_base * dt, 0, vel_long_k / wheel_base * dt, 0,
             (x2_k - x1_k)/wheel_base * dt, 0]
   * A_y1 = [0, 1 - vel_long_k / wheel_base * dt, 0, vel_long_k / wheel_base * dt,
             (y2_k - y1_k)/wheel_base * dt, 0]
   * A_x2 = [-vel_long_k / wheel_base * dt, vel_lat_k / wheel_base * dt,
             1 + vel_long_k / wheel_base * dt, - vel_lat_k / wheel_base * dt,
             (x2_k - x1_k)/wheel_base * dt, - (y2_k - y1_k)/wheel_base * dt]
   * A_y2 = [-vel_lat_k / wheel_base * dt, -vel_long_k / wheel_base * dt,
             vel_lat_k / wheel_base * dt, 1 + vel_long_k / wheel_base * dt,
             (y2_k - y1_k)/wheel_base * dt, (x2_k - x1_k)/wheel_base * dt]
   * A_vx = [0, 0, 0, 0, 1, 0]
   * A_vy = [0, 0, 0, 0, 0, exp(-dt / 2.0)]
   */

  // Current state vector X_t
  StateVec X_t;
  ekf.getX(X_t);

  const double & x1 = X_t(IDX::X1);
  const double & y1 = X_t(IDX::Y1);
  const double & x2 = X_t(IDX::X2);
  const double & y2 = X_t(IDX::Y2);
  const double & vel_long = X_t(IDX::U);
  const double & vel_lat = X_t(IDX::V);

  const double yaw = std::atan2(y2 - y1, x2 - x1);
  const double wheel_base = std::hypot(x2 - x1, y2 - y1);
  const double sin_yaw = (y2 - y1) / wheel_base;
  const double cos_yaw = (x2 - x1) / wheel_base;
  const double wheel_base_inv_dt = dt / wheel_base;
  const double sin_yaw_dt = sin_yaw * dt;
  const double cos_yaw_dt = cos_yaw * dt;

  // Predict state vector X t+1
  StateVec X_next_t;
  X_next_t(IDX::X1) = x1 + vel_long * cos_yaw_dt;
  X_next_t(IDX::Y1) = y1 + vel_long * sin_yaw_dt;
  X_next_t(IDX::X2) = x2 + vel_long * cos_yaw_dt - vel_lat * sin_yaw_dt;
  X_next_t(IDX::Y2) = y2 + vel_long * sin_yaw_dt + vel_lat * cos_yaw_dt;
  X_next_t(IDX::U) = vel_long;  // velocity does not change
  // Apply exponential decay to slip angle over time, with a half-life
  constexpr double half_life = 1.5;                    // [s] half-life of the exponential decay
  constexpr double gamma = 0.69314718056 / half_life;  // natural logarithm of 2 / half-life
  const double decay_rate = std::exp(-gamma * dt);     // decay rate for half-life
  X_next_t(IDX::V) = vel_lat * decay_rate;             // lateral velocity decays exponentially

  // State transition matrix A
  ProcessMat A;
  A.setZero();

  A(IDX::X1, IDX::X1) = 1.0 - vel_long * wheel_base_inv_dt;
  A(IDX::X1, IDX::X2) = vel_long * wheel_base_inv_dt;
  A(IDX::X1, IDX::U) = cos_yaw_dt;

  A(IDX::Y1, IDX::Y1) = 1.0 - vel_long * wheel_base_inv_dt;
  A(IDX::Y1, IDX::Y2) = vel_long * wheel_base_inv_dt;
  A(IDX::Y1, IDX::U) = sin_yaw_dt;

  A(IDX::X2, IDX::X1) = -vel_long * wheel_base_inv_dt;
  A(IDX::X2, IDX::Y1) = vel_lat * wheel_base_inv_dt;
  A(IDX::X2, IDX::X2) = 1.0 + vel_long * wheel_base_inv_dt;
  A(IDX::X2, IDX::Y2) = -vel_lat * wheel_base_inv_dt;
  A(IDX::X2, IDX::U) = cos_yaw_dt;
  A(IDX::X2, IDX::V) = -sin_yaw_dt;

  A(IDX::Y2, IDX::X1) = -vel_lat * wheel_base_inv_dt;
  A(IDX::Y2, IDX::Y1) = -vel_long * wheel_base_inv_dt;
  A(IDX::Y2, IDX::X2) = vel_lat * wheel_base_inv_dt;
  A(IDX::Y2, IDX::Y2) = 1.0 + vel_long * wheel_base_inv_dt;
  A(IDX::Y2, IDX::U) = sin_yaw_dt;
  A(IDX::Y2, IDX::V) = cos_yaw_dt;

  // velocity does not change
  A(IDX::U, IDX::U) = 1.0;
  A(IDX::V, IDX::V) = decay_rate;

  // Process noise covariance Q
  double q_stddev_yaw_rate = motion_params_.q_stddev_yaw_rate_min;
  if (vel_long > 0.01) {
    /* uncertainty of the yaw rate is limited by the following:
     *  - centripetal acceleration a_lat : d(yaw)/dt = w = a_lat/vel_long
     *  - or maximum slip angle slip_max : w = vel_long*sin(slip_max)/wheel_base
     */
    q_stddev_yaw_rate = std::min(
      motion_params_.q_stddev_acc_lat / vel_long,
      vel_long * std::sin(motion_params_.q_max_slip_angle) / wheel_base);  // [rad/s]
    q_stddev_yaw_rate = std::clamp(
      q_stddev_yaw_rate, motion_params_.q_stddev_yaw_rate_min,
      motion_params_.q_stddev_yaw_rate_max);
  }
  const double q_stddev_head = q_stddev_yaw_rate * wheel_base * dt;  // yaw uncertainty

  const double dt2 = dt * dt;
  const double dt4 = dt2 * dt2;

  const double q_cov_long = 0.25 * motion_params_.q_cov_acc_long * dt4;
  const double q_cov_lat = 0.25 * motion_params_.q_cov_acc_lat * dt4;
  const double q_cov_long2 = q_cov_long + motion_params_.q_cov_length * dt2;
  const double q_cov_lat2 = q_cov_lat + q_stddev_head * q_stddev_head;

  StateMat Q;
  Q.setZero();
  const double sin_2yaw = std::sin(2.0 * yaw);
  Q(IDX::X1, IDX::X1) = (q_cov_long * cos_yaw * cos_yaw + q_cov_lat * sin_yaw * sin_yaw);
  Q(IDX::X1, IDX::Y1) = (0.5f * (q_cov_long - q_cov_lat) * sin_2yaw);
  Q(IDX::Y1, IDX::X1) = Q(IDX::X1, IDX::Y1);
  Q(IDX::Y1, IDX::Y1) = (q_cov_long * sin_yaw * sin_yaw + q_cov_lat * cos_yaw * cos_yaw);

  Q(IDX::X2, IDX::X2) = (q_cov_long2 * cos_yaw * cos_yaw + q_cov_lat2 * sin_yaw * sin_yaw);
  Q(IDX::X2, IDX::Y2) = (0.5f * (q_cov_long2 - q_cov_lat2) * sin_2yaw);
  Q(IDX::Y2, IDX::X2) = Q(IDX::X2, IDX::Y2);
  Q(IDX::Y2, IDX::Y2) = (q_cov_long2 * sin_yaw * sin_yaw + q_cov_lat2 * cos_yaw * cos_yaw);

  // covariance between X1 and X2, Y1 and Y2, shares the same covariance of rear axle
  constexpr double cross_coefficient =
    0.1;  // [m^2] coefficient for covariance between front and rear axle
  Q(IDX::X1, IDX::X2) = Q(IDX::X1, IDX::X1) * cross_coefficient;
  Q(IDX::X2, IDX::X1) = Q(IDX::X1, IDX::X1) * cross_coefficient;
  Q(IDX::Y1, IDX::Y2) = Q(IDX::Y1, IDX::Y1) * cross_coefficient;
  Q(IDX::Y2, IDX::Y1) = Q(IDX::Y1, IDX::Y1) * cross_coefficient;
  Q(IDX::X1, IDX::Y2) = Q(IDX::X1, IDX::Y1) * cross_coefficient;
  Q(IDX::Y2, IDX::X1) = Q(IDX::X1, IDX::Y1) * cross_coefficient;
  Q(IDX::Y1, IDX::X2) = Q(IDX::X1, IDX::Y1) * cross_coefficient;
  Q(IDX::X2, IDX::Y1) = Q(IDX::X1, IDX::Y1) * cross_coefficient;

  // covariance of velocity
  const double q_cov_vel_long = motion_params_.q_cov_acc_long * dt2;
  const double q_cov_vel_lat = motion_params_.q_cov_acc_lat * dt2;
  Q(IDX::U, IDX::U) = q_cov_vel_long;
  Q(IDX::V, IDX::V) = q_cov_vel_lat;

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
  const double wheel_base_inv = 1.0 / wheel_base;
  const double wheel_base_inv_sq = wheel_base_inv * wheel_base_inv;

  // set position
  pose.position.x = (X(IDX::X1) * motion_params_.lf_ratio + X(IDX::X2) * motion_params_.lr_ratio) *
                    motion_params_.wheel_base_ratio_inv;
  pose.position.y = (X(IDX::Y1) * motion_params_.lf_ratio + X(IDX::Y2) * motion_params_.lr_ratio) *
                    motion_params_.wheel_base_ratio_inv;
  // do not change z

  // set orientation
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();

  // set twist
  twist.linear.x = X(IDX::U);
  twist.linear.y = X(IDX::V) / motion_params_.wheel_pos_ratio;  // scaled by wheel position ratio
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = X(IDX::V) * wheel_base_inv;

  constexpr double default_cov = 0.1 * 0.1;
  // set pose covariance
  pose_cov[XYZRPY_COV_IDX::X_X] = P(IDX::X1, IDX::X1);
  pose_cov[XYZRPY_COV_IDX::X_Y] = P(IDX::X1, IDX::Y1);
  pose_cov[XYZRPY_COV_IDX::Y_X] = P(IDX::Y1, IDX::X1);
  pose_cov[XYZRPY_COV_IDX::Y_Y] = P(IDX::Y1, IDX::Y1);
  pose_cov[XYZRPY_COV_IDX::YAW_YAW] = P(IDX::X2, IDX::X2) * cos(yaw) * wheel_base_inv_sq +
                                      P(IDX::Y2, IDX::Y2) * sin(yaw) * wheel_base_inv_sq;
  pose_cov[XYZRPY_COV_IDX::Z_Z] = default_cov;
  pose_cov[XYZRPY_COV_IDX::ROLL_ROLL] = default_cov;
  pose_cov[XYZRPY_COV_IDX::PITCH_PITCH] = default_cov;

  // set twist covariance
  twist_cov[XYZRPY_COV_IDX::X_X] = P(IDX::U, IDX::U);
  twist_cov[XYZRPY_COV_IDX::Y_Y] = P(IDX::V, IDX::V);
  twist_cov[XYZRPY_COV_IDX::YAW_YAW] =
    P(IDX::V, IDX::V) * wheel_base_inv_sq /
    (motion_params_.wheel_pos_ratio * motion_params_.wheel_pos_ratio);
  twist_cov[XYZRPY_COV_IDX::Z_Z] = default_cov;
  twist_cov[XYZRPY_COV_IDX::ROLL_ROLL] = default_cov;
  twist_cov[XYZRPY_COV_IDX::PITCH_PITCH] = default_cov;

  return true;
}

}  // namespace autoware::multi_object_tracker
