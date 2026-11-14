// Copyright 2024 TIER IV, Inc.
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

#include "autoware/multi_object_tracker/tracker/motion_model/cv_motion_model.hpp"

#include "autoware/multi_object_tracker/tracker/motion_model/motion_model_math.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <autoware_utils_geometry/msg/covariance.hpp>
#include <tf2/utils.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace autoware::multi_object_tracker
{
// cspell: ignore CV
// Constant Velocity (CV) motion model
using autoware_utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;

CVMotionModel::CVMotionModel() : logger_(rclcpp::get_logger("CVMotionModel"))
{
  // set prediction parameters
  constexpr double dt_max = 0.11;  // [s] maximum time interval for prediction
  setMaxDeltaTime(dt_max);
}

void CVMotionModel::setMotionParams(
  const double & q_stddev_x, const double & q_stddev_y, const double & q_stddev_vx,
  const double & q_stddev_vy)
{
  // set process noise covariance parameters
  motion_params_.q_cov_x = q_stddev_x * q_stddev_x;
  motion_params_.q_cov_y = q_stddev_y * q_stddev_y;
  motion_params_.q_cov_vx = q_stddev_vx * q_stddev_vx;
  motion_params_.q_cov_vy = q_stddev_vy * q_stddev_vy;
}

void CVMotionModel::setMotionLimits(const double & max_vx, const double & max_vy)
{
  // set motion limitations
  motion_params_.max_vx = max_vx;
  motion_params_.max_vy = max_vy;
}

bool CVMotionModel::initialize(
  const rclcpp::Time & time, const double & x, const double & y,
  const std::array<double, 36> & pose_cov, const double & vx, const double & vy,
  const std::array<double, 36> & twist_cov)
{
  // initialize state vector X
  StateVec X;
  X << x, y, vx, vy;

  // initialize covariance matrix P
  StateMat P = StateMat::Zero();
  P(IDX::X, IDX::X) = pose_cov[XYZRPY_COV_IDX::X_X];
  P(IDX::Y, IDX::Y) = pose_cov[XYZRPY_COV_IDX::Y_Y];
  P(IDX::VX, IDX::VX) = twist_cov[XYZRPY_COV_IDX::X_X];
  P(IDX::VY, IDX::VY) = twist_cov[XYZRPY_COV_IDX::Y_Y];

  return MotionModel::initialize(time, X, P);
}

bool CVMotionModel::updateStatePose(
  const double & x, const double & y, const std::array<double, 36> & pose_cov)
{
  // check if the state is initialized
  if (!checkInitialized()) return false;

  // update state, without velocity
  constexpr int DIM_Y = 2;

  // update state
  Eigen::Matrix<double, DIM_Y, 1> Y;
  Y << x, y;

  Eigen::Matrix<double, DIM_Y, DIM> C = Eigen::Matrix<double, DIM_Y, DIM>::Zero();
  C(0, IDX::X) = 1.0;
  C(1, IDX::Y) = 1.0;

  Eigen::Matrix<double, DIM_Y, DIM_Y> R = Eigen::Matrix<double, DIM_Y, DIM_Y>::Zero();
  R(0, 0) = pose_cov[XYZRPY_COV_IDX::X_X];
  R(0, 1) = pose_cov[XYZRPY_COV_IDX::X_Y];
  R(1, 0) = pose_cov[XYZRPY_COV_IDX::Y_X];
  R(1, 1) = pose_cov[XYZRPY_COV_IDX::Y_Y];

  return ekf_.update(Y, C, R);
}

bool CVMotionModel::updateStatePoseVel(
  const double & x, const double & y, const std::array<double, 36> & pose_cov, const double & vx,
  const double & vy, const std::array<double, 36> & twist_cov)
{
  // check if the state is initialized
  if (!checkInitialized()) return false;

  // update state with velocity
  constexpr int DIM_Y = 4;

  // update state
  Eigen::Matrix<double, DIM_Y, 1> Y;
  Y << x, y, vx, vy;

  Eigen::Matrix<double, DIM_Y, DIM> C = Eigen::Matrix<double, DIM_Y, DIM>::Zero();
  C(0, IDX::X) = 1.0;
  C(1, IDX::Y) = 1.0;
  C(2, IDX::VX) = 1.0;
  C(3, IDX::VY) = 1.0;

  Eigen::Matrix<double, DIM_Y, DIM_Y> R = Eigen::Matrix<double, DIM_Y, DIM_Y>::Zero();
  R(0, 0) = pose_cov[XYZRPY_COV_IDX::X_X];
  R(0, 1) = pose_cov[XYZRPY_COV_IDX::X_Y];
  R(1, 0) = pose_cov[XYZRPY_COV_IDX::Y_X];
  R(1, 1) = pose_cov[XYZRPY_COV_IDX::Y_Y];
  R(2, 2) = twist_cov[XYZRPY_COV_IDX::X_X];
  R(2, 3) = twist_cov[XYZRPY_COV_IDX::X_Y];
  R(3, 2) = twist_cov[XYZRPY_COV_IDX::Y_X];
  R(3, 3) = twist_cov[XYZRPY_COV_IDX::Y_Y];

  return ekf_.update(Y, C, R);
}

bool CVMotionModel::limitStates()
{
  StateVec X_t;
  StateMat P_t;
  ekf_.getX(X_t);
  ekf_.getP(P_t);
  X_t(IDX::VX) = motion_model_math::clampSymmetric(X_t(IDX::VX), motion_params_.max_vx);
  X_t(IDX::VY) = motion_model_math::clampSymmetric(X_t(IDX::VY), motion_params_.max_vy);
  ekf_.init(X_t, P_t);

  return true;
}

bool CVMotionModel::adjustPosition(const double & x, const double & y)
{
  // check if the state is initialized
  if (!checkInitialized()) return false;

  // adjust position
  StateVec X_t;
  StateMat P_t;
  ekf_.getX(X_t);
  ekf_.getP(P_t);
  X_t(IDX::X) += x;
  X_t(IDX::Y) += y;
  ekf_.init(X_t, P_t);

  return true;
}

bool CVMotionModel::predictStateStep(const double dt, KalmanFilter & ekf) const
{
  /*  Motion model: Constant velocity model
   *
   * x_{k+1}   = x_k + vx_k * dt
   * y_{k+1}   = y_k + vx_k * dt
   * vx_{k+1}  = vx_k
   * vy_{k+1}  = vy_k
   *
   */

  /*  Jacobian Matrix
   *
   * A = [ 1, 0, dt,  0]
   *     [ 0, 1,  0, dt]
   *     [ 0, 0,  1,  0]
   *     [ 0, 0,  0,  1]
   */

  // Current state vector X t
  StateVec X_t;
  ekf.getX(X_t);

  // Predict state vector X t+1
  StateVec X_next_t;  // predicted state
  X_next_t(IDX::X) = X_t(IDX::X) + X_t(IDX::VX) * dt;
  X_next_t(IDX::Y) = X_t(IDX::Y) + X_t(IDX::VY) * dt;
  X_next_t(IDX::VX) = X_t(IDX::VX);
  X_next_t(IDX::VY) = X_t(IDX::VY);

  // State transition matrix A
  StateMat A = StateMat::Identity();
  A(IDX::X, IDX::VX) = dt;
  A(IDX::Y, IDX::VY) = dt;

  // Process noise covariance Q
  StateMat Q = StateMat::Zero();
  Q(IDX::X, IDX::X) = motion_params_.q_cov_x * dt * dt;
  Q(IDX::X, IDX::Y) = 0.0;
  Q(IDX::Y, IDX::Y) = motion_params_.q_cov_y * dt * dt;
  Q(IDX::Y, IDX::X) = 0.0;
  Q(IDX::VX, IDX::VX) = motion_params_.q_cov_vx * dt * dt;
  Q(IDX::VY, IDX::VY) = motion_params_.q_cov_vy * dt * dt;

  // control-input model B and control-input u are not used
  // Eigen::MatrixXd B = Eigen::MatrixXd::Zero(DIM, DIM);
  // Eigen::MatrixXd u = Eigen::MatrixXd::Zero(DIM, 1);

  // predict state
  return ekf.predict(X_next_t, A, Q);
}

bool CVMotionModel::getPredictedState(
  const rclcpp::Time & time, geometry_msgs::msg::Pose & pose, std::array<double, 36> & pose_cov,
  geometry_msgs::msg::Twist & twist, std::array<double, 36> & twist_cov) const
{
  // get predicted state
  StateVec X;
  StateMat P;
  if (!MotionModel::getPredictedState(time, X, P)) {
    return false;
  }

  // set position
  pose.position.x = X(IDX::X);
  pose.position.y = X(IDX::Y);
  pose.position.z = z_;
  pose.orientation = orientation_;

  // get yaw for twist rotation from own orientation member
  const double yaw = tf2::getYaw(orientation_);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  // set twist (rotate world-frame velocity into body frame)
  twist.linear.x = X(IDX::VX) * cos_yaw + X(IDX::VY) * sin_yaw;
  twist.linear.y = -X(IDX::VX) * sin_yaw + X(IDX::VY) * cos_yaw;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;

  // set pose covariance
  pose_cov[XYZRPY_COV_IDX::X_X] = P(IDX::X, IDX::X);
  pose_cov[XYZRPY_COV_IDX::X_Y] = P(IDX::X, IDX::Y);
  pose_cov[XYZRPY_COV_IDX::Y_X] = P(IDX::Y, IDX::X);
  pose_cov[XYZRPY_COV_IDX::Y_Y] = P(IDX::Y, IDX::Y);
  pose_cov[XYZRPY_COV_IDX::Z_Z] = motion_model_math::kUnobservedCov;
  pose_cov[XYZRPY_COV_IDX::ROLL_ROLL] = motion_model_math::kUnobservedCov;
  pose_cov[XYZRPY_COV_IDX::PITCH_PITCH] = motion_model_math::kUnobservedCov;
  pose_cov[XYZRPY_COV_IDX::YAW_YAW] = motion_model_math::kUnobservedCov;

  // set twist covariance (rotate velocity covariance into body frame)
  Eigen::Matrix2d twist_cov_mat;
  twist_cov_mat(0, 0) = P(IDX::VX, IDX::VX);
  twist_cov_mat(0, 1) = P(IDX::VX, IDX::VY);
  twist_cov_mat(1, 0) = P(IDX::VY, IDX::VX);
  twist_cov_mat(1, 1) = P(IDX::VY, IDX::VY);
  const Eigen::Matrix2d twist_cov_rotated =
    motion_model_math::rotateCov2D(twist_cov_mat, cos_yaw, sin_yaw);
  twist_cov[XYZRPY_COV_IDX::X_X] = twist_cov_rotated(0, 0);
  twist_cov[XYZRPY_COV_IDX::X_Y] = twist_cov_rotated(0, 1);
  twist_cov[XYZRPY_COV_IDX::Y_X] = twist_cov_rotated(1, 0);
  twist_cov[XYZRPY_COV_IDX::Y_Y] = twist_cov_rotated(1, 1);
  twist_cov[XYZRPY_COV_IDX::Z_Z] = motion_model_math::kUnobservedCov;
  twist_cov[XYZRPY_COV_IDX::ROLL_ROLL] = motion_model_math::kUnobservedCov;
  twist_cov[XYZRPY_COV_IDX::PITCH_PITCH] = motion_model_math::kUnobservedCov;
  twist_cov[XYZRPY_COV_IDX::YAW_YAW] = motion_model_math::kUnobservedCov;

  return true;
}

}  // namespace autoware::multi_object_tracker
