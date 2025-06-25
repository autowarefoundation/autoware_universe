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

#include "autoware/multi_object_tracker/tracker/motion_model/static_motion_model.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <autoware_utils/ros/msg_covariance.hpp>

#include <tf2/utils.h>

namespace autoware::multi_object_tracker
{
using autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;

StaticMotionModel::StaticMotionModel() : logger_(rclcpp::get_logger("StaticMotionModel"))
{
  // set prediction parameters
  constexpr double dt_max = 0.11;  // [s] maximum time interval for prediction
  setMaxDeltaTime(dt_max);
}

void StaticMotionModel::setMotionParams(const double & q_stddev_x, const double & q_stddev_y)
{
  // set process noise covariance parameters
  motion_params_.q_cov_x = q_stddev_x * q_stddev_x;
  motion_params_.q_cov_y = q_stddev_y * q_stddev_y;
}

bool StaticMotionModel::initialize(
  const rclcpp::Time & time, const double & x, const double & y,
  const std::array<double, 36> & pose_cov)
{
  // initialize state vector X
  StateVec X(DIM, 1);
  X << x, y;

  // initialize covariance matrix P
  StateMat P = StateMat::Zero(DIM, DIM);
  P(IDX::X, IDX::X) = pose_cov[XYZRPY_COV_IDX::X_X];
  P(IDX::Y, IDX::Y) = pose_cov[XYZRPY_COV_IDX::Y_Y];

  return MotionModel::initialize(time, X, P);
}

bool StaticMotionModel::updateStatePose(
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

bool StaticMotionModel::adjustPosition(const double & x, const double & y)
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

bool StaticMotionModel::predictStateStep(const double dt, KalmanFilter & ekf) const
{
  /*  Motion model: static
   *
   * x_{k+1}   = x_k
   * y_{k+1}   = y_k
   *
   */

  /*  Jacobian Matrix
   *
   * A = [ 1, 0]
   *     [ 0, 1]
   */

  // Current state vector X t
  StateVec X_t;
  ekf.getX(X_t);

  // Predict state vector X t+1
  StateVec X_next_t;  // predicted state
  X_next_t(IDX::X) = X_t(IDX::X);
  X_next_t(IDX::Y) = X_t(IDX::Y);

  // State transition matrix A
  StateMat A = StateMat::Identity();

  // Process noise covariance Q
  StateMat Q = StateMat::Zero();
  Q(IDX::X, IDX::X) = motion_params_.q_cov_x * dt * dt;
  Q(IDX::X, IDX::Y) = 0.0;
  Q(IDX::Y, IDX::Y) = motion_params_.q_cov_y * dt * dt;
  Q(IDX::Y, IDX::X) = 0.0;

  // predict state
  return ekf.predict(X_next_t, A, Q);
}

bool StaticMotionModel::getPredictedState(
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
  // do not change z

  // set twist
  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;

  // set pose covariance
  constexpr double zz_cov = 0.1 * 0.1;   // TODO(yukkysaito) Currently tentative
  constexpr double rr_cov = 0.1 * 0.1;   // TODO(yukkysaito) Currently tentative
  constexpr double pp_cov = 0.1 * 0.1;   // TODO(yukkysaito) Currently tentative
  constexpr double yaw_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  pose_cov[XYZRPY_COV_IDX::X_X] = P(IDX::X, IDX::X);
  pose_cov[XYZRPY_COV_IDX::X_Y] = P(IDX::X, IDX::Y);
  pose_cov[XYZRPY_COV_IDX::Y_X] = P(IDX::Y, IDX::X);
  pose_cov[XYZRPY_COV_IDX::Y_Y] = P(IDX::Y, IDX::Y);
  pose_cov[XYZRPY_COV_IDX::Z_Z] = zz_cov;
  pose_cov[XYZRPY_COV_IDX::ROLL_ROLL] = rr_cov;
  pose_cov[XYZRPY_COV_IDX::PITCH_PITCH] = pp_cov;
  pose_cov[XYZRPY_COV_IDX::YAW_YAW] = yaw_cov;

  // set twist covariance
  constexpr double cov_const = 0.1 * 0.1;  // [m^2]
  twist_cov[XYZRPY_COV_IDX::X_X] = cov_const;
  twist_cov[XYZRPY_COV_IDX::X_Y] = 0.0;
  twist_cov[XYZRPY_COV_IDX::Y_X] = 0.0;
  twist_cov[XYZRPY_COV_IDX::Y_Y] = cov_const;
  twist_cov[XYZRPY_COV_IDX::Z_Z] = 0.0;
  twist_cov[XYZRPY_COV_IDX::ROLL_ROLL] = cov_const;
  twist_cov[XYZRPY_COV_IDX::PITCH_PITCH] = cov_const;
  twist_cov[XYZRPY_COV_IDX::YAW_YAW] = cov_const;

  return true;
}

}  // namespace autoware::multi_object_tracker
