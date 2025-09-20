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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MOTION_MODEL__MOTION_MODEL_BASE_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MOTION_MODEL__MOTION_MODEL_BASE_HPP_

#include "autoware/multi_object_tracker/kalman_filter_template.hpp"

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <geometry_msgs/msg/twist.hpp>

namespace autoware::multi_object_tracker
{
template <int StateSize, int MeasurementSize = 2>
class MotionModel
{
private:
  bool is_initialized_{false};
  double dt_max_{0.11};  // [s] maximum time interval for prediction

protected:
  rclcpp::Time last_update_time_;
  KalmanFilterTemplate<StateSize, MeasurementSize> ekf_;
  // Cache for prediction to avoid repeated allocations
  mutable KalmanFilterTemplate<StateSize, MeasurementSize> tmp_ekf_for_no_update_;
  // Dimensionality constants (accessible to derived classes)
  static constexpr int DIM = StateSize;

public:
  // Add these type aliases for convenience
  using KalmanFilter = KalmanFilterTemplate<StateSize, MeasurementSize>;
  using StateVec = typename KalmanFilter::StateVec;
  using StateMat = typename KalmanFilter::StateMat;
  using MeasVec = typename KalmanFilter::MeasVec;
  using MeasMat = typename KalmanFilter::MeasMat;
  using ProcessMat = typename KalmanFilter::ProcessMat;
  using MeasModelMat = typename KalmanFilter::MeasModelMat;

  MotionModel() : last_update_time_(rclcpp::Time(0, 0)) {}
  virtual ~MotionModel() = default;

  bool checkInitialized() const noexcept { return is_initialized_; }
  double getDeltaTime(const rclcpp::Time & time) const noexcept
  {
    return (time - last_update_time_).seconds();
  }
  void setMaxDeltaTime(const double dt_max) noexcept { dt_max_ = dt_max; }
  double getStateElement(unsigned int idx) const noexcept { return ekf_.getXelement(idx); }
  void getStateVector(StateVec & X) const noexcept { ekf_.getX(X); }

  bool initialize(const rclcpp::Time & time, const StateVec & X, const StateMat & P);
  bool predictState(const rclcpp::Time & time);
  bool getPredictedState(const rclcpp::Time & time, StateVec & X, StateMat & P) const;

  virtual bool predictStateStep(
    const double dt, KalmanFilterTemplate<StateSize, MeasurementSize> & ekf) const = 0;
  virtual bool getPredictedState(
    const rclcpp::Time & time, geometry_msgs::msg::Pose & pose, std::array<double, 36> & pose_cov,
    geometry_msgs::msg::Twist & twist, std::array<double, 36> & twist_cov) const = 0;
};

// === Implementation ===

template <int StateSize, int MeasurementSize>
bool MotionModel<StateSize, MeasurementSize>::initialize(
  const rclcpp::Time & time, const StateVec & X, const StateMat & P)
{
  // initialize Kalman filter
  ekf_.init(X, P);

  // set last_update_time_
  last_update_time_ = time;

  // set initialized flag
  is_initialized_ = true;

  return true;
}

template <int StateSize, int MeasurementSize>
bool MotionModel<StateSize, MeasurementSize>::predictState(const rclcpp::Time & time)
{
  // check if the state is initialized
  if (!checkInitialized()) return false;

  const double dt = getDeltaTime(time);
  if (dt < 0.0) {
    return false;
  }
  if (dt < 1e-6 /*1usec*/) {
    return true;
  }

  // Optimized: Use integer arithmetic for better performance
  const uint32_t repeat = static_cast<uint32_t>(std::floor(dt / dt_max_)) + 1;
  const double dt_ = dt / static_cast<double>(repeat);

  // Optimized: Pre-allocate time increment to avoid repeated calculations
  const rclcpp::Duration dt_duration = rclcpp::Duration::from_seconds(dt_);

  for (uint32_t i = 0; i < repeat; ++i) {
    if (!predictStateStep(dt_, ekf_)) return false;
    last_update_time_ += dt_duration;
  }
  // reset the last_update_time_ to the prediction time
  last_update_time_ = time;
  return true;
}

template <int StateSize, int MeasurementSize>
bool MotionModel<StateSize, MeasurementSize>::getPredictedState(
  const rclcpp::Time & time, StateVec & X, StateMat & P) const
{
  // check if the state is initialized
  if (!checkInitialized()) return false;

  const double dt = getDeltaTime(time);
  if (dt < 1e-6 /*1usec*/) {
    // no prediction, return the current state
    ekf_.getX(X);
    ekf_.getP(P);
    return true;
  }

  // copy the predicted state and covariance
  tmp_ekf_for_no_update_ = ekf_;
  // multi-step prediction
  // if dt is too large, shorten dt and repeat prediction
  const uint32_t repeat = static_cast<uint32_t>(std::floor(dt / dt_max_)) + 1;
  const double dt_ = dt / static_cast<double>(repeat);
  for (uint32_t i = 0; i < repeat; ++i) {
    if (!predictStateStep(dt_, tmp_ekf_for_no_update_)) return false;
  }
  tmp_ekf_for_no_update_.getX(X);
  tmp_ekf_for_no_update_.getP(P);
  return true;
}

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MOTION_MODEL__MOTION_MODEL_BASE_HPP_
