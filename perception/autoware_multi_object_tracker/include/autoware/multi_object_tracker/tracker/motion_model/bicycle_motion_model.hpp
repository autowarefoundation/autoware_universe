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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MOTION_MODEL__BICYCLE_MOTION_MODEL_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MOTION_MODEL__BICYCLE_MOTION_MODEL_HPP_

#include "autoware/multi_object_tracker/object_model/object_model.hpp"
#include "autoware/multi_object_tracker/tracker/motion_model/motion_model_base.hpp"

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace autoware::multi_object_tracker
{

class BicycleMotionModel : public MotionModel<6>
{
private:
  // attributes
  rclcpp::Logger logger_;

  // motion parameters: process noise and motion limits
  struct MotionParams
  {
    double q_stddev_acc_long = 3.43;         // [m/s^2] uncertain longitudinal acceleration, 0.35G
    double q_stddev_acc_lat = 1.47;          // [m/s^2] uncertain longitudinal acceleration, 0.15G
    double q_cov_acc_long = 11.8;            // [m/s^2] uncertain longitudinal acceleration, 0.35G
    double q_cov_acc_lat = 2.16;             // [m/s^2] uncertain lateral acceleration, 0.15G
    double q_stddev_yaw_rate_min = 0.02618;  // [rad/s] uncertain yaw change rate, 1.5deg/s
    double q_stddev_yaw_rate_max = 0.2618;   // [rad/s] uncertain yaw change rate, 15deg/s
    double q_cov_slip_rate_min =
      2.7416e-5;  // [rad^2/s^2] uncertain slip angle change rate, 0.3 deg/s
    double q_cov_slip_rate_max = 0.03046;  // [rad^2/s^2] uncertain slip angle change rate, 10 deg/s
    double q_max_slip_angle = 0.5236;      // [rad] max slip angle, 30deg
    double lf_ratio = 0.3;   // [-] ratio of the distance from the center to the front wheel
    double lr_ratio = 0.25;  // [-] ratio of the distance from the center to the rear wheel
    double lf_min = 1.0;     // [m] minimum distance from the center to the front wheel
    double lr_min = 1.0;     // [m] minimum distance from the center to the rear wheel
    double wheel_base_ratio_inv =
      1 / (lf_ratio + lr_ratio);  // [-] inverse of the sum of lf_ratio and lr_ratio
    double max_vel = 27.8;        // [m/s] maximum velocity, 100km/h
    double max_slip = 0.5236;     // [rad] maximum slip angle, 30deg
    double max_reverse_vel =
      -1.389;  // [m/s] maximum reverse velocity, -5km/h. The value is expected to be negative
    double wheel_pos_ratio =
      (lf_ratio + lr_ratio) /
      lr_ratio;  // [-] distance ratio of the wheel base over center-to-rear-wheel
    double wheel_gamma_front =
      (0.5 - lf_ratio) / (lf_ratio + lr_ratio);  // [-] protrusion from front wheel position ratio
    double wheel_gamma_rear =
      (0.5 - lr_ratio) / (lf_ratio + lr_ratio);  // [-] protrusion from rear wheel position ratio
    double q_cov_length = 0.25;                  // [m^2] covariance of the length uncertainty, 0.5m
  } motion_params_;

public:
  BicycleMotionModel();

  // bicycle model state indices
  // X1, Y1: position of the rear wheel
  // X2, Y2: position of the front wheel
  // U: longitudinal velocity
  // V: lateral velocity of the front wheel
  enum IDX { X1 = 0, Y1 = 1, X2 = 2, Y2 = 3, U = 4, V = 5 };

  bool initialize(
    const rclcpp::Time & time, const double & x, const double & y, const double & yaw,
    const std::array<double, 36> & pose_cov, const double & vel_long, const double & vel_long_cov,
    const double & vel_lat, const double & vel_lat_cov, const double & length);

  void setMotionParams(
    object_model::MotionProcessNoise process_noise, object_model::BicycleModelState bicycle_state,
    object_model::MotionProcessLimit process_limit);

  double getYawState() const;
  double getLength() const;

  bool updateStatePose(
    const double & x, const double & y, const std::array<double, 36> & pose_cov,
    const double & length);

  bool updateStatePoseHead(
    const double & x, const double & y, const double & yaw, const std::array<double, 36> & pose_cov,
    const double & length);

  bool updateStatePoseVel(
    const double & x, const double & y, const std::array<double, 36> & pose_cov, const double & yaw,
    const double & vel_long, const double & vel_lat, const std::array<double, 36> & twist_cov,
    const double & length);

  bool updateStatePoseHeadVel(
    const double & x, const double & y, const double & yaw, const std::array<double, 36> & pose_cov,
    const double & vel_long, const double & vel_lat, const std::array<double, 36> & twist_cov,
    const double & length);

  bool updateStatePoseRear(
    const double & xr, const double & yr, const std::array<double, 36> & pose_cov);

  bool updateStatePoseFront(
    const double & xf, const double & yf, const std::array<double, 36> & pose_cov);

  enum class LengthUpdateAnchor { CENTER, FRONT, REAR };
  bool updateStateLength(
    const double & new_length, const LengthUpdateAnchor anchor = LengthUpdateAnchor::CENTER);

  bool adjustPosition(const double & delta_x, const double & delta_y);

  bool limitStates();

  bool predictStateStep(const double dt, KalmanFilter & ekf) const override;

  bool getPredictedState(
    const rclcpp::Time & time, geometry_msgs::msg::Pose & pose, std::array<double, 36> & pose_cov,
    geometry_msgs::msg::Twist & twist, std::array<double, 36> & twist_cov) const override;
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MOTION_MODEL__BICYCLE_MOTION_MODEL_HPP_
