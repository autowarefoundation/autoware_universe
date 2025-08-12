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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MOTION_MODEL__BICYCLE_MOTION_MODEL_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MOTION_MODEL__BICYCLE_MOTION_MODEL_HPP_

#include "autoware/multi_object_tracker/tracker/motion_model/motion_model_base.hpp"

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
    double lf_ratio = 0.3;     // [-] ratio of the distance from the center to the front wheel
    double lr_ratio = 0.25;    // [-] ratio of the distance from the center to the rear wheel
    double lf_min = 1.0;       // [m] minimum distance from the center to the front wheel
    double lr_min = 1.0;       // [m] minimum distance from the center to the rear wheel
    double max_vel = 27.8;     // [m/s] maximum velocity, 100km/h
    double max_slip = 0.5236;  // [rad] maximum slip angle, 30deg
    double max_reverse_vel =
      -1.389;  // [m/s] maximum reverse velocity, -5km/h. The value is expected to be negative
  } motion_params_;

public:
  BicycleMotionModel();

  // bicycle model state indices
  // X1, Y1: position of the rear wheel
  // X2, Y2: position of the front wheel
  // VX: longitudinal velocity
  // VY: lateral velocity of the front wheel
  enum IDX { X1 = 0, Y1 = 1, X2 = 2, Y2 = 3, VX = 4, VY = 5 };

  bool initialize(
    const rclcpp::Time & time, const double & x, const double & y, const double & yaw,
    const std::array<double, 36> & pose_cov, const double & vel, const double & vel_cov,
    const double & slip, const double & slip_cov, const double & length);

  void setMotionParams(
    const double & q_stddev_acc_long, const double & q_stddev_acc_lat,
    const double & q_stddev_yaw_rate_min, const double & q_stddev_yaw_rate_max,
    const double & q_stddev_slip_rate_min, const double & q_stddev_slip_rate_max,
    const double & q_max_slip_angle, const double & lf_ratio, const double & lf_min,
    const double & lr_ratio, const double & lr_min);

  void setMotionLimits(const double & max_vel, const double & max_slip);

  double getYawState() const;
  double getLength() const;

  bool updateStatePose(const double & x, const double & y, const std::array<double, 36> & pose_cov, const double & length);

  bool updateStatePoseHead(
    const double & x, const double & y, const double & yaw,
    const std::array<double, 36> & pose_cov, const double & length);

  bool updateStatePoseVel(
    const double & x, const double & y, const std::array<double, 36> & pose_cov, const double & vel_x,
    const double & vel_y, const std::array<double, 36> & twist_cov, const double & length);

  bool updateStatePoseHeadVel(
    const double & x, const double & y, const double & yaw, const std::array<double, 36> & pose_cov,
    const double & vel_x, const double & vel_y, const std::array<double, 36> & twist_cov, const double & length);

  bool adjustPosition(const double & delta_x, const double & delta_y);

  bool limitStates();

  bool predictStateStep(const double dt, KalmanFilter & ekf) const override;

  bool getPredictedState(
    const rclcpp::Time & time, geometry_msgs::msg::Pose & pose, std::array<double, 36> & pose_cov,
    geometry_msgs::msg::Twist & twist, std::array<double, 36> & twist_cov) const override;

};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MOTION_MODEL__BICYCLE_MOTION_MODEL_HPP_
