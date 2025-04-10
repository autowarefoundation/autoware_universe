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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MOTION_MODEL__CTRV_MOTION_MODEL_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MOTION_MODEL__CTRV_MOTION_MODEL_HPP_

#include "autoware/kalman_filter/kalman_filter.hpp"
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

// cspell: ignore CTRV
class CTRVMotionModel : public MotionModel
{
private:
  // attributes
  rclcpp::Logger logger_;

  // motion parameters: process noise and motion limits
  struct MotionParams
  {
    double q_cov_x = 0.025;          // [m^2/s^2] uncertain position in x, 0.5m/s
    double q_cov_y = 0.025;          // [m^2/s^2] uncertain position in y, 0.5m/s
    double q_cov_yaw = 0.1218;       // [rad^2/s^2] uncertain yaw angle, 20deg/s
    double q_cov_vel = 8.6436;       // [m^2/s^4] uncertain velocity, 0.3G m/s^2
    double q_cov_wz = 0.5236;        // [rad^2/s^4] uncertain yaw rate, 30deg/s^2
    double max_vel = 2.78;           // [m/s] maximum velocity
    double max_wz = 0.5236;          // [rad/s] maximum yaw rate, 30deg/s
    double max_reverse_vel = -1.38;  // [m/s] maximum reverse velocity, -5km/h
  } motion_params_;

public:
  CTRVMotionModel();

  enum IDX { X = 0, Y = 1, YAW = 2, VEL = 3, WZ = 4 };
  const char DIM = 5;

  bool initialize(
    const rclcpp::Time & time, const double & x, const double & y, const double & yaw,
    const std::array<double, 36> & pose_cov, const double & vel, const double & vel_cov,
    const double & wz, const double & wz_cov);

  void setMotionParams(
    const double & q_stddev_x, const double & q_stddev_y, const double & q_stddev_yaw,
    const double & q_stddev_vx, const double & q_stddev_wz);

  void setMotionLimits(const double & max_vel, const double & max_wz);

  bool updateStatePose(const double & x, const double & y, const std::array<double, 36> & pose_cov);

  bool updateStatePoseHead(
    const double & x, const double & y, const double & yaw,
    const std::array<double, 36> & pose_cov);

  bool updateStatePoseHeadVel(
    const double & x, const double & y, const double & yaw, const std::array<double, 36> & pose_cov,
    const double & vel, const std::array<double, 36> & twist_cov);

  bool adjustPosition(const double & x, const double & y);

  bool limitStates();

  bool predictStateStep(const double dt, KalmanFilter & ekf) const override;

  bool getPredictedState(
    const rclcpp::Time & time, geometry_msgs::msg::Pose & pose, std::array<double, 36> & pose_cov,
    geometry_msgs::msg::Twist & twist, std::array<double, 36> & twist_cov) const override;
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MOTION_MODEL__CTRV_MOTION_MODEL_HPP_
