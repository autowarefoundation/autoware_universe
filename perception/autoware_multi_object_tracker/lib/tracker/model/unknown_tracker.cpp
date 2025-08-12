// Copyright 2020 Tier IV, Inc.
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

#include "autoware/multi_object_tracker/tracker/model/unknown_tracker.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <autoware_utils/ros/msg_covariance.hpp>

#include <bits/stdc++.h>
#include <tf2/utils.h>

#include <cmath>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace autoware::multi_object_tracker
{

UnknownTracker::UnknownTracker(
  const rclcpp::Time & time, const types::DynamicObject & object,
  const bool enable_velocity_estimation, const bool enable_motion_output)
: Tracker(time, object),
  logger_(rclcpp::get_logger("UnknownTracker")),
  enable_velocity_estimation_(enable_velocity_estimation),
  enable_motion_output_(enable_motion_output)
{
  tracker_type_ = TrackerType::UNKNOWN;

  if (enable_velocity_estimation_) {
    // Set motion model parameters
    {
      constexpr double q_stddev_x = 1.5;         // [m/s]
      constexpr double q_stddev_y = 1.5;         // [m/s]
      constexpr double q_stddev_vx = 9.8 * 0.5;  // [m/(s*s)]
      constexpr double q_stddev_vy = 9.8 * 0.5;  // [m/(s*s)]
      motion_model_.setMotionParams(q_stddev_x, q_stddev_y, q_stddev_vx, q_stddev_vy);
    }

    // Set motion limits
    motion_model_.setMotionLimits(
      autoware_utils::kmph2mps(60), /* [m/s] maximum velocity, x */
      autoware_utils::kmph2mps(60)  /* [m/s] maximum velocity, y */
    );

    // Set initial state
    {
      using autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
      const double x = object.pose.position.x;
      const double y = object.pose.position.y;
      auto pose_cov = object.pose_covariance;
      auto twist_cov = object.twist_covariance;
      const double yaw = tf2::getYaw(object.pose.orientation);

      double vx = 0.0;
      double vy = 0.0;
      if (object.kinematics.has_twist) {
        const double & vel_x = object.twist.linear.x;
        const double & vel_y = object.twist.linear.y;
        vx = std::cos(yaw) * vel_x - std::sin(yaw) * vel_y;
        vy = std::sin(yaw) * vel_x + std::cos(yaw) * vel_y;
      }

      if (!object.kinematics.has_position_covariance) {
        constexpr double p0_stddev_x = 1.0;  // [m]
        constexpr double p0_stddev_y = 1.0;  // [m]

        const double p0_cov_x = std::pow(p0_stddev_x, 2.0);
        const double p0_cov_y = std::pow(p0_stddev_y, 2.0);

        const double cos_yaw = std::cos(yaw);
        const double sin_yaw = std::sin(yaw);
        const double sin_2yaw = std::sin(2.0 * yaw);
        pose_cov[XYZRPY_COV_IDX::X_X] = p0_cov_x * cos_yaw * cos_yaw + p0_cov_y * sin_yaw * sin_yaw;
        pose_cov[XYZRPY_COV_IDX::X_Y] = 0.5 * (p0_cov_x - p0_cov_y) * sin_2yaw;
        pose_cov[XYZRPY_COV_IDX::Y_Y] = p0_cov_x * sin_yaw * sin_yaw + p0_cov_y * cos_yaw * cos_yaw;
        pose_cov[XYZRPY_COV_IDX::Y_X] = pose_cov[XYZRPY_COV_IDX::X_Y];
      }

      if (!object.kinematics.has_twist_covariance) {
        constexpr double p0_stddev_vx = autoware_utils::kmph2mps(10);  // [m/s]
        constexpr double p0_stddev_vy = autoware_utils::kmph2mps(10);  // [m/s]
        const double p0_cov_vx = std::pow(p0_stddev_vx, 2.0);
        const double p0_cov_vy = std::pow(p0_stddev_vy, 2.0);
        twist_cov[XYZRPY_COV_IDX::X_X] = p0_cov_vx;
        twist_cov[XYZRPY_COV_IDX::X_Y] = 0.0;
        twist_cov[XYZRPY_COV_IDX::Y_X] = 0.0;
        twist_cov[XYZRPY_COV_IDX::Y_Y] = p0_cov_vy;
      }

      // rotate twist covariance matrix, since it is in the vehicle coordinate system
      Eigen::MatrixXd twist_cov_rotate(2, 2);
      twist_cov_rotate(0, 0) = twist_cov[XYZRPY_COV_IDX::X_X];
      twist_cov_rotate(0, 1) = twist_cov[XYZRPY_COV_IDX::X_Y];
      twist_cov_rotate(1, 0) = twist_cov[XYZRPY_COV_IDX::Y_X];
      twist_cov_rotate(1, 1) = twist_cov[XYZRPY_COV_IDX::Y_Y];
      Eigen::MatrixXd R_yaw = Eigen::Rotation2Dd(-yaw).toRotationMatrix();
      Eigen::MatrixXd twist_cov_rotated = R_yaw * twist_cov_rotate * R_yaw.transpose();
      twist_cov[XYZRPY_COV_IDX::X_X] = twist_cov_rotated(0, 0);
      twist_cov[XYZRPY_COV_IDX::X_Y] = twist_cov_rotated(0, 1);
      twist_cov[XYZRPY_COV_IDX::Y_X] = twist_cov_rotated(1, 0);
      twist_cov[XYZRPY_COV_IDX::Y_Y] = twist_cov_rotated(1, 1);

      // initialize motion model
      motion_model_.initialize(time, x, y, pose_cov, vx, vy, twist_cov);
    }
  } else {
    // Set motion model parameters
    constexpr double q_stddev_x = 5.0;  // [m/s]
    constexpr double q_stddev_y = q_stddev_x;
    static_motion_model_.setMotionParams(q_stddev_x, q_stddev_y);

    // Set initial state
    using autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
    auto pose_cov = object.pose_covariance;
    if (!object.kinematics.has_position_covariance) {
      constexpr double p0_stddev_x = 1.5;  // [m]
      constexpr double p0_stddev_y = 1.5;  // [m]

      const double p0_cov_x = p0_stddev_x * p0_stddev_x;
      const double p0_cov_y = p0_stddev_y * p0_stddev_y;

      pose_cov[XYZRPY_COV_IDX::X_X] = p0_cov_x;
      pose_cov[XYZRPY_COV_IDX::Y_Y] = p0_cov_y;
      pose_cov[XYZRPY_COV_IDX::X_Y] = 0.0;
      pose_cov[XYZRPY_COV_IDX::Y_X] = 0.0;
    }
    static_motion_model_.initialize(time, object.pose.position.x, object.pose.position.y, pose_cov);
  }
}

bool UnknownTracker::predict(const rclcpp::Time & time)
{
  if (enable_velocity_estimation_) {
    return motion_model_.predictState(time);
  } else {
    return static_motion_model_.predictState(time);
  }

  return true;
}

bool UnknownTracker::measureWithPose(const types::DynamicObject & object)
{
  bool is_updated = true;

  if (enable_velocity_estimation_) {
    // update motion model
    const double x = object.pose.position.x;
    const double y = object.pose.position.y;

    is_updated = motion_model_.updateStatePose(x, y, object.pose_covariance);
    motion_model_.limitStates();

  } else {
    // update static motion model
    const double x = object.pose.position.x;
    const double y = object.pose.position.y;

    is_updated = static_motion_model_.updateStatePose(x, y, object.pose_covariance);
  }

  // position z
  constexpr double gain = 0.1;
  object_.pose.position.z = (1.0 - gain) * object_.pose.position.z + gain * object.pose.position.z;

  return is_updated;
}

bool UnknownTracker::measure(
  const types::DynamicObject & object, const rclcpp::Time & time,
  const types::InputChannel & /*channel_info*/)
{
  // update object shape
  object_.shape = object.shape;
  object_.pose = object.pose;
  object_.area = types::getArea(object.shape);
  last_pose_ = object.pose;

  if (enable_velocity_estimation_) {
    // check time gap
    const double dt = motion_model_.getDeltaTime(time);
    if (0.01 /*10msec*/ < dt) {
      RCLCPP_WARN(
        logger_,
        "UnknownTracker::measure There is a large gap between predicted time and measurement time. "
        "(%f)",
        dt);
    }
  }

  // update object
  measureWithPose(object);

  return true;
}

bool UnknownTracker::getTrackedObject(
  const rclcpp::Time & time, types::DynamicObject & object, const bool to_publish) const
{
  auto time_object = time;

  if (to_publish) {
    // if it is for publish, limit the time to the last updated time
    const auto last_measurement_time = getLatestMeasurementTime();
    time_object = time.seconds() > last_measurement_time.seconds() ? last_measurement_time : time;
  }
  // else, allow extrapolation

  // get the object
  object = object_;

  if (enable_velocity_estimation_) {
    // predict from motion model
    if (!motion_model_.getPredictedState(
          time_object, object.pose, object.pose_covariance, object.twist,
          object.twist_covariance)) {
      RCLCPP_WARN(logger_, "UnknownTracker::getTrackedObject: Failed to get predicted state.");
      return false;
    }
  } else {
    // predict from static motion model
    if (!static_motion_model_.getPredictedState(
          time_object, object.pose, object.pose_covariance, object.twist,
          object.twist_covariance)) {
      RCLCPP_WARN(logger_, "UnknownTracker::getTrackedObject: Failed to get predicted state.");
      return false;
    }
  }

  if (to_publish) {
    // back to the input pose to match with the polygon shape
    object.pose = last_pose_;
    if (!enable_motion_output_) {
      object.twist.linear.x = 0.0;
      object.twist.linear.y = 0.0;
    }
  }

  return true;
}

}  // namespace autoware::multi_object_tracker
