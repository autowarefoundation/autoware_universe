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
//
//
// Author: v1.0 Yukihiro Saito
//
#define EIGEN_MPL2_ONLY

#include "autoware/multi_object_tracker/tracker/model/bicycle_tracker.hpp"

#include "autoware/multi_object_tracker/object_model/shapes.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <autoware_utils/ros/msg_covariance.hpp>

#include <bits/stdc++.h>
#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace autoware::multi_object_tracker
{
BicycleTracker::BicycleTracker(const rclcpp::Time & time, const types::DynamicObject & object)
: Tracker(time, object), logger_(rclcpp::get_logger("BicycleTracker"))
{
  if (object.shape.type != autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    // set default initial size
    auto & object_extension = object_.shape.dimensions;
    object_extension.x = object_model_.init_size.length;
    object_extension.y = object_model_.init_size.width;
    object_extension.z = object_model_.init_size.height;
  }
  object_.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;

  // set maximum and minimum size
  limitObjectExtension(object_model_);

  // Set motion model parameters
  {
    const double q_stddev_acc_long = object_model_.process_noise.acc_long;
    const double q_stddev_acc_lat = object_model_.process_noise.acc_lat;
    const double q_stddev_yaw_rate_min = object_model_.process_noise.yaw_rate_min;
    const double q_stddev_yaw_rate_max = object_model_.process_noise.yaw_rate_max;
    const double q_stddev_slip_rate_min = object_model_.bicycle_state.slip_rate_stddev_min;
    const double q_stddev_slip_rate_max = object_model_.bicycle_state.slip_rate_stddev_max;
    const double q_max_slip_angle = object_model_.bicycle_state.slip_angle_max;
    const double lf_ratio = object_model_.bicycle_state.wheel_pos_ratio_front;
    const double lf_min = object_model_.bicycle_state.wheel_pos_front_min;
    const double lr_ratio = object_model_.bicycle_state.wheel_pos_ratio_rear;
    const double lr_min = object_model_.bicycle_state.wheel_pos_rear_min;
    motion_model_.setMotionParams(
      q_stddev_acc_long, q_stddev_acc_lat, q_stddev_yaw_rate_min, q_stddev_yaw_rate_max,
      q_stddev_slip_rate_min, q_stddev_slip_rate_max, q_max_slip_angle, lf_ratio, lf_min, lr_ratio,
      lr_min);
  }

  // Set motion limits
  {
    const double max_vel = object_model_.process_limit.vel_long_max;
    const double max_slip = object_model_.bicycle_state.slip_angle_max;
    motion_model_.setMotionLimits(max_vel, max_slip);  // maximum velocity and slip angle
  }

  // Set initial state
  {
    using autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
    const double x = object.pose.position.x;
    const double y = object.pose.position.y;
    const double yaw = tf2::getYaw(object.pose.orientation);

    auto pose_cov = object.pose_covariance;
    if (!object.kinematics.has_position_covariance) {
      // initial state covariance
      const auto & p0_cov_x = object_model_.initial_covariance.pos_x;
      const auto & p0_cov_y = object_model_.initial_covariance.pos_y;
      const auto & p0_cov_yaw = object_model_.initial_covariance.yaw;

      const double cos_yaw = std::cos(yaw);
      const double sin_yaw = std::sin(yaw);
      const double sin_2yaw = std::sin(2.0 * yaw);
      pose_cov[XYZRPY_COV_IDX::X_X] = p0_cov_x * cos_yaw * cos_yaw + p0_cov_y * sin_yaw * sin_yaw;
      pose_cov[XYZRPY_COV_IDX::X_Y] = 0.5 * (p0_cov_x - p0_cov_y) * sin_2yaw;
      pose_cov[XYZRPY_COV_IDX::Y_Y] = p0_cov_x * sin_yaw * sin_yaw + p0_cov_y * cos_yaw * cos_yaw;
      pose_cov[XYZRPY_COV_IDX::Y_X] = pose_cov[XYZRPY_COV_IDX::X_Y];
      pose_cov[XYZRPY_COV_IDX::YAW_YAW] = p0_cov_yaw;
    }

    double vel = 0.0;
    double vel_cov = object_model_.initial_covariance.vel_long;
    if (object.kinematics.has_twist) {
      vel = object.twist.linear.x;
    }
    if (object.kinematics.has_twist_covariance) {
      vel_cov = object.twist_covariance[XYZRPY_COV_IDX::X_X];
    }

    const double slip = 0.0;
    const double slip_cov = object_model_.bicycle_state.init_slip_angle_cov;
    const double & length = object_.shape.dimensions.x;

    // initialize motion model
    motion_model_.initialize(time, x, y, yaw, pose_cov, vel, vel_cov, slip, slip_cov, length);
  }
}

bool BicycleTracker::predict(const rclcpp::Time & time)
{
  return motion_model_.predictState(time);
}

bool BicycleTracker::measureWithPose(const types::DynamicObject & object)
{
  // get measurement yaw angle to update
  const double tracked_yaw = motion_model_.getStateElement(IDX::YAW);
  double measurement_yaw = 0.0;
  bool is_yaw_available = shapes::getMeasurementYaw(object, tracked_yaw, measurement_yaw);

  // update
  bool is_updated = false;
  {
    const double x = object.pose.position.x;
    const double y = object.pose.position.y;
    const double yaw = measurement_yaw;

    if (is_yaw_available) {
      is_updated = motion_model_.updateStatePoseHead(x, y, yaw, object.pose_covariance);
    } else {
      is_updated = motion_model_.updateStatePose(x, y, object.pose_covariance);
    }
    motion_model_.limitStates();
  }

  // position z
  constexpr double gain = 0.1;
  object_.pose.position.z = (1.0 - gain) * object_.pose.position.z + gain * object.pose.position.z;

  return is_updated;
}

bool BicycleTracker::measureWithShape(const types::DynamicObject & object)
{
  if (object.shape.type != autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    // do not update shape if the input is not a bounding box
    return false;
  }

  // check bound box size abnormality
  constexpr double size_max = 30.0;  // [m]
  constexpr double size_min = 0.1;   // [m]
  if (
    object.shape.dimensions.x > size_max || object.shape.dimensions.x < size_min ||
    object.shape.dimensions.y > size_max || object.shape.dimensions.y < size_min ||
    object.shape.dimensions.z > size_max || object.shape.dimensions.z < size_min) {
    return false;
  }

  // update object size
  constexpr double gain = 0.1;
  constexpr double gain_inv = 1.0 - gain;
  auto & object_extension = object_.shape.dimensions;
  object_extension.x = gain_inv * object_extension.x + gain * object.shape.dimensions.x;
  object_extension.y = gain_inv * object_extension.y + gain * object.shape.dimensions.y;
  object_extension.z = gain_inv * object_extension.z + gain * object.shape.dimensions.z;

  // set shape type, which is bounding box
  object_.shape.type = object.shape.type;

  // set maximum and minimum size
  limitObjectExtension(object_model_);

  // update motion model
  motion_model_.updateExtendedState(object_extension.x);

  return true;
}

bool BicycleTracker::measure(const types::DynamicObject & object, const rclcpp::Time & time)
{
  // check time gap
  const double dt = motion_model_.getDeltaTime(time);
  if (0.01 /*10msec*/ < dt) {
    RCLCPP_WARN(
      logger_,
      "BicycleTracker::measure There is a large gap between predicted time and measurement time. "
      "(%f)",
      dt);
  }

  // update object
  measureWithPose(object);
  measureWithShape(object);

  return true;
}

bool BicycleTracker::getTrackedObject(
  const rclcpp::Time & time, types::DynamicObject & object) const
{
  object = object_;

  // predict from motion model
  auto & pose = object.pose;
  auto & pose_cov = object.pose_covariance;
  auto & twist = object.twist;
  auto & twist_cov = object.twist_covariance;
  if (!motion_model_.getPredictedState(time, pose, pose_cov, twist, twist_cov)) {
    RCLCPP_WARN(logger_, "BicycleTracker::getTrackedObject: Failed to get predicted state.");
    return false;
  }

  // set shape
  const auto origin_yaw = tf2::getYaw(object_.pose.orientation);
  const auto ekf_pose_yaw = tf2::getYaw(pose.orientation);
  object.shape.footprint =
    autoware_utils::rotate_polygon(object.shape.footprint, origin_yaw - ekf_pose_yaw);

  return true;
}

}  // namespace autoware::multi_object_tracker
