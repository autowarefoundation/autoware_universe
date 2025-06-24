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

#include "autoware/multi_object_tracker/tracker/model/pedestrian_tracker.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
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
PedestrianTracker::PedestrianTracker(const rclcpp::Time & time, const types::DynamicObject & object)
: Tracker(time, object), logger_(rclcpp::get_logger("PedestrianTracker"))
{
  if (object.shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
    // set default initial size
    auto & object_extension = object_.shape.dimensions;
    object_extension.x = object_model_.init_size.length;
    object_extension.y = object_model_.init_size.width;
    object_extension.z = object_model_.init_size.height;
  }
  // set maximum and minimum size
  limitObjectExtension(object_model_);

  // Set motion model parameters
  {
    const double q_stddev_x = object_model_.process_noise.vel_long;
    const double q_stddev_y = object_model_.process_noise.vel_lat;
    const double q_stddev_yaw = object_model_.process_noise.yaw_rate;
    const double q_stddev_vx = object_model_.process_noise.acc_long;
    const double q_stddev_wz = object_model_.process_noise.acc_turn;
    motion_model_.setMotionParams(q_stddev_x, q_stddev_y, q_stddev_yaw, q_stddev_vx, q_stddev_wz);
  }

  // Set motion limits
  {
    const double max_vel = object_model_.process_limit.vel_long_max;
    const double max_turn_rate = object_model_.process_limit.yaw_rate_max;
    motion_model_.setMotionLimits(max_vel, max_turn_rate);  // maximum velocity and slip angle
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
    double wz = 0.0;
    if (object.kinematics.has_twist) {
      vel = object.twist.linear.x;
      wz = object.twist.angular.z;
    }

    double vel_cov = object_model_.initial_covariance.vel_long;
    double wz_cov = object_model_.initial_covariance.yaw_rate;
    if (object.kinematics.has_twist_covariance) {
      vel_cov = object.twist_covariance[XYZRPY_COV_IDX::X_X];
      wz_cov = object.twist_covariance[XYZRPY_COV_IDX::YAW_YAW];
    }

    // initialize motion model
    motion_model_.initialize(time, x, y, yaw, pose_cov, vel, vel_cov, wz, wz_cov);
  }
}

bool PedestrianTracker::predict(const rclcpp::Time & time)
{
  return motion_model_.predictState(time);
}

bool PedestrianTracker::measureWithPose(const types::DynamicObject & object)
{
  // update motion model
  bool is_updated = false;
  {
    const double x = object.pose.position.x;
    const double y = object.pose.position.y;

    is_updated = motion_model_.updateStatePose(x, y, object.pose_covariance);
    motion_model_.limitStates();
  }

  // position z
  constexpr double gain = 0.1;
  object_.pose.position.z = (1.0 - gain) * object_.pose.position.z + gain * object.pose.position.z;

  // remove cached object
  removeCache();

  return is_updated;
}

bool PedestrianTracker::measureWithShape(const types::DynamicObject & object)
{
  if (object.shape.type != autoware_perception_msgs::msg::Shape::POLYGON) {
    constexpr double size_max = 30.0;  // [m]
    constexpr double size_min = 0.1;   // [m]
    if (
      object.shape.dimensions.x > size_max || object.shape.dimensions.x < size_min ||
      object.shape.dimensions.y > size_max || object.shape.dimensions.y < size_min ||
      object.shape.dimensions.z > size_max || object.shape.dimensions.z < size_min) {
      return false;
    }
    constexpr double gain = 0.5;
    constexpr double gain_inv = 1.0 - gain;
    auto & object_extension = object_.shape.dimensions;
    object_extension.x = gain_inv * object_extension.x + gain * object.shape.dimensions.x;
    object_extension.y = gain_inv * object_extension.y + gain * object.shape.dimensions.y;
    object_extension.z = gain_inv * object_extension.z + gain * object.shape.dimensions.z;

    // update shape type, bounding box or cylinder
    object_.shape.type = object.shape.type;
    object_.area = types::getArea(object.shape);

    // set maximum and minimum size
    limitObjectExtension(object_model_);
  } else {
    // do not update polygon shape
    return false;
  }

  // update shape type
  object_.shape.type = object.shape.type;

  return true;
}

bool PedestrianTracker::measure(
  const types::DynamicObject & object, const rclcpp::Time & time,
  const types::InputChannel & channel_info)
{
  // check time gap
  const double dt = motion_model_.getDeltaTime(time);
  if (0.01 /*10msec*/ < dt) {
    RCLCPP_WARN(
      logger_,
      "PedestrianTracker::measure There is a large gap between predicted time and measurement "
      "time. (%f)",
      dt);
  }

  // update object
  measureWithPose(object);
  if (channel_info.trust_extension) {
    measureWithShape(object);
  }

  return true;
}

bool PedestrianTracker::getTrackedObject(
  const rclcpp::Time & time, types::DynamicObject & object,
  [[maybe_unused]] const bool to_publish) const
{
  // try to return cached object
  if (getCachedObject(time, object)) {
    return true;
  }

  object = object_;
  object.time = time;

  // predict from motion model
  auto & pose = object.pose;
  auto & pose_cov = object.pose_covariance;
  auto & twist = object.twist;
  auto & twist_cov = object.twist_covariance;
  if (!motion_model_.getPredictedState(time, pose, pose_cov, twist, twist_cov)) {
    RCLCPP_WARN(logger_, "PedestrianTracker::getTrackedObject: Failed to get predicted state.");
    return false;
  }

  // cache object
  updateCache(object, time);

  return true;
}

}  // namespace autoware::multi_object_tracker
