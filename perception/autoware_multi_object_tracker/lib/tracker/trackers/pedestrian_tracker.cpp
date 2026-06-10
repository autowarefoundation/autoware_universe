// Copyright 2020 TIER IV, Inc.
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

#include "autoware/multi_object_tracker/tracker/trackers/pedestrian_tracker.hpp"

#include <tf2/utils.hpp>

#include <algorithm>

namespace autoware::multi_object_tracker
{

PedestrianTracker::PedestrianTracker(const rclcpp::Time & time, const types::DynamicObject & object)
: Tracker(time, object),
  logger_(rclcpp::get_logger("PedestrianTracker")),
  shape_model_(object_model_)
{
  tracker_type_ = TrackerType::PEDESTRIAN;

  // Initialize shape manager: handles POLYGON/CYLINDER/BBOX normalization and clamping
  shape_model_.init(object);

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
    motion_model_.setMotionLimits(max_vel, max_turn_rate);
  }

  // Set initial state
  {
    using autoware_utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;
    const double x = object.pose.position.x;
    const double y = object.pose.position.y;
    const double yaw = tf2::getYaw(object.pose.orientation);

    auto pose_cov = object.pose_covariance;
    if (!object.kinematics.has_position_covariance) {
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

    motion_model_.initialize(time, x, y, yaw, pose_cov, vel, vel_cov, wz, wz_cov);
    motion_model_.setZ(object.pose.position.z);
    committed_yaw_ = yaw;
  }
}

bool PedestrianTracker::predict(const rclcpp::Time & time)
{
  return motion_model_.predictState(time);
}

bool PedestrianTracker::updateKinematics(const types::DynamicObject & object)
{
  bool is_updated = false;
  {
    const double x = object.pose.position.x;
    const double y = object.pose.position.y;

    is_updated = motion_model_.updateStatePose(x, y, object.pose_covariance);
    motion_model_.limitStates();
  }

  // Low-pass filter on z position.
  constexpr double gain = 0.1;
  motion_model_.updateZ(object.pose.position.z, gain);

  return is_updated;
}

bool PedestrianTracker::measure(
  const types::DynamicObject & object, const rclcpp::Time & time,
  const types::InputChannel & channel_info)
{
  const double dt = motion_model_.getDeltaTime(time);
  if (0.01 /*10msec*/ < dt) {
    RCLCPP_WARN(
      logger_,
      "PedestrianTracker::measure There is a large gap between predicted time and measurement "
      "time. (%f)",
      dt);
  }

  updateKinematics(object);

  // Use the committed tracker heading (1-frame-stale, refreshed by commitState) for POLYGON-branch
  // projection — committed_yaw_ preserves the legacy snapshot semantics.
  const double tracker_yaw = committed_yaw_;
  shape_model_.update(object, channel_info.trust_extension, tracker_yaw);

  removeCache();
  return true;
}

bool PedestrianTracker::getMotionState(
  const rclcpp::Time & time, geometry_msgs::msg::Pose & pose, std::array<double, 36> & pose_cov,
  geometry_msgs::msg::Twist & twist, std::array<double, 36> & twist_cov) const
{
  return motion_model_.getPredictedState(time, pose, pose_cov, twist, twist_cov);
}

bool PedestrianTracker::getTrackedObject(
  const rclcpp::Time & time, types::DynamicObject & object, const bool to_publish) const
{
  if (!getCachedObject(time, object)) {
    if (!populateKinematicObject(time, time, object)) {
      RCLCPP_WARN(logger_, "PedestrianTracker::getTrackedObject: Failed to get predicted state.");
      return false;
    }
    updateCache(object, time);
  }

  // Export shape from extend manager (type selection: CYLINDER vs BOUNDING_BOX)
  assembleShapeTo(object, to_publish);

  if (to_publish) {
    using autoware_utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;
    auto & twist = object.twist;
    constexpr double vel_cov_buffer = 0.7;
    constexpr double vel_too_low_ignore = 0.35;
    const double vel_long = std::abs(twist.linear.x);
    if (vel_long > vel_too_low_ignore) {
      const double vel_limit =
        std::max(std::sqrt(object.twist_covariance[XYZRPY_COV_IDX::X_X]) - vel_cov_buffer, 0.0);

      if (vel_long < vel_limit) {
        twist.linear.x = twist.linear.x > 0 ? vel_too_low_ignore : -vel_too_low_ignore;
      } else {
        twist.linear.x =
          twist.linear.x > 0 ? twist.linear.x - vel_limit : twist.linear.x + vel_limit;
      }
    }
  }

  return true;
}

}  // namespace autoware::multi_object_tracker
