// Copyright 2026 TIER IV, Inc.
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

#include "autoware/multi_object_tracker/tracker/model/static_tracker.hpp"

#include "autoware/multi_object_tracker/object_model/shapes.hpp"

#include <autoware_utils_geometry/msg/covariance.hpp>

#include <autoware_perception_msgs/msg/shape.hpp>

#include <cmath>

namespace autoware::multi_object_tracker
{

StaticTracker::StaticTracker(const rclcpp::Time & time, const types::DynamicObject & object)
: Tracker(time, object), logger_(rclcpp::get_logger("StaticTracker"))
{
  tracker_type_ = TrackerType::STATIC;

  // Set motion model parameters
  constexpr double q_stddev_x = 0.5;  // [m/s]
  constexpr double q_stddev_y = q_stddev_x;
  motion_model_.setMotionParams(q_stddev_x, q_stddev_y);

  // Set initial state
  using autoware_utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;
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

  motion_model_.initialize(time, object.pose.position.x, object.pose.position.y, pose_cov);
}

bool StaticTracker::predict(const rclcpp::Time & time)
{
  return motion_model_.predictState(time);
}

bool StaticTracker::measureWithPose(const types::DynamicObject & object)
{
  const double x = object.pose.position.x;
  const double y = object.pose.position.y;

  return motion_model_.updateStatePose(x, y, object.pose_covariance);
}

bool StaticTracker::measure(
  const types::DynamicObject & object, const rclcpp::Time & /*time*/,
  const types::InputChannel & /*channel_info*/)
{
  object_.shape = object.shape;
  object_.pose = object.pose;
  object_.area = types::getArea(object.shape);

  measureWithPose(object);

  return true;
}

bool StaticTracker::getTrackedObject(
  const rclcpp::Time & time, types::DynamicObject & object, const bool to_publish) const
{
  auto time_object = time;

  if (to_publish) {
    const auto last_measurement_time = getLatestMeasurementTime();
    time_object = time.seconds() > last_measurement_time.seconds() ? last_measurement_time : time;
  }

  object = object_;
  object.time = time;
  object.kinematics.is_stationary = true;

  if (!motion_model_.getPredictedState(
        time_object, object.pose, object.pose_covariance, object.twist, object.twist_covariance)) {
    RCLCPP_WARN(logger_, "StaticTracker::getTrackedObject: Failed to get predicted state.");
    return false;
  }

  if (to_publish && object.shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
    types::DynamicObject converted;
    if (shapes::convertConvexHullToBoundingBox(object, converted, ego_pos_)) {
      object = converted;
    }
  }

  return true;
}

}  // namespace autoware::multi_object_tracker
