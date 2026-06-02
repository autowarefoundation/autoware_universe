// Copyright 2021 TIER IV, Inc.
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

#include "autoware/map_based_prediction/path_generator/path_generator.hpp"

#include "autoware/map_based_prediction/path_generator/frenet.hpp"

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/interpolation/spline_interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::map_based_prediction
{
using autoware_utils::ScopedTimeTrack;

PathGenerator::PathGenerator(const double sampling_time_interval)
: sampling_time_interval_(sampling_time_interval)
{
  min_crosswalk_user_velocity_ = 0.1;
}

PathGenerator::PathGenerator(
  const double sampling_time_interval, const double min_crosswalk_user_velocity)
: sampling_time_interval_(sampling_time_interval),
  min_crosswalk_user_velocity_(min_crosswalk_user_velocity)
{
}

void PathGenerator::setTimeKeeper(std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_ptr)
{
  time_keeper_ = std::move(time_keeper_ptr);
}

PredictedPath PathGenerator::generatePathForNonVehicleObject(
  const TrackedObject & object, const double duration) const
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  return generateStraightPath(object, duration);
}

PredictedPath PathGenerator::generatePathToTargetPoint(
  const TrackedObject & object, const Eigen::Vector2d & point) const
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  PredictedPath predicted_path{};
  const double ep = 0.001;

  const auto & obj_pos = object.kinematics.pose_with_covariance.pose.position;
  const auto & obj_vel = object.kinematics.twist_with_covariance.twist.linear;

  const Eigen::Vector2d pedestrian_to_entry_point(point.x() - obj_pos.x, point.y() - obj_pos.y);
  const auto velocity = std::max(std::hypot(obj_vel.x, obj_vel.y), min_crosswalk_user_velocity_);
  const auto arrival_time = pedestrian_to_entry_point.norm() / velocity;

  const auto pedestrian_to_entry_point_normalized = pedestrian_to_entry_point.normalized();
  const auto pedestrian_to_entry_point_orientation = autoware_utils::create_quaternion_from_yaw(
    std::atan2(pedestrian_to_entry_point_normalized.y(), pedestrian_to_entry_point_normalized.x()));

  for (double dt = 0.0; dt < arrival_time + ep; dt += sampling_time_interval_) {
    geometry_msgs::msg::Pose world_frame_pose;
    world_frame_pose.position.x =
      obj_pos.x + velocity * pedestrian_to_entry_point_normalized.x() * dt;
    world_frame_pose.position.y =
      obj_pos.y + velocity * pedestrian_to_entry_point_normalized.y() * dt;
    world_frame_pose.position.z = obj_pos.z;
    world_frame_pose.orientation = pedestrian_to_entry_point_orientation;
    predicted_path.path.push_back(world_frame_pose);
    if (predicted_path.path.size() >= predicted_path.path.max_size()) {
      break;
    }
  }

  predicted_path.confidence = 1.0;
  predicted_path.time_step = rclcpp::Duration::from_seconds(sampling_time_interval_);

  return predicted_path;
}

PredictedPathWithArrivalIndex PathGenerator::generatePathForCrosswalkUser(
  const TrackedObject & object, const CrosswalkEdgePoints & reachable_crosswalk,
  const double duration) const
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  PredictedPathWithArrivalIndex predicted_path;
  const double ep = 0.001;

  const auto & obj_pos = object.kinematics.pose_with_covariance.pose.position;
  const auto & obj_vel = object.kinematics.twist_with_covariance.twist.linear;

  const Eigen::Vector2d pedestrian_to_entry_point(
    reachable_crosswalk.front_center_point.x() - obj_pos.x,
    reachable_crosswalk.front_center_point.y() - obj_pos.y);
  const Eigen::Vector2d entry_to_exit_point(
    reachable_crosswalk.back_center_point.x() - reachable_crosswalk.front_center_point.x(),
    reachable_crosswalk.back_center_point.y() - reachable_crosswalk.front_center_point.y());
  const auto velocity = std::max(std::hypot(obj_vel.x, obj_vel.y), min_crosswalk_user_velocity_);
  const auto arrival_time = pedestrian_to_entry_point.norm() / velocity;

  const auto pedestrian_to_entry_point_normalized = pedestrian_to_entry_point.normalized();
  const auto pedestrian_to_entry_point_orientation = autoware_utils::create_quaternion_from_yaw(
    std::atan2(pedestrian_to_entry_point_normalized.y(), pedestrian_to_entry_point_normalized.x()));
  const auto entry_to_exit_point_normalized = entry_to_exit_point.normalized();
  const auto entry_to_exit_point_orientation = autoware_utils::create_quaternion_from_yaw(
    std::atan2(entry_to_exit_point_normalized.y(), entry_to_exit_point_normalized.x()));

  predicted_path.arrival_index =
    static_cast<size_t>(1 + std::ceil(arrival_time / sampling_time_interval_));
  for (double dt = 0.0; dt < duration + ep; dt += sampling_time_interval_) {
    geometry_msgs::msg::Pose world_frame_pose;
    if (dt < arrival_time) {
      world_frame_pose.position.x =
        obj_pos.x + velocity * pedestrian_to_entry_point_normalized.x() * dt;
      world_frame_pose.position.y =
        obj_pos.y + velocity * pedestrian_to_entry_point_normalized.y() * dt;
      world_frame_pose.position.z = obj_pos.z;
      world_frame_pose.orientation = pedestrian_to_entry_point_orientation;
      predicted_path.path.push_back(world_frame_pose);
    } else {
      world_frame_pose.position.x =
        reachable_crosswalk.front_center_point.x() +
        velocity * entry_to_exit_point_normalized.x() * (dt - arrival_time);
      world_frame_pose.position.y =
        reachable_crosswalk.front_center_point.y() +
        velocity * entry_to_exit_point_normalized.y() * (dt - arrival_time);
      world_frame_pose.position.z = obj_pos.z;
      world_frame_pose.orientation = entry_to_exit_point_orientation;
      predicted_path.path.push_back(world_frame_pose);
    }
    if (predicted_path.path.size() >= predicted_path.path.max_size()) {
      break;
    }
  }

  predicted_path.confidence = 1.0;
  predicted_path.time_step = rclcpp::Duration::from_seconds(sampling_time_interval_);

  return predicted_path;
}

PredictedPath PathGenerator::generatePathForLowSpeedVehicle(
  const TrackedObject & object, const double duration) const
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  PredictedPath path;
  path.time_step = rclcpp::Duration::from_seconds(sampling_time_interval_);
  const double ep = 0.001;
  for (double dt = 0.0; dt < duration + ep; dt += sampling_time_interval_) {
    path.path.push_back(object.kinematics.pose_with_covariance.pose);
  }

  return path;
}

PredictedPath PathGenerator::generatePathForOffLaneVehicle(
  const TrackedObject & object, const double duration) const
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  return generateStraightPath(object, duration);
}

PredictedPath PathGenerator::generatePathForOnLaneVehicle(
  const TrackedObject & object, const PosePath & ref_path, const double duration,
  const double lateral_duration, const double path_width, const double speed_limit) const
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  if (ref_path.size() < 2) {
    return generateStraightPath(object, duration);
  }

  // if the object is moving backward, we generate a straight path
  if (object.kinematics.twist_with_covariance.twist.linear.x < 0.0) {
    return generateStraightPath(object, duration);
  }

  // get object width
  double object_width = 5.0;  // a large number
  if (object.shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    object_width = object.shape.dimensions.y;
  } else if (object.shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
    object_width = object.shape.dimensions.x;
  }
  // Calculate the backlash width, which represents the maximum distance the object can be biased
  // from the reference path
  constexpr double margin =
    0.5;  // Set a safety margin of 0.5m for the object to stay away from the edge of the lane
  double backlash_width = (path_width - object_width) / 2.0 - margin;
  backlash_width = std::max(backlash_width, 0.0);  // minimum is 0.0

  return generatePolynomialPath(
    object, ref_path, duration, lateral_duration, path_width, backlash_width, speed_limit);
}

PredictedPath PathGenerator::generateStraightPath(
  const TrackedObject & object, const double longitudinal_duration) const
{
  const auto & object_pose = object.kinematics.pose_with_covariance.pose;
  const auto & object_twist = object.kinematics.twist_with_covariance.twist;
  constexpr double ep = 0.001;
  const double duration = longitudinal_duration + ep;

  PredictedPath path;
  path.time_step = rclcpp::Duration::from_seconds(sampling_time_interval_);
  path.path.reserve(static_cast<size_t>((duration) / sampling_time_interval_));
  for (double dt = 0.0; dt < duration; dt += sampling_time_interval_) {
    const auto future_obj_pose = autoware_utils::calc_offset_pose(
      object_pose, object_twist.linear.x * dt, object_twist.linear.y * dt, 0.0);
    path.path.push_back(future_obj_pose);
  }

  return path;
}

PredictedPath PathGenerator::generatePolynomialPath(
  const TrackedObject & object, const PosePath & ref_path, const double duration,
  const double lateral_duration, const double path_width, const double backlash_width,
  const double speed_limit) const
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  // Get current Frenet Point
  const double ref_path_len = autoware::motion_utils::calcArcLength(ref_path);
  const auto current_point = getFrenetPoint(
    object, ref_path.at(0), duration, speed_limit, use_vehicle_acceleration_,
    acceleration_exponential_half_life_);

  // Step 1. Set Target Frenet Point
  // Note that we do not set position s,
  // since we don't know the target longitudinal position
  FrenetPoint terminal_point;
  terminal_point.s_vel = std::hypot(current_point.s_vel, current_point.d_vel);
  terminal_point.s_acc = 0.0;
  terminal_point.d_vel = 0.0;
  terminal_point.d_acc = 0.0;

  // if the object is behind of the reference path adjust the lateral_duration to reach the start of
  // the reference path
  double lateral_duration_adjusted = lateral_duration;
  if (current_point.s < 0.0) {
    const double distance_to_start = -current_point.s;
    const double duration_to_reach = distance_to_start / terminal_point.s_vel;
    lateral_duration_adjusted = std::max(lateral_duration, duration_to_reach);
  }

  // calculate terminal d position, based on backlash width
  {
    if (backlash_width < 0.01 /*m*/) {
      // If the backlash width is less than 0.01m, do not consider the backlash width and reduce
      // calculation cost
      terminal_point.d = 0.0;
    } else {
      const double return_width = path_width / 2.0;  // [m]
      const double current_momentum_d =
        current_point.d + 0.5 * current_point.d_vel * lateral_duration_adjusted;
      const double momentum_d_abs = std::abs(current_momentum_d);

      if (momentum_d_abs < backlash_width) {
        // If the object momentum is within the backlash width, we set the target d position to the
        // current momentum
        terminal_point.d = current_momentum_d;
      } else if (
        momentum_d_abs >= backlash_width && momentum_d_abs < backlash_width + return_width) {
        // If the object momentum is within the return zone, we set the target d position close to
        // the zero gradually
        terminal_point.d =
          (backlash_width + return_width - momentum_d_abs) * backlash_width / return_width;
        terminal_point.d *= (current_momentum_d > 0) ? 1 : -1;
      } else {
        // If the object momentum is outside the backlash width + return zone, we set the target d
        // position to 0
        terminal_point.d = 0.0;
      }
    }
  }

  // Step 2. Generate Predicted Path on a Frenet coordinate
  const auto frenet_predicted_path = generateFrenetPath(
    current_point, terminal_point, ref_path_len, duration, lateral_duration_adjusted,
    sampling_time_interval_);

  // Step 3. Interpolate Reference Path for converting predicted path coordinate
  const auto interpolated_ref_path = interpolateReferencePath(ref_path, frenet_predicted_path);

  if (frenet_predicted_path.size() < 2 || interpolated_ref_path.size() < 2) {
    return generateStraightPath(object, duration);
  }

  // Step 4. Convert predicted trajectory from Frenet to Cartesian coordinate
  return convertToPredictedPath(
    object, frenet_predicted_path, interpolated_ref_path, sampling_time_interval_);
}

}  // namespace autoware::map_based_prediction
