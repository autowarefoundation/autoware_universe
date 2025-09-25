// Copyright 2025 TIER IV, Inc.
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

#include "autoware/trajectory_safety_filter/filters/collision_filter.hpp"

#include <autoware_utils_geometry/geometry.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <any>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::trajectory_safety_filter::plugin
{

namespace
{
tf2::Vector3 get_velocity_in_world_coordinate(const TrajectoryPoint & point)
{
  // Transform local velocity (in vehicle frame) to world frame
  const auto & pose = point.pose;
  const float yaw = tf2::getYaw(pose.orientation);
  const float cos_yaw = std::cos(yaw);
  const float sin_yaw = std::sin(yaw);

  // Rotate velocity from local to world frame
  const float vx_world =
    cos_yaw * point.longitudinal_velocity_mps - sin_yaw * point.lateral_velocity_mps;
  const float vy_world =
    sin_yaw * point.longitudinal_velocity_mps + cos_yaw * point.lateral_velocity_mps;

  return tf2::Vector3(vx_world, vy_world, 0.0f);
}

/**
 * @brief Calculates the time to collision (TTC) between two trajectory points.
 *
 * This function estimates the TTC by projecting the relative velocity of the two points
 * along the direction of their displacement. The calculation assumes infinitesimal points
 * rather than performing a full 2D collision check, which simplifies the computation.
 *
 * @param point1 The first trajectory point.
 * @param point2 The second trajectory point.
 * @return The time to collision in seconds. Returns 0.0f if the points are coincident,
 *         and std::numeric_limits<float>::infinity() if the relative velocity is negligible.
 */
float time_to_collision(const TrajectoryPoint & point1, const TrajectoryPoint & point2)
{
  constexpr float eps = 1e-6;

  const auto displacement =
    autoware_utils_geometry::point_2_tf_vector(point1.pose.position, point2.pose.position);
  const float distance = displacement.length();
  if (distance < eps) return 0.0f;

  const auto dir = displacement.normalized();

  const auto v1 = get_velocity_in_world_coordinate(point1);
  const auto v2 = get_velocity_in_world_coordinate(point2);

  const float relative_velocity = tf2::tf2Dot(dir, v1) - tf2::tf2Dot(dir, v2);

  if (std::abs(relative_velocity) < eps) return std::numeric_limits<float>::infinity();
  return distance / relative_velocity;
}
}  // namespace

CollisionFilter::ObjectStateCache CollisionFilter::precompute_object_positions(
  const autoware_perception_msgs::msg::PredictedObjects & objects, double max_time) const
{
  ObjectStateCache cache;
  const size_t time_steps = static_cast<size_t>(max_time / TIME_RESOLUTION) + 1;

  cache.reserve(objects.objects.size());

  for (const auto & object : objects.objects) {
    std::vector<nav_msgs::msg::Odometry> object_timeline;
    object_timeline.reserve(time_steps);

    const auto max_confidence_path = std::max_element(
      object.kinematics.predicted_paths.begin(), object.kinematics.predicted_paths.end(),
      [](const auto & a, const auto & b) { return a.confidence < b.confidence; });

    if (
      max_confidence_path == object.kinematics.predicted_paths.end() ||
      max_confidence_path->path.empty()) {
      // If no predicted path, use initial state for all time steps
      nav_msgs::msg::Odometry initial_state;
      initial_state.pose.pose = object.kinematics.initial_pose_with_covariance.pose;
      initial_state.twist.twist = object.kinematics.initial_twist_with_covariance.twist;

      for (size_t i = 0; i < time_steps; ++i) {
        object_timeline.push_back(initial_state);
      }
      continue;
    }
    const auto & predicted_path = max_confidence_path->path;
    const float dt = rclcpp::Duration(max_confidence_path->time_step).seconds();

    // Precompute positions at TIME_RESOLUTION intervals
    for (size_t i = 0; i < time_steps; ++i) {
      const double query_time = i * TIME_RESOLUTION;

      if (dt <= 0.0f || predicted_path.size() < 2) {
        // Use initial state
        nav_msgs::msg::Odometry state;
        state.pose.pose = object.kinematics.initial_pose_with_covariance.pose;
        state.twist.twist = object.kinematics.initial_twist_with_covariance.twist;
        object_timeline.push_back(state);
        continue;
      }

      const float max_path_time = dt * static_cast<float>(predicted_path.size() - 1);

      if (query_time > max_path_time) {
        // Beyond prediction horizon, use last predicted point
        const auto & last_pose = predicted_path.back();
        nav_msgs::msg::Odometry state;
        state.pose.pose = last_pose;
        // Velocity estimation from last two points
        if (predicted_path.size() >= 2) {
          const auto & prev_pose = predicted_path[predicted_path.size() - 2];
          const float dx = last_pose.position.x - prev_pose.position.x;
          const float dy = last_pose.position.y - prev_pose.position.y;
          // Set velocity in world frame
          state.twist.twist.linear.x = dx / dt;
          state.twist.twist.linear.y = dy / dt;
          state.twist.twist.linear.z = 0.0;
        } else {
          state.twist.twist.linear.x = 0.0;
          state.twist.twist.linear.y = 0.0;
          state.twist.twist.linear.z = 0.0;
        }
        object_timeline.push_back(state);
        continue;
      }

      // Interpolate between predicted points
      const auto idx = static_cast<size_t>(query_time / dt);
      const size_t next_idx = std::min(idx + 1, predicted_path.size() - 1);
      const double ratio = (query_time - static_cast<float>(idx) * dt) / dt;

      nav_msgs::msg::Odometry state;

      // Use calc_interpolated_pose for position and orientation interpolation
      state.pose.pose = autoware_utils_geometry::calc_interpolated_pose(
        predicted_path[idx], predicted_path[next_idx], ratio, false);  // false = use SLERP

      // Velocity in world frame
      const float dx = predicted_path[next_idx].position.x - predicted_path[idx].position.x;
      const float dy = predicted_path[next_idx].position.y - predicted_path[idx].position.y;
      state.twist.twist.linear.x = dx / dt;
      state.twist.twist.linear.y = dy / dt;
      state.twist.twist.linear.z = 0.0;

      object_timeline.push_back(state);
    }
    cache.push_back(std::move(object_timeline));
  }

  return cache;
}

nav_msgs::msg::Odometry CollisionFilter::get_cached_state(
  const ObjectStateCache & cache, size_t object_idx, double time) const
{
  if (object_idx >= cache.size()) {
    return nav_msgs::msg::Odometry{};
  }

  const auto & object_timeline = cache[object_idx];
  const auto time_idx = static_cast<size_t>(time / TIME_RESOLUTION);

  if (time_idx >= object_timeline.size() - 1) {
    return object_timeline.empty() ? nav_msgs::msg::Odometry{} : object_timeline.back();
  }

  const double remainder = time - time_idx * TIME_RESOLUTION;

  if (remainder < 1e-6) {
    // Exact match
    return object_timeline[time_idx];
  }

  // Linear interpolation between two cached states
  const auto & state1 = object_timeline[time_idx];
  const auto & state2 = object_timeline[time_idx + 1];
  const double ratio = remainder / TIME_RESOLUTION;

  nav_msgs::msg::Odometry interpolated;

  // Use calc_interpolated_pose for position and orientation
  interpolated.pose.pose = autoware_utils_geometry::calc_interpolated_pose(
    state1.pose.pose, state2.pose.pose, ratio, false);  // false = use SLERP

  // Interpolate velocity linearly
  interpolated.twist.twist.linear.x =
    state1.twist.twist.linear.x +
    ratio * (state2.twist.twist.linear.x - state1.twist.twist.linear.x);
  interpolated.twist.twist.linear.y =
    state1.twist.twist.linear.y +
    ratio * (state2.twist.twist.linear.y - state1.twist.twist.linear.y);
  interpolated.twist.twist.linear.z =
    state1.twist.twist.linear.z +
    ratio * (state2.twist.twist.linear.z - state1.twist.twist.linear.z);

  return interpolated;
}

bool CollisionFilter::check_collision(
  const TrajectoryPoint & traj_point, const ObjectStateCache & cache, size_t object_idx,
  double time_from_start) const
{
  const auto object_odometry = get_cached_state(cache, object_idx, time_from_start);

  // Convert Odometry to TrajectoryPoint for collision check
  TrajectoryPoint object_point;
  object_point.pose = object_odometry.pose.pose;
  object_point.longitudinal_velocity_mps = object_odometry.twist.twist.linear.x;
  object_point.lateral_velocity_mps = object_odometry.twist.twist.linear.y;

  const float ttc = time_to_collision(traj_point, object_point);
  return ttc >= 0.0 && ttc < params_.min_ttc;
}

void CollisionFilter::set_parameters(const std::unordered_map<std::string, std::any> & params)
{
  auto get_value = [&params](const std::string & key, auto & value) {
    auto it = params.find(key);
    if (it != params.end()) {
      try {
        value = std::any_cast<std::decay_t<decltype(value)>>(it->second);
      } catch (const std::bad_any_cast &) {
        // Keep default value if cast fails
      }
    }
  };

  // Map from parameter structure
  get_value("time", params_.max_check_time);
  get_value("min_value", params_.min_ttc);
}

bool CollisionFilter::is_feasible(
  const TrajectoryPoints & traj_points, const FilterContext & context)
{
  if (!context.predicted_objects || context.predicted_objects->objects.empty()) {
    return true;  // No objects to check collision with
  }

  const auto cache =
    precompute_object_positions(*context.predicted_objects, params_.max_check_time);

  // Check each trajectory point within time horizon
  for (const auto & point : traj_points) {
    const double time_from_start = rclcpp::Duration(point.time_from_start).seconds();
    if (time_from_start > params_.max_check_time) {
      break;
    }

    // Check collision with each cached object
    for (size_t obj_idx = 0; obj_idx < cache.size(); ++obj_idx) {
      if (check_collision(point, cache, obj_idx, time_from_start)) {
        return false;
      }
    }
  }

  return true;
}
}  // namespace autoware::trajectory_safety_filter::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_safety_filter::plugin::CollisionFilter,
  autoware::trajectory_safety_filter::plugin::SafetyFilterInterface)
