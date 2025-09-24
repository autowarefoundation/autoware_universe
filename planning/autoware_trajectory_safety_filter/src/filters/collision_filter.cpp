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

float time_to_collision(
  const TrajectoryPoint & ego_point, const rclcpp::Duration & duration,
  const autoware_perception_msgs::msg::PredictedObject & object, const float max_ttc_value = 100.0f)
{
  // Find the path with highest confidence
  const auto max_confidence_path = std::max_element(
    object.kinematics.predicted_paths.begin(), object.kinematics.predicted_paths.end(),
    [](const auto & a, const auto & b) { return a.confidence < b.confidence; });
  if (max_confidence_path == object.kinematics.predicted_paths.end()) return max_ttc_value;

  const auto & object_path = max_confidence_path->path;
  if (object_path.size() < 2) {
    if (duration.seconds() == 0.0f) {
      TrajectoryPoint object_point;
      object_point.pose = object.kinematics.initial_pose_with_covariance.pose;
      object_point.longitudinal_velocity_mps =
        object.kinematics.initial_twist_with_covariance.twist.linear.x;
      object_point.lateral_velocity_mps =
        object.kinematics.initial_twist_with_covariance.twist.linear.y;
      float ttc = time_to_collision(ego_point, object_point);
      return std::min(ttc, max_ttc_value);
    }
    return max_ttc_value;
  }

  const float dt = rclcpp::Duration(max_confidence_path->time_step).seconds();
  if (dt <= 0.0f) return max_ttc_value;

  const float max_time =
    dt * static_cast<float>(object_path.size() - 1);  // max time of the object path
  const float query_time = duration.seconds();
  if (query_time < 0.0f) return max_ttc_value;
  if (query_time > max_time) return max_ttc_value;

  const size_t nearest_index =
    std::min(static_cast<size_t>(query_time / dt), object_path.size() - 2);
  const float t_i = static_cast<float>(nearest_index) * dt;
  const float ratio = std::clamp((query_time - t_i) / dt, 0.0f, 1.0f);

  const auto object_pose = autoware_utils_geometry::calc_interpolated_pose(
    object_path.at(nearest_index), object_path.at(nearest_index + 1),
    ratio);  // for boundary check
  const auto segment = autoware_utils_geometry::point_2_tf_vector(
    object_path.at(nearest_index).position, object_path.at(nearest_index + 1).position);
  const float segment_length = segment.length();

  // Calculate object velocity (first in world coordinates)
  TrajectoryPoint obj{};
  obj.pose = object_pose;
  if (segment_length > 1e-6) {
    const auto dir_w = segment / segment_length;
    const float v = segment_length / dt;  // velocity scalar in world

    // world -> body rotation
    const float yaw = tf2::getYaw(obj.pose.orientation);
    const float c = std::cos(yaw);
    const float s = std::sin(yaw);
    const float vx_w = dir_w.x() * v;
    const float vy_w = dir_w.y() * v;

    obj.longitudinal_velocity_mps = c * vx_w + s * vy_w;  // body-long
    obj.lateral_velocity_mps = -s * vx_w + c * vy_w;      // body-lat
  } else {
    obj.longitudinal_velocity_mps = 0.0f;
    obj.lateral_velocity_mps = 0.0f;
  }

  const float ttc = time_to_collision(ego_point, obj);
  return std::min(ttc, max_ttc_value);
}
}  // namespace

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

  // Check each trajectory point within time horizon
  for (const auto & point : traj_points) {
    const double time_from_start = rclcpp::Duration(point.time_from_start).seconds();
    if (time_from_start > params_.max_check_time) {
      break;
    }

    if (check_collision(point, *context.predicted_objects, point.time_from_start)) {
      return false;
    }
  }

  return true;
}

bool CollisionFilter::check_collision(
  const TrajectoryPoint & traj_point,
  const autoware_perception_msgs::msg::PredictedObjects & objects,
  const rclcpp::Duration & duration) const
{
  for (const auto & object : objects.objects) {
    const double ttc = time_to_collision(traj_point, duration, object);

    // Check if TTC is below minimum threshold
    if (ttc < params_.min_ttc && ttc >= 0) {
      return true;  // Potential collision detected
    }
  }
  return false;  // No collision
}
}  // namespace autoware::trajectory_safety_filter::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_safety_filter::plugin::CollisionFilter,
  autoware::trajectory_safety_filter::plugin::SafetyFilterInterface)
