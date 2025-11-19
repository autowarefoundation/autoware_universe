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

#include "autoware/trajectory_ranker/metrics/metrics_utils.hpp"

#include "autoware/trajectory_ranker/utils.hpp"

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/duration.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2/utils.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

namespace autoware::trajectory_ranker::metrics::utils
{

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Vector3;

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

geometry_msgs::msg::Point transform_to_relative_coordinate2_d(
  const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Pose & origin)
{
  // translation
  geometry_msgs::msg::Point trans_p;
  trans_p.x = point.x - origin.position.x;
  trans_p.y = point.y - origin.position.y;

  // rotation (use inverse matrix of rotation)
  float yaw = tf2::getYaw(origin.orientation);

  geometry_msgs::msg::Point res;
  res.x = (cos(yaw) * trans_p.x) + (sin(yaw) * trans_p.y);
  res.y = ((-1) * sin(yaw) * trans_p.x) + (cos(yaw) * trans_p.y);
  res.z = origin.position.z;

  return res;
}

float calc_radius(
  const geometry_msgs::msg::Point & target, const geometry_msgs::msg::Pose & current_pose)
{
  constexpr float RADIUS_MAX = 1e9;
  const float denominator = 2 * transform_to_relative_coordinate2_d(target, current_pose).y;
  const float numerator =
    autoware_utils_geometry::calc_squared_distance2d(target, current_pose.position);

  if (fabs(denominator) > 0) {
    return numerator / denominator;
  }
  return RADIUS_MAX;
}

float curvature(
  const geometry_msgs::msg::Point & target, const geometry_msgs::msg::Pose & current_pose)
{
  constexpr float KAPPA_MAX = 1e9;
  const float radius = calc_radius(target, current_pose);

  if (fabs(radius) > 0) {
    return 1 / radius;
  }
  return KAPPA_MAX;
}

float pure_pursuit(const std::shared_ptr<TrajectoryPoints> & points, const Pose & ego_pose)
{
  if (!points || points->empty()) {
    return 0.0f;
  }

  const auto target_point = [&points, ego_pose]() {
    constexpr float lookahead_distance = 10.0f;
    const auto p = autoware::motion_utils::calcLongitudinalOffsetPoint(
      *points, ego_pose.position, lookahead_distance);

    if (p.has_value()) return p.value();

    return autoware_utils_geometry::get_point(points->back());
  }();

  return curvature(target_point, ego_pose);
}

float steer_command(
  const std::shared_ptr<TrajectoryPoints> & points, const Pose & ego_pose, const float wheel_base)
{
  return std::atan(wheel_base * pure_pursuit(points, ego_pose));
}

float time_to_collision(
  const std::shared_ptr<TrajectoryPoints> & points,
  const std::shared_ptr<PredictedObjects> & objects, const size_t idx, const float max_ttc_value)
{
  if (!objects || objects->objects.empty()) return max_ttc_value;
  if (!points || idx >= points->size()) return max_ttc_value;

  const auto & ego_point = points->at(idx);
  float best = std::numeric_limits<float>::infinity();

  for (const auto & object : objects->objects) {
    const auto time = ego_point.time_from_start;
    const float ttc = time_to_collision(ego_point, time, object, max_ttc_value);
    if (std::isfinite(ttc) && ttc >= 0.0f) {
      best = std::min(best, ttc);
    }
  }

  if (!std::isfinite(best)) return max_ttc_value;
  return std::min(best, max_ttc_value);
}

float time_to_collision(const TrajectoryPoint & point1, const TrajectoryPoint & point2)
{
  constexpr float eps = 1e-6;

  const auto displacement =
    autoware_utils_geometry::point_2_tf_vector(point1.pose.position, point2.pose.position);
  const float distance = displacement.length();
  if (distance < eps) return 0.0f;

  const auto dir = displacement.normalized();

  const auto v1 = utils::get_velocity_in_world_coordinate(point1);
  const auto v2 = utils::get_velocity_in_world_coordinate(point2);

  const float relative_velocity = tf2::tf2Dot(dir, v1) - tf2::tf2Dot(dir, v2);

  if (std::abs(relative_velocity) < eps) return std::numeric_limits<float>::infinity();
  return distance / relative_velocity;
}

float time_to_collision(
  const TrajectoryPoint & ego_point, const rclcpp::Duration & duration,
  const autoware_perception_msgs::msg::PredictedObject & object, const float max_ttc_value)
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

}  // namespace autoware::trajectory_ranker::metrics::utils
