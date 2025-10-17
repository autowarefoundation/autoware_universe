// Copyright 2021 Tier IV, Inc.
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

#include "autoware/planning_evaluator/metrics/obstacle_metrics.hpp"

#include "autoware/planning_evaluator/metrics/metrics_utils.hpp"
#include "autoware_utils/geometry/geometry.hpp"

#include <Eigen/Core>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>

#include "autoware_planning_msgs/msg/trajectory_point.hpp"

#include <boost/geometry.hpp>

#include <algorithm>
#include <limits>

namespace planning_diagnostics
{
namespace metrics
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_utils::calc_distance2d;
namespace bg = boost::geometry;

Accumulator<double> calcDistanceToObstacle(
  const PredictedObjects & obstacles, const Trajectory & traj, const VehicleInfo & vehicle_info)
{
  Accumulator<double> stat;

  // Get local ego footprint once
  const autoware_utils::LinearRing2d local_ego_footprint = vehicle_info.createFootprint();

  for (const TrajectoryPoint & p : traj.points) {
    double min_dist = std::numeric_limits<double>::max();
    for (const auto & object : obstacles.objects) {
      // Use footprint-based distance calculation
      const auto dist = utils::calc_ego_object_distance(local_ego_footprint, p.pose, object);
      min_dist = std::min(min_dist, dist);
    }
    stat.add(min_dist);
  }
  return stat;
}

Accumulator<double> calcTimeToCollision(
  const PredictedObjects & obstacles, const Trajectory & traj, const VehicleInfo & vehicle_info,
  const double distance_threshold)
{
  Accumulator<double> stat;

  if (traj.points.empty() || obstacles.objects.empty()) {
    return stat;
  }

  // Get local ego footprint once
  const autoware_utils::LinearRing2d local_ego_footprint = vehicle_info.createFootprint();

  // Dynamic obstacle state
  struct DynamicObstacle
  {
    autoware_perception_msgs::msg::PredictedObject object;
    size_t path_index = 1;  // Next target point index in path
    geometry_msgs::msg::Vector3 direction;
    double velocity;

    explicit DynamicObstacle(const autoware_perception_msgs::msg::PredictedObject & obj)
    : object(obj)
    {
      // Calculate velocity
      const auto & twist = obj.kinematics.initial_twist_with_covariance.twist.linear;
      velocity = std::sqrt(twist.x * twist.x + twist.y * twist.y);

      // Set default direction from object's orientation (fallback for no path)
      const auto & quat = obj.kinematics.initial_pose_with_covariance.pose.orientation;
      direction.x = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
      direction.y = 2.0 * (quat.x * quat.y + quat.w * quat.z);
    }

    void updatePosition(double dt)
    {
      if (velocity < 1e-3) return;

      auto & pose = object.kinematics.initial_pose_with_covariance.pose;
      double remaining_dist = velocity * dt;

      // Use a more conservative threshold to handle floating-point precision issues
      constexpr double DISTANCE_THRESHOLD = 1e-3;

      // Move along path segments, handling multiple points if dt is large
      if (!object.kinematics.predicted_paths.empty()) {
        const auto & path = object.kinematics.predicted_paths[0].path;

        while (remaining_dist > DISTANCE_THRESHOLD && path_index < path.size()) {
          const auto & target = path[path_index];
          const double seg_dist = calc_distance2d(pose, target);

          // Handle case where path points are too close together
          if (seg_dist < DISTANCE_THRESHOLD) {
            pose = target;
            path_index++;
            continue;
          }

          if (seg_dist <= remaining_dist) {
            // Move to this path point and continue to next
            pose = target;
            remaining_dist -= seg_dist;
            path_index++;

            // Update direction for next segment
            if (path_index < path.size()) {
              const auto & next_target = path[path_index];
              const double dx = next_target.position.x - pose.position.x;
              const double dy = next_target.position.y - pose.position.y;
              const double norm = std::sqrt(dx * dx + dy * dy);
              if (norm > 1e-6) {
                direction.x = dx / norm;
                direction.y = dy / norm;
              }
            }
          } else {
            // Move partway to this path point
            const double ratio = remaining_dist / seg_dist;
            const double dx = target.position.x - pose.position.x;
            const double dy = target.position.y - pose.position.y;
            pose.position.x += ratio * dx;
            pose.position.y += ratio * dy;
            // Calculate orientation from movement direction
            const double half_yaw = std::atan2(dy, dx) * 0.5;
            pose.orientation = geometry_msgs::msg::Quaternion{};
            pose.orientation.z = std::sin(half_yaw);
            pose.orientation.w = std::cos(half_yaw);
            remaining_dist = 0.0;
          }
        }
      }

      // If path exhausted but still have distance, continue in last direction
      if (remaining_dist > DISTANCE_THRESHOLD) {
        pose.position.x += remaining_dist * direction.x;
        pose.position.y += remaining_dist * direction.y;
        const double half_yaw = std::atan2(direction.y, direction.x) * 0.5;
        pose.orientation = geometry_msgs::msg::Quaternion{};
        pose.orientation.z = std::sin(half_yaw);
        pose.orientation.w = std::cos(half_yaw);
      }
    }
  };

  // Initialize dynamic obstacles
  std::vector<DynamicObstacle> dynamic_obstacles(
    obstacles.objects.begin(), obstacles.objects.end());

  TrajectoryPoint p0 = traj.points.front();
  double t = 0.0;
  constexpr double MIN_VELOCITY_THRESHOLD = 1e-3;

  for (const TrajectoryPoint & p : traj.points) {
    if (std::abs(p0.longitudinal_velocity_mps) < MIN_VELOCITY_THRESHOLD) {
      p0 = p;
      continue;
    }

    const double dt = calc_distance2d(p0, p) / std::abs(p0.longitudinal_velocity_mps);
    t += dt;

    // Update obstacle positions and check collisions
    for (auto & obstacle : dynamic_obstacles) {
      obstacle.updatePosition(dt);

      const auto distance =
        utils::calc_ego_object_distance(local_ego_footprint, p.pose, obstacle.object);
      if (distance <= distance_threshold) {
        stat.add(t);
        return stat;
      }
    }
    p0 = p;
  }
  return stat;
}

}  // namespace metrics
}  // namespace planning_diagnostics
