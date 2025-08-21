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

#include "autoware_utils/geometry/geometry.hpp"

#include <Eigen/Core>

#include "autoware_planning_msgs/msg/trajectory_point.hpp"

#include <algorithm>
#include <limits>

namespace planning_diagnostics
{
namespace metrics
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_utils::calc_distance2d;

Accumulator<double> calcDistanceToObstacle(
  const PredictedObjects & obstacles, const Trajectory & traj)
{
  Accumulator<double> stat;
  for (const TrajectoryPoint & p : traj.points) {
    double min_dist = std::numeric_limits<double>::max();
    for (const auto & object : obstacles.objects) {
      // TODO(Maxime CLEMENT): take into account the shape, not only the centroid
      const auto dist = calc_distance2d(object.kinematics.initial_pose_with_covariance.pose, p);
      min_dist = std::min(min_dist, dist);
    }
    stat.add(min_dist);
  }
  return stat;
}

Accumulator<double> calcTimeToCollision(
  const PredictedObjects & obstacles, const Trajectory & traj, const double distance_threshold)
{
  Accumulator<double> stat;

  if (traj.points.empty()) {
    return stat;
  }

  /** TODO(Maxime CLEMENT):
   * this implementation assumes static obstacles and does not work for dynamic obstacles
   */
  TrajectoryPoint p0 = traj.points.front();
  double t = 0.0;  // [s] time from start of trajectory
  constexpr double MIN_VELOCITY_THRESHOLD = 1e-3;
  for (const TrajectoryPoint & p : traj.points) {
    const double traj_dist = calc_distance2d(p0, p);
    if (std::abs(p0.longitudinal_velocity_mps) > MIN_VELOCITY_THRESHOLD) {
      const double dt = traj_dist / std::abs(p0.longitudinal_velocity_mps);
      t += dt;
      for (auto obstacle : obstacles.objects) {
        const double obstacle_dist =
          calc_distance2d(p, obstacle.kinematics.initial_pose_with_covariance.pose);
        // TODO(Maxime CLEMENT): take shape into consideration
        if (obstacle_dist <= distance_threshold) {
          stat.add(t);
          break;
        }
      }
    }
    if (stat.count() > 0) {
      break;
    }
    p0 = p;
  }
  return stat;
}

}  // namespace metrics
}  // namespace planning_diagnostics
