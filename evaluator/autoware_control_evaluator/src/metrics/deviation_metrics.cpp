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

#include "autoware/control_evaluator/metrics/deviation_metrics.hpp"

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware_utils/geometry/geometry.hpp"
#include "autoware_utils/geometry/pose_deviation.hpp"

namespace control_diagnostics
{
namespace metrics
{
using autoware_planning_msgs::msg::Trajectory;

double calcLateralDeviation(const Trajectory & traj, const Point & point)
{
  const size_t nearest_index = autoware::motion_utils::findNearestIndex(traj.points, point);
  return autoware_utils::calc_lateral_deviation(traj.points[nearest_index].pose, point);
}

double calcYawDeviation(const Trajectory & traj, const Pose & pose)
{
  const size_t nearest_index = autoware::motion_utils::findNearestIndex(traj.points, pose.position);
  return autoware_utils::calc_yaw_deviation(traj.points[nearest_index].pose, pose);
}

double calcLongitudinalDeviation(const Pose & base_pose, const Point & target_point)
{
  return autoware_utils::calc_longitudinal_deviation(base_pose, target_point);
}

double calcLateralDeviation(const Pose & base_pose, const Point & target_point)
{
  return autoware_utils::calc_lateral_deviation(base_pose, target_point);
}

double calcYawDeviation(const Pose & base_pose, const Pose & target_pose)
{
  return autoware_utils::calc_yaw_deviation(base_pose, target_pose);
}

}  // namespace metrics
}  // namespace control_diagnostics
