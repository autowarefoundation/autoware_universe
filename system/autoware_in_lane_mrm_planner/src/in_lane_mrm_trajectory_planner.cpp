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

#include "in_lane_mrm_trajectory_planner.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <tf2/utils.h>

namespace autoware::in_lane_mrm_planner
{

InLaneMrmTrajectoryPlanner::InLaneMrmTrajectoryPlanner(
  PathPlanner & path_planner, const Params & params, const VehicleInfo & vehicle_info)
: path_planner_(path_planner), params_(params), vehicle_info_(vehicle_info)
{
  (void)vehicle_info_;
}

double InLaneMrmTrajectoryPlanner::compute_lateral_offset(
  const PathWithLaneId & centerline_path, const geometry_msgs::msg::Pose & pose)
{
  return autoware::motion_utils::calcLateralOffset(centerline_path.points, pose.position);
}

PathWithLaneId InLaneMrmTrajectoryPlanner::apply_lateral_offset(
  const PathWithLaneId & centerline, const double d)
{
  PathWithLaneId offset_path = centerline;
  for (auto & point : offset_path.points) {
    const double yaw = tf2::getYaw(point.point.pose.orientation);
    point.point.pose.position.x += d * (-std::sin(yaw));
    point.point.pose.position.y += d * std::cos(yaw);
  }
  return offset_path;
}

TrajectoryShiftParams InLaneMrmTrajectoryPlanner::make_shift_params() const
{
  TrajectoryShiftParams shift_params;
  const auto & path_shift = params_.path_planning.path_shift;
  shift_params.minimum_shift_length = path_shift.minimum_shift_length;
  shift_params.minimum_shift_yaw = path_shift.minimum_shift_yaw;
  shift_params.minimum_shift_distance = path_shift.minimum_shift_distance;
  shift_params.min_speed_for_curvature = path_shift.min_speed_for_curvature;
  shift_params.lateral_accel_limit = path_shift.lateral_accel_limit;
  return shift_params;
}

std::optional<Trajectory> InLaneMrmTrajectoryPlanner::plan(const Odometry & odom) const
{
  const auto & pose = odom.pose.pose;
  const double ego_velocity = odom.twist.twist.linear.x;
  const double ego_yaw_rate = odom.twist.twist.angular.z;

  const auto centerline_opt = path_planner_.plan_path(pose, ego_velocity);
  if (!centerline_opt) {
    return std::nullopt;
  }

  const double lateral_offset = compute_lateral_offset(*centerline_opt, pose);
  const auto offset_path = apply_lateral_offset(*centerline_opt, lateral_offset);

  auto trajectory = path_planner_.convert_path_to_trajectory(
    offset_path, params_.path_planning.output.delta_arc_length);
  trajectory.header = centerline_opt->header;

  if (params_.path_planning.path_shift.enable) {
    trajectory = path_planner_.shift_trajectory_to_ego(
      trajectory, pose, ego_velocity, ego_yaw_rate, make_shift_params(),
      params_.path_planning.output.delta_arc_length);
  }

  return trajectory;
}

}  // namespace autoware::in_lane_mrm_planner
