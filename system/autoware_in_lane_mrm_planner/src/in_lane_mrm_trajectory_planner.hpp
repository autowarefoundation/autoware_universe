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

#ifndef IN_LANE_MRM_TRAJECTORY_PLANNER_HPP_
#define IN_LANE_MRM_TRAJECTORY_PLANNER_HPP_

#include "path_planner.hpp"
#include "type_alias.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <optional>

namespace autoware::in_lane_mrm_planner
{

// Orchestrates Route Lane Resolver (PathPlanner) and MRM lateral-offset steps in one call:
// plan_path → apply_lateral_offset → convert_path_to_trajectory → shift_trajectory_to_ego.
// InLaneMrmPlannerNode owns the PathPlanner instance and injects it here (approach A).
class InLaneMrmTrajectoryPlanner
{
public:
  InLaneMrmTrajectoryPlanner(
    PathPlanner & path_planner, const Params & params, const VehicleInfo & vehicle_info);

  std::optional<Trajectory> plan(const Odometry & odom) const;

  static double compute_lateral_offset(
    const PathWithLaneId & centerline_path, const geometry_msgs::msg::Pose & pose);

  static PathWithLaneId apply_lateral_offset(const PathWithLaneId & centerline, double d);

private:
  TrajectoryShiftParams make_shift_params() const;

  PathPlanner & path_planner_;
  Params params_;
  VehicleInfo vehicle_info_;
};

}  // namespace autoware::in_lane_mrm_planner

#endif  // IN_LANE_MRM_TRAJECTORY_PLANNER_HPP_
