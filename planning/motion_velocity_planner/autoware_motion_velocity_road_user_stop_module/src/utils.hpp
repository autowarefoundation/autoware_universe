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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/primitives/Polygon.h>

#include <vector>

namespace autoware::motion_velocity_planner::road_user_stop::utils
{

/**
 * @brief Convert a lanelet2 BasicPolygon2d to autoware_utils_geometry Polygon2d.
 * @param poly Lanelet2 BasicPolygon2d.
 * @return A Polygon2d with the same point sequence.
 */
autoware_utils_geometry::Polygon2d to_polygon_2d(const lanelet::BasicPolygon2d & poly);

/**
 * @brief Convert a lanelet2 BasicPolygon3d to autoware_utils_geometry Polygon2d.
 * @param poly Lanelet2 BasicPolygon3d.
 * @return A Polygon2d with the same point sequence (z-coordinate ignored).
 */
autoware_utils_geometry::Polygon2d to_polygon_2d(const lanelet::BasicPolygon3d & poly);

/**
 * @brief create one-step polygons from front bumper onwards (based on
 * polygon_utils::create_one_step_polygons)
 * @details this function creates polygons only from the front bumper position onwards,
 * excluding the rear part of the vehicle. this is useful for forward collision detection
 * where we only care about objects in front of the vehicle.
 * @param traj_points trajectory points
 * @param vehicle_info vehicle information
 * @param current_ego_pose current ego pose
 * @param lat_margin lateral margin for polygon expansion
 * @param enable_to_consider_current_pose whether to consider current pose error
 * @param time_to_convergence time for pose error to converge
 * @param decimate_trajectory_step_length step length for trajectory decimation
 * @return vector of polygons from front bumper onwards
 */
std::vector<autoware_utils_geometry::Polygon2d> create_one_step_polygons_from_front(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & traj_points,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const geometry_msgs::msg::Pose & current_ego_pose, const double lat_margin,
  const bool enable_to_consider_current_pose, const double time_to_convergence,
  const double decimate_trajectory_step_length);

}  // namespace autoware::motion_velocity_planner::road_user_stop::utils

#endif  // UTILS_HPP_
