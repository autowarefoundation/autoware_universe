// Copyright 2025 Tier IV, Inc.
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

#ifndef AUTOWARE__PLANNING_EVALUATOR__METRICS__METRICS_UTILS_HPP_
#define AUTOWARE__PLANNING_EVALUATOR__METRICS__METRICS_UTILS_HPP_

#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include "autoware_perception_msgs/msg/predicted_object.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

#include <sstream>
#include <iomanip>
#include <string>

namespace planning_diagnostics
{
namespace metrics
{
namespace utils
{
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;

/**
 * @brief convert UUID to string representation
 * @param [in] uuid UUID message
 * @return hex string representation of UUID
 */
inline std::string uuid_to_string(const unique_identifier_msgs::msg::UUID & uuid)
{
  std::stringstream ss;
  ss << std::hex << std::setfill('0');
  for (const auto & byte : uuid.uuid) {
    ss << std::setw(2) << static_cast<int>(byte);
  }
  return ss.str();
}

/**
 * @brief find the index in the trajectory at the given distance of the given index
 * @param [in] traj input trajectory
 * @param [in] curr_id index
 * @param [in] distance distance
 * @return index of the trajectory point at distance ahead of traj[curr_id]
 */
size_t getIndexAfterDistance(const Trajectory & traj, const size_t curr_id, const double distance);

/**
 * @brief trim a trajectory from the current ego pose to some fixed time or distance
 * @param [in] traj input trajectory to trim
 * @param [in] max_dist_m [m] maximum distance ahead of the ego pose
 * @param [in] max_time_s [s] maximum time ahead of the ego pose
 * @return sub-trajectory starting from the ego pose and of maximum length max_dist_m, maximum
 * duration max_time_s
 */
Trajectory get_lookahead_trajectory(
  const Trajectory & traj, const Pose & ego_pose, const double max_dist_m, const double max_time_s);

/**
 * @brief calculate the total distance from ego position to the end of trajectory
 * @details finds the nearest point to ego position on the trajectory and calculates
 *          the cumulative distance by summing up the distances between consecutive points
 *          from that position to the end of the trajectory.
 *
 * @param [in] traj input trajectory to calculate distance along
 * @param [in] ego_pose current ego vehicle pose
 * @return total distance from ego position to trajectory end in meters
 */
double calc_lookahead_trajectory_distance(const Trajectory & traj, const Pose & ego_pose);

/**
 * @brief create ego vehicle footprint polygon at a specific pose
 * @param [in] local_ego_footprint ego vehicle footprint in local coordinates
 * @param [in] ego_pose current ego vehicle pose in world coordinates
 * @return ego footprint polygon transformed to world coordinates
 */
autoware_utils::Polygon2d create_pose_footprint(
  const autoware_utils::LinearRing2d & local_ego_footprint, const Pose & ego_pose);

/**
 * @brief create an elongated footprint polygon by combining ego footprints along trajectory
 * @details Uses adaptive resampling based on vehicle info to balance accuracy and
 *          performance. Resamples trajectory at 0.2x vehicle length intervals and fills gaps
 *          larger than 0.5x vehicle length with intermediate points to ensure continuity.
 * @param [in] vehicle_info vehicle information containing dimensions
 * @param [in] traj trajectory to sweep the footprint along
 * @param [in] ego_pose current ego vehicle pose in world coordinates
 * @return combined polygon representing the swept area of ego footprint along trajectory
 */
autoware_utils::Polygon2d create_trajectory_footprint(
  const VehicleInfo & vehicle_info, const Trajectory & traj, const Pose & ego_pose);

}  // namespace utils
}  // namespace metrics
}  // namespace planning_diagnostics
#endif  // AUTOWARE__PLANNING_EVALUATOR__METRICS__METRICS_UTILS_HPP_
