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

#include "parameters.hpp"
#include "slow_down_interpolator.hpp"

#include <map>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace autoware::motion_velocity_planner::experimental::utils
{

/**
 * @brief Erase elements from an associative container if they match a given predicate.
 *
 * This function iterates over a `std::map` or `std::unordered_map` and removes all key-value
 * pairs for which the predicate returns true. The predicate must accept a `std::pair<Key, Value>`.
 *
 * @tparam Container Must be either `std::map` or `std::unordered_map`.
 * @tparam Predicate Unary function or lambda that returns true for elements to be removed.
 * @param container The map to remove elements from.
 * @param pred      The predicate used to test elements for removal.
 */
template <
  typename Container, typename Predicate,
  typename = std::enable_if_t<
    std::is_same_v<
      Container, std::map<typename Container::key_type, typename Container::mapped_type>> ||
    std::is_same_v<
      Container,
      std::unordered_map<typename Container::key_type, typename Container::mapped_type>>>>
void erase_if(Container & container, Predicate pred)
{
  for (auto it = container.begin(); it != container.end();) {
    if (pred(*it)) {  // For map, *it is a std::pair, so pred should handle pair<Key, Value>
      it = container.erase(it);
    } else {
      ++it;
    }
  }
}

/**
 * @brief Remove elements from a sequential container based on a predicate.
 *
 * This utility function uses `std::remove_if` followed by `erase` to remove all elements
 * that satisfy the predicate. Intended for use with containers like `std::vector`, `std::deque`,
 * etc.
 *
 * @tparam Container A sequence container supporting `erase` and iterators.
 * @tparam Predicate Unary function or lambda returning true for elements to be removed.
 * @param container The container to remove elements from.
 * @param pred      The predicate to determine which elements to remove.
 */
template <typename Container, typename Predicate>
void remove_if(Container & container, Predicate pred)
{
  auto remove_itr = std::remove_if(container.begin(), container.end(), pred);
  container.erase(remove_itr, container.end());
}

/**
 * @brief Convert a 2D point to a 3D geometry_msgs::msg::Point with z = 0.
 *
 * @param point A 2D point represented by `Point2d`.
 * @return A ROS-compatible 3D point with z set to 0.
 */
inline geometry_msgs::msg::Point to_geom_pt(const Point2d & point)
{
  return autoware_utils::to_msg(point.to_3d(0.0));
}

/**
 * @brief Convert a ROS 3D point into a 2D point by discarding the z-component.
 *
 * @param point A ROS point with x, y, z coordinates.
 * @return A 2D point containing only the x and y values.
 */
inline Point2d to_pt2d(const geometry_msgs::msg::Point & point)
{
  return {point.x, point.y};
}

/**
 * @brief Create grouped intervals of departure points for both sides of the ego vehicle.
 *
 * This function scans through the departure points on the left and right sides of the ego vehicle
 * and groups nearby points into meaningful intervals. A new interval starts when a valid departure
 * point is found, and subsequent points are added to the same interval if:
 * - Their type is one of the allowed types specified in `enable_type`.
 * - They are within one vehicle length of the previous point.
 * - They are not marked as removable.
 *
 * An interval must contain at least two valid departure points to be included in the result.
 * Each interval includes the start and end pose along the reference trajectory.
 *
 * @param aw_ref_traj     Reference trajectory used to compute poses along the path.
 * @param departure_points Departure points for left and right sides of the ego vehicle.
 * @param vehicle_length   Length of the ego vehicle, used to determine grouping distance.
 * @param enable_type      Set of departure types to include in the interval grouping.
 * @return A list of departure intervals representing potential boundary departure risks.
 */
DepartureIntervals init_departure_intervals(
  const trajectory::Trajectory<TrajectoryPoint> & aw_ref_traj,
  const Side<DeparturePoints> & departure_points, const double vehicle_length,
  const std::unordered_set<DepartureType> & enable_type);

/**
 * @brief Update and merge departure intervals based on current trajectory and ego state.
 *
 * This function:
 * 1. Recomputes interval start/end poses using the latest trajectory.
 * 2. Removes intervals already passed by the ego or shifted significantly.
 * 3. Marks departure points that fall within intervals or are near interval ends.
 * 4. Merges overlapping intervals on the same side to simplify the list.
 *
 * @param[out] departure_intervals Updated list of departure intervals.
 * @param[in,out] departure_points Per-side departure points; may be marked removable.
 * @param[in] aw_ref_traj Reference trajectory.
 * @param[in] vehicle_length_m Used to extend intervals if points are nearby.
 * @param[in] ref_traj_fr_pt First trajectory point (used when interval starts at front).
 * @param[in] ego_dist_from_traj_front Ego’s current distance along the trajectory.
 * @param[in] th_pt_shift_dist_m Threshold distance for detecting shifted points.
 * @param[in] th_pt_shift_angle_rad Threshold angle for detecting shifted points.
 * @param[in] enable_type Set of enabled departure types to consider for intervals.
 * @param[in] enable_type Enabled departure types.
 * @param[in] is_reset_interval Flags to reset departure intervals if no departure point found.
 * @param[in] is_departure_persist Checks to insert departure point to departure intervals.
 */
void update_departure_intervals(
  DepartureIntervals & departure_intervals, Side<DeparturePoints> & departure_points,
  const trajectory::Trajectory<TrajectoryPoint> & aw_ref_traj, const double vehicle_length_m,
  const TrajectoryPoint & ref_traj_fr_pt, const double ego_dist_from_traj_front,
  const double th_pt_shift_dist_m, const double th_pt_shift_angle_rad,
  const std::unordered_set<DepartureType> & enable_type, const bool is_reset_interval,
  const bool is_departure_persist);

/**
 * @brief Refresh and add critical departure points based on updated trajectory.
 *
 * Removes outdated or shifted critical points, and adds new ones if they are
 * sufficiently far from existing ones (hysteresis threshold).
 *
 * @param[in] new_departure_points New departure points from the current cycle.
 * @param[in,out] critical_departure_points Persistent list of critical points to update.
 * @param[in] aw_ref_traj Reference trajectory.
 * @param[in] th_point_merge_distance_m Min distance to avoid adding duplicates.
 * @param[in] offset_from_ego Ignore points before this arc-length.
 * @param[in] th_pt_shift_dist_m Threshold distance to detect point drift.
 * @param[in] th_pt_shift_angle_rad Threshold angle to detect pose change.
 */
void update_critical_departure_points(
  const Side<DeparturePoints> & new_departure_points,
  CriticalDeparturePoints & critical_departure_points,
  const trajectory::Trajectory<TrajectoryPoint> & aw_ref_traj,
  const double th_point_merge_distance_m, const double offset_from_ego,
  const double th_pt_shift_dist_m, const double th_pt_shift_angle_rad);

/**
 * @brief Build slow-down segments ahead of the ego vehicle.
 *
 * For each active departure interval:
 *   • Pick the interval edge still in front of the ego.
 *   • Use `slow_down_interpolator` to get a target velocity that respects
 *     longitudinal and lateral clearance.
 *   • Emit a tuple (start_pose, end_pose, target_vel) when the ego has not
 *     yet reached that edge and the computed target is valid.
 *
 * @param ref_traj_pts         Reference trajectory.
 * @param departure_intervals  Boundary-departure intervals to check.
 * @param slow_down_interpolator  Provides (rel_dist, vel, accel) lookup.
 * @param vehicle_info         Needed for longitudinal footprint checks.
 * @param boundary_segments     (reserved, currently unused)
 * @param curr_vel             Current ego speed [m/s].
 * @param ego_dist_on_traj_m   Ego arc-length position on the trajectory.
 * @return Vector of (start_pose, end_pose, target_vel) triples.
 */
std::vector<std::tuple<Pose, Pose, double>> get_slow_down_intervals(
  const trajectory::Trajectory<TrajectoryPoint> & ref_traj_pts,
  const DepartureIntervals & departure_intervals,
  const SlowDownInterpolator & slow_down_interpolator, const double curr_vel, const double curr_acc,
  const double ego_dist_on_traj_m);

/**
 * @brief Detect whether a pose has shifted beyond distance or yaw limits.
 *
 * @param prev_iter_pt       Pose from the previous frame/iteration.
 * @param curr_iter_pt       Current pose.
 * @param th_shift_m         Linear shift threshold [m].
 * @param th_yaw_diff_rad    Yaw-difference threshold [rad].
 * @return Pair {shift_m, yaw_diff_rad} if either threshold is exceeded,
 *         otherwise std::nullopt.
 */
std::optional<std::pair<double, double>> is_point_shifted(
  const Pose & prev_iter_pt, const Pose & curr_iter_pt, const double th_shift_m,
  const double th_yaw_diff_rad);
}  // namespace autoware::motion_velocity_planner::experimental::utils
#endif  // UTILS_HPP_
