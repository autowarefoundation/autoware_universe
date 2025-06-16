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

namespace autoware::motion_velocity_planner::utils
{

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

template <typename Container, typename Predicate>
void remove_if(Container & container, Predicate pred)
{
  auto remove_itr = std::remove_if(container.begin(), container.end(), pred);
  container.erase(remove_itr, container.end());
}

inline geometry_msgs::msg::Point to_geom_pt(const Point2d & point)
{
  return autoware_utils::to_msg(point.to_3d(0.0));
}

inline Point2d to_pt2d(const geometry_msgs::msg::Point & point)
{
  return {point.x, point.y};
}
/**
 * @brief Initialize grouped departure intervals for each side of the vehicle.
 *
 * @param aw_ref_traj        The reference trajectory used to compute global poses from trajectory
 * distances.
 * @param departure_points   Departure points for both sides of the ego vehicle, indexed by SideKey.
 * @param vehicle_info       Vehicle information used to determine grouping threshold (e.g., length
 * offset).
 * @return A vector of `DepartureInterval`s for both sides of the vehicle, merged into one list.
 */
DepartureIntervals init_departure_intervals(
  const trajectory::Trajectory<TrajectoryPoint> & aw_ref_traj,
  const Side<DeparturePoints> & departure_points, const double vehicle_length,
  const std::unordered_set<DepartureType> & enable_type);

void update_departure_intervals(
  DepartureIntervals & departure_intervals, Side<DeparturePoints> & departure_points,
  const trajectory::Trajectory<TrajectoryPoint> & aw_ref_traj, const double vehicle_length_m,
  const TrajectoryPoint & ref_traj_fr_pt, const double ego_dist_from_traj_front,
  const double th_pt_shift_dist_m, const double th_pt_shift_angle_rad,
  const std::unordered_set<DepartureType> & enable_type);

void update_critical_departure_points(
  const Side<DeparturePoints> & new_departure_points,
  CriticalDeparturePoints & critical_departure_points,
  const trajectory::Trajectory<TrajectoryPoint> & aw_ref_traj, const double th_dist_hysteresis_m,
  const double offset_from_ego, const double th_pt_shift_dist_m,
  const double th_pt_shift_angle_rad);

std::vector<std::tuple<Pose, Pose, double>> get_slow_down_intervals(
  const trajectory::Trajectory<TrajectoryPoint> & ref_traj_pts,
  const DepartureIntervals & departure_intervals,
  const SlowDownInterpolator & slow_down_interpolator, const VehicleInfo & vehicle_info,
  const BoundarySideWithIdx & boundary_segments, const double curr_vel,
  const double ego_dist_on_traj_m);

std::optional<std::pair<double, double>> is_point_shifted(
  const Pose & prev_iter_pt, const Pose & curr_iter_pt, const double th_shift_m,
  const double th_yaw_diff_rad);
}  // namespace autoware::motion_velocity_planner::utils
#endif  // UTILS_HPP_
