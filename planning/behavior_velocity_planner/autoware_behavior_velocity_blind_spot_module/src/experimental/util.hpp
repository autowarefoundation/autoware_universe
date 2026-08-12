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

#ifndef EXPERIMENTAL__UTIL_HPP_
#define EXPERIMENTAL__UTIL_HPP_

#include <autoware/lanelet2_utils/intersection.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware/trajectory/path_point_with_lane_id.hpp>
#include <autoware_utils/geometry/geometry.hpp>

// #include <lanelet2_core/primitives/Lanelet.h>

#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{

using autoware::experimental::lanelet2_utils::TurnDirection;
using Trajectory = autoware::experimental::trajectory::Trajectory<
  autoware_internal_planning_msgs::msg::PathPointWithLaneId>;

struct StopLinePositions
{
  std::optional<double> default_s;  //<! stop line for traffic light
  double instant_s;                 //<! stop line ahead of current_pose by the braking distance
  double critical_s;                //<! stop line for conflict_area
};

/**
 * @brief return lane_id on the interval [lane_id1, lane_id2, ..., lane_id) (lane_id argument is
 * excluded) in the road connection order
 */
std::vector<lanelet::Id> find_lane_ids_up_to(const Trajectory & path, const lanelet::Id lane_id);

/**
 * @brief generate the attention_area for blind_spot (see document figure)
 * @param lane_ids_upto_intersection the lane ids upto the intersection itself, excluding the
 * intersection lane itself
 * @param lane_id the lane_id of the intersection lane
 */
std::optional<lanelet::CompoundPolygon3d> generate_attention_area(
  const lanelet::ConstLanelet & road_lanelets_before_turning_merged,
  const lanelet::ConstLanelets & blind_side_lanelets_before_turning,
  const lanelet::ConstLineString3d & virtual_blind_side_boundary_after_turning,
  const lanelet::ConstLineString3d & virtual_ego_straight_path_after_turning,
  const lanelet::ConstLanelet & intersection_lanelet, const Trajectory & path,
  const TurnDirection & turn_direction, const double ego_width);

/**
 * @brief generate virtual LineString which is normal to the entry line of `intersection_lanelet`,
 * starting from the intersection point of `path`, OR the boundary of sibling straight lanelet of
 * `intersection_lanelet` if such lane exists
 * @param intersection_lanelet the intersection lanelet where the ego vehicle is turning
 * @param path the ego vehicle's path
 * @param turn_direction the turn direction of the intersection lanelet
 * @param ego_width the width of the ego vehicle
 * @return virtual straight path after turning
 */
std::optional<lanelet::ConstLineString3d> generate_virtual_ego_straight_path_after_turning(
  const lanelet::ConstLanelet & intersection_lanelet, const Trajectory & path,
  const TurnDirection & turn_direction, const double ego_width);

/**
 * @brief generate a polygon representing the Path along the intersection lane, with given
 * `ego_width` width
 */
std::optional<lanelet::ConstLanelet> generate_ego_path_polygon(
  const Trajectory & path, const double ego_width);

/**
 * @brief generate default stop line at the entry of `intersection_lanelet`, and critical stopline
 * just before `virtual_ego_straight_path_after_turning`
 * @return stop line indices on the mutated `path`
 */
std::optional<StopLinePositions> generate_stop_points(
  const autoware_utils::LinearRing2d & footprint, const double ego_length, const double ego_s,
  const lanelet::ConstLanelet & intersection_lanelet,
  const lanelet::ConstLineString3d & virtual_ego_straight_path_after_turning,
  const double braking_distance, const double critical_stopline_margin, Trajectory & path);
}  // namespace autoware::behavior_velocity_planner::experimental

#endif  // EXPERIMENTAL__UTIL_HPP_
