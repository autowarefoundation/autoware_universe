// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_VELOCITY_BLIND_SPOT_MODULE__UTIL_HPP_
#define AUTOWARE__BEHAVIOR_VELOCITY_BLIND_SPOT_MODULE__UTIL_HPP_

#include <autoware/lanelet2_utils/intersection.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{

/**
 * @brief  wrapper class of interpolated path with lane id
 */
struct InterpolatedPathInfo
{
  /** the interpolated path */
  autoware_internal_planning_msgs::msg::PathWithLaneId path;
  /** discretization interval of interpolation */
  double ds{0.0};
  /** the intersection lanelet id */
  lanelet::Id lane_id{0};
  /** the range of indices for the path points with associative lane id */
  std::pair<size_t, size_t> lane_id_interval;
};

std::optional<InterpolatedPathInfo> generateInterpolatedPathInfo(
  const lanelet::Id lane_id,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & input_path, rclcpp::Logger logger);

/**
 * @brief return lane_id on the interval [lane_id1, lane_id2, ..., lane_id) (lane_id argument is
 * excluded) in the road connection order
 */
std::vector<lanelet::Id> find_lane_ids_upto(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path, const lanelet::Id lane_id);

/**
 * @brief obtain the index where the footprint on the path intersects with `line` for the first time
 * on the interval of `interpolated_path_info`
 */
std::optional<size_t> get_first_index_intersects_line(
  const lanelet::ConstLineString2d & line, const InterpolatedPathInfo & interpolated_path_info,
  const autoware_utils::LinearRing2d & footprint, const double vehicle_length);

/**
 * @brief generate a linestring consisting of two points of the entry part of `lanelet`
 * @note returned linestring is from left to right of `lanelet`'s boundary
 */
lanelet::ConstLineString3d get_entry_line(const lanelet::ConstLanelet & lanelet);

/**
 * @brief generate the attention_area for blind_spot(see document figure)
 * @param lane_ids_upto_intersection the lane ids upto the intersection itself, excluding the
 * intersection lane itself
 * @param lane_id the lane_id of the intersection lane
 */
std::optional<lanelet::CompoundPolygon3d> generate_attention_area(
  const lanelet::ConstLanelet & road_lanelets_before_turning_merged,
  const lanelet::ConstLanelets & blind_side_lanelets_before_turning,
  const lanelet::ConstLineString3d & virtual_blind_side_boundary_after_turning,
  const lanelet::ConstLineString3d & virtual_ego_straight_path_after_turning,
  const lanelet::ConstLanelet & intersection_lanelet,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const autoware::experimental::lanelet2_utils::TurnDirection & turn_direction,
  const double ego_width);

/**
 * @brief collect the lanelets before the intersection upto given `backward_attention_length`
 * @return non-empty list of Lanelets in the order of driving direction, or null.
 */
std::optional<std::pair<lanelet::ConstLanelets, lanelet::ConstLanelets>>
generate_blind_side_lanelets_before_turning(
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const autoware::experimental::lanelet2_utils::TurnDirection & turn_direction,
  const double backward_attention_length,
  const std::vector<lanelet::Id> & lane_ids_upto_intersection,
  const lanelet::Id intersection_lane_id);

/**
 * @brief return the extend outer boundary of `leftmost_lanelet`
 */
lanelet::ConstLineString3d generate_virtual_blind_side_boundary_after_turning(
  const lanelet::ConstLanelet & outermost_lanelet,
  const autoware::experimental::lanelet2_utils::TurnDirection & turn_direction,
  const double extend_length);

/**
 * @brief generate virtual LineString which is normal to the entry line of `intersection_lanelet`,
 * starting from the intersection point of `path`, OR the boundary of sibling straight lanelet of
 * `intersection_lanelet` if such lane exists
 */
std::optional<lanelet::LineString3d> generate_virtual_ego_straight_path_after_turning(
  const lanelet::ConstLanelet & intersection_lanelet,
  const lanelet::routing::RoutingGraphConstPtr routing_graph_ptr,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const autoware::experimental::lanelet2_utils::TurnDirection & turn_direction,
  const double ego_width);

/**
 * @brief generate a polygon representing the Path along the intersection lane, with given
 * `ego_width` width
 */
std::optional<lanelet::ConstLanelet> generate_ego_path_polygon(
  const InterpolatedPathInfo & interpolated_path_info, const double ego_width);

struct StopPoints
{
  std::optional<size_t> default_stopline;  //<! stopline for traffic light
  std::size_t instant_stopline;   //<! stopline ahead of current_pose by the braking distance
  std::size_t critical_stopline;  //<! stopline for conflict_area
};

/**
 * @brief generate default stopline at the entry of `intersection_lanelet`, and critical stopline
 * just before `virtual_ego_straight_path_after_turning`
 * @return stop line indices on the mutated `path`
 */
std::optional<StopPoints> generate_stop_points(
  const InterpolatedPathInfo & interpolated_path_info,
  const autoware_utils::LinearRing2d & footprint, const double ego_length,
  const lanelet::ConstLanelet & intersection_lanelet,
  const lanelet::ConstLineString3d & virtual_ego_straight_path_after_turning,
  const geometry_msgs::msg::Pose & current_pose, const double braking_distance,
  const double critical_stopline_margin, const double ego_nearest_dist_threshold,
  const double ego_nearest_yaw_threshold,
  autoware_internal_planning_msgs::msg::PathWithLaneId * path);

}  // namespace autoware::behavior_velocity_planner

#endif  // AUTOWARE__BEHAVIOR_VELOCITY_BLIND_SPOT_MODULE__UTIL_HPP_
