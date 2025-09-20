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

#ifndef AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__BIDIRECTIONAL_LANELETS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__BIDIRECTIONAL_LANELETS_HPP_

#include "autoware/behavior_path_bidirectional_traffic_module/parameter.hpp"
#include "autoware/trajectory/pose.hpp"
#include "autoware/trajectory/utils/find_intervals.hpp"
#include "autoware/universe_utils/geometry/boost_geometry.hpp"

#include <autoware_internal_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/Forward.h>

#include <memory>
#include <optional>
#include <utility>
#include <vector>
namespace autoware::behavior_path_planner
{

/**
 * @brief Class representing a set of connected bidirectional lanelets
 */
class ConnectedBidirectionalLanelets
{
public:
  friend class TestKeepLeft;

  using SharedPtr = std::shared_ptr<ConnectedBidirectionalLanelets>;
  using SharedConstPtr = std::shared_ptr<const ConnectedBidirectionalLanelets>;

  /**
   * @brief Search for all bidirectional lanelet pairs in the map
   * @param map Lanelet2 map
   * @param get_next_lanelets Function to get next lanelets
   * @param get_prev_lanelets Function to get previous lanelets
   * @return A list of connected bidirectional lanelet sets
   */
  static std::vector<ConnectedBidirectionalLanelets::SharedConstPtr>
  search_bidirectional_lanes_on_map(
    const lanelet::LaneletMap & map,
    const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> & get_next_lanelets,
    const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> & get_prev_lanelets);

  /**
   * @brief Construct a bidirectional lanelet pair from two lanelets
   * @param one_bidirectional_lanelet_pair A pair of lanelets
   * @param get_next_lanelets Function to get next lanelets
   * @param get_prev_lanelets Function to get previous lanelets
   * @return A pair of connected bidirectional lanelet sets
   */
  static std::pair<SharedConstPtr, SharedConstPtr> make_bidirectional_lane_pair(
    const std::pair<lanelet::ConstLanelet, lanelet::ConstLanelet> & one_bidirectional_lanelet_pair,
    const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> & get_next_lanelets,
    const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> & get_prev_lanelets);

  /**
   * @brief Get interval of trajectory overlapping with this lane
   * @param trajectory Input trajectory with lane IDs
   * @return Optional interval if overlap exists
   */
  [[nodiscard]] std::optional<experimental::trajectory::Interval> get_overlap_interval(
    const experimental::trajectory::Trajectory<
      autoware_internal_planning_msgs::msg::PathPointWithLaneId> & trajectory) const;
  /**
   * @brief Check if object is on this lane (by pose and polygon)
   * @param obj_pose Object pose
   * @param obj_polygon Object polygon in 2D
   * @return True if object lies on this lane
   */
  [[nodiscard]] bool is_object_on_this_lane(
    const geometry_msgs::msg::Pose & obj_pose,
    const autoware::universe_utils::Polygon2d & obj_polygon) const;

  /**
   * @brief Check if predicted object is on this lane
   * @param obj Predicted object message
   * @return True if object lies on this lane
   */
  [[nodiscard]] bool is_object_on_this_lane(
    const autoware_perception_msgs::msg::PredictedObject & obj) const;

  /**
   * @brief Get the opposite direction lanelet set
   * @return Opposite lanelet set
   */
  [[nodiscard]] SharedConstPtr get_opposite() const;

  /**
   * @brief Get centerline trajectory of lane
   * @return Centerline trajectory
   */
  [[nodiscard]] experimental::trajectory::Trajectory<geometry_msgs::msg::Pose> get_center_line()
    const;

  /**
   * @brief Get lanelets in this set
   * @return Vector of lanelets
   */
  [[nodiscard]] const lanelet::ConstLanelets & get_lanelets() const;

  /**
   * @brief Get inflow lanelets
   * @return Lanelets leading into this set
   */
  [[nodiscard]] const lanelet::ConstLanelets & get_lanelets_before_entering() const;

  /**
   * @brief Get outflow lanelets
   * @return Lanelets leading out from this set
   */
  [[nodiscard]] const lanelet::ConstLanelets & get_lanelets_after_exiting() const;

  /**
   * @brief Compute average lane width
   * @return Average width in meters
   */
  [[nodiscard]] double average_lane_width() const;

private:
  const lanelet::ConstLanelets bidirectional_lanelets_;
  const lanelet::ConstLanelets lanelets_before_entering_;
  const lanelet::ConstLanelets lanelets_after_entering_;
  mutable std::weak_ptr<ConnectedBidirectionalLanelets> opposite_;

  ConnectedBidirectionalLanelets(
    lanelet::ConstLanelets bidirectional_lanelets, lanelet::ConstLanelets lanelets_before_entering,
    lanelet::ConstLanelets lanelets_after_entering);

  /**
   * @brief Internal helper to recursively search connected bidirectional lanelets
   */
  static std::pair<ConnectedBidirectionalLanelets, ConnectedBidirectionalLanelets>
  search_connected_bidirectional_lanelets(
    const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> & get_next_lanelets,
    const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> & get_prev_lanelets,
    const lanelet::ConstLanelet & lanelet_a, const lanelet::ConstLanelet & lanelet_b);
};

/**
 * @brief Find which bidirectional lanelet the ego vehicle is in
 * @param ego_pose Pose of the ego vehicle
 * @param ego_params Ego vehicle parameters
 * @param all_bidirectional_lanes List of all connected bidirectional lanelets
 * @return Optional bidirectional lane containing ego
 */
std::optional<ConnectedBidirectionalLanelets::SharedConstPtr>
get_bidirectional_lanelets_where_ego_is(
  const geometry_msgs::msg::Pose & ego_pose, const EgoParameters & ego_params,
  const std::vector<ConnectedBidirectionalLanelets::SharedConstPtr> & all_bidirectional_lanes);

}  // namespace autoware::behavior_path_planner

// clang-format off
#endif  // AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__BIDIRECTIONAL_LANELETS_HPP_  // NOLINT
// clang-format on
