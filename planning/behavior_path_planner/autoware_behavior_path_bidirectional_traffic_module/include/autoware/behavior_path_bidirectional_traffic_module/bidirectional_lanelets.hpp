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
class ConnectedBidirectionalLanelets
{
public:
  using SharedPtr = std::shared_ptr<ConnectedBidirectionalLanelets>;
  using SharedConstPtr = std::shared_ptr<const ConnectedBidirectionalLanelets>;

  static std::vector<ConnectedBidirectionalLanelets::SharedConstPtr>
  search_bidirectional_lanes_on_map(
    const lanelet::LaneletMap & map,
    const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> & get_next_lanelets,
    const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> & get_prev_lanelets);

  static std::pair<SharedConstPtr, SharedConstPtr> make_bidirectional_lane_pair(
    const std::pair<lanelet::ConstLanelet, lanelet::ConstLanelet> & one_bidirectional_lanelet_pair,
    const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> & get_next_lanelets,
    const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> & get_prev_lanelets);

  [[nodiscard]] std::optional<trajectory::Interval> get_overlap_interval(
    const trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> &
      trajectory) const;
  [[nodiscard]] bool is_object_on_this_lane(
    const geometry_msgs::msg::Pose & obj_pose,
    const autoware::universe_utils::Polygon2d & obj_polygon) const;
  [[nodiscard]] bool is_object_on_this_lane(
    const autoware_perception_msgs::msg::PredictedObject & obj) const;

  [[nodiscard]] SharedConstPtr get_opposite() const;
  [[nodiscard]] trajectory::Trajectory<geometry_msgs::msg::Pose> get_center_line() const;

  [[nodiscard]] const lanelet::ConstLanelets & get_lanelets() const;
  [[nodiscard]] const lanelet::ConstLanelets & get_lanelets_before_entering() const;
  [[nodiscard]] const lanelet::ConstLanelets & get_lanelets_after_exiting() const;
  [[nodiscard]] const lanelet::ConstLanelets & get_intersection_lanelets() const;

  [[nodiscard]] double average_lane_width() const;

  [[nodiscard]] std::pair<bool, std::vector<geometry_msgs::msg::Pose>>
  check_if_is_possible_to_stop_and_suggest_alternative_stop_poses(
    const geometry_msgs::msg::Pose & stopping_pose, const EgoParameters & ego_params) const;

private:
  const lanelet::ConstLanelets bidirectional_lanelets_;
  const lanelet::ConstLanelets lanelets_before_entering_;
  const lanelet::ConstLanelets lanelets_after_entering_;
  mutable std::weak_ptr<ConnectedBidirectionalLanelets> opposite_;

  ConnectedBidirectionalLanelets(
    lanelet::ConstLanelets bidirectional_lanelets, lanelet::ConstLanelets lanelets_before_entering,
    lanelet::ConstLanelets lanelets_after_entering);

  static std::pair<ConnectedBidirectionalLanelets, ConnectedBidirectionalLanelets>
  search_connected_bidirectional_lanelets(
    const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> & get_next_lanelets,
    const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> & get_prev_lanelets,
    const lanelet::ConstLanelet & lanelet_a, const lanelet::ConstLanelet & lanelet_b);
};

std::optional<ConnectedBidirectionalLanelets::SharedConstPtr>
get_bidirectional_lanelets_where_ego_is(
  const geometry_msgs::msg::Pose & ego_pose, const EgoParameters & ego_params,
  const std::vector<ConnectedBidirectionalLanelets::SharedConstPtr> & all_bidirectional_lanes);

bool check_for_bidirectional_lanelets_in_trajectory(
  const trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> &
    trajectory,
  const std::vector<ConnectedBidirectionalLanelets> & bidirectional_lanelets);

}  // namespace autoware::behavior_path_planner

// clang-format off
#endif  // AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__BIDIRECTIONAL_LANELETS_HPP_  // NOLINT
// clang-format on
