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

#ifndef AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__CONNECTED_BIDIRECTIONAL_LANELETS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__CONNECTED_BIDIRECTIONAL_LANELETS_HPP_

#include "autoware/trajectory/forward.hpp"
#include "autoware/trajectory/pose.hpp"
#include "autoware/trajectory/utils/find_intervals.hpp"
#include "autoware/universe_utils/geometry/boost_geometry.hpp"

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <tier4_planning_msgs/msg/path_point_with_lane_id.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{
class ConnectedBidirectionalLanelets
{
private:
  std::vector<lanelet::ConstLanelet> bidirectional_lanes_;
  std::shared_ptr<ConnectedBidirectionalLanelets> opposite_bidirectional_lanes_;

  void set_opposite_bidirectional_lanes(
    const ConnectedBidirectionalLanelets & opposite_bidirectional_lanes)
  {
    opposite_bidirectional_lanes_ = std::make_shared<ConnectedBidirectionalLanelets>(
      opposite_bidirectional_lanes.bidirectional_lanes_.begin(),
      opposite_bidirectional_lanes.bidirectional_lanes_.end());
  }

  static std::pair<ConnectedBidirectionalLanelets, ConnectedBidirectionalLanelets>
  search_connected_bidirectional_lanelets(
    const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> & get_next_lanelets,
    const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> &
      get_previous_lanelets,
    const lanelet::ConstLanelet & lanelet_a, const lanelet::ConstLanelet & lanelet_b);

public:
  static std::vector<ConnectedBidirectionalLanelets> search_bidirectional_lanes_on_map(
    const lanelet::LaneletMap & map,
    const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> & get_next_lanelets,
    const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> &
      get_previous_lanelets);

  template <class Iterator>
  ConnectedBidirectionalLanelets(const Iterator & begin, const Iterator & end)
  : bidirectional_lanes_(begin, end)
  {
  }

  ConnectedBidirectionalLanelets(const ConnectedBidirectionalLanelets & other);

  ConnectedBidirectionalLanelets & operator=(const ConnectedBidirectionalLanelets & other);

  ConnectedBidirectionalLanelets(ConnectedBidirectionalLanelets && other) noexcept = default;

  ConnectedBidirectionalLanelets & operator=(ConnectedBidirectionalLanelets && other) noexcept =
    default;

  ~ConnectedBidirectionalLanelets() = default;

  [[nodiscard]] std::optional<trajectory::Interval> get_overlap_interval(
    const trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> & trajectory) const;

  [[nodiscard]] bool is_object_on_this_lane(
    const geometry_msgs::msg::Pose & obj_pose,
    const autoware::universe_utils::Polygon2d & obj_polygon) const;

  [[nodiscard]] bool is_object_on_this_lane(
    const autoware_perception_msgs::msg::PredictedObject & obj) const;

  [[nodiscard]] ConnectedBidirectionalLanelets get_opposite_bidirectional_lanes() const;

  [[nodiscard]] trajectory::Trajectory<geometry_msgs::msg::Pose> get_center_line() const;

  [[nodiscard]] trajectory::Trajectory<geometry_msgs::msg::Pose> get_left_line() const;

  bool is_inside_intersection(const geometry_msgs::msg::Pose & ego_pose) const;
};

std::optional<ConnectedBidirectionalLanelets> get_current_bidirectional_lane(
  const geometry_msgs::msg::Pose & ego_pose,
  const autoware::universe_utils::Polygon2d & ego_polygon,
  const std::vector<ConnectedBidirectionalLanelets> & all_bidirectional_lanes);

}  // namespace autoware::behavior_path_planner

// clang-format off
#endif  // AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__CONNECTED_BIDIRECTIONAL_LANELETS_HPP_  // NOLINT
// clang-format on
