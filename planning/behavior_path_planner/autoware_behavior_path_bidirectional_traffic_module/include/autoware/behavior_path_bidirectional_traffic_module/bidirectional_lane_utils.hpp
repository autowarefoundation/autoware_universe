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

#ifndef AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__BIDIRECTIONAL_LANE_UTILS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__BIDIRECTIONAL_LANE_UTILS_HPP_

#include "autoware/trajectory/forward.hpp"
#include "autoware/trajectory/utils/find_intervals.hpp"

#include <tier4_planning_msgs/msg/path_point_with_lane_id.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <functional>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{
class ConnectedBidirectionalLanelets
{
private:
  std::vector<lanelet::ConstLanelet> bidirectional_lanes_;
  ConnectedBidirectionalLanelets * opposite_bidirectional_lanes_{nullptr};

  template <class Iterator>
  ConnectedBidirectionalLanelets(const Iterator & begin, const Iterator & end)
  : bidirectional_lanes_(begin, end)
  {
  }

  void set_opposite_bidirectional_lanes(
    ConnectedBidirectionalLanelets * opposite_bidirectional_lanes)
  {
    opposite_bidirectional_lanes_ = opposite_bidirectional_lanes;
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

  [[nodiscard]] std::optional<trajectory::Interval> get_overlap_interval(
    const trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> & trajectory) const;
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__BIDIRECTIONAL_LANE_UTILS_HPP_
