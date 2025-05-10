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

#ifndef AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__UTILS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__UTILS_HPP_

#include <autoware/trajectory/forward.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/Forward.h>

#include <functional>
#include <unordered_map>

namespace autoware::behavior_path_planner
{

double compute_length_of_lanelets(const lanelet::ConstLanelet & lanelet);

bool has_common_part(const lanelet::Ids & lane_ids1, const lanelet::Ids & lane_ids2);

lanelet::ConstLanelets filter_lanelets_by_ids(
  const lanelet::ConstLanelets & all_lanelets, const lanelet::Ids & target_lane_ids);

lanelet::ConstLanelets filter_intersection_lanelets(const lanelet::ConstLanelets & all_lanelets);

lanelet::ConstLanelets get_inflow_lanelets(
  const lanelet::ConstLanelets & lanelets,
  const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> & get_prev_lanelets);

lanelet::ConstLanelets get_outflow_lanelets(
  const lanelet::ConstLanelets & lanelets,
  const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> & get_next_lanelets);

size_t uuid_to_key(const std::array<uint8_t, 16> & uuid);

struct ConstLaneletsHash
{
  size_t operator()(const lanelet::ConstLanelets & lanelets) const;
};

template <typename KeyType, typename ValueType>
using ConstLaneletsHashMap = std::unordered_map<KeyType, ValueType, ConstLaneletsHash>;

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__UTILS_HPP_
