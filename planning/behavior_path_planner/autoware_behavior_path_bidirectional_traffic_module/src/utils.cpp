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

#include "autoware/behavior_path_bidirectional_traffic_module/utils.hpp"

#include "autoware_utils/system/lru_cache.hpp"

#include <boost/geometry/algorithms/length.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <unordered_set>

namespace autoware::behavior_path_planner
{

bool has_common_part(const lanelet::Ids & lane_ids1, const lanelet::Ids & lane_ids2)
{
  std::unordered_set<lanelet::Id> lane_ids_set1(lane_ids1.begin(), lane_ids1.end());
  for (const lanelet::Id & lane_id2 : lane_ids2) {
    if (lane_ids_set1.find(lane_id2) != lane_ids_set1.end()) {
      return true;
    }
  }
  return false;
}

double compute_length_of_lanelets(const lanelet::ConstLanelet & lanelet)
{
  static autoware_utils::LRUCache<lanelet::Id, double> lanelet_length_cache(1000);
  if (lanelet_length_cache.contains(lanelet.id())) {
    return lanelet_length_cache.get(lanelet.id()).value();
  }
  auto centerline = lanelet::utils::to2D(lanelet.centerline());
  const auto length = static_cast<double>(boost::geometry::length(centerline));
  lanelet_length_cache.put(lanelet.id(), length);
  return length;
}

lanelet::ConstLanelets filter_lanelets_by_ids(
  const lanelet::ConstLanelets & all_lanelets, const lanelet::Ids & target_lane_ids)
{
  lanelet::ConstLanelets filtered_lanelets;
  std::unordered_set<lanelet::Id> target_lane_ids_set(
    target_lane_ids.begin(), target_lane_ids.end());
  for (const auto & lanelet : all_lanelets) {
    if (target_lane_ids_set.find(lanelet.id()) != target_lane_ids_set.end()) {
      filtered_lanelets.emplace_back(lanelet);
    }
  }
  return filtered_lanelets;
}

lanelet::ConstLanelets filter_intersection_lanelets(const lanelet::ConstLanelets & all_lanelets)
{
  std::vector<lanelet::ConstLanelet> intersection_lanelets;
  for (const lanelet::ConstLanelet & lane : all_lanelets) {
    std::string_view turn_direction = lane.attributeOr("turn_direction", "none");
    if (turn_direction == "straight") {
      intersection_lanelets.emplace_back(lane);
    }
  }
  return intersection_lanelets;
}

lanelet::ConstLanelets get_inflow_lanelets(
  const lanelet::ConstLanelets & lanelets,
  const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> & get_prev_lanelets)
{
  std::unordered_set<lanelet::ConstLanelet> inflow_set;
  std::unordered_set<lanelet::ConstLanelet> lanelets_set(lanelets.begin(), lanelets.end());

  for (const lanelet::ConstLanelet & lanelet : lanelets) {
    for (const lanelet::ConstLanelet & prev_lanelet : get_prev_lanelets(lanelet)) {
      if (lanelets_set.find(prev_lanelet) == lanelets_set.end()) {
        inflow_set.insert(prev_lanelet);
      }
    }
  }
  return {inflow_set.begin(), inflow_set.end()};
}

lanelet::ConstLanelets get_outflow_lanelets(
  const lanelet::ConstLanelets & lanelets,
  const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> & get_next_lanelets)
{
  std::unordered_set<lanelet::ConstLanelet> outflow_set;
  std::unordered_set<lanelet::ConstLanelet> lanelets_set(lanelets.begin(), lanelets.end());

  for (const auto & lanelet : lanelets) {
    for (const auto & next_lanelet : get_next_lanelets(lanelet)) {
      // Add only if it is not in the given lanelets
      if (lanelets_set.find(next_lanelet) == lanelets_set.end()) {
        outflow_set.insert(next_lanelet);
      }
    }
  }

  return {outflow_set.begin(), outflow_set.end()};
}

size_t uuid_to_key(const std::array<uint8_t, 16> & uuid)
{
  return std::accumulate(uuid.begin(), uuid.end(), size_t{0}, [](size_t seed, uint8_t byte) {
    return seed ^ (static_cast<size_t>(byte) + 0x9e3779b9 + (seed << 6) + (seed >> 2));
  });
}

size_t ConstLaneletsHash::operator()(const lanelet::ConstLanelets & lanelets) const
{
  size_t seed = lanelets.size();
  for (const auto & lanelet : lanelets) {
    seed ^= std::hash<lanelet::Id>{}(lanelet.id()) + 0x9e3779b9 + (seed << 6U) + (seed >> 2U);
  }

  return seed;
}

}  // namespace autoware::behavior_path_planner
