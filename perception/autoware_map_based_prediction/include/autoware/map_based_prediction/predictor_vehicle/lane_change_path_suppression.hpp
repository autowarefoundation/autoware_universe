// Copyright 2021 TIER IV, Inc.
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

#ifndef AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VEHICLE__LANE_CHANGE_PATH_SUPPRESSION_HPP_
#define AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VEHICLE__LANE_CHANGE_PATH_SUPPRESSION_HPP_

#include "autoware/map_based_prediction/data_structure.hpp"

#include <geometry_msgs/msg/point.hpp>

#include <lanelet2_routing/RoutingGraph.h>

#include <cstddef>
#include <set>
#include <vector>

namespace autoware::map_based_prediction::detail
{
struct LaneChangePathSuppressionInfo
{
  Maneuver detected_maneuver;
  bool lane_ends_within_horizon;
  /// Whether the current lanelet this candidate came from has a lane-follow candidate.
  bool center_exists{false};
  lanelet::Id current_lanelet_id{lanelet::InvalId};
  double lane_end_distance{-1.0};
};

bool laneEndsWithinHorizon(
  const lanelet::routing::LaneletPath & path, const geometry_msgs::msg::Point & position,
  double travel_distance, const lanelet::routing::RoutingGraph & graph);

bool laneEndsWithinHorizon(
  const lanelet::ConstLanelet & current_lanelet, const geometry_msgs::msg::Point & position,
  double travel_distance, const lanelet::routing::RoutingGraph & graph,
  size_t * expanded_lanelets = nullptr, double * lane_end_distance = nullptr);

/// Suppress lane-change candidates whose origin lanelet has a lane-follow candidate, was judged
/// lane-follow, and (when keep_when_lane_ends) does not end within the rescue horizon.
std::vector<LaneletPathWithPathInfo> suppressUndetectedLaneChangePaths(
  const std::vector<LaneletPathWithPathInfo> & paths,
  const std::vector<LaneChangePathSuppressionInfo> & origins, bool enable, bool keep_when_lane_ends,
  std::vector<LaneletPathWithPathInfo> * suppressed_paths = nullptr);

/// Select held candidates whose origin lanelet has no surviving lane-follow prediction.
std::vector<LaneletPathWithPathInfo> selectPathsToRestore(
  const std::vector<LaneletPathWithPathInfo> & suppressed_paths,
  const std::set<lanelet::Id> & surviving_center_origins);
}  // namespace autoware::map_based_prediction::detail

#endif  // AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VEHICLE__LANE_CHANGE_PATH_SUPPRESSION_HPP_
