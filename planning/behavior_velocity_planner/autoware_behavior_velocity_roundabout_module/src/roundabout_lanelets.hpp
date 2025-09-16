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

#ifndef ROUNDABOUT_LANELETS_HPP_
#define ROUNDABOUT_LANELETS_HPP_

#include <autoware/behavior_velocity_intersection_module/interpolated_path_info.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>

#include <lanelet2_core/primitives/CompoundPolygon.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_routing/Forward.h>

#include <optional>
#include <vector>

namespace autoware::behavior_velocity_planner
{

/**
 * @brief see the document for more details of RoundaboutLanelets
 */
struct RoundaboutLanelets
{
public:
  /**
   * update attention lanelets information
   */
  void update(
    const InterpolatedPathInfo & interpolated_path_info,
    const autoware_utils::LinearRing2d & footprint, const double vehicle_length,
    [[maybe_unused]] lanelet::routing::RoutingGraphPtr routing_graph_ptr);

  const lanelet::ConstLanelets & attention() const { return attention_; }
  const std::vector<std::optional<lanelet::ConstLineString3d>> & attention_stoplines() const
  {
    return attention_stoplines_;
  }
  const lanelet::ConstLanelets & adjacent() const { return adjacent_; }
  const lanelet::ConstLanelets & attention_non_preceding() const
  {
    return attention_non_preceding_;
  }
  const std::vector<lanelet::CompoundPolygon3d> & attention_area() const { return attention_area_; }
  const std::vector<lanelet::CompoundPolygon3d> & adjacent_area() const { return adjacent_area_; }
  const std::optional<lanelet::ConstLanelet> & first_attention_lane() const
  {
    return first_attention_lane_;
  }
  const std::optional<lanelet::CompoundPolygon3d> & first_attention_area() const
  {
    return first_attention_area_;
  }

  /**
   * the set of attention lanelets which is topologically merged
   */
  lanelet::ConstLanelets attention_;
  std::vector<lanelet::CompoundPolygon3d> attention_area_;

  /**
   * the stop lines for each attention_lanelets associated.
   */
  std::vector<std::optional<lanelet::ConstLineString3d>> attention_stoplines_;

  /**
   * the conflicting part of attention lanelets
   */
  lanelet::ConstLanelets attention_non_preceding_;
  std::vector<lanelet::CompoundPolygon3d> attention_non_preceding_area_;

  /**
   * the stop lines for each attention_non_preceding_
   */
  std::vector<std::optional<lanelet::ConstLineString3d>> attention_non_preceding_stoplines_;

  /**
   * the adjacent lanelets of the objective roundabout lanelet
   */
  lanelet::ConstLanelets adjacent_;
  std::vector<lanelet::CompoundPolygon3d> adjacent_area_;

  /**
   * the first attention lanelet which ego path points intersect for the first time
   */
  std::optional<lanelet::ConstLanelet> first_attention_lane_{std::nullopt};
  std::optional<lanelet::CompoundPolygon3d> first_attention_area_{std::nullopt};
};

/**
 * @brief Struct representing the lanelets along a path
 */
struct PathLanelets
{
  lanelet::ConstLanelet
    ego_or_entry2exit;  // this is `assigned lane` part of the path(not from
                        // ego) if ego is before the roundabout, otherwise from ego to exit
  lanelet::ConstLanelets all;
};
}  // namespace autoware::behavior_velocity_planner

#endif  // ROUNDABOUT_LANELETS_HPP_
