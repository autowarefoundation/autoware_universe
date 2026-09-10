// Copyright 2026 TIER IV, inc.
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

#ifndef AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__ROAD_BOUNDARY_HPP_
#define AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__ROAD_BOUNDARY_HPP_

#include "autoware/map_based_prediction/path_generator/path_generator.hpp"
#include "autoware/map_based_prediction/utils.hpp"

#include <autoware_perception_msgs/msg/predicted_objects.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

namespace autoware::map_based_prediction
{

/// Trims VRU predicted paths that jump out onto the road at the right/left boundary of
/// road-subtype lanelets. Objects that are already inside a road lanelet are left untouched.
class RoadBoundaryModule
{
public:
  RoadBoundaryModule() = default;

  /// @pre lanelet_map_ptr is non-null when building from a map; nullptr clears the layer.
  void build_from_map(std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr);

  void set_object_deceleration(const utils::ObjectDecelerationParams & params)
  {
    object_deceleration_params_ = params;
  }

  /// Predicate returning true when the given crosswalk / walkway lanelet has a red signal.
  using CrosswalkSignalRedFn = std::function<bool(const lanelet::ConstLanelet &)>;

  /// Crosswalk rings converted once at map load, keyed by the lanelet id.
  using CrosswalkPolygonMap = std::unordered_map<lanelet::Id, lanelet::BasicPolygon2d>;

  /// Return the object's predicted paths trimmed where they first cross a road boundary.
  /// When @p object_within_road is true the paths are returned unchanged. A path is cut only when
  /// the object can decelerate to a stop before the boundary; otherwise the jump-out is treated as
  /// unavoidable and the path is kept.
  ///
  /// A crossing that falls inside a crosswalk / walkway is normally exempt (kept), because it
  /// represents a legitimate crossing. The exemption is overridden when @p is_crosswalk_signal_red
  /// reports the crosswalk's signal as red: in that case the jump-out is cut like any other.
  [[nodiscard]] std::vector<PredictedPath> cut_paths_crossing_road_boundary(
    const autoware_perception_msgs::msg::PredictedObject & predicted_object,
    bool object_within_road, const CrosswalkSignalRedFn & is_crosswalk_signal_red) const;

private:
  lanelet::LaneletMapConstUPtr road_boundary_layer_{nullptr};
  // Crosswalk / walkway lanelets. A boundary crossing that falls inside one of these is treated as
  // a legitimate crosswalk crossing and is not cut.
  lanelet::LaneletMapConstUPtr crosswalk_layer_{nullptr};
  CrosswalkPolygonMap crosswalk_polygons_;
  utils::ObjectDecelerationParams object_deceleration_params_{};
};

}  // namespace autoware::map_based_prediction

#endif  // AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__ROAD_BOUNDARY_HPP_
