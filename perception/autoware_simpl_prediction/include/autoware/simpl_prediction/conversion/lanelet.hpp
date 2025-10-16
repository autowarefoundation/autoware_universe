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

#ifndef AUTOWARE__SIMPL_PREDICTION__CONVERSION__LANELET_HPP_
#define AUTOWARE__SIMPL_PREDICTION__CONVERSION__LANELET_HPP_

#include "autoware/simpl_prediction/archetype/polyline.hpp"

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/CompoundPolygon.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/utility/Optional.h>

#include <mutex>
#include <optional>
#include <vector>

namespace autoware::simpl_prediction::conversion
{
/**
 * @brief This class responsible for converting lanelet objects to `PolylineData`.
 */
class LaneletConverter
{
public:
  /**
   * @brief Convert a lanelet map to the polyline data.
   *
   * @param lanelet_map_ptr Read-only pointer to lanelet map.
   */
  void convert(const lanelet::LaneletMapConstPtr lanelet_map_ptr);

  /**
   * @brief Return polylines if inner container contains any polylines.
   */
  std::optional<std::vector<archetype::Polyline>> polylines();

private:
  /**
   * @brief Convert a linestring to the set of polylines.
   *
   * @param linestring Linestring instance.
   * @param label Label.
   */
  archetype::Polyline from_linestring(
    const lanelet::ConstLineString3d & linestring,
    const archetype::MapLabel & label) const noexcept;

  /**
   * @brief Convert a polygon to the set of polylines.
   *
   * @param polygon Polygon instance.
   * @param label Label.
   */
  archetype::Polyline from_polygon(
    lanelet::Id id, const lanelet::CompoundPolygon3d & polygon,
    const archetype::MapLabel & label) const noexcept;

  std::vector<archetype::Polyline> container_;  //!< Vector of converted polylines.
  std::mutex container_mtx_;                    //!< Mutex guard for map points container.
};
}  // namespace autoware::simpl_prediction::conversion
#endif  // AUTOWARE__SIMPL_PREDICTION__CONVERSION__LANELET_HPP_
