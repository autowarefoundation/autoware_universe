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

#ifndef AUTOWARE__SIMPL__CONVERSION__LANELET_HPP_
#define AUTOWARE__SIMPL__CONVERSION__LANELET_HPP_

#include "autoware/simpl/archetype/agent.hpp"
#include "autoware/simpl/archetype/map.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/CompoundPolygon.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/utility/Optional.h>

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace autoware::simpl::conversion
{
/**
 * @brief Insert lane points into the container from the end of it.
 *
 * @param points Sequence of points to be inserted.
 * @param container Points container.
 */
inline void insert_lane_points(
  const std::vector<archetype::MapPoint> & points, std::vector<archetype::MapPoint> & container)
{
  container.reserve(container.size() * 2);
  container.insert(container.end(), points.begin(), points.end());
}

/**
 * @brief Try to retrieve the `type` attribute of the lanelet object.
 *
 * @param lanelet Lanelet object.
 */
inline lanelet::Optional<std::string> to_type(const lanelet::ConstLanelet & lanelet) noexcept
{
  return lanelet.hasAttribute("type") ? lanelet.attribute("type").as<std::string>()
                                      : lanelet::Optional<std::string>();
}

/**
 * @brief Try to retrieve the `type` attribute of the linestring object.
 *
 * @param linestring LineString object.
 */
inline lanelet::Optional<std::string> to_type(
  const lanelet::ConstLineString3d & linestring) noexcept
{
  return linestring.hasAttribute("type") ? linestring.attribute("type").as<std::string>()
                                         : lanelet::Optional<std::string>();
}

/**
 * @brief Try to retrieve the `subtype` attribute of the lanelet object.
 *
 * @param lanelet Lanelet object.
 */
inline lanelet::Optional<std::string> to_subtype(const lanelet::ConstLanelet & lanelet) noexcept
{
  return lanelet.hasAttribute("subtype") ? lanelet.attribute("subtype").as<std::string>()
                                         : lanelet::Optional<std::string>();
}

/**
 * @brief Try to retrieve the `subtype` attribute of the linestring object.
 *
 * @param linestring LineString object.
 */
inline lanelet::Optional<std::string> to_subtype(
  const lanelet::ConstLineString3d & linestring) noexcept
{
  return linestring.hasAttribute("subtype") ? linestring.attribute("subtype").as<std::string>()
                                            : lanelet::Optional<std::string>();
}

/**
 * @brief Check if the specified lanelet is the turnable intersection.
 *
 * @param lanelet Lanelet instance.
 * @return true if the lanelet has the attribute named turn_direction.
 */
inline bool is_turnable_intersection(const lanelet::ConstLanelet & lanelet) noexcept
{
  return lanelet.hasAttribute("turn_direction") &&
         lanelet.attribute("turn_direction").as<std::string>().value() != "straight";
}

/**
 * @brief Check if the specified lanelet subtype is kind of lane.
 *
 * @param subtype
 * @return True if the lanelet subtype is the one of the (road, highway, road_shoulder,
 * pedestrian_lane, bicycle_lane, walkway).
 */
inline bool is_lane_like(const lanelet::Optional<std::string> & subtype) noexcept
{
  if (!subtype) {
    return false;
  }
  const auto & subtype_str = subtype.value();
  return (
    subtype_str == "road" || subtype_str == "highway" || subtype_str == "road_shoulder" ||
    subtype_str == "pedestrian_lane" || subtype_str == "bicycle_lane" || subtype_str == "walkway");
}

/**
 * @brief Check if the specified lanelet subtype is kind of the roadway.
 *
 * @param subtype Subtype of the corresponding lanelet.
 * @return True if the subtype is the one of the (road, highway, road_shoulder).
 */
inline bool is_roadway_like(const lanelet::Optional<std::string> & subtype) noexcept
{
  if (!subtype) {
    return false;
  }
  const auto & subtype_str = subtype.value();
  return subtype_str == "road" || subtype_str == "highway" || subtype_str == "road_shoulder";
}

/**
 * @brief Check if the specified linestring is kind of the boundary.
 *
 * @param linestring 3D linestring.
 * @return True if the type is the one of the (line_thin, line_thick, road_boarder) and the subtype
 * is not virtual.
 */
inline bool is_boundary_like(const lanelet::ConstLineString3d & linestring)
{
  const auto type = to_type(linestring);
  const auto subtype = to_subtype(linestring);
  if (!type || !subtype) {
    return false;
  }

  const auto & type_str = type.value();
  const auto & subtype_str = subtype.value();
  return (type_str == "line_thin" || type_str == "line_thick" || type_str == "road_boarder") &&
         subtype_str != "virtual";
}

/**
 * @brief Check if the specified linestring is the kind of crosswalk.
 *
 * @param subtype Subtype of the corresponding polygon.
 * @return True if the lanelet subtype is the one of the (crosswalk,).
 */
inline bool is_crosswalk_like(const lanelet::Optional<std::string> & subtype)
{
  if (!subtype) {
    return false;
  }

  const auto & subtype_str = subtype.value();
  return subtype_str == "crosswalk";
}

/**
 * @brief This class responsible for converting lanelet objects to `PolylineData`.
 */
class LaneletConverter
{
public:
  /**
   * @brief Construct a new LaneletConverter object.
   *
   * @param lanelet_map_ptr Pointer to the lanelet map.
   */
  explicit LaneletConverter(const lanelet::LaneletMapConstPtr lanelet_map_ptr);

  /**
   * @brief Convert a lanelet map to the polyline data except of points whose distance from the
   * specified position is farther than the threshold.
   *
   * @param state_from Origin to check the distance from this.
   * @param distance_threshold Distance threshold
   * @return std::optional<archetype::MapPoint>
   */
  std::optional<std::vector<archetype::MapPoint>> convert(
    const archetype::AgentState & position_from, double distance_threshold) const;

private:
  /**
   * @brief Convert a linestring to the set of polylines.
   *
   * @param linestring Linestring instance.
   * @param label Label.
   * @param state_from Origin to check the distance from this.
   * @param distance_threshold Distance threshold from the specified position.
   * @return std::vector<archetype::MapPoint>
   */
  std::vector<archetype::MapPoint> from_linestring(
    const lanelet::ConstLineString3d & linestring, const archetype::MapLabel & label,
    const archetype::AgentState & state_from, double distance_threshold) const noexcept;

  /**
   * @brief Convert a polygon to the set of polylines.
   *
   * @param polygon Polygon instance.
   * @param label Label.
   * @param state_from Origin to check the distance from this.
   * @param distance_threshold Distance threshold from the specified position.
   * @return std::vector<archetype::MapPoint>
   */
  std::vector<archetype::MapPoint> from_polygon(
    const lanelet::CompoundPolygon3d & polygon, const archetype::MapLabel & label,
    const archetype::AgentState & state_from, double distance_threshold) const noexcept;

  lanelet::LaneletMapConstPtr lanelet_map_ptr_;  //!< Pointer to the lanelet map.
};
}  // namespace autoware::simpl::conversion
#endif  // AUTOWARE__SIMPL__CONVERSION__LANELET_HPP_
