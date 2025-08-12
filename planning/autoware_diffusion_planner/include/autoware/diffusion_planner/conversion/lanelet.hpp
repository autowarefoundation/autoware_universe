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

#ifndef AUTOWARE__DIFFUSION_PLANNER__CONVERSION__LANELET_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__CONVERSION__LANELET_HPP_

#include "autoware/diffusion_planner/polyline.hpp"

#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <geometry_msgs/msg/detail/point__struct.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/CompoundPolygon.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/utility/Optional.h>

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner
{
using TrafficSignal = autoware_perception_msgs::msg::TrafficLightGroup;
using TrafficSignalArray = autoware_perception_msgs::msg::TrafficLightGroupArray;
using TrafficLightIdMap = std::unordered_map<lanelet::Id, TrafficSignal>;
using autoware_perception_msgs::msg::TrafficLightElement;

/**
 * @brief Insert lane points into the container from the end of it.
 *
 * @param points Sequence of points to be inserted.
 * @param container Points container.
 */
inline void insert_lane_points(
  const std::vector<LanePoint> & points, std::vector<LanePoint> & container)
{
  container.reserve(container.size() * 2);
  container.insert(container.end(), points.begin(), points.end());
}

inline lanelet::Optional<std::string> to_type_name(const lanelet::ConstLanelet & lanelet)
{
  return lanelet.hasAttribute("type") ? lanelet.attribute("type").as<std::string>()
                                      : lanelet::Optional<std::string>();
}

inline lanelet::Optional<std::string> to_type_name(const lanelet::ConstLineString3d & linestring)
{
  return linestring.hasAttribute("type") ? linestring.attribute("type").as<std::string>()
                                         : lanelet::Optional<std::string>();
}

/**
 * @brief Extract the subtype name from a lanelet.
 *
 * @param lanelet Lanelet instance.
 * @return std::optional<string>
 */
inline lanelet::Optional<std::string> to_subtype_name(
  const lanelet::ConstLanelet & lanelet) noexcept
{
  return lanelet.hasAttribute("subtype") ? lanelet.attribute("subtype").as<std::string>()
                                         : lanelet::Optional<std::string>();
}

/**
 * @brief Extract the subtype name from a 3D linestring.
 *
 * @param linestring 3D linestring instance.
 * @return lanelet::Optional<std::string>
 */
inline lanelet::Optional<std::string> to_subtype_name(
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
  return lanelet.hasAttribute("turn_direction");
}

/**
 * @brief Check if the specified lanelet subtype is kind of lane.
 *
 * @param subtype
 * @return True if the lanelet subtype is the one of the (road, highway, road_shoulder,
 * pedestrian_lane, bicycle_lane, walkway).
 */
inline bool is_lane_like(const lanelet::Optional<std::string> & subtype)
{
  if (!subtype) {
    return false;
  }
  const auto & subtype_str = subtype.value();
  return (
    subtype_str == "road" || subtype_str == "highway" || subtype_str == "road_shoulder" ||
    subtype_str == "bicycle_lane");
  // subtype_str == "pedestrian_lane" || subtype_str == "bicycle_lane" || subtype_str == "walkway"
}

/**
 * @brief Check if the specified lanelet subtype is kind of the roadway.
 *
 * @param subtype Subtype of the corresponding lanelet.
 * @return True if the subtype is the one of the (road, highway, road_shoulder).
 */
inline bool is_roadway_like(const lanelet::Optional<std::string> & subtype)
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
  const auto type = to_type_name(linestring);
  if (!type) {
    return false;
  }

  const auto & type_str = type.value();
  return (
    type_str == "line_thin" || type_str == "line_thick" || type_str == "road_boarder" ||
    type_str == "virtual");
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

struct LaneSegment
{
  int64_t id;
  Polyline polyline;
  bool is_intersection{false};
  std::vector<BoundarySegment> left_boundaries;
  std::vector<BoundarySegment> right_boundaries;
  std::optional<float> speed_limit_mps{std::nullopt};

  LaneSegment(
    int64_t id, Polyline polyline, bool is_intersection,
    const std::vector<BoundarySegment> & left_boundaries,
    const std::vector<BoundarySegment> & right_boundaries, std::optional<float> speed_limit_mps)
  : id(id),
    polyline(std::move(polyline)),
    is_intersection(is_intersection),
    left_boundaries(left_boundaries),
    right_boundaries(right_boundaries),
    speed_limit_mps(speed_limit_mps)
  {
  }
};

/**
 * @brief A class to convert lanelet map to polyline.
 */
class LaneletConverter
{
public:
  /**
   * @brief Construct a new Lanelet Converter object
   *
   * @param lanelet_map_ptr Pointer of loaded lanelet map.
   */
  explicit LaneletConverter(const lanelet::LaneletMapConstPtr lanelet_map_ptr)
  : lanelet_map_ptr_(lanelet_map_ptr)
  {
  }

  /**
   * @brief Convert a lanelet map to line segment data
   * @return std::vector<LaneSegment>
   */
  [[nodiscard]] std::vector<LaneSegment> convert_to_lane_segments(
    const int64_t num_lane_points) const;

  /**
   * @brief Convert a linestring to the set of polylines.
   *
   * @param linestring Linestring instance.
   * @param position Origin to check the distance from this.
   * @param distance_threshold Distance threshold from the specified position.
   * @return std::vector<LanePoint>
   */
  [[nodiscard]] static std::vector<LanePoint> from_linestring(
    const lanelet::ConstLineString3d & linestring, const geometry_msgs::msg::Point & position,
    double distance_threshold) noexcept;

  [[nodiscard]] static std::vector<LanePoint> from_linestring(
    const lanelet::ConstLineString3d & linestring) noexcept;

  /**
   * @brief Convert a polygon to the set of polylines.
   *
   * @param polygon Polygon instance.
   * @param position Origin to check the distance from this.
   * @param distance_threshold Distance threshold from the specified position.
   * @return std::vector<LanePoint>
   */
  [[nodiscard]] static std::vector<LanePoint> from_polygon(
    const lanelet::CompoundPolygon3d & polygon, const geometry_msgs::msg::Point & position,
    double distance_threshold) noexcept;

  [[nodiscard]] static std::vector<LanePoint> from_polygon(
    const lanelet::CompoundPolygon3d & polygon) noexcept;

private:
  /**
   * @brief Convert any geometry type to the set of lane points.
   *
   * @tparam GeometryType The type of the geometry (e.g., LineString3d, CompoundPolygon3d).
   * @param geometry Geometry instance.
   * @param position Origin to check the distance from this.
   * @param distance_threshold Distance threshold from the specified position.
   * @return std::vector<LanePoint>
   */
  template <typename GeometryType>
  [[nodiscard]] static std::vector<LanePoint> from_geometry(
    const GeometryType & geometry, const geometry_msgs::msg::Point & position,
    double distance_threshold) noexcept;

  /**
   * @brief Convert any geometry type to the set of lane points.
   *
   * @tparam GeometryType The type of the geometry (e.g., LineString3d, CompoundPolygon3d).
   * @param geometry Geometry instance.
   * @return std::vector<LanePoint>
   */
  template <typename GeometryType>
  [[nodiscard]] static std::vector<LanePoint> from_geometry(const GeometryType & geometry) noexcept;

  lanelet::LaneletMapConstPtr lanelet_map_ptr_;  //!< Pointer of lanelet map.
};

std::vector<LanePoint> interpolate_points(const std::vector<LanePoint> & input, size_t num_points);
}  // namespace autoware::diffusion_planner

#endif  // AUTOWARE__DIFFUSION_PLANNER__CONVERSION__LANELET_HPP_
