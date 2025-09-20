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

#include "autoware/simpl_prediction/conversion/lanelet.hpp"

#include "autoware/simpl_prediction/archetype/map.hpp"
#include "autoware/simpl_prediction/archetype/polyline.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::simpl_prediction::conversion
{
namespace
{
/**
 * @brief Key-value mapping of the lanelet subtype and `MapLabel`.
 */
const std::unordered_map<std::string, archetype::MapLabel> MAP_LABEL_MAPPING = {
  {"road", archetype::MapLabel::ROADWAY},
  {"highway", archetype::MapLabel::ROADWAY},
  {"road_shoulder", archetype::MapLabel::ROADWAY},
  {"bicycle_lane", archetype::MapLabel::BIKE_LANE},
  {"dashed", archetype::MapLabel::DASHED},
  {"solid", archetype::MapLabel::SOLID},
  {"dashed_dashed", archetype::MapLabel::DOUBLE_DASH},
  {"virtual", archetype::MapLabel::UNKNOWN},
  {"road_border", archetype::MapLabel::SOLID},
  {"crosswalk", archetype::MapLabel::CROSSWALK},
  {"unknown", archetype::MapLabel::UNKNOWN},
};

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
lanelet::Optional<std::string> to_subtype(const lanelet::ConstLanelet & lanelet) noexcept
{
  return lanelet.hasAttribute("subtype") ? lanelet.attribute("subtype").as<std::string>()
                                         : lanelet::Optional<std::string>();
}

/**
 * @brief Try to retrieve the `subtype` attribute of the linestring object.
 *
 * @param linestring LineString object.
 */
lanelet::Optional<std::string> to_subtype(const lanelet::ConstLineString3d & linestring) noexcept
{
  return linestring.hasAttribute("subtype") ? linestring.attribute("subtype").as<std::string>()
                                            : lanelet::Optional<std::string>();
}

archetype::MapLabel to_boundary_label(const lanelet::ConstLineString3d & linestring)
{
  if (auto t_type = to_type(linestring); t_type && MAP_LABEL_MAPPING.count(*t_type)) {
    return MAP_LABEL_MAPPING.at(*t_type);
  } else if (auto t_subtype = to_subtype(linestring);
             t_subtype && MAP_LABEL_MAPPING.count(*t_subtype)) {
    return MAP_LABEL_MAPPING.at(*t_subtype);
  } else {
    return archetype::MapLabel::UNKNOWN;
  }
}

/**
 * @brief Check if the specified lanelet subtype is kind of the roadway.
 *
 * @param subtype Subtype of the corresponding lanelet.
 * @return True if the subtype is the one of the (road, highway, road_shoulder).
 */
bool is_roadway_like(const lanelet::Optional<std::string> & subtype) noexcept
{
  if (!subtype) {
    return false;
  }
  const auto & subtype_str = subtype.value();
  return subtype_str == "road" || subtype_str == "highway" || subtype_str == "road_shoulder" ||
         subtype_str == "bicycle_lane";
}

/**
 * @brief Check if the specified linestring is kind of the boundary.
 *
 * @param linestring 3D linestring.
 * @return True if the type is the one of the (line_thin, line_thick, road_boarder, virtual).
 */
bool is_boundary_like(const lanelet::ConstLineString3d & linestring)
{
  const auto type = to_type(linestring);
  if (!type) {
    return false;
  }

  const auto & type_str = type.value();
  return (
    type_str == "line_thin" || type_str == "line_thick" || type_str == "road_border" ||
    type_str == "virtual");
}

/**
 * @brief Check if the specified linestring is the kind of crosswalk.
 *
 * @param subtype Subtype of the corresponding polygon.
 * @return True if the lanelet subtype is the one of the (crosswalk,).
 */
bool is_crosswalk_like(const lanelet::Optional<std::string> & subtype)
{
  if (!subtype) {
    return false;
  }

  const auto & subtype_str = subtype.value();
  return subtype_str == "crosswalk";
}

/**
 * @brief Check if the specified lanelet id has already been taken and contained in the set of ids.
 *
 * @param taken_boundary_ids Vector of taken lanelet ids.
 * @param id Target lanelet id.
 * @return True if the target lanelet id is contained in the taken boundary ids.
 */
bool is_taken_boundary(const lanelet::Ids & taken_boundary_ids, const lanelet::Id & id)
{
  return std::find(taken_boundary_ids.begin(), taken_boundary_ids.end(), id) !=
         taken_boundary_ids.end();
}

archetype::Polyline interpolate_waypoints(
  const archetype::Polyline & input, size_t num_waypoint = 20)
{
  if (input.size() < 2 || num_waypoint < 2) {
    return input;
  }

  // 1. compute cumulative distances
  std::vector<double> cumulative_distances(input.size(), 0.0);
  for (size_t i = 1; i < input.size(); ++i) {
    cumulative_distances[i] = cumulative_distances[i - 1] + input[i].distance_from(input[i - 1]);
  }
  const auto & total_distance = cumulative_distances.back();

  // 2. generate target arc lengths
  std::vector<double> target_distances(num_waypoint, 0.0);
  double step = total_distance / static_cast<double>(num_waypoint - 1);
  for (size_t i = 0; i < num_waypoint; ++i) {
    target_distances[i] = static_cast<double>(i) * step;
  }

  // 3. interpolate new points
  std::vector<archetype::MapPoint> waypoints;
  size_t segment_idx = 0;
  for (const auto & target : target_distances) {
    // move to the correct segment
    while (segment_idx + 1 < cumulative_distances.size() &&
           cumulative_distances[segment_idx + 1] < target) {
      ++segment_idx;
    }

    // interpolate between input[segment_idx] and input[segment_idx + 1]
    const double & start = cumulative_distances[segment_idx];
    const double & end = cumulative_distances[segment_idx + 1];
    double denom = end - start;
    double t = (std::abs(denom) > 1e-3) ? (target - start) / denom : 0.0;
    waypoints.emplace_back(input[segment_idx].lerp(input[segment_idx + 1], t));
  }

  return archetype::Polyline(input.id(), waypoints);
}
}  // namespace

void LaneletConverter::convert(const lanelet::LaneletMapConstPtr lanelet_map_ptr)
{
  std::lock_guard<std::mutex> lock(container_mtx_);
  container_.clear();
  lanelet::Ids taken_boundary_ids;
  for (const auto & lanelet : lanelet_map_ptr->laneletLayer) {
    const auto lanelet_subtype = to_subtype(lanelet);
    if (!lanelet_subtype || MAP_LABEL_MAPPING.count(lanelet_subtype.value()) == 0) {
      continue;
    }
    const auto label = MAP_LABEL_MAPPING.at(lanelet_subtype.value());
    if (is_roadway_like(lanelet_subtype)) {
      // convert centerlines
      const auto roadway_points = from_linestring(lanelet.centerline3d(), label);
      container_.emplace_back(interpolate_waypoints(roadway_points));

      // left boundary
      const auto left_bound = lanelet.leftBound3d();
      if (!is_taken_boundary(taken_boundary_ids, left_bound.id())) {
        const auto bound_points = from_linestring(left_bound, to_boundary_label(left_bound));
        container_.emplace_back(interpolate_waypoints(bound_points));
        taken_boundary_ids.emplace_back(left_bound.id());
      }

      // right boundary
      const auto right_bound = lanelet.rightBound3d();
      if (!is_taken_boundary(taken_boundary_ids, right_bound.id())) {
        const auto bound_points = from_linestring(right_bound, to_boundary_label(right_bound));
        container_.emplace_back(interpolate_waypoints(bound_points));
        taken_boundary_ids.emplace_back(right_bound.id());
      }
    } else if (is_crosswalk_like(lanelet_subtype)) {
      const auto points = from_polygon(lanelet.id(), lanelet.polygon3d(), label);
      container_.emplace_back(interpolate_waypoints(points));
    }
  }

  // parse linestring layers
  for (const auto & linestring : lanelet_map_ptr->lineStringLayer) {
    if (is_boundary_like(linestring) && !is_taken_boundary(taken_boundary_ids, linestring.id())) {
      const auto points = from_linestring(linestring, to_boundary_label(linestring));
      container_.emplace_back(interpolate_waypoints(points));
      taken_boundary_ids.emplace_back(linestring.id());
    }
  }
}

std::optional<std::vector<archetype::Polyline>> LaneletConverter::polylines()
{
  std::lock_guard<std::mutex> lock(container_mtx_);
  return container_.empty() ? std::nullopt : std::make_optional(container_);
}

archetype::Polyline LaneletConverter::from_linestring(
  const lanelet::ConstLineString3d & linestring, const archetype::MapLabel & label) const noexcept
{
  std::vector<archetype::MapPoint> waypoints;
  for (auto itr = linestring.begin(); itr != linestring.end(); ++itr) {
    waypoints.emplace_back(itr->x(), itr->y(), itr->z(), label);
  }
  return archetype::Polyline(linestring.id(), waypoints);
}

archetype::Polyline LaneletConverter::from_polygon(
  lanelet::Id id, const lanelet::CompoundPolygon3d & polygon,
  const archetype::MapLabel & label) const noexcept
{
  std::vector<archetype::MapPoint> waypoints;
  for (auto itr = polygon.begin(); itr != polygon.end(); ++itr) {
    waypoints.emplace_back(itr->x(), itr->y(), itr->z(), label);
  }
  return archetype::Polyline(id, waypoints);
}
}  // namespace autoware::simpl_prediction::conversion
