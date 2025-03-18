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

#include "autoware/simpl/conversion/lanelet.hpp"

#include "autoware/simpl/archetype/agent.hpp"
#include "autoware/simpl/archetype/map.hpp"

#include <geometry_msgs/msg/point.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/CompoundPolygon.h>
#include <lanelet2_core/primitives/LineString.h>

#include <cmath>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::simpl::conversion
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
}  // namespace

LaneletConverter::LaneletConverter(const lanelet::LaneletMapConstPtr lanelet_map_ptr)
: lanelet_map_ptr_(lanelet_map_ptr)
{
}

std::optional<std::vector<archetype::MapPoint>> LaneletConverter::convert(
  const archetype::AgentState & state_from, double distance_threshold) const
{
  std::vector<archetype::MapPoint> container;
  for (const auto & lanelet : lanelet_map_ptr_->laneletLayer) {
    const auto lanelet_subtype = to_subtype(lanelet);
    const auto label = MAP_LABEL_MAPPING.at(lanelet_subtype.value());
    if (is_lane_like(lanelet_subtype)) {
      // Convert centerlines
      if (is_roadway_like(lanelet_subtype)) {
        const auto points =
          from_linestring(lanelet.centerline3d(), label, state_from, distance_threshold);
        insert_lane_points(points, container);
      }
      // Convert boundaries except of virtual lines
      if (!is_turnable_intersection(lanelet)) {
        const auto left_bound = lanelet.leftBound3d();
        if (is_boundary_like(left_bound)) {
          const auto points = from_linestring(left_bound, label, state_from, distance_threshold);
          insert_lane_points(points, container);
        }
        const auto right_bound = lanelet.rightBound3d();
        if (is_boundary_like(right_bound)) {
          const auto points = from_linestring(right_bound, label, state_from, distance_threshold);
          insert_lane_points(points, container);
        }
      }
    } else if (is_crosswalk_like(lanelet_subtype)) {
      const auto points = from_polygon(lanelet.polygon3d(), label, state_from, distance_threshold);
      insert_lane_points(points, container);
    }
  }

  // parse linestring layers
  for (const auto & linestring : lanelet_map_ptr_->lineStringLayer) {
    if (is_boundary_like(linestring)) {
      const auto subtype = to_subtype(linestring);
      const auto label = MAP_LABEL_MAPPING.at(subtype.value());
      const auto points = from_linestring(linestring, label, state_from, distance_threshold);
      insert_lane_points(points, container);
    }
  }

  return container.size() == 0 ? std::nullopt : std::make_optional(container);
}

std::vector<archetype::MapPoint> LaneletConverter::from_linestring(
  const lanelet::ConstLineString3d & linestring, const archetype::MapLabel & label,
  const archetype::AgentState & state_from, double distance_threshold) const noexcept
{
  std::vector<archetype::MapPoint> output;
  for (auto itr = linestring.begin(); itr != linestring.end(); ++itr) {
    if (auto distance =
          std::hypot(itr->x() - state_from.x, itr->y() - state_from.y, itr->z() - state_from.z);
        distance > distance_threshold) {
      continue;
    }

    double dx, dy, dz;
    constexpr double epsilon = 1e-6;
    if (itr == linestring.begin()) {
      dx = 0.0;
      dy = 0.0;
      dz = 0.0;
    } else {
      dx = itr->x() - (itr - 1)->x();
      dy = itr->y() - (itr - 1)->y();
      dz = itr->z() - (itr - 1)->z();
      const auto norm = std::hypot(dx, dy, dz);
      dx /= (norm + epsilon);
      dy /= (norm + epsilon);
      dz /= (norm + epsilon);
    }
    output.emplace_back(itr->x(), itr->y(), itr->z(), dx, dy, dz, label);
  }
  return output;
}

std::vector<archetype::MapPoint> LaneletConverter::from_polygon(
  const lanelet::CompoundPolygon3d & polygon, const archetype::MapLabel & label,
  const archetype::AgentState & state_from, double distance_threshold) const noexcept
{
  std::vector<archetype::MapPoint> output;
  for (auto itr = polygon.begin(); itr != polygon.end(); ++itr) {
    if (auto distance =
          std::hypot(itr->x() - state_from.x, itr->y() - state_from.y, itr->z() - state_from.z);
        distance > distance_threshold) {
      continue;
    }

    double dx, dy, dz;
    constexpr double epsilon = 1e-6;
    if (itr == polygon.begin()) {
      dx = 0.0;
      dy = 0.0;
      dz = 0.0;
    } else {
      dx = itr->x() - (itr - 1)->x();
      dy = itr->y() - (itr - 1)->y();
      dz = itr->z() - (itr - 1)->z();
      const auto norm = std::hypot(dx, dy, dz);
      dx /= (norm + epsilon);
      dy /= (norm + epsilon);
      dz /= (norm + epsilon);
    }
    output.emplace_back(itr->x(), itr->y(), itr->z(), dx, dy, dz, label);
  }
  return output;
}
}  // namespace autoware::simpl::conversion
