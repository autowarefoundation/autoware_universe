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

#include "autoware/map_based_prediction/predictor_vru/road_boundary.hpp"

#include "autoware/map_based_prediction/utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/object_recognition_utils/object_classification.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::map_based_prediction
{

namespace
{
// Crosswalk polygons are expanded by this margin before the containment test so that intersection
// points lying exactly on (or just outside) the crosswalk edge are still recognised as inside.
constexpr double kCrosswalkContainmentMargin = 0.5;  // [m]

std::optional<lanelet::ConstLanelet> find_crosswalk_at_point(
  const lanelet::LaneletMap * crosswalk_layer,
  const RoadBoundaryModule::CrosswalkPolygonMap & crosswalk_polygons,
  const lanelet::BasicPoint2d & point)
{
  if (!crosswalk_layer) {
    return std::nullopt;
  }
  const lanelet::BasicPoint2d margin(kCrosswalkContainmentMargin, kCrosswalkContainmentMargin);
  const lanelet::BoundingBox2d query(point - margin, point + margin);
  for (const auto & crosswalk : crosswalk_layer->laneletLayer.search(query)) {
    // distance() is 0 when the point is inside the polygon and the gap to the edge otherwise, so
    // this is equivalent to a within() test against a polygon expanded by the margin.
    if (
      boost::geometry::distance(point, crosswalk_polygons.at(crosswalk.id())) <=
      kCrosswalkContainmentMargin) {
      return crosswalk;
    }
  }
  return std::nullopt;
}

// A crossing on this segment counts as a cut point unless every intersection with the boundary is
// exempt. An intersection is exempt when it falls inside a crosswalk / walkway whose signal is not
// red (no signal or non-red) i.e. a legitimate crossing. An intersection outside any crosswalk, or
// inside a crosswalk with a red signal, is a cuttable jump-out.
bool segment_has_cuttable_crossing(
  const lanelet::BasicLineString2d & path_segment, const lanelet::BasicLineString2d & boundary_2d,
  const lanelet::LaneletMap * crosswalk_layer,
  const RoadBoundaryModule::CrosswalkPolygonMap & crosswalk_polygons,
  const RoadBoundaryModule::CrosswalkSignalRedFn & is_crosswalk_signal_red)
{
  lanelet::BasicPoints2d intersections;
  boost::geometry::intersection(path_segment, boundary_2d, intersections);
  // No points either for a disjoint segment or for a collinear overlap that cannot be resolved
  // into points; the latter falls back to cutting.
  if (intersections.empty()) {
    return boost::geometry::intersects(path_segment, boundary_2d);
  }
  for (const auto & intersection : intersections) {
    const auto crosswalk =
      find_crosswalk_at_point(crosswalk_layer, crosswalk_polygons, intersection);
    const bool exempt = crosswalk.has_value() && !is_crosswalk_signal_red(crosswalk.value());
    if (!exempt) {
      return true;
    }
  }
  return false;
}

std::optional<size_t> find_road_boundary_crossing_index(
  const lanelet::LaneletMap & road_boundary_layer, const lanelet::LaneletMap * crosswalk_layer,
  const RoadBoundaryModule::CrosswalkPolygonMap & crosswalk_polygons,
  const RoadBoundaryModule::CrosswalkSignalRedFn & is_crosswalk_signal_red,
  const PredictedPath & predicted_path)
{
  const auto & path = predicted_path.path;
  if (path.size() < 2) {
    return std::nullopt;
  }

  const auto predicted_path_ls = utils::to_linestring_2d(path, path.size() - 1);

  const auto candidates =
    road_boundary_layer.lineStringLayer.search(lanelet::geometry::boundingBox2d(predicted_path_ls));
  std::vector<lanelet::BasicLineString2d> crossed_boundaries{};
  for (const auto & candidate : candidates) {
    auto boundary_2d = lanelet::utils::to2D(candidate.basicLineString());
    if (boost::geometry::intersects(predicted_path_ls, boundary_2d)) {
      crossed_boundaries.push_back(std::move(boundary_2d));
    }
  }
  if (crossed_boundaries.empty()) {
    return std::nullopt;
  }

  for (auto i = 0UL; i + 1 < predicted_path_ls.size(); ++i) {
    const lanelet::BasicLineString2d path_segment(
      lanelet::BasicPoints2d{predicted_path_ls.at(i), predicted_path_ls.at(i + 1)});
    for (const auto & boundary_2d : crossed_boundaries) {
      if (
        segment_has_cuttable_crossing(
          path_segment, boundary_2d, crosswalk_layer, crosswalk_polygons,
          is_crosswalk_signal_red)) {
        return i;
      }
    }
  }
  return std::nullopt;
}
}  // namespace

void RoadBoundaryModule::build_from_map(std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr)
{
  road_boundary_layer_ = nullptr;
  crosswalk_layer_ = nullptr;
  crosswalk_polygons_.clear();
  if (!lanelet_map_ptr) {
    return;
  }

  std::unordered_map<lanelet::Id, std::pair<lanelet::LineString3d, size_t>> bound_usage;
  lanelet::ConstLanelets crosswalks;
  for (const auto & lanelet : lanelet_map_ptr->laneletLayer) {
    const std::string subtype = lanelet.attributeOr(lanelet::AttributeName::Subtype, "none");
    if (
      subtype == lanelet::AttributeValueString::Crosswalk ||
      subtype == lanelet::AttributeValueString::Walkway) {
      crosswalks.push_back(lanelet);
      continue;
    }
    if (subtype != lanelet::AttributeValueString::Road && subtype != "road_shoulder") {
      continue;
    }
    for (const auto & bound : {lanelet.leftBound(), lanelet.rightBound()}) {
      auto [it, inserted] = bound_usage.try_emplace(
        bound.id(),
        lanelet::LineString3d(std::const_pointer_cast<lanelet::LineStringData>(bound.constData())),
        0UL);
      ++it->second.second;
    }
  }
  // A bound shared by two road-subtype lanelets is an interior lane divider; a path coming from
  // outside the road always reaches an outer edge first, so dividers are never the first crossing
  // and are left out of the layer.
  lanelet::LineStrings3d boundaries;
  for (auto & [id, bound_with_usage] : bound_usage) {
    if (bound_with_usage.second == 1) {
      boundaries.push_back(bound_with_usage.first);
    }
  }
  road_boundary_layer_ = lanelet::utils::createMap(boundaries);
  crosswalk_layer_ = lanelet::utils::createConstMap(crosswalks, {});
  for (const auto & crosswalk : crosswalks) {
    crosswalk_polygons_.emplace(crosswalk.id(), crosswalk.polygon2d().basicPolygon());
  }
}

std::vector<PredictedPath> RoadBoundaryModule::cut_paths_crossing_road_boundary(
  const autoware_perception_msgs::msg::PredictedObject & predicted_object,
  const bool object_within_road, const CrosswalkSignalRedFn & is_crosswalk_signal_red) const
{
  std::vector<PredictedPath> cut_paths = predicted_object.kinematics.predicted_paths;
  // Objects already inside a road lanelet are left untouched so that a path exiting the road
  // (i.e. its first boundary crossing) is not mistaken for a jump-out and cut.
  if (!road_boundary_layer_ || object_within_road) {
    return cut_paths;
  }

  const auto & linear_velocity =
    predicted_object.kinematics.initial_twist_with_covariance.twist.linear;
  const double object_speed = std::hypot(linear_velocity.x, linear_velocity.y);
  const double deceleration = object_deceleration_params_.get(
    autoware::object_recognition_utils::getHighestProbLabel(predicted_object.classification));

  for (PredictedPath & predicted_path : cut_paths) {
    const std::optional<size_t> crossing_index = find_road_boundary_crossing_index(
      *road_boundary_layer_, crosswalk_layer_.get(), crosswalk_polygons_, is_crosswalk_signal_red,
      predicted_path);
    if (!crossing_index) {
      continue;
    }
    // Only cut when the object can decelerate to a stop before the boundary. If it cannot stop in
    // time, the jump-out onto the road is unavoidable and the full path is kept.
    const double distance_to_cross =
      autoware::motion_utils::calcSignedArcLength(predicted_path.path, 0, crossing_index.value());
    if (utils::distance_to_stop_with_deceleration(object_speed, deceleration) > distance_to_cross) {
      continue;
    }
    // Keep the last pose before the path crosses into the road. The crossing index is the start of
    // a segment, so it is always a valid index into the path.
    predicted_path.path.resize(crossing_index.value() + 1);
  }
  return cut_paths;
}

}  // namespace autoware::map_based_prediction
