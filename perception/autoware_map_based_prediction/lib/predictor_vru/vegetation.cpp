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

#include "autoware/map_based_prediction/predictor_vru/vegetation.hpp"

#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/logging.hpp>

#include <autoware_perception_msgs/msg/shape.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::map_based_prediction
{

namespace
{
using Point2d = autoware_utils_geometry::Point2d;
using Polygon2d = autoware_utils_geometry::Polygon2d;

autoware_utils_geometry::Polygon2d toPolygon2d(const lanelet::ConstPolygon3d & lanelet_polygon)
{
  autoware_utils_geometry::Polygon2d polygon;
  boost::geometry::convert(lanelet::utils::to2D(lanelet_polygon.basicPolygon()), polygon);
  boost::geometry::correct(polygon);
  return polygon;
}

// Radius of the smallest origin-centred circle covering the footprint.
double footprintRadius(const autoware_perception_msgs::msg::Shape & shape)
{
  using autoware_perception_msgs::msg::Shape;
  if (shape.type == Shape::CYLINDER) {
    return shape.dimensions.x * 0.5;
  }
  if (shape.type == Shape::POLYGON) {
    double radius = 0.0;
    for (const auto & point : shape.footprint.points) {
      radius = std::max(radius, static_cast<double>(std::hypot(point.x, point.y)));
    }
    return radius;
  }
  return std::hypot(shape.dimensions.x, shape.dimensions.y) * 0.5;
}

Polygon2d convexPolygonCoveringFootprints(
  const Polygon2d & start_polygon, const Polygon2d & end_polygon)
{
  boost::geometry::model::multi_point<Point2d> points;
  for (const auto & point : start_polygon.outer()) {
    points.push_back(point);
  }
  for (const auto & point : end_polygon.outer()) {
    points.push_back(point);
  }

  Polygon2d hull;
  boost::geometry::convex_hull(points, hull);
  boost::geometry::correct(hull);
  return hull;
}

// Centerline bounding box grown by the footprint radius, so every polygon the swept footprint
// can reach is inside the queried area.
lanelet::BoundingBox2d searchBox(
  const lanelet::BasicLineString2d & predicted_path_ls,
  const autoware_perception_msgs::msg::Shape & object_shape)
{
  const auto bbox = lanelet::geometry::boundingBox2d(predicted_path_ls);
  const double radius = footprintRadius(object_shape);
  const lanelet::BasicPoint2d margin(radius, radius);
  return {bbox.min() - margin, bbox.max() + margin};
}

// The linestring holds the path positions up to and including last_idx.
std::optional<size_t> findVegetationCrossingIndex(
  const PredictedPath & predicted_path, const lanelet::BasicLineString2d & predicted_path_ls,
  const autoware_perception_msgs::msg::Shape & object_shape,
  const std::vector<const Polygon2d *> & vegetation_polygons, const size_t last_idx)
{
  const auto & path = predicted_path.path;
  if (path.size() < 2 || vegetation_polygons.empty()) {
    return std::nullopt;
  }

  const double footprint_radius = footprintRadius(object_shape);
  const bool is_cylinder = object_shape.type == autoware_perception_msgs::msg::Shape::CYLINDER;
  // The swept footprint of a cylinder is the centerline buffered by its radius, so distance
  // tests against the centerline are exact; other shapes use them as a conservative gate.
  const auto initial_footprint =
    is_cylinder ? Polygon2d{} : autoware_utils_geometry::to_polygon2d(path.front(), object_shape);

  std::vector<const Polygon2d *> candidate_polygons;
  for (const auto * vegetation_polygon : vegetation_polygons) {
    if (boost::geometry::distance(predicted_path_ls, *vegetation_polygon) > footprint_radius) {
      continue;
    }
    // Object already overlapping vegetation: its path leaves the area, so no crossing is reported.
    const bool initially_overlapping =
      is_cylinder ? boost::geometry::distance(predicted_path_ls.front(), *vegetation_polygon) <=
                      footprint_radius
                  : boost::geometry::intersects(initial_footprint, *vegetation_polygon);
    if (initially_overlapping) {
      return std::nullopt;
    }
    candidate_polygons.push_back(vegetation_polygon);
  }
  if (candidate_polygons.empty()) {
    return std::nullopt;
  }

  // Segment loop outermost so the crossing closest to the object is the one reported.
  if (is_cylinder) {
    for (auto i = 0UL; i + 1 <= last_idx; ++i) {
      const lanelet::BasicLineString2d segment(
        lanelet::BasicPoints2d{predicted_path_ls.at(i), predicted_path_ls.at(i + 1)});
      for (const auto * candidate_polygon : candidate_polygons) {
        if (boost::geometry::distance(segment, *candidate_polygon) <= footprint_radius) {
          return i;
        }
      }
    }
    return std::nullopt;
  }

  auto segment_start_footprint = initial_footprint;
  for (auto i = 0UL; i + 1 <= last_idx; ++i) {
    auto segment_end_footprint =
      autoware_utils_geometry::to_polygon2d(path.at(i + 1), object_shape);
    const auto swept_polygon =
      convexPolygonCoveringFootprints(segment_start_footprint, segment_end_footprint);
    for (const auto * candidate_polygon : candidate_polygons) {
      if (boost::geometry::intersects(swept_polygon, *candidate_polygon)) {
        return i;
      }
    }
    segment_start_footprint = std::move(segment_end_footprint);
  }
  return std::nullopt;
}

bool doesPathCrossVegetation(
  const PredictedPathWithArrivalIndex & predicted_path,
  const lanelet::BasicLineString2d & predicted_path_ls,
  const autoware_perception_msgs::msg::Shape & object_shape,
  const std::vector<const Polygon2d *> & vegetation_polygons)
{
  if (predicted_path.path.size() < 2) {
    return false;
  }
  const size_t last_idx = std::min(predicted_path.arrival_index, predicted_path.path.size() - 1);
  return findVegetationCrossingIndex(
           predicted_path, predicted_path_ls, object_shape, vegetation_polygons, last_idx)
    .has_value();
}
}  // namespace

void VegetationModule::buildFromMap(std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr)
{
  vegetation_layer_ = nullptr;
  polygons_2d_.clear();
  if (!lanelet_map_ptr) {
    return;
  }

  lanelet::Polygons3d vegetations;
  for (const auto & polygon : lanelet_map_ptr->polygonLayer) {
    const std::string type = polygon.attributeOr(lanelet::AttributeName::Type, "none");
    const std::string subtype = polygon.attributeOr(lanelet::AttributeName::Subtype, "none");
    if (type != "area" || subtype != "vegetation") {
      continue;
    }
    auto polygon_2d = toPolygon2d(polygon);
    boost::geometry::unique(polygon_2d);
    // A closed outer ring needs three distinct vertices plus the closing point.
    if (polygon_2d.outer().size() < 4 || !boost::geometry::is_valid(polygon_2d)) {
      RCLCPP_WARN(
        rclcpp::get_logger("map_based_prediction.vegetation"),
        "Skipping invalid vegetation polygon %ld in the lanelet map", polygon.id());
      continue;
    }
    polygons_2d_.emplace(polygon.id(), std::move(polygon_2d));
    vegetations.emplace_back(std::const_pointer_cast<lanelet::LineStringData>(polygon.constData()));
  }
  // The layer stays null when nothing is left, so per-path queries short-circuit.
  if (!vegetations.empty()) {
    vegetation_layer_ = lanelet::utils::createMap(vegetations);
  }
}

std::vector<const Polygon2d *> VegetationModule::lookupCachedPolygons(
  const lanelet::ConstPolygons3d & candidates) const
{
  std::vector<const Polygon2d *> polygons;
  polygons.reserve(candidates.size());
  for (const auto & candidate : candidates) {
    polygons.push_back(&polygons_2d_.at(candidate.id()));
  }
  return polygons;
}

bool VegetationModule::doesPathCrossAnyVegetationBeforeCrosswalk(
  const PredictedPathWithArrivalIndex & predicted_path,
  const lanelet::BasicLineString2d & predicted_path_ls,
  const autoware_perception_msgs::msg::Shape & object_shape) const
{
  if (!vegetation_layer_ || predicted_path_ls.empty()) {
    return false;
  }
  const auto candidates =
    vegetation_layer_->polygonLayer.search(searchBox(predicted_path_ls, object_shape));
  return doesPathCrossVegetation(
    predicted_path, predicted_path_ls, object_shape, lookupCachedPolygons(candidates));
}

PredictedPath VegetationModule::cutPathsCrossingVegetation(
  const PredictedPath & predicted_path, const lanelet::BasicLineString2d & predicted_path_ls,
  const autoware_perception_msgs::msg::Shape & object_shape) const
{
  PredictedPath cut_path = predicted_path;
  if (cut_path.path.empty() || !vegetation_layer_) {
    return cut_path;
  }

  const auto candidates =
    vegetation_layer_->polygonLayer.search(searchBox(predicted_path_ls, object_shape));
  const std::optional<size_t> crossing_index = findVegetationCrossingIndex(
    predicted_path, predicted_path_ls, object_shape, lookupCachedPolygons(candidates),
    predicted_path.path.size() - 1);
  if (crossing_index) {
    cut_path.path.resize(std::min(*crossing_index + 1, cut_path.path.size()));
  }
  return cut_path;
}
}  // namespace autoware::map_based_prediction
