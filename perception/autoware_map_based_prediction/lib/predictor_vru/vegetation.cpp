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

#include <autoware_perception_msgs/msg/shape.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::map_based_prediction
{

namespace
{
using Point2d = autoware_utils_geometry::Point2d;
using Polygon2d = autoware_utils_geometry::Polygon2d;
using Box2d = autoware_utils_geometry::Box2d;

autoware_utils_geometry::Polygon2d toPolygon2d(const lanelet::ConstPolygon3d & lanelet_polygon)
{
  autoware_utils_geometry::Polygon2d polygon;
  boost::geometry::convert(lanelet::utils::to2D(lanelet_polygon.basicPolygon()), polygon);
  boost::geometry::correct(polygon);
  return polygon;
}

Polygon2d convexPolygonCoveringSegmentFootprints(
  const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & end_pose,
  const autoware_perception_msgs::msg::Shape & shape)
{
  const auto start_polygon = autoware_utils_geometry::to_polygon2d(start_pose, shape);
  const auto end_polygon = autoware_utils_geometry::to_polygon2d(end_pose, shape);

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

Box2d buildPathFootprintBBox(
  const std::vector<geometry_msgs::msg::Pose> & path,
  const autoware_perception_msgs::msg::Shape & object_shape, const size_t last_idx)
{
  boost::geometry::model::multi_point<Point2d> path_footprint_points;
  for (auto i = 0UL; i <= last_idx; ++i) {
    const auto footprint = autoware_utils_geometry::to_polygon2d(path.at(i), object_shape);
    for (const auto & point : footprint.outer()) {
      path_footprint_points.push_back(point);
    }
  }
  return boost::geometry::return_envelope<Box2d>(path_footprint_points);
}

std::optional<size_t> findVegetationCrossingIndex(
  const PredictedPath & predicted_path, const autoware_perception_msgs::msg::Shape & object_shape,
  const lanelet::ConstPolygons3d & vegetation_polygons, const size_t last_idx)
{
  const auto & path = predicted_path.path;
  if (path.size() < 2 || vegetation_polygons.empty()) {
    return std::nullopt;
  }

  const auto path_bbox = buildPathFootprintBBox(path, object_shape, last_idx);
  const auto initial_footprint = autoware_utils_geometry::to_polygon2d(path.front(), object_shape);

  std::vector<Polygon2d> candidate_polygons;
  for (const auto & vegetation_polygon : vegetation_polygons) {
    const auto vegetation_polygon_2d = toPolygon2d(vegetation_polygon);
    if (!boost::geometry::intersects(path_bbox, vegetation_polygon_2d)) {
      continue;
    }
    // Object already overlapping vegetation: its path leaves the area, so no crossing is reported.
    if (boost::geometry::intersects(initial_footprint, vegetation_polygon_2d)) {
      return std::nullopt;
    }
    candidate_polygons.push_back(vegetation_polygon_2d);
  }
  if (candidate_polygons.empty()) {
    return std::nullopt;
  }

  // Segment loop outermost so the crossing closest to the object is the one reported.
  for (auto i = 0UL; i + 1 <= last_idx; ++i) {
    const auto swept_polygon =
      convexPolygonCoveringSegmentFootprints(path.at(i), path.at(i + 1), object_shape);
    for (const auto & candidate_polygon : candidate_polygons) {
      if (boost::geometry::intersects(swept_polygon, candidate_polygon)) {
        return i;
      }
    }
  }
  return std::nullopt;
}

bool doesPathCrossVegetation(
  const PredictedPathWithArrivalIndex & predicted_path,
  const autoware_perception_msgs::msg::Shape & object_shape,
  const lanelet::ConstPolygons3d & vegetation_polygons)
{
  if (predicted_path.path.size() < 2) {
    return false;
  }
  const size_t last_idx = std::min(predicted_path.arrival_index, predicted_path.path.size() - 1);
  return findVegetationCrossingIndex(predicted_path, object_shape, vegetation_polygons, last_idx)
    .has_value();
}
}  // namespace

void VegetationModule::buildFromMap(std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr)
{
  if (!lanelet_map_ptr) {
    vegetation_layer_ = nullptr;
    return;
  }

  lanelet::Polygons3d vegetations;
  for (const auto & polygon : lanelet_map_ptr->polygonLayer) {
    const std::string type = polygon.attributeOr(lanelet::AttributeName::Type, "none");
    const std::string subtype = polygon.attributeOr(lanelet::AttributeName::Subtype, "none");
    if (type == "area" && subtype == "vegetation") {
      vegetations.emplace_back(
        std::const_pointer_cast<lanelet::LineStringData>(polygon.constData()));
    }
  }
  vegetation_layer_ = lanelet::utils::createMap(vegetations);
}

bool VegetationModule::doesPathCrossAnyVegetationBeforeCrosswalk(
  const PredictedPathWithArrivalIndex & predicted_path,
  const autoware_perception_msgs::msg::Shape & object_shape) const
{
  if (!vegetation_layer_) {
    return false;
  }
  if (predicted_path.path.empty()) {
    return false;
  }
  lanelet::BasicLineString2d predicted_path_ls;
  const size_t last_idx = std::min(predicted_path.arrival_index, predicted_path.path.size() - 1);
  for (auto i = 0UL; i <= last_idx; ++i) {
    const auto & pt = predicted_path.path.at(i);
    predicted_path_ls.emplace_back(pt.position.x, pt.position.y);
  }
  const auto candidates =
    vegetation_layer_->polygonLayer.search(lanelet::geometry::boundingBox2d(predicted_path_ls));
  return doesPathCrossVegetation(predicted_path, object_shape, candidates);
}

PredictedPath VegetationModule::cutPathsCrossingVegetation(
  const PredictedPath & predicted_path,
  const autoware_perception_msgs::msg::Shape & object_shape) const
{
  PredictedPath cut_path = predicted_path;
  if (cut_path.path.empty() || !vegetation_layer_) {
    return cut_path;
  }

  lanelet::BasicLineString2d predicted_path_ls;
  for (const auto & pt : predicted_path.path) {
    predicted_path_ls.emplace_back(pt.position.x, pt.position.y);
  }
  const auto candidates =
    vegetation_layer_->polygonLayer.search(lanelet::geometry::boundingBox2d(predicted_path_ls));
  const std::optional<size_t> crossing_index = findVegetationCrossingIndex(
    predicted_path, object_shape, candidates, predicted_path.path.size() - 1);
  if (crossing_index) {
    cut_path.path.resize(std::min(*crossing_index + 1, cut_path.path.size()));
  }
  return cut_path;
}
}  // namespace autoware::map_based_prediction
