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

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::map_based_prediction
{

namespace
{
double calc_footprint_search_margin(const autoware_perception_msgs::msg::Shape & shape)
{
  if (shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    const auto hx = shape.dimensions.x * 0.5;
    const auto hy = shape.dimensions.y * 0.5;
    return std::hypot(hx, hy);
  }
  if (shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
    return shape.dimensions.x * 0.5;
  }
  return std::max(shape.dimensions.x, shape.dimensions.y) * 0.5;
}

bool has_required_info(const autoware_perception_msgs::msg::PredictedObject & predicted_object)
{
  using autoware_perception_msgs::msg::Shape;
  if (predicted_object.kinematics.predicted_paths.empty()) {
    return false;
  }
  const auto & shape = predicted_object.shape;
  switch (shape.type) {
    case Shape::BOUNDING_BOX:
      return shape.dimensions.x > 0.0 && shape.dimensions.y > 0.0;
    case Shape::CYLINDER:
      return shape.dimensions.x > 0.0;
    case Shape::POLYGON:
      return !shape.footprint.points.empty();
    default:
      return false;
  }
}

std::vector<autoware_utils_geometry::Polygon2d> collect_candidate_vegetation_polygons(
  const lanelet::LaneletMap & vegetation_layer, const std::vector<PredictedPath> & predicted_paths,
  const autoware_perception_msgs::msg::Shape & object_shape)
{
  lanelet::BoundingBox2d search_bbox;
  const auto search_margin = calc_footprint_search_margin(object_shape);
  const lanelet::BasicPoint2d offset(search_margin, search_margin);
  for (const auto & predicted_path : predicted_paths) {
    for (const auto & pose : predicted_path.path) {
      const lanelet::BasicPoint2d center(pose.position.x, pose.position.y);
      search_bbox.extend(center - offset);
      search_bbox.extend(center + offset);
    }
  }

  const auto candidates = vegetation_layer.polygonLayer.search(search_bbox);
  std::vector<autoware_utils_geometry::Polygon2d> vegetation_polygons_2d;
  vegetation_polygons_2d.reserve(candidates.size());
  for (const auto & candidate : candidates) {
    autoware_utils_geometry::Polygon2d polygon;
    boost::geometry::convert(lanelet::utils::to2D(candidate.basicPolygon()), polygon);
    boost::geometry::correct(polygon);
    vegetation_polygons_2d.push_back(polygon);
  }
  return vegetation_polygons_2d;
}

std::optional<size_t> find_vegetation_crossing_index(
  const PredictedPath & predicted_path, const autoware_perception_msgs::msg::Shape & object_shape,
  const std::vector<autoware_utils_geometry::Polygon2d> & vegetation_polygons_2d)
{
  if (vegetation_polygons_2d.empty()) {
    return std::nullopt;
  }
  for (auto i = 0UL; i < predicted_path.path.size(); ++i) {
    const autoware_utils_geometry::Polygon2d footprint =
      autoware_utils_geometry::to_polygon2d(predicted_path.path.at(i), object_shape);
    for (const auto & vegetation_polygon : vegetation_polygons_2d) {
      // NOTE: intersects_convex (GJK) treats both polygons as convex. A non-convex vegetation area
      // is evaluated as its convex hull, but this works effectively.
      if (autoware_utils_geometry::intersects_convex(footprint, vegetation_polygon)) {
        return i;
      }
    }
  }
  return std::nullopt;
}
}  // namespace

void VegetationModule::build_from_map(std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr)
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

std::vector<PredictedPath> VegetationModule::cut_paths_crossing_vegetation(
  const autoware_perception_msgs::msg::PredictedObject & predicted_object) const
{
  std::vector<PredictedPath> cut_paths = predicted_object.kinematics.predicted_paths;
  if (!has_required_info(predicted_object) || !vegetation_layer_) {
    return cut_paths;
  }

  const autoware_perception_msgs::msg::Shape & object_shape = predicted_object.shape;
  const std::vector<autoware_utils_geometry::Polygon2d> candidate_polygons =
    collect_candidate_vegetation_polygons(*vegetation_layer_, cut_paths, object_shape);

  for (PredictedPath & predicted_path : cut_paths) {
    const std::optional<size_t> crossing_index =
      find_vegetation_crossing_index(predicted_path, object_shape, candidate_polygons);
    if (crossing_index) {
      predicted_path.path.resize(std::max<size_t>(*crossing_index, 1UL));
      // resize keep [0, crossing_index-1]: the crossing point and the rest are dropped
    }
  }
  return cut_paths;
}
}  // namespace autoware::map_based_prediction
