// Copyright 2024 Tier IV, Inc.
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

#include "autoware/multi_object_tracker/object_model/shapes.hpp"

#include <Eigen/Geometry>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <tf2/utils.hpp>

#include <autoware_perception_msgs/msg/shape.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace autoware::multi_object_tracker
{
namespace shapes
{
inline double getSumArea(const std::vector<autoware_utils_geometry::Polygon2d> & polygons)
{
  return std::accumulate(
    polygons.begin(), polygons.end(), 0.0, [](double acc, autoware_utils_geometry::Polygon2d p) {
      return acc + boost::geometry::area(p);
    });
}

inline double getIntersectionArea(
  const autoware_utils_geometry::Polygon2d & source_polygon,
  const autoware_utils_geometry::Polygon2d & target_polygon)
{
  std::vector<autoware_utils_geometry::Polygon2d> intersection_polygons;
  boost::geometry::intersection(source_polygon, target_polygon, intersection_polygons);
  return getSumArea(intersection_polygons);
}

inline double getUnionArea(
  const autoware_utils_geometry::Polygon2d & source_polygon,
  const autoware_utils_geometry::Polygon2d & target_polygon)
{
  std::vector<autoware_utils_geometry::Polygon2d> union_polygons;
  boost::geometry::union_(source_polygon, target_polygon, union_polygons);
  return getSumArea(union_polygons);
}

inline double getConvexShapeArea(
  const autoware_utils_geometry::Polygon2d & source_polygon,
  const autoware_utils_geometry::Polygon2d & target_polygon)
{
  boost::geometry::model::multi_polygon<autoware_utils_geometry::Polygon2d> union_polygons;
  boost::geometry::union_(source_polygon, target_polygon, union_polygons);

  autoware_utils_geometry::Polygon2d hull;
  boost::geometry::convex_hull(union_polygons, hull);
  return boost::geometry::area(hull);
}

double get1dIoU(
  const types::DynamicObject & source_object, const types::DynamicObject & target_object)
{
  constexpr double min_union_length = 0.1;  // As 0.01 used in 2dIoU, use 0.1 here
  constexpr double min_length = 1e-3;       // As 1e-6 used in 2dIoU, use 1e-3 here
  // Compute radii from dimensions (use max of x and y as diameter)
  const double r_src =
    std::max(source_object.shape.dimensions.x, source_object.shape.dimensions.y) * 0.5;
  const double r_tgt =
    std::max(target_object.shape.dimensions.x, target_object.shape.dimensions.y) * 0.5;
  // if radius is smaller than the minimum length, return 0.0
  if (r_src < min_length || r_tgt < min_length) return 0.0;
  // Ensure r1 is the larger radius
  const double r1 = std::max(r_tgt, r_src);
  const double r2 = std::min(r_tgt, r_src);
  const auto dx = source_object.pose.position.x - target_object.pose.position.x;
  const auto dy = source_object.pose.position.y - target_object.pose.position.y;
  // distance between centers
  const auto dist = std::sqrt(dx * dx + dy * dy);
  // if distance is larger than the sum of radius, return 0.0
  if (dist > r1 + r2 - min_length) return 0.0;
  // if distance is smaller than the difference of radius, return the ratio of the smaller radius to
  // the larger radius
  // Square used to mimic area ratio behavior as a rough 2D approximation
  if (dist < r1 - r2) return (r2 * r2) / (r1 * r1);
  // if distance is between the difference and the sum of radii, return the ratio of the
  // intersection length to the union length
  if (r1 + r2 + dist < min_union_length) return 0.0;
  const double intersection_length = r1 + r2 - dist;
  const double iou = intersection_length * r2 / (r1 * r1) * 0.5;
  return iou;
}

double get2dIoU(
  const types::DynamicObject & source_object, const types::DynamicObject & target_object,
  const double min_union_area)
{
  static const double MIN_AREA = 1e-6;

  const auto source_polygon =
    autoware_utils_geometry::to_polygon2d(source_object.pose, source_object.shape);
  if (boost::geometry::area(source_polygon) < MIN_AREA) return 0.0;
  const auto target_polygon =
    autoware_utils_geometry::to_polygon2d(target_object.pose, target_object.shape);
  if (boost::geometry::area(target_polygon) < MIN_AREA) return 0.0;

  const double intersection_area = getIntersectionArea(source_polygon, target_polygon);
  if (intersection_area < MIN_AREA) return 0.0;
  const double union_area = getUnionArea(source_polygon, target_polygon);

  const double iou =
    union_area < min_union_area ? 0.0 : std::min(1.0, intersection_area / union_area);
  return iou;
}

double get2dGeneralizedIoU(
  const types::DynamicObject & source_object, const types::DynamicObject & target_object)
{
  static const double MIN_AREA = 1e-6;

  const auto source_polygon =
    autoware_utils_geometry::to_polygon2d(source_object.pose, source_object.shape);
  const double source_area = boost::geometry::area(source_polygon);
  const auto target_polygon =
    autoware_utils_geometry::to_polygon2d(target_object.pose, target_object.shape);
  const double target_area = boost::geometry::area(target_polygon);
  if (source_area < MIN_AREA && target_area < MIN_AREA) return -1.0;

  const double intersection_area = getIntersectionArea(source_polygon, target_polygon);
  const double union_area = getUnionArea(source_polygon, target_polygon);
  const double iou = union_area < 0.01 ? 0.0 : std::min(1.0, intersection_area / union_area);
  const double convex_shape_area = getConvexShapeArea(source_polygon, target_polygon);

  return iou - (convex_shape_area - union_area) / convex_shape_area;
}

bool get2dPrecisionRecallGIoU(
  const types::DynamicObject & source_object, const types::DynamicObject & target_object,
  double & precision, double & recall, double & generalized_iou)
{
  static const double MIN_AREA = 1e-6;

  const auto source_polygon =
    autoware_utils_geometry::to_polygon2d(source_object.pose, source_object.shape);
  const double source_area = boost::geometry::area(source_polygon);
  if (source_area < MIN_AREA) return false;
  const auto target_polygon =
    autoware_utils_geometry::to_polygon2d(target_object.pose, target_object.shape);
  const double target_area = boost::geometry::area(target_polygon);
  if (target_area < MIN_AREA) return false;

  const double intersection_area = getIntersectionArea(source_polygon, target_polygon);
  const double union_area = getUnionArea(source_polygon, target_polygon);
  const double convex_shape_area = getConvexShapeArea(source_polygon, target_polygon);
  const double iou = union_area < 0.01 ? 0.0 : std::min(1.0, intersection_area / union_area);

  precision = source_area < MIN_AREA ? 0.0 : std::min(1.0, intersection_area / source_area);
  recall = source_area < MIN_AREA ? 0.0 : std::min(1.0, intersection_area / target_area);
  generalized_iou = iou - (convex_shape_area - union_area) / convex_shape_area;

  return true;
}

/**
 * @brief convert convex hull shape object to bounding box object
 * @param input_object: input convex hull objects
 * @param output_object: output bounding box objects
 */
bool convertConvexHullToBoundingBox(
  const types::DynamicObject & input_object, types::DynamicObject & output_object)
{
  // check footprint size
  const auto & points = input_object.shape.footprint.points;
  if (points.size() < 3) {
    return false;
  }

  // Pre-allocate boundary values using first point
  float max_x = points[0].x;
  float max_y = points[0].y;
  float max_z = points[0].z;
  float min_x = points[0].x;
  float min_y = points[0].y;
  float min_z = points[0].z;

  // Start from second point since we used first point for initialization
  for (size_t i = 1; i < points.size(); ++i) {
    const auto & point = points[i];
    // Use direct comparison instead of std::max/min
    if (point.x > max_x) max_x = point.x;
    if (point.y > max_y) max_y = point.y;
    if (point.z > max_z) max_z = point.z;
    if (point.x < min_x) min_x = point.x;
    if (point.y < min_y) min_y = point.y;
    if (point.z < min_z) min_z = point.z;
  }

  // calc new center in local coordinates - avoid division by 2.0 twice
  const double center_x = (max_x + min_x) * 0.5;
  const double center_y = (max_y + min_y) * 0.5;

  // transform to global for the object's position
  const double yaw = tf2::getYaw(input_object.pose.orientation);
  const double cos_yaw = cos(yaw);
  const double sin_yaw = sin(yaw);
  const double dx = center_x * cos_yaw - center_y * sin_yaw;
  const double dy = center_x * sin_yaw + center_y * cos_yaw;

  // set output parameters - avoid unnecessary copying
  output_object = input_object;
  output_object.pose.position.x += dx;
  output_object.pose.position.y += dy;

  output_object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  output_object.shape.dimensions.x = max_x - min_x;
  output_object.shape.dimensions.y = max_y - min_y;
  output_object.shape.dimensions.z = max_z - min_z;

  // adjust footprint points in local coordinates - use references to avoid copies
  for (auto & point : output_object.shape.footprint.points) {
    point.x -= center_x;
    point.y -= center_y;
  }

  return true;
}

}  // namespace shapes

}  // namespace autoware::multi_object_tracker
