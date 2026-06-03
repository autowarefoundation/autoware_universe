// Copyright 2024 TIER IV, Inc.
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
#include <utility>
#include <vector>

namespace
{
constexpr double MIN_AREA = 1e-6;
constexpr double INVALID_SCORE = -1.0;
}  // namespace

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

bool convertConvexHullToBoundingBox(
  const types::DynamicObject & input_object, types::DynamicObject & output_object,
  const std::optional<geometry_msgs::msg::Point> & ego_pos)
{
  const auto & points = input_object.shape.footprint.points;
  if (points.size() < 3) {
    return false;
  }

  // Transform ego position into the object's local frame for the ego-facing edge filter.
  // Footprint points are defined in local frame (object-relative 2D coords).
  double ego_local_x = 0.0, ego_local_y = 0.0;
  const bool use_ego = ego_pos.has_value();
  if (use_ego) {
    const double yaw = tf2::getYaw(input_object.pose.orientation);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const double dgx = ego_pos->x - input_object.pose.position.x;
    const double dgy = ego_pos->y - input_object.pose.position.y;
    ego_local_x = cos_yaw * dgx + sin_yaw * dgy;
    ego_local_y = -sin_yaw * dgx + cos_yaw * dgy;
  }

  const size_t n = points.size();
  double best_scaled_area = std::numeric_limits<double>::max();
  size_t best_i = 0;
  double best_min_u = 0.0, best_max_u = 0.0, best_min_v = 0.0, best_max_v = 0.0;
  bool found_any = false;

  auto tryEdge = [&](const size_t i) {
    const auto & p0 = points[i];
    const auto & p1 = points[(i + 1) % n];
    const double ex = p1.x - p0.x;
    const double ey = p1.y - p0.y;
    const double len_sq = ex * ex + ey * ey;
    if (len_sq < 1e-12) return;

    double min_u = std::numeric_limits<double>::max();
    double max_u = std::numeric_limits<double>::lowest();
    double min_v = std::numeric_limits<double>::max();
    double max_v = std::numeric_limits<double>::lowest();
    for (const auto & p : points) {
      const double u = p.x * ex + p.y * ey;
      const double v = -p.x * ey + p.y * ex;
      if (u < min_u) min_u = u;
      if (u > max_u) max_u = u;
      if (v < min_v) min_v = v;
      if (v > max_v) max_v = v;
    }
    // scaled_area = actual_area * len_sq; compare without sqrt
    const double scaled_area = (max_u - min_u) * (max_v - min_v) / len_sq;
    if (scaled_area < best_scaled_area) {
      best_scaled_area = scaled_area;
      best_i = i;
      best_min_u = min_u;
      best_max_u = max_u;
      best_min_v = min_v;
      best_max_v = max_v;
      found_any = true;
    }
  };

  // Ego-facing pass: outward normal of CCW edge (ex,ey) is (ey,-ex).
  // Edge faces ego when (ey,-ex)·(ego_local_x,ego_local_y) > 0, i.e. ey*ego_local_x - ex*ego_local_y > 0.
  if (use_ego) {
    for (size_t i = 0; i < n; ++i) {
      const auto & p0 = points[i];
      const auto & p1 = points[(i + 1) % n];
      const double ex = p1.x - p0.x;
      const double ey = p1.y - p0.y;
      if (ey * ego_local_x - ex * ego_local_y <= 0.0) continue;
      tryEdge(i);
    }
  }

  // Fallback: full per-edge search (no ego, CW polygon, or no ego-facing edges).
  if (!found_any) {
    for (size_t i = 0; i < n; ++i) {
      tryEdge(i);
    }
  }

  if (!found_any) return false;

  // Recover actual dimensions and center from the winning edge (one sqrt + one atan2).
  const auto & p0 = points[best_i];
  const auto & p1 = points[(best_i + 1) % n];
  const double ex = p1.x - p0.x;
  const double ey = p1.y - p0.y;
  const double edge_len = std::sqrt(ex * ex + ey * ey);
  const double len_sq = ex * ex + ey * ey;

  // Bbox dimensions in local frame
  const double dim_along = (best_max_u - best_min_u) / edge_len;
  const double dim_perp = (best_max_v - best_min_v) / edge_len;

  // Bbox center in local frame: invert the unnormalized projection transform.
  // [x; y] = (1/len_sq) * [[ex, -ey]; [ey, ex]] * [cu; cv]
  const double cu = (best_min_u + best_max_u) * 0.5;
  const double cv = (best_min_v + best_max_v) * 0.5;
  const double center_local_x = (ex * cu - ey * cv) / len_sq;
  const double center_local_y = (ey * cu + ex * cv) / len_sq;

  // Winning edge orientation in local frame → global yaw offset
  const double bbox_yaw_local = std::atan2(ey, ex);

  const double obj_yaw = tf2::getYaw(input_object.pose.orientation);
  const double cos_yaw = std::cos(obj_yaw);
  const double sin_yaw = std::sin(obj_yaw);

  output_object = input_object;

  // Rotate local center offset to global and add to object position
  output_object.pose.position.x += cos_yaw * center_local_x - sin_yaw * center_local_y;
  output_object.pose.position.y += sin_yaw * center_local_x + cos_yaw * center_local_y;

  // Set global bbox orientation (object yaw + edge yaw in local frame)
  const double half = (obj_yaw + bbox_yaw_local) * 0.5;
  output_object.pose.orientation.x = 0.0;
  output_object.pose.orientation.y = 0.0;
  output_object.pose.orientation.z = std::sin(half);
  output_object.pose.orientation.w = std::cos(half);

  output_object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  output_object.shape.dimensions.x = dim_along;
  output_object.shape.dimensions.y = dim_perp;

  // Shift footprint to be relative to new center (still in original local frame axes)
  for (auto & point : output_object.shape.footprint.points) {
    point.x -= static_cast<float>(center_local_x);
    point.y -= static_cast<float>(center_local_y);
  }

  return true;
}

std::pair<double, double> getObjectZRange(const types::DynamicObject & object)
{
  const double center_z = object.pose.position.z;
  const double height = object.shape.dimensions.z;
  const double min_z = center_z - height / 2.0;
  const double max_z = center_z + height / 2.0;
  return {min_z, max_z};
}

double get3dGeneralizedIoU(
  const types::DynamicObject & source_object, const types::DynamicObject & target_object)
{
  const auto source_polygon =
    autoware_utils_geometry::to_polygon2d(source_object.pose, source_object.shape);
  if (boost::geometry::area(source_polygon) < MIN_AREA) return INVALID_SCORE;
  const auto target_polygon =
    autoware_utils_geometry::to_polygon2d(target_object.pose, target_object.shape);
  if (boost::geometry::area(target_polygon) < MIN_AREA) return INVALID_SCORE;

  const double union_area = getUnionArea(source_polygon, target_polygon);
  if (union_area < MIN_AREA) return INVALID_SCORE;

  const double intersection_area = getIntersectionArea(source_polygon, target_polygon);
  const double convex_area = getConvexShapeArea(source_polygon, target_polygon);

  const auto [z_min_src, z_max_src] = getObjectZRange(source_object);
  const auto [z_min_tgt, z_max_tgt] = getObjectZRange(target_object);

  const double height_overlap =
    std::max(0.0, std::min(z_max_src, z_max_tgt) - std::max(z_min_src, z_min_tgt));

  if (height_overlap <= 0.0) return INVALID_SCORE;

  const double total_height = std::max(z_max_src, z_max_tgt) - std::min(z_min_src, z_min_tgt);

  const double iou =
    std::clamp((intersection_area * height_overlap) / (union_area * total_height), 0.0, 1.0);

  return iou - (convex_area - union_area) / convex_area;
}

}  // namespace shapes

}  // namespace autoware::multi_object_tracker
