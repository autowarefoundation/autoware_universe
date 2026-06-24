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

#include "autoware/multi_object_tracker/object_model/shapes_transform.hpp"

#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <tf2/utils.hpp>

#include <autoware_perception_msgs/msg/shape.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/geometry.hpp>

#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace
{
struct OrientedExtent
{
  double min_along, max_along, min_lat, max_lat;
};

// Project polygon footprint points onto a unit axis (cos_u, sin_u) and its perpendicular.
template <typename PointContainer>
inline OrientedExtent computeOrientedExtent(
  const PointContainer & points, const double cos_u, const double sin_u)
{
  OrientedExtent ext{
    std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest(),
    std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest()};
  for (const auto & p : points) {
    const double along = p.x * cos_u + p.y * sin_u;
    const double lat = -p.x * sin_u + p.y * cos_u;
    if (along < ext.min_along) ext.min_along = along;
    if (along > ext.max_along) ext.max_along = along;
    if (lat < ext.min_lat) ext.min_lat = lat;
    if (lat > ext.max_lat) ext.max_lat = lat;
  }
  return ext;
}
}  // namespace

namespace autoware::multi_object_tracker
{
namespace shapes
{

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
  double best_area = std::numeric_limits<double>::max();
  size_t best_i = 0;
  OrientedExtent best_ext{};
  bool found_any = false;

  auto tryEdge = [&](const size_t i) -> bool {
    const auto & p0 = points[i];
    const auto & p1 = points[(i + 1) % n];
    const double ex = p1.x - p0.x, ey = p1.y - p0.y;
    const double len_sq = ex * ex + ey * ey;
    if (len_sq < 1e-12) return false;
    const double edge_len = std::sqrt(len_sq);
    const auto ext = computeOrientedExtent(points, ex / edge_len, ey / edge_len);
    const double area = (ext.max_along - ext.min_along) * (ext.max_lat - ext.min_lat);
    if (area < best_area) {
      best_area = area;
      best_i = i;
      best_ext = ext;
      return true;
    }
    return false;
  };

  // Ego-facing pass: outward normal of CCW edge (ex,ey) is (ey,-ex).
  // Edge faces ego when (ey,-ex)·(ego_local_x,ego_local_y) > 0, i.e. ey*ego_local_x -
  // ex*ego_local_y > 0.
  if (use_ego) {
    for (size_t i = 0; i < n; ++i) {
      const auto & p0 = points[i];
      const auto & p1 = points[(i + 1) % n];
      const double ex = p1.x - p0.x;
      const double ey = p1.y - p0.y;
      if (ey * ego_local_x - ex * ego_local_y <= 0.0) continue;
      found_any |= tryEdge(i);
    }
  }

  // Fallback: full per-edge search (no ego, CW polygon, or no ego-facing edges).
  if (!found_any) {
    for (size_t i = 0; i < n; ++i) {
      found_any |= tryEdge(i);
    }
    if (!found_any) return false;
  }

  // Recover edge direction and bbox geometry from the winning edge.
  const auto & p0 = points[best_i];
  const auto & p1 = points[(best_i + 1) % n];
  const double ex = p1.x - p0.x, ey = p1.y - p0.y;
  const double edge_len = std::sqrt(ex * ex + ey * ey);
  const double cos_u = ex / edge_len, sin_u = ey / edge_len;

  const double dim_along = best_ext.max_along - best_ext.min_along;
  const double dim_perp = best_ext.max_lat - best_ext.min_lat;

  // Bbox center in local frame: inverse of the normalized projection.
  const double cu = (best_ext.min_along + best_ext.max_along) * 0.5;
  const double cv = (best_ext.min_lat + best_ext.max_lat) * 0.5;
  const double center_local_x = cu * cos_u - cv * sin_u;
  const double center_local_y = cu * sin_u + cv * cos_u;

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

  // Shift footprint to new center and rotate into new object frame (which is rotated by
  // bbox_yaw_local)
  for (auto & point : output_object.shape.footprint.points) {
    const float dx = point.x - static_cast<float>(center_local_x);
    const float dy = point.y - static_cast<float>(center_local_y);
    point.x = static_cast<float>(cos_u) * dx + static_cast<float>(sin_u) * dy;
    point.y = -static_cast<float>(sin_u) * dx + static_cast<float>(cos_u) * dy;
  }

  return true;
}

std::optional<types::DynamicObject> alignClusterToOrientation(
  const types::DynamicObject & cluster, const double target_yaw)
{
  if (
    cluster.shape.type != autoware_perception_msgs::msg::Shape::POLYGON ||
    cluster.shape.footprint.points.empty()) {
    return std::nullopt;
  }

  // Compose the two rotations (cluster local → map, map → target frame) into one.
  const double phi = target_yaw - tf2::getYaw(cluster.pose.orientation);
  const auto ext =
    computeOrientedExtent(cluster.shape.footprint.points, std::cos(phi), std::sin(phi));

  const double long_center = (ext.min_along + ext.max_along) * 0.5;
  const double lat_center = (ext.min_lat + ext.max_lat) * 0.5;
  const double cos_tr = std::cos(target_yaw), sin_tr = std::sin(target_yaw);

  types::DynamicObject aligned = cluster;

  // Convert aligned object to bounding box
  aligned.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  aligned.shape.footprint.points.clear();

  // Update position, orientation, and dimensions
  aligned.pose.position.x = cluster.pose.position.x + long_center * cos_tr - lat_center * sin_tr;
  aligned.pose.position.y = cluster.pose.position.y + long_center * sin_tr + lat_center * cos_tr;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, target_yaw);
  aligned.pose.orientation = tf2::toMsg(q);

  aligned.shape.dimensions.x = ext.max_along - ext.min_along;
  aligned.shape.dimensions.y = ext.max_lat - ext.min_lat;

  return aligned;
}

geometry_msgs::msg::Polygon unionFootprints(
  const geometry_msgs::msg::Polygon & a, const geometry_msgs::msg::Polygon & b)
{
  if (a.points.empty()) return b;
  if (b.points.empty()) return a;

  const auto to_boost = [](const geometry_msgs::msg::Polygon & fp) {
    autoware_utils_geometry::Polygon2d poly;
    for (const auto & p : fp.points) {
      poly.outer().emplace_back(p.x, p.y);
    }
    boost::geometry::correct(poly);
    return poly;
  };

  // Extract exterior ring into msg::Polygon; skip Boost's closing duplicate point.
  const auto to_msg = [](const autoware_utils_geometry::Polygon2d & poly) {
    geometry_msgs::msg::Polygon out;
    const auto & ring = poly.outer();
    const size_t n = ring.size() > 1u ? ring.size() - 1u : ring.size();
    out.points.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      geometry_msgs::msg::Point32 p;
      p.x = static_cast<float>(ring[i].x());
      p.y = static_cast<float>(ring[i].y());
      p.z = 0.0f;
      out.points.push_back(p);
    }
    return out;
  };

  const auto poly_a = to_boost(a);
  const auto poly_b = to_boost(b);

  std::vector<autoware_utils_geometry::Polygon2d> union_result;
  boost::geometry::union_(poly_a, poly_b, union_result);
  if (union_result.empty()) return a;

  // Single connected result — extract directly.
  if (union_result.size() == 1u) {
    return to_msg(union_result[0]);
  }

  // Disjoint components: compute convex hull of all component vertices so that both
  // footprints are covered without discarding the smaller one.
  autoware_utils_geometry::Polygon2d all_points;
  for (const auto & comp : union_result) {
    for (const auto & pt : comp.outer()) {
      all_points.outer().push_back(pt);
    }
  }
  autoware_utils_geometry::Polygon2d hull;
  boost::geometry::convex_hull(all_points, hull);
  return to_msg(hull);
}

geometry_msgs::msg::Polygon transformFootprint(
  const geometry_msgs::msg::Polygon & footprint, const geometry_msgs::msg::Pose & src_pose,
  const geometry_msgs::msg::Pose & dst_pose)
{
  const double src_yaw = tf2::getYaw(src_pose.orientation);
  const double dst_yaw = tf2::getYaw(dst_pose.orientation);
  const double d_yaw = src_yaw - dst_yaw;
  const double cos_d = std::cos(d_yaw);
  const double sin_d = std::sin(d_yaw);
  const double cos_dst = std::cos(dst_yaw);
  const double sin_dst = std::sin(dst_yaw);
  const double wx = src_pose.position.x - dst_pose.position.x;
  const double wy = src_pose.position.y - dst_pose.position.y;
  const double t_x = cos_dst * wx + sin_dst * wy;
  const double t_y = -sin_dst * wx + cos_dst * wy;

  geometry_msgs::msg::Polygon result;
  result.points.resize(footprint.points.size());
  for (size_t i = 0; i < footprint.points.size(); ++i) {
    const auto & p = footprint.points[i];
    result.points[i].x = static_cast<float>(cos_d * p.x - sin_d * p.y + t_x);
    result.points[i].y = static_cast<float>(sin_d * p.x + cos_d * p.y + t_y);
    result.points[i].z = p.z;
  }
  return result;
}

}  // namespace shapes

}  // namespace autoware::multi_object_tracker
