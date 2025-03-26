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
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>

#include <autoware_perception_msgs/msg/shape.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/geometry.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

namespace autoware::multi_object_tracker
{
namespace shapes
{
inline double getSumArea(const std::vector<autoware_utils::Polygon2d> & polygons)
{
  return std::accumulate(
    polygons.begin(), polygons.end(), 0.0,
    [](double acc, autoware_utils::Polygon2d p) { return acc + boost::geometry::area(p); });
}

inline double getIntersectionArea(
  const autoware_utils::Polygon2d & source_polygon,
  const autoware_utils::Polygon2d & target_polygon)
{
  std::vector<autoware_utils::Polygon2d> intersection_polygons;
  boost::geometry::intersection(source_polygon, target_polygon, intersection_polygons);
  return getSumArea(intersection_polygons);
}

inline double getUnionArea(
  const autoware_utils::Polygon2d & source_polygon,
  const autoware_utils::Polygon2d & target_polygon)
{
  std::vector<autoware_utils::Polygon2d> union_polygons;
  boost::geometry::union_(source_polygon, target_polygon, union_polygons);
  return getSumArea(union_polygons);
}

double get2dIoU(
  const types::DynamicObject & source_object, const types::DynamicObject & target_object,
  const double min_union_area)
{
  static const double MIN_AREA = 1e-6;

  const auto source_polygon = autoware_utils::to_polygon2d(source_object.pose, source_object.shape);
  if (boost::geometry::area(source_polygon) < MIN_AREA) return 0.0;
  const auto target_polygon = autoware_utils::to_polygon2d(target_object.pose, target_object.shape);
  if (boost::geometry::area(target_polygon) < MIN_AREA) return 0.0;

  const double intersection_area = getIntersectionArea(source_polygon, target_polygon);
  if (intersection_area < MIN_AREA) return 0.0;
  const double union_area = getUnionArea(source_polygon, target_polygon);

  const double iou =
    union_area < min_union_area ? 0.0 : std::min(1.0, intersection_area / union_area);
  return iou;
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
  if (input_object.shape.footprint.points.size() < 3) {
    return false;
  }

  // look for bounding box boundary
  float max_x = -std::numeric_limits<float>::infinity();
  float max_y = -std::numeric_limits<float>::infinity();
  float min_x = std::numeric_limits<float>::infinity();
  float min_y = std::numeric_limits<float>::infinity();
  float max_z = -std::numeric_limits<float>::infinity();
  float min_z = std::numeric_limits<float>::infinity();

  for (const auto & point : input_object.shape.footprint.points) {
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    max_z = std::max(max_z, point.z);
    min_z = std::min(min_z, point.z);
  }

  // calc new center
  const Eigen::Vector2d center{input_object.pose.position.x, input_object.pose.position.y};
  const auto yaw = tf2::getYaw(input_object.pose.orientation);
  const Eigen::Matrix2d R_inv = Eigen::Rotation2Dd(-yaw).toRotationMatrix();
  const Eigen::Vector2d new_local_center{(max_x + min_x) / 2.0, (max_y + min_y) / 2.0};
  const Eigen::Vector2d new_center = center + R_inv.transpose() * new_local_center;

  // set output parameters
  output_object = input_object;
  output_object.pose.position.x = new_center.x();
  output_object.pose.position.y = new_center.y();

  output_object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  output_object.shape.dimensions.x = max_x - min_x;
  output_object.shape.dimensions.y = max_y - min_y;
  output_object.shape.dimensions.z = max_z - min_z;

  return true;
}

bool getMeasurementYaw(
  const types::DynamicObject & object, const double & predicted_yaw, double & measurement_yaw)
{
  measurement_yaw = tf2::getYaw(object.pose.orientation);

  // check orientation sign is known or not, and fix the limiting delta yaw
  double limiting_delta_yaw = M_PI_2;
  if (object.kinematics.orientation_availability == types::OrientationAvailability::AVAILABLE) {
    limiting_delta_yaw = M_PI;
  }
  // limiting delta yaw, even the availability is unknown
  while (std::fabs(predicted_yaw - measurement_yaw) > limiting_delta_yaw) {
    if (measurement_yaw < predicted_yaw) {
      measurement_yaw += 2 * limiting_delta_yaw;
    } else {
      measurement_yaw -= 2 * limiting_delta_yaw;
    }
  }
  // return false if the orientation is unknown
  return object.kinematics.orientation_availability != types::OrientationAvailability::UNAVAILABLE;
}

enum BBOX_IDX {
  FRONT_SURFACE = 0,
  RIGHT_SURFACE = 1,
  REAR_SURFACE = 2,
  LEFT_SURFACE = 3,
  FRONT_R_CORNER = 4,
  REAR_R_CORNER = 5,
  REAR_L_CORNER = 6,
  FRONT_L_CORNER = 7,
  INSIDE = 8,
  INVALID = -1
};

/**
 * @brief Determine the Nearest Corner or Surface of detected object observed from ego vehicle
 *
 * @param x: object x coordinate in map frame
 * @param y: object y coordinate in map frame
 * @param yaw: object yaw orientation in map frame
 * @param width: object bounding box width
 * @param length: object bounding box length
 * @param self_transform: Ego vehicle position in map frame
 * @return int index
 */
void getNearestCornerOrSurface(
  const geometry_msgs::msg::Transform & self_transform, types::DynamicObject & object)
{
  const double x = object.pose.position.x;
  const double y = object.pose.position.y;
  const double yaw = tf2::getYaw(object.pose.orientation);
  const double width = object.shape.dimensions.y;
  const double length = object.shape.dimensions.x;

  // get local vehicle pose
  const double x0 = self_transform.translation.x;
  const double y0 = self_transform.translation.y;

  // localize self vehicle pose to object coordinate
  // R.T (X0-X)
  const double xl = std::cos(yaw) * (x0 - x) + std::sin(yaw) * (y0 - y);
  const double yl = -std::sin(yaw) * (x0 - x) + std::cos(yaw) * (y0 - y);

  // Determine anchor point
  //     x+ (front)
  //         __
  // y+     |  | y-
  // (left) |  | (right)
  //         --
  //     x- (rear)
  double anchor_x = 0;
  double anchor_y = 0;
  if (xl > length / 2.0) {
    anchor_x = length / 2.0;
  } else if (xl > -length / 2.0) {
    anchor_x = 0;
  } else {
    anchor_x = -length / 2.0;
  }
  if (yl > width / 2.0) {
    anchor_y = width / 2.0;
  } else if (yl > -width / 2.0) {
    anchor_y = 0;
  } else {
    anchor_y = -width / 2.0;
  }

  object.anchor_point.x = anchor_x;
  object.anchor_point.y = anchor_y;
}

void calcAnchorPointOffset(
  const types::DynamicObject & this_object, Eigen::Vector2d & tracking_offset,
  types::DynamicObject & updating_object)
{
  // copy value
  const geometry_msgs::msg::Point anchor_vector = updating_object.anchor_point;
  // invalid anchor
  if (anchor_vector.x <= 1e-6 && anchor_vector.y <= 1e-6) {
    return;
  }
  double input_yaw = tf2::getYaw(updating_object.pose.orientation);

  // current object width and height
  const double length = this_object.shape.dimensions.x;
  const double width = this_object.shape.dimensions.y;

  // update offset
  tracking_offset = Eigen::Vector2d(anchor_vector.x, anchor_vector.y);
  if (tracking_offset.x() > 0) {
    tracking_offset.x() -= length / 2.0;
  } else if (tracking_offset.x() < 0) {
    tracking_offset.x() += length / 2.0;
  }
  if (tracking_offset.y() > 0) {
    tracking_offset.y() -= width / 2.0;
  } else if (tracking_offset.y() < 0) {
    tracking_offset.y() += width / 2.0;
  }

  // offset input object
  const Eigen::Matrix2d R = Eigen::Rotation2Dd(input_yaw).toRotationMatrix();
  const Eigen::Vector2d rotated_offset = R * tracking_offset;
  updating_object.pose.position.x += rotated_offset.x();
  updating_object.pose.position.y += rotated_offset.y();
}

}  // namespace shapes

}  // namespace autoware::multi_object_tracker
