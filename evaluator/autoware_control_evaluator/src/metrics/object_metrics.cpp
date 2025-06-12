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

#include "autoware/control_evaluator/metrics/object_metrics.hpp"

#include "autoware_utils/geometry/geometry.hpp"

#include <boost/geometry.hpp>
#include <tf2/utils.h>

namespace control_diagnostics
{
namespace metrics
{
namespace bg = boost::geometry;

geometry_msgs::msg::Point32 createPoint32(const double x, const double y, const double z)
{
  geometry_msgs::msg::Point32 p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

autoware_utils::Polygon2d createEgoPolygon(
    const geometry_msgs::msg::Pose & ego_pose,
    const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  const autoware_utils::LinearRing2d local_ego_footprint = vehicle_info.createFootprint();
  const autoware_utils::LinearRing2d ego_footprint = autoware_utils::transform_vector(
    local_ego_footprint, autoware_utils::pose2transform(ego_pose));

  autoware_utils::Polygon2d ego_polygon;
  ego_polygon.outer() = ego_footprint;

  bg::correct(ego_polygon);

  return ego_polygon;
}

autoware_utils::Polygon2d createObjPolygon(
    const geometry_msgs::msg::Pose & pose,
    const geometry_msgs::msg::Polygon & footprint)
{
    geometry_msgs::msg::Polygon transformed_polygon{};
    geometry_msgs::msg::TransformStamped geometry_tf{};
    geometry_tf.transform = autoware_utils::pose2transform(pose);
    tf2::doTransform(footprint, transformed_polygon, geometry_tf);

    autoware_utils::Polygon2d object_polygon;
    for (const auto & p : transformed_polygon.points) {
        object_polygon.outer().push_back(autoware_utils::Point2d(p.x, p.y));
    }

    bg::correct(object_polygon);

    return object_polygon;
}

autoware_utils_geometry::Polygon2d createObjPolygon(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 & size)
{
  const double length_m = size.x / 2.0;
  const double width_m = size.y / 2.0;

  geometry_msgs::msg::Polygon polygon{};

  polygon.points.push_back(createPoint32(length_m, -width_m, 0.0));
  polygon.points.push_back(createPoint32(length_m, width_m, 0.0));
  polygon.points.push_back(createPoint32(-length_m, width_m, 0.0));
  polygon.points.push_back(createPoint32(-length_m, -width_m, 0.0));

  return createObjPolygon(pose, polygon);
}

autoware_utils_geometry::Polygon2d createObjPolygonForCylinder(
  const geometry_msgs::msg::Pose & pose, const double diameter)
{
  geometry_msgs::msg::Polygon polygon{};

  const double radius = diameter * 0.5;
  // add hexagon points
  for (int i = 0; i < 6; ++i) {
    const double angle = 2.0 * M_PI * static_cast<double>(i) / 6.0;
    const double x = radius * std::cos(angle);
    const double y = radius * std::sin(angle);
    polygon.points.push_back(createPoint32(x, y, 0.0));
  }

  return createObjPolygon(pose, polygon);
}

autoware_utils_geometry::Polygon2d createObjPolygon(
  const autoware_perception_msgs::msg::PredictedObject & object)
{
  const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;

  switch (object.shape.type) {
    case autoware_perception_msgs::msg::Shape::POLYGON:
      return createObjPolygon(object_pose, object.shape.footprint);
    case autoware_perception_msgs::msg::Shape::CYLINDER:
      return createObjPolygonForCylinder(object_pose, object.shape.dimensions.x);
    case autoware_perception_msgs::msg::Shape::BOUNDING_BOX:
      return createObjPolygon(object_pose, object.shape.dimensions);
    default:
      return createObjPolygon(object_pose, object.shape.dimensions);
  }
}

double calcPolygonDistance(
  const autoware_utils::Polygon2d & polygon1,
  const autoware_utils::Polygon2d & polygon2)
{
  return bg::distance(polygon1, polygon2);
}

}  // namespace metrics
}  // namespace control_diagnostics