// Copyright 2023 TIER IV, Inc.
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

#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/footprints.hpp"

#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/parameters.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp"

#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <boost/geometry/strategies/transform/matrix_transformers.hpp>

#include <tf2/utils.h>

namespace autoware::behavior_path_planner::drivable_area_expansion
{
Polygon2d translate_polygon(const Polygon2d & polygon, const double x, const double y)
{
  Polygon2d translated_polygon;
  const boost::geometry::strategy::transform::translate_transformer<double, 2, 2> translation(x, y);
  boost::geometry::transform(polygon, translated_polygon, translation);
  return translated_polygon;
}

Polygon2d create_footprint(const geometry_msgs::msg::Pose & pose, const Polygon2d & base_footprint)
{
  const auto angle = tf2::getYaw(pose.orientation);
  return translate_polygon(
    autoware_utils::rotate_polygon(base_footprint, angle), pose.position.x, pose.position.y);
}

MultiPolygon2d create_object_footprints(
  const autoware_perception_msgs::msg::PredictedObjects & objects,
  const DrivableAreaExpansionParameters & params)
{
  using behavior_path_planner::utils::path_safety_checker::filter::velocity_filter;

  MultiPolygon2d footprints;
  if (!params.object_exclusion.exclude_dynamic && !params.object_exclusion.exclude_static) {
    return footprints;
  }

  auto get_base_footprint = [&](const auto & object) {
    const auto front = object.shape.dimensions.x / 2 + params.object_exclusion.front_offset;
    const auto rear = -object.shape.dimensions.x / 2 - params.object_exclusion.rear_offset;
    const auto left = object.shape.dimensions.y / 2 + params.object_exclusion.left_offset;
    const auto right = -object.shape.dimensions.y / 2 - params.object_exclusion.right_offset;
    Polygon2d footprint;
    footprint.outer() = {
      Point2d{front, left}, Point2d{front, right}, Point2d{rear, right}, Point2d{rear, left},
      Point2d{front, left}};
    return footprint;
  };

  for (const auto & object : objects.objects) {
    const auto base_footprint = get_base_footprint(object);
    if (params.object_exclusion.exclude_dynamic) {
      for (const auto & path : object.kinematics.predicted_paths)
        for (const auto & pose : path.path)
          footprints.push_back(create_footprint(pose, base_footprint));
      continue;
    }

    if (
      params.object_exclusion.exclude_static &&
      velocity_filter(
        object.kinematics.initial_twist_with_covariance.twist,
        -std::numeric_limits<double>::epsilon(), params.object_exclusion.stopped_obj_vel_th)) {
      footprints.push_back(
        create_footprint(object.kinematics.initial_pose_with_covariance.pose, base_footprint));
    }
  }
  return footprints;
}
}  // namespace autoware::behavior_path_planner::drivable_area_expansion
