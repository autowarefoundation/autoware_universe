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

#ifndef AUTOWARE__CONTROL_EVALUATOR__METRICS__OBJECT_METRICS_HPP_
#define AUTOWARE__CONTROL_EVALUATOR__METRICS__OBJECT_METRICS_HPP_

#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace control_diagnostics
{

namespace metrics
{

/**
 * @brief Create the ego vehicle polygon.
 * @param [in] ego_pose ego pose
 * @param [in] vehicle_info vehicle info
 * @return ego vehicle polygon
 **/
autoware_utils::Polygon2d createEgoPolygon(
  const geometry_msgs::msg::Pose & ego_pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info);

/**
 * @brief Create the object polygon.
 * @param [in] object object
 * @return object polygon
 **/
autoware_utils::Polygon2d createObjPolygon(
  const autoware_perception_msgs::msg::PredictedObject & object);

/**
 * @brief Calculate the distance between two polygons.
 * @param [in] polygon1 polygon1
 * @param [in] polygon2 polygon2
 * @return distance
 **/
double calcPolygonDistance(
  const autoware_utils::Polygon2d & polygon1, const autoware_utils::Polygon2d & polygon2);

}  // namespace metrics
}  // namespace control_diagnostics

#endif  // AUTOWARE__CONTROL_EVALUATOR__METRICS__OBJECT_METRICS_HPP_
