// Copyright 2026 TIER IV, Inc.
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

#include "autoware/obstacle_proximity_checker/obstacle_proximity_checker.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <limits>
#include <string>
#include <unordered_map>

namespace autoware::obstacle_proximity_checker
{
namespace
{
namespace bg = boost::geometry;
using Point2d = bg::model::d2::point_xy<double>;
using Polygon2d = bg::model::polygon<Point2d>;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_utils::create_point;

const std::unordered_map<int, std::string> kLabelMap = {
  {ObjectClassification::UNKNOWN, "unknown"}, {ObjectClassification::CAR, "car"},
  {ObjectClassification::TRUCK, "truck"},     {ObjectClassification::BUS, "bus"},
  {ObjectClassification::TRAILER, "trailer"}, {ObjectClassification::MOTORCYCLE, "motorcycle"},
  {ObjectClassification::BICYCLE, "bicycle"}, {ObjectClassification::PEDESTRIAN, "pedestrian"}};

geometry_msgs::msg::Pose createBaseLinkOrigin()
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(0.0);
  return pose;
}

}  // namespace

ProximityChecker::ProximityChecker(
  const Parameters & parameters, const vehicle_info_utils::VehicleInfo & vehicle_info)
: parameters_(parameters), vehicle_info_(vehicle_info)
{
}

CheckResult ProximityChecker::check(
  const Inputs & input, const double contact_distance_threshold) const
{
  const auto nearest_obstacle = getNearestObstacle(input);

  CheckResult result;
  result.nearest_obstacle = nearest_obstacle;
  result.is_obstacle_found = isObstacleFound(nearest_obstacle, contact_distance_threshold);
  return result;
}

void ProximityChecker::update_parameters(const Parameters & parameters)
{
  parameters_ = parameters;
}

bool ProximityChecker::getUseDynamicObject() const
{
  for (const auto & [label, enabled] : parameters_.object_type_enable_check) {
    (void)label;
    if (enabled) {
      return true;
    }
  }
  return false;
}

bool ProximityChecker::isObstacleFound(
  const std::optional<ProximityObstacle> & nearest_obstacle,
  const double contact_distance_threshold) const
{
  if (!nearest_obstacle.has_value()) {
    return false;
  }

  return nearest_obstacle.value().nearest_distance < contact_distance_threshold;
}

std::optional<ProximityObstacle> ProximityChecker::getNearestObstacle(const Inputs & input) const
{
  const auto nearest_pointcloud = getNearestObstacleByPointCloud(input);
  const auto nearest_object = getNearestObstacleByDynamicObject(input);
  if (!nearest_pointcloud.has_value() && !nearest_object.has_value()) {
    return std::nullopt;
  }

  if (!nearest_pointcloud.has_value()) {
    return nearest_object;
  }

  if (!nearest_object.has_value()) {
    return nearest_pointcloud;
  }

  return nearest_pointcloud.value().nearest_distance < nearest_object.value().nearest_distance
           ? nearest_pointcloud
           : nearest_object;
}

std::optional<ProximityObstacle> ProximityChecker::getNearestObstacleByPointCloud(
  const Inputs & input) const
{
  if (!parameters_.pointcloud_enable_check || !input.pointcloud_in_base_link) {
    return std::nullopt;
  }

  if (input.pointcloud_in_base_link->empty()) {
    return std::nullopt;
  }

  const auto & pointcloud_param = parameters_.obstacle_types_map.at("pointcloud");
  const double front_margin = pointcloud_param.surround_check_front_distance;
  const double side_margin = pointcloud_param.surround_check_side_distance;
  const double back_margin = pointcloud_param.surround_check_back_distance;
  const double base_to_front = vehicle_info_.max_longitudinal_offset_m + front_margin;
  const double base_to_rear = vehicle_info_.rear_overhang_m + back_margin;
  const double width = vehicle_info_.vehicle_width_m + side_margin * 2;
  const auto ego_polygon =
    autoware_utils::to_footprint(createBaseLinkOrigin(), base_to_front, base_to_rear, width);

  geometry_msgs::msg::Point nearest_point_base_link;
  double minimum_distance = std::numeric_limits<double>::max();
  bool was_minimum_distance_updated = false;
  for (const auto & point : *input.pointcloud_in_base_link) {
    const Point2d boost_point(point.x, point.y);
    const auto distance_to_object = bg::distance(ego_polygon, boost_point);

    if (distance_to_object < minimum_distance) {
      nearest_point_base_link = create_point(point.x, point.y, point.z);
      minimum_distance = distance_to_object;
      was_minimum_distance_updated = true;
    }
  }

  if (!was_minimum_distance_updated) {
    return std::nullopt;
  }

  ProximityObstacle obstacle;
  obstacle.is_point_cloud = true;
  obstacle.nearest_distance = minimum_distance;
  obstacle.nearest_point =
    autoware::universe_utils::transformPoint(nearest_point_base_link, input.ego_pose);
  obstacle.uuid = unique_identifier_msgs::msg::UUID();
  return obstacle;
}

std::optional<ProximityObstacle> ProximityChecker::getNearestObstacleByDynamicObject(
  const Inputs & input) const
{
  if (!input.objects || !getUseDynamicObject()) {
    return std::nullopt;
  }

  autoware_perception_msgs::msg::PredictedObject nearest_object;
  double minimum_distance = std::numeric_limits<double>::max();
  bool was_minimum_distance_updated = false;
  for (const auto & object : input.objects->objects) {
    const int label = object.classification.front().label;
    const auto label_iter = kLabelMap.find(label);
    if (label_iter == kLabelMap.end()) {
      continue;
    }

    const auto & str_label = label_iter->second;
    const auto enable_check_iter = parameters_.object_type_enable_check.find(str_label);
    if (
      enable_check_iter == parameters_.object_type_enable_check.end() ||
      !enable_check_iter->second) {
      continue;
    }

    const auto & object_param = parameters_.obstacle_types_map.at(str_label);
    const double front_margin = object_param.surround_check_front_distance;
    const double side_margin = object_param.surround_check_side_distance;
    const double back_margin = object_param.surround_check_back_distance;
    const double base_to_front = vehicle_info_.max_longitudinal_offset_m + front_margin;
    const double base_to_rear = vehicle_info_.rear_overhang_m + back_margin;
    const double width = vehicle_info_.vehicle_width_m + side_margin * 2;
    const auto ego_polygon =
      autoware_utils::to_footprint(input.ego_pose, base_to_front, base_to_rear, width);
    const auto object_polygon = autoware_utils::to_polygon2d(object);
    const auto distance_to_object = bg::distance(ego_polygon, object_polygon);

    if (distance_to_object < minimum_distance) {
      nearest_object = object;
      minimum_distance = distance_to_object;
      was_minimum_distance_updated = true;
    }
  }

  if (!was_minimum_distance_updated) {
    return std::nullopt;
  }

  ProximityObstacle obstacle;
  obstacle.is_point_cloud = false;
  obstacle.nearest_distance = minimum_distance;
  obstacle.nearest_point = nearest_object.kinematics.initial_pose_with_covariance.pose.position;
  obstacle.uuid = nearest_object.object_id;
  return obstacle;
}

}  // namespace autoware::obstacle_proximity_checker
