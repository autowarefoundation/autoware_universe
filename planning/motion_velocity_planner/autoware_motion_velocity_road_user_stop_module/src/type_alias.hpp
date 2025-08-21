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

#ifndef TYPE_ALIAS_HPP_
#define TYPE_ALIAS_HPP_

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_internal_planning_msgs/msg/planning_factor.hpp>
#include <autoware_internal_planning_msgs/msg/safety_factor_array.hpp>
#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace autoware::motion_velocity_planner
{
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_internal_planning_msgs::msg::PlanningFactor;
using autoware_internal_planning_msgs::msg::SafetyFactor;
using autoware_internal_planning_msgs::msg::SafetyFactorArray;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::Shape;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_utils::LineString2d;
using autoware_utils_geometry::Polygon2d;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::Odometry;
using unique_identifier_msgs::msg::UUID;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
}  // namespace autoware::motion_velocity_planner

#endif  // TYPE_ALIAS_HPP_
