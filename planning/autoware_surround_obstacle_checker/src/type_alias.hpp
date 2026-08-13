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

#include <autoware/motion_utils/vehicle/vehicle_state_checker.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_internal_planning_msgs/msg/planning_factor_array.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit_clear_command.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace autoware::surround_obstacle_checker
{

using autoware::motion_utils::VehicleStopChecker;
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_internal_planning_msgs::msg::ControlPoint;
using autoware_internal_planning_msgs::msg::PlanningFactor;
using autoware_internal_planning_msgs::msg::PlanningFactorArray;
using autoware_internal_planning_msgs::msg::VelocityLimit;
using autoware_internal_planning_msgs::msg::VelocityLimitClearCommand;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::Shape;
using geometry_msgs::msg::PolygonStamped;
using unique_identifier_msgs::msg::UUID;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;

}  // namespace autoware::surround_obstacle_checker

#endif  // TYPE_ALIAS_HPP_
