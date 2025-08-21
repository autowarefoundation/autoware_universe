// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__COMPONENT_INTERFACE_SPECS_UNIVERSE__PLANNING_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_SPECS_UNIVERSE__PLANNING_HPP_

#include <rclcpp/qos.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/route_state.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/srv/clear_route.hpp>
#include <autoware_planning_msgs/srv/set_lanelet_route.hpp>
#include <autoware_planning_msgs/srv/set_waypoint_route.hpp>

namespace autoware::component_interface_specs_universe::planning
{

struct SetLaneletRoute
{
  using Service = autoware_planning_msgs::srv::SetLaneletRoute;
  static constexpr char name[] = "/planning/set_lanelet_route";
};

struct SetWaypointRoute
{
  using Service = autoware_planning_msgs::srv::SetWaypointRoute;
  static constexpr char name[] = "/planning/set_waypoint_route";
};

struct ClearRoute
{
  using Service = autoware_planning_msgs::srv::ClearRoute;
  static constexpr char name[] = "/planning/clear_route";
};

struct RouteState
{
  using Message = autoware_planning_msgs::msg::RouteState;
  static constexpr char name[] = "/planning/route_state";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
};

struct LaneletRoute
{
  using Message = autoware_planning_msgs::msg::LaneletRoute;
  static constexpr char name[] = "/planning/route";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
};

struct Trajectory
{
  using Message = autoware_planning_msgs::msg::Trajectory;
  static constexpr char name[] = "/planning/trajectory";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

}  // namespace autoware::component_interface_specs_universe::planning

#endif  // AUTOWARE__COMPONENT_INTERFACE_SPECS_UNIVERSE__PLANNING_HPP_
