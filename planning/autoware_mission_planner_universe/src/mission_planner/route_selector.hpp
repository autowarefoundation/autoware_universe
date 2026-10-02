// Copyright 2024 The Autoware Contributors
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

#ifndef MISSION_PLANNER__ROUTE_SELECTOR_HPP_
#define MISSION_PLANNER__ROUTE_SELECTOR_HPP_

#include <autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp>
#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/route_state.hpp>
#include <autoware_planning_msgs/srv/clear_route.hpp>
#include <autoware_planning_msgs/srv/set_lanelet_route.hpp>
#include <autoware_planning_msgs/srv/set_waypoint_route.hpp>

#include <optional>
#include <variant>

namespace autoware::mission_planner_universe
{

using autoware_common_msgs::msg::ResponseStatus;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::RouteState;
using autoware_planning_msgs::srv::ClearRoute;
using autoware_planning_msgs::srv::SetLaneletRoute;
using autoware_planning_msgs::srv::SetWaypointRoute;
using unique_identifier_msgs::msg::UUID;

class RouteInterface
{
public:
  explicit RouteInterface(rclcpp::Clock::SharedPtr clock);
  RouteState::_state_type get_state() const;
  std::optional<LaneletRoute> get_route() const;
  void change_route();
  void change_state(RouteState::_state_type state);
  void update_state(const RouteState & state);
  void update_route(const LaneletRoute & route);

  AUTOWARE_SERVICE_PTR(ClearRoute) srv_clear_route;
  AUTOWARE_SERVICE_PTR(SetLaneletRoute) srv_set_lanelet_route;
  AUTOWARE_SERVICE_PTR(SetWaypointRoute) srv_set_waypoint_route;
  AUTOWARE_PUBLISHER_PTR(RouteState) pub_state_;
  AUTOWARE_PUBLISHER_PTR(LaneletRoute) pub_route_;

private:
  RouteState state_;
  std::optional<LaneletRoute> route_;
  rclcpp::Clock::SharedPtr clock_;
};

class RouteSelector : public autoware::agnocast_wrapper::Node
{
public:
  explicit RouteSelector(const rclcpp::NodeOptions & options);
  void publish_processing_time(autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch);

private:
  using WaypointRequest = SetWaypointRoute::Request::SharedPtr;
  using LaneletRequest = SetLaneletRoute::Request::SharedPtr;

  RouteInterface main_;
  RouteInterface mrm_;

  rclcpp::CallbackGroup::SharedPtr group_;
  AUTOWARE_CLIENT_PTR(ClearRoute) cli_clear_route_;
  AUTOWARE_CLIENT_PTR(SetWaypointRoute) cli_set_waypoint_route_;
  AUTOWARE_CLIENT_PTR(SetLaneletRoute) cli_set_lanelet_route_;
  AUTOWARE_SUBSCRIPTION_PTR(RouteState) sub_state_;
  AUTOWARE_SUBSCRIPTION_PTR(LaneletRoute) sub_route_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Float64Stamped) pub_processing_time_;

  bool initialized_;
  bool mrm_operating_;
  std::variant<std::monostate, WaypointRequest, LaneletRequest> main_request_;

  void on_state(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(RouteState) & msg);
  void on_route(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(LaneletRoute) & msg);

  void on_clear_route_main(ClearRoute::Request::SharedPtr req, ClearRoute::Response::SharedPtr res);
  void on_set_waypoint_route_main(
    SetWaypointRoute::Request::SharedPtr req, SetWaypointRoute::Response::SharedPtr res);
  void on_set_lanelet_route_main(
    SetLaneletRoute::Request::SharedPtr req, SetLaneletRoute::Response::SharedPtr res);

  void on_clear_route_mrm(ClearRoute::Request::SharedPtr req, ClearRoute::Response::SharedPtr res);
  void on_set_waypoint_route_mrm(
    SetWaypointRoute::Request::SharedPtr req, SetWaypointRoute::Response::SharedPtr res);
  void on_set_lanelet_route_mrm(
    SetLaneletRoute::Request::SharedPtr req, SetLaneletRoute::Response::SharedPtr res);

  ResponseStatus resume_main_route(ClearRoute::Request::SharedPtr req);
};

}  // namespace autoware::mission_planner_universe

#endif  // MISSION_PLANNER__ROUTE_SELECTOR_HPP_
