// Copyright 2019 Autoware Foundation
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

#ifndef MISSION_PLANNER__MISSION_PLANNER_HPP_
#define MISSION_PLANNER__MISSION_PLANNER_HPP_

#include "arrival_checker.hpp"

#include <autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp>
#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/agnocast_wrapper/polling_subscriber.hpp>
#include <autoware/agnocast_wrapper/tf2.hpp>
#include <autoware/mission_planner_universe/mission_planner_plugin.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_utils/ros/logger_level_configure.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/srv/set_route.hpp>
#include <autoware_adapi_v1_msgs/srv/set_route_points.hpp>
#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/route_state.hpp>
#include <autoware_planning_msgs/srv/clear_route.hpp>
#include <autoware_planning_msgs/srv/set_lanelet_route.hpp>
#include <autoware_planning_msgs/srv/set_preferred_primitive.hpp>
#include <autoware_planning_msgs/srv/set_waypoint_route.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tier4_planning_msgs/msg/reroute_availability.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::mission_planner_universe
{

using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_planning_msgs::msg::LaneletPrimitive;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;
using autoware_planning_msgs::msg::PoseWithUuidStamped;
using autoware_planning_msgs::msg::RouteState;
using autoware_planning_msgs::srv::ClearRoute;
using autoware_planning_msgs::srv::SetLaneletRoute;
using autoware_planning_msgs::srv::SetPreferredPrimitive;
using autoware_planning_msgs::srv::SetWaypointRoute;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Odometry;
using std_msgs::msg::Header;
using tier4_planning_msgs::msg::RerouteAvailability;
using unique_identifier_msgs::msg::UUID;
using visualization_msgs::msg::MarkerArray;

class MissionPlanner : public autoware::agnocast_wrapper::Node
{
public:
  explicit MissionPlanner(const rclcpp::NodeOptions & options);
  void publish_processing_time(autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch);

private:
  ArrivalChecker arrival_checker_;
  pluginlib::ClassLoader<PlannerPlugin> plugin_loader_;
  std::shared_ptr<PlannerPlugin> planner_;

  std::string map_frame_;
  autoware::agnocast_wrapper::Buffer tf_buffer_;
  autoware::agnocast_wrapper::TransformListener tf_listener_;
  Pose transform_pose(const Pose & pose, const Header & header);

  AUTOWARE_SERVICE_PTR(ClearRoute) srv_clear_route;
  AUTOWARE_SERVICE_PTR(SetLaneletRoute) srv_set_lanelet_route;
  AUTOWARE_SERVICE_PTR(SetPreferredPrimitive) srv_set_preferred_primitive;
  AUTOWARE_SERVICE_PTR(SetWaypointRoute) srv_set_waypoint_route;
  AUTOWARE_PUBLISHER_PTR(RouteState) pub_state_;
  AUTOWARE_PUBLISHER_PTR(LaneletRoute) pub_route_;

  AUTOWARE_SUBSCRIPTION_PTR(PoseWithUuidStamped) sub_modified_goal_;
  AUTOWARE_SUBSCRIPTION_PTR(Odometry) sub_odometry_;
  AUTOWARE_SUBSCRIPTION_PTR(OperationModeState) sub_operation_mode_state_;
  autoware::agnocast_wrapper::polling::PollingSubscriber<RerouteAvailability>::SharedPtr
    sub_reroute_availability_;

  AUTOWARE_SUBSCRIPTION_PTR(LaneletMapBin) sub_vector_map_;
  AUTOWARE_PUBLISHER_PTR(MarkerArray) pub_marker_;
  AUTOWARE_MESSAGE_CONST_SHARED_PTR(Odometry) odometry_;
  AUTOWARE_MESSAGE_CONST_SHARED_PTR(OperationModeState) operation_mode_state_;
  AUTOWARE_MESSAGE_CONST_SHARED_PTR(LaneletMapBin) map_ptr_;
  RouteState state_;
  std::optional<LaneletRoute::SharedPtr> original_route_;
  LaneletRoute::ConstSharedPtr current_route_;
  lanelet::LaneletMapPtr lanelet_map_ptr_{nullptr};

  void on_odometry(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(Odometry) & msg);
  void on_operation_mode_state(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(OperationModeState) & msg);
  void on_map(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(LaneletMapBin) & msg);
  void on_modified_goal(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(PoseWithUuidStamped) & msg);

  void on_clear_route(
    const ClearRoute::Request::SharedPtr req, const ClearRoute::Response::SharedPtr res);
  void on_set_lanelet_route(
    const SetLaneletRoute::Request::SharedPtr req, const SetLaneletRoute::Response::SharedPtr res);
  void on_set_preferred_primitive(
    const SetPreferredPrimitive::Request::SharedPtr req,
    const SetPreferredPrimitive::Response::SharedPtr res);
  void on_set_waypoint_route(
    const SetWaypointRoute::Request::SharedPtr req,
    const SetWaypointRoute::Response::SharedPtr res);

  void change_state(RouteState::_state_type state);
  void change_route();
  void change_route(const LaneletRoute & route);
  void cancel_route();
  LaneletRoute create_route(const SetLaneletRoute::Request & req);
  LaneletRoute create_route(const SetWaypointRoute::Request & req);
  LaneletRoute create_route(const PoseWithUuidStamped & msg);
  LaneletRoute create_route(
    const Header & header, const std::vector<LaneletSegment> & segments, const Pose & goal_pose,
    const UUID & uuid, const bool allow_goal_modification);
  LaneletRoute create_route(
    const Header & header, const std::vector<Pose> & waypoints, const Pose & start_pose,
    const Pose & goal_pose, const UUID & uuid, const bool allow_goal_modification);

  void print_pose_log(
    const std::string & route_type, const Pose & initial_pose, const Pose & goal_pose);

  AUTOWARE_TIMER_PTR data_check_timer_;
  void check_initialization();
  bool is_mission_planner_ready_;

  double reroute_time_threshold_;
  double minimum_reroute_length_;
  // flag to allow reroute in autonomous driving mode.
  // if false, reroute fails. if true, only safe reroute is allowed.
  bool allow_reroute_in_autonomous_mode_;
  float goal_lanelet_transparency_;
  bool check_reroute_safety(const LaneletRoute & original_route, const LaneletRoute & target_route);

  std::unique_ptr<autoware_utils::BasicLoggerLevelConfigure<autoware::agnocast_wrapper::Node>>
    logger_configure_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Float64Stamped) pub_processing_time_;
};

}  // namespace autoware::mission_planner_universe

#endif  // MISSION_PLANNER__MISSION_PLANNER_HPP_
