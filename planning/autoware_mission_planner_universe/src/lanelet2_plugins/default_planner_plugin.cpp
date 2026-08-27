// Copyright 2025 Autoware Foundation
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

#include "default_planner_plugin.hpp"

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <memory>

namespace autoware::mission_planner_universe::lanelet2
{

void DefaultPlannerPlugin::initialize_common(rclcpp::Node * node)
{
  node_ = node;

  const auto durable_qos = rclcpp::QoS(1).transient_local();
  pub_goal_footprint_marker_ =
    node_->create_publisher<MarkerArray>("~/debug/goal_footprint", durable_qos);

  DefaultPlannerParameters param;
  param.goal_angle_threshold_deg = node_->declare_parameter<double>("goal_angle_threshold_deg");
  param.enable_correct_goal_pose = node_->declare_parameter<bool>("enable_correct_goal_pose");
  param.consider_no_drivable_lanes = node_->declare_parameter<bool>("consider_no_drivable_lanes");
  param.check_footprint_inside_lanes =
    node_->declare_parameter<bool>("check_footprint_inside_lanes");
  param.allow_area = node_->declare_parameter<bool>("allow_area", false);

  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(*node_).getVehicleInfo();
  planner_ = std::make_unique<DefaultPlanner>(param, vehicle_info);
}

void DefaultPlannerPlugin::initialize(rclcpp::Node * node)
{
  initialize_common(node);
  map_subscriber_ = node_->create_subscription<LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{10}.transient_local(),
    [this](const LaneletMapBin::ConstSharedPtr msg) { planner_->set_map(*msg); });
}

void DefaultPlannerPlugin::initialize(rclcpp::Node * node, const LaneletMapBin::ConstSharedPtr msg)
{
  initialize_common(node);
  planner_->set_map(*msg);
}

bool DefaultPlannerPlugin::ready() const
{
  return planner_ && planner_->ready();
}

PlannerPlugin::LaneletRoute DefaultPlannerPlugin::plan(const RoutePoints & points)
{
  const auto result = planner_->plan(points);
  if (result.warning_message) {
    RCLCPP_WARN(node_->get_logger(), "%s", result.warning_message->c_str());
  }
  if (result.goal_footprint) {
    pub_goal_footprint_marker_->publish(
      DefaultPlanner::visualize_debug_footprint(*result.goal_footprint));
  }
  return result.route;
}

void DefaultPlannerPlugin::updateRoute(const LaneletRoute & route)
{
  planner_->updateRoute(route);
}

void DefaultPlannerPlugin::clearRoute()
{
  planner_->clearRoute();
}

PlannerPlugin::MarkerArray DefaultPlannerPlugin::visualize(
  const LaneletRoute & route, float goal_lanelet_transparency) const
{
  return planner_->visualize(route, goal_lanelet_transparency);
}

const autoware::route_handler::RouteHandler & DefaultPlannerPlugin::getRouteHandler() const
{
  return planner_->getRouteHandler();
}

}  // namespace autoware::mission_planner_universe::lanelet2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::mission_planner_universe::lanelet2::DefaultPlannerPlugin,
  autoware::mission_planner_universe::PlannerPlugin)
