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

#ifndef AUTOWARE__MISSION_PLANNER_UNIVERSE__MISSION_PLANNER_PLUGIN_HPP_
#define AUTOWARE__MISSION_PLANNER_UNIVERSE__MISSION_PLANNER_PLUGIN_HPP_

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <functional>
#include <vector>

namespace autoware::mission_planner_universe
{

class PlannerPlugin
{
public:
  using RoutePoints = std::vector<geometry_msgs::msg::Pose>;
  using LaneletRoute = autoware_planning_msgs::msg::LaneletRoute;
  using LaneletMapBin = autoware_map_msgs::msg::LaneletMapBin;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  /// @brief Everything a planner plugin needs from the node that hosts it. Holding the node
  /// interfaces instead of the node itself keeps this interface independent of the node type, so
  /// that a plugin can be hosted by any node that provides them.
  struct Context
  {
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters;
    rclcpp::Logger logger{rclcpp::get_logger("mission_planner_plugin")};
    rclcpp::Clock::SharedPtr clock;
    std::function<void(const MarkerArray &)> publish_debug_marker;
    autoware::vehicle_info_utils::VehicleInfo vehicle_info;
  };

  virtual ~PlannerPlugin() = default;
  virtual void initialize(const Context & context) = 0;
  virtual void set_map(const LaneletMapBin & map) = 0;
  virtual bool ready() const = 0;
  virtual LaneletRoute plan(const RoutePoints & points) = 0;
  virtual MarkerArray visualize(
    const LaneletRoute & route, float goal_lanelet_transparency = 0.05) const = 0;
  virtual void updateRoute(const LaneletRoute & route) = 0;
  virtual void clearRoute() = 0;
  virtual const autoware::route_handler::RouteHandler & getRouteHandler() const = 0;

  /// @brief Build a context from a node. Works with any node type that provides the rclcpp node
  /// interfaces and create_publisher (rclcpp::Node, autoware::agnocast_wrapper::Node, ...).
  template <typename NodeT>
  static Context make_context(NodeT & node)
  {
    const auto durable_qos = rclcpp::QoS(1).transient_local();
    auto debug_marker_publisher =
      node.template create_publisher<MarkerArray>("~/debug/goal_footprint", durable_qos);

    Context context;
    context.parameters = node.get_node_parameters_interface();
    context.logger = node.get_logger();
    context.clock = node.get_clock();
    // The publisher is owned by this callback, so it lives as long as the plugin keeps the context.
    context.publish_debug_marker = [debug_marker_publisher](const MarkerArray & markers) {
      debug_marker_publisher->publish(markers);
    };
    context.vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo();
    return context;
  }

  /// @brief Convenience entry point for callers that own a node and already have the map.
  template <typename NodeT>
  void initialize(NodeT * node, const LaneletMapBin::ConstSharedPtr msg)
  {
    initialize(make_context(*node));
    set_map(*msg);
  }
};

}  // namespace autoware::mission_planner_universe

#endif  // AUTOWARE__MISSION_PLANNER_UNIVERSE__MISSION_PLANNER_PLUGIN_HPP_
