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

#ifndef LANELET2_PLUGINS__DEFAULT_PLANNER_PLUGIN_HPP_
#define LANELET2_PLUGINS__DEFAULT_PLANNER_PLUGIN_HPP_

#include <autoware/mission_planner_universe/default_planner.hpp>
#include <autoware/mission_planner_universe/mission_planner_plugin.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace autoware::mission_planner_universe::lanelet2
{

/// @brief Adapts DefaultPlanner, which owns no ROS interface, to the pluginlib-loaded
/// PlannerPlugin. It declares the parameters, feeds the map and publishes the goal footprint on
/// behalf of the planner, so that out-of-package callers that load the planner through pluginlib
/// keep working unchanged.
///
/// initialize() must be called before any other method.
class DefaultPlannerPlugin : public mission_planner_universe::PlannerPlugin
{
public:
  void initialize(rclcpp::Node * node) override;
  void initialize(rclcpp::Node * node, const LaneletMapBin::ConstSharedPtr msg) override;
  [[nodiscard]] bool ready() const override;
  LaneletRoute plan(const RoutePoints & points) override;
  void updateRoute(const LaneletRoute & route) override;
  void clearRoute() override;
  [[nodiscard]] MarkerArray visualize(
    const LaneletRoute & route, float goal_lanelet_transparency = 0.05) const override;
  [[nodiscard]] const autoware::route_handler::RouteHandler & getRouteHandler() const override;

private:
  void initialize_common(rclcpp::Node * node);

  rclcpp::Node * node_{nullptr};
  rclcpp::Subscription<LaneletMapBin>::SharedPtr map_subscriber_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_goal_footprint_marker_;
  std::unique_ptr<DefaultPlanner> planner_;
};

}  // namespace autoware::mission_planner_universe::lanelet2

#endif  // LANELET2_PLUGINS__DEFAULT_PLANNER_PLUGIN_HPP_
