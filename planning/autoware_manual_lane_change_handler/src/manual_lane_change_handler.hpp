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

#ifndef MANUAL_LANE_CHANGE_HANDLER_HPP_
#define MANUAL_LANE_CHANGE_HANDLER_HPP_

#include <autoware/mission_planner_universe/mission_planner_plugin.hpp>
#include <autoware/mission_planner_universe/service_utils.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_planning_msgs/msg/lanelet_primitive.hpp>
#include <autoware_planning_msgs/srv/set_lanelet_route.hpp>
#include <autoware_planning_msgs/srv/set_preferred_primitive.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_external_api_msgs/srv/set_preferred_lane.hpp>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::manual_lane_change_handler
{

using autoware::mission_planner_universe::PlannerPlugin;
using autoware_planning_msgs::msg::LaneletPrimitive;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::srv::SetPreferredPrimitive;
using tier4_external_api_msgs::srv::SetPreferredLane;

struct LaneChangeRequestResult
{
  std::vector<LaneletPrimitive> preferred_primitives;
  bool success;
  std::string message;
};

enum class DIRECTION {
  MANUAL_LEFT,
  MANUAL_RIGHT,
  AUTO,
};

class ManualLaneChangeHandler : public rclcpp::Node
{
public:
  explicit ManualLaneChangeHandler(const rclcpp::NodeOptions & options);

  void publish_processing_time(autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch)
  {
    autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
    processing_time_msg.stamp = get_clock()->now();
    processing_time_msg.data = stop_watch.toc();
    pub_processing_time_->publish(processing_time_msg);
  }

private:
  std::vector<autoware_planning_msgs::msg::LaneletPrimitive> sort_primitives_left_to_right(
    const route_handler::RouteHandler & route_handler,
    autoware_planning_msgs::msg::LaneletPrimitive preferred_primitive,
    std::vector<autoware_planning_msgs::msg::LaneletPrimitive> primitives);

  lanelet::ConstLanelet get_lanelet_by_id(const int64_t id)
  {
    return planner_->getRouteHandler().getLaneletMapPtr()->laneletLayer.get(id);
  }

  void route_callback(const LaneletRoute::ConstSharedPtr msg);
  void set_preferred_lane(
    const SetPreferredLane::Request::SharedPtr req,
    const SetPreferredLane::Response::SharedPtr res);
  LaneChangeRequestResult process_lane_change_request(
    const int64_t ego_lanelet_id, const SetPreferredLane::Request::SharedPtr req);

  rclcpp::Service<SetPreferredLane>::SharedPtr srv_set_preferred_lane;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<LaneletRoute>::SharedPtr sub_route_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    pub_processing_time_;

  pluginlib::ClassLoader<PlannerPlugin> plugin_loader_;
  std::shared_ptr<PlannerPlugin> planner_;

  nav_msgs::msg::Odometry::ConstSharedPtr odometry_;
  std::shared_ptr<LaneletRoute> current_route_;
  std::optional<LaneletRoute::ConstSharedPtr> original_route_;
  rclcpp::Client<autoware_planning_msgs::srv::SetPreferredPrimitive>::SharedPtr client_;
  rclcpp::Logger logger_;
};

}  // namespace autoware::manual_lane_change_handler

#endif  // MANUAL_LANE_CHANGE_HANDLER_HPP_
