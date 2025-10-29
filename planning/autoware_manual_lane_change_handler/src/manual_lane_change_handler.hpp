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
  explicit ManualLaneChangeHandler(const rclcpp::NodeOptions & options)
  : Node("manual_lane_change_handler", options),
    plugin_loader_(
      "autoware_mission_planner_universe", "autoware::mission_planner_universe::PlannerPlugin"),
    current_route_(nullptr),
    original_route_{std::nullopt},
    logger_(rclcpp::get_logger("ManualLaneChangeHandler"))
  {
    planner_ = plugin_loader_.createSharedInstance(
      "autoware::mission_planner_universe::lanelet2::DefaultPlanner");
    planner_->initialize(this);

    get_lanelet_by_id_ = [&](const int64_t id) {
      return planner_->getRouteHandler().getLaneletMapPtr()->laneletLayer.get(id);
    };

    sub_odometry_ = create_subscription<nav_msgs::msg::Odometry>(
      "~/input/odometry", rclcpp::QoS(1),
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) { odometry_ = msg; });

    sub_route_ = create_subscription<LaneletRoute>(
      "/planning/mission_planning/route", rclcpp::QoS(1).transient_local(),
      [this](const LaneletRoute::ConstSharedPtr msg) {
        RCLCPP_INFO(logger_, "Received new route with %zu segments", msg->segments.size());
        auto route = *msg;
        planner_->updateRoute(*msg);

        std::for_each(route.segments.begin(), route.segments.end(), [&](auto & segment) {
          const auto & route_handler = planner_->getRouteHandler();
          segment.primitives = sortPrimitivesLeftToRight(
            route_handler, segment.preferred_primitive, segment.primitives);
        });

        current_route_ = std::make_shared<LaneletRoute>(route);
        planner_->updateRoute(*current_route_);
      });

    srv_set_preferred_lane = create_service<SetPreferredLane>(
      "~/set_preferred_lane",
      service_utils::handle_exception(&ManualLaneChangeHandler::set_preferred_lane, this));

    client_ = this->create_client<SetPreferredPrimitive>(
      "/planning/mission_planning/mission_planner/set_preferred_primitive");

    pub_processing_time_ =
      this->create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
        "~/debug/processing_time_ms", 1);
  }

  void publish_processing_time(autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch)
  {
    autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
    processing_time_msg.stamp = get_clock()->now();
    processing_time_msg.data = stop_watch.toc();
    pub_processing_time_->publish(processing_time_msg);
  }

private:
  std::vector<autoware_planning_msgs::msg::LaneletPrimitive> sortPrimitivesLeftToRight(
    const route_handler::RouteHandler & route_handler,
    autoware_planning_msgs::msg::LaneletPrimitive preferred_primitive,
    std::vector<autoware_planning_msgs::msg::LaneletPrimitive> primitives);

  void set_preferred_lane(
    const SetPreferredLane::Request::SharedPtr req, const SetPreferredLane::Response::SharedPtr res)
  {
    // Wait for the service to be available
    if (!client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(logger_, "Service /planning/set_preferred_primitive not available.");
      res->status.success = false;
      res->status.message = "Service /planning/set_preferred_primitive not available.";
      return;
    }

    if (!current_route_) {
      res->status.success = false;
      res->status.message = "No current route available.";
      return;
    }

    lanelet::ConstLanelet closest_lanelet =
      get_lanelet_by_id_(current_route_->segments.front().preferred_primitive.id);
    const bool found_closest_lane = planner_->getRouteHandler().getClosestLaneletWithinRoute(
      odometry_->pose.pose, &closest_lanelet);

    if (!found_closest_lane) {
      res->status.success = false;
      res->status.message = "Failed to find closest lanelet.";

      return;
    }

    RCLCPP_INFO_STREAM(logger_, "Closest lanelet ID to ego: " << closest_lanelet.id());

    LaneChangeRequestResult lane_change_request_result =
      this->process_lane_change_request(closest_lanelet.id(), req);

    if (!lane_change_request_result.success) {
      res->status.success = false;
      res->status.message = lane_change_request_result.message;
      return;
    }

    std::shared_ptr<SetPreferredPrimitive::Request> set_preferred_primitive_req =
      std::make_shared<SetPreferredPrimitive::Request>();
    set_preferred_primitive_req->preferred_primitives =
      lane_change_request_result.preferred_primitives;
    set_preferred_primitive_req->reset = req->lane_change_direction != 2;
    set_preferred_primitive_req->uuid = current_route_->uuid;

    auto future = client_->async_send_request(
      set_preferred_primitive_req,
      [this](rclcpp::Client<SetPreferredPrimitive>::SharedFuture future) {
        auto response = future.get();
        if (response->status.success) {
          RCLCPP_INFO(
            this->get_logger(), "Lane change request successful: %s",
            response->status.message.c_str());
        } else {
          RCLCPP_WARN(
            this->get_logger(), "Failed to set preferred primitive: %s",
            response->status.message.c_str());
        }
      });

    res->status.success = lane_change_request_result.success;
    res->status.message = lane_change_request_result.message;
  }

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
  std::function<lanelet::ConstLanelet(const int64_t)> get_lanelet_by_id_;
  std::optional<LaneletRoute::ConstSharedPtr> original_route_;
  rclcpp::Client<autoware_planning_msgs::srv::SetPreferredPrimitive>::SharedPtr client_;
  rclcpp::Logger logger_;
};

}  // namespace autoware::manual_lane_change_handler

#endif  // MANUAL_LANE_CHANGE_HANDLER_HPP_
