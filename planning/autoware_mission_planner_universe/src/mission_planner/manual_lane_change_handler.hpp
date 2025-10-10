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

#ifndef MISSION_PLANNER__MANUAL_LANE_CHANGE_HANDLER_HPP_
#define MISSION_PLANNER__MANUAL_LANE_CHANGE_HANDLER_HPP_

#include "service_utils.hpp"

#include <autoware/mission_planner_universe/mission_planner_plugin.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/srv/set_lanelet_route.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_planning_msgs/srv/set_preferred_lane.hpp>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

#include <optional>
#include <string>

namespace autoware::mission_planner_universe
{

using autoware_planning_msgs::msg::LaneletRoute;
using tier4_planning_msgs::srv::SetPreferredLane;

struct LaneChangeRequestResult
{
  LaneletRoute route;
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

        // trim route from the current position

        const auto segment_it =
          std::find_if(route.segments.begin(), route.segments.end(), [&](const auto & segment) {
            return std::any_of(
              segment.primitives.begin(), segment.primitives.end(), [&](const auto & primitive) {
                const auto lanelet = lanelet_map_ptr_->laneletLayer.get(primitive.id);
                return lanelet::utils::isInLanelet(odometry_->pose.pose, lanelet);
              });
          });

        // erase segments before the current segment
        if (segment_it != route.segments.end()) {
          route.segments.erase(route.segments.begin(), segment_it);
        }

        current_route_ = std::make_shared<LaneletRoute>(route);
        planner_->updateRoute(*current_route_);
      });

    srv_set_preferred_lane = create_service<SetPreferredLane>(
      "~/set_preferred_lane",
      service_utils::handle_exception(&ManualLaneChangeHandler::set_preferred_lane, this));

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
    auto client = this->create_client<autoware_planning_msgs::srv::SetLaneletRoute>(
      "/planning/set_lanelet_route");
    // Wait for the service to be available
    if (!client->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(logger_, "Service /planning/set_lanelet_route not available.");
      res->status.success = false;
      res->status.message = "Service /planning/set_lanelet_route not available.";
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

    std::shared_ptr<autoware_planning_msgs::srv::SetLaneletRoute::Request> set_lanelet_route_req =
      std::make_shared<autoware_planning_msgs::srv::SetLaneletRoute::Request>();
    set_lanelet_route_req->header = current_route_->header;
    set_lanelet_route_req->goal_pose = current_route_->goal_pose;

    // Generate a new UUID for the route
    boost::uuids::random_generator gen;
    boost::uuids::uuid uuid = gen();
    std::copy(uuid.begin(), uuid.end(), set_lanelet_route_req->uuid.uuid.begin());
    set_lanelet_route_req->allow_modification = current_route_->allow_modification;
    set_lanelet_route_req->segments = lane_change_request_result.route.segments;
    set_lanelet_route_req->emphasise_goal_lanes =
      req->lane_change_direction != 2;  // Emphasise goal lanes for manual lane changes

    auto future = client->async_send_request(
      set_lanelet_route_req,
      [this](rclcpp::Client<autoware_planning_msgs::srv::SetLaneletRoute>::SharedFuture future) {
        auto response = future.get();
        if (response->status.success) {
          RCLCPP_INFO(this->get_logger(), "Successfully set lanelet route!");
        } else {
          RCLCPP_WARN(
            this->get_logger(), "Failed to set lanelet route: %s",
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
  rclcpp::Logger logger_;
};

}  // namespace autoware::mission_planner_universe

#endif  // MISSION_PLANNER__MANUAL_LANE_CHANGE_HANDLER_HPP_
