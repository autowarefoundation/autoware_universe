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

#include "manual_lane_change_handler.hpp"

#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace autoware::manual_lane_change_handler
{

ManualLaneChangeHandler::ManualLaneChangeHandler(const rclcpp::NodeOptions & options)
: Node("manual_lane_change_handler", options),
  plugin_loader_(
    "autoware_mission_planner_universe", "autoware::mission_planner_universe::PlannerPlugin"),
  current_route_(nullptr),
  logger_(rclcpp::get_logger("ManualLaneChangeHandler"))
{
  planner_ = plugin_loader_.createSharedInstance(
    "autoware::mission_planner_universe::lanelet2::DefaultPlanner");
  planner_->initialize(this);

  sub_odometry_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", rclcpp::QoS(1),
    [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) { odometry_ = msg; });

  sub_route_ = create_subscription<LaneletRoute>(
    "/planning/mission_planning/route", rclcpp::QoS(1).transient_local(),
    std::bind(&ManualLaneChangeHandler::route_callback, this, std::placeholders::_1));

  srv_set_preferred_lane = create_service<SetPreferredLane>(
    "~/set_preferred_lane",
    service_utils::handle_exception(&ManualLaneChangeHandler::set_preferred_lane, this));

  client_ = this->create_client<SetPreferredPrimitive>(
    "/planning/mission_planning/mission_planner/set_preferred_primitive");

  pub_processing_time_ = this->create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "~/debug/processing_time_ms", 1);
  pub_shift_number_ = this->create_publisher<autoware_internal_debug_msgs::msg::Int32Stamped>(
    "~/debug/shift_number", 1);
}

std::vector<autoware_planning_msgs::msg::LaneletPrimitive>
ManualLaneChangeHandler::sort_primitives_left_to_right(
  const route_handler::RouteHandler & route_handler,
  autoware_planning_msgs::msg::LaneletPrimitive preferred_primitive,
  std::vector<autoware_planning_msgs::msg::LaneletPrimitive> primitives)
{
  using Primitive = autoware_planning_msgs::msg::LaneletPrimitive;

  std::deque<Primitive> sorted_primitives;

  auto find_primitive = [&](lanelet::Id id) -> std::optional<Primitive> {
    auto it = std::find_if(
      primitives.begin(), primitives.end(), [&](const Primitive & p) { return p.id == id; });
    if (it != primitives.end()) return *it;
    return std::nullopt;
  };

  lanelet::ConstLanelet current = route_handler.getLaneletsFromId(preferred_primitive.id);
  // Walk left lanes
  for (auto left = route_handler.getLeftLanelet(current, true); left;
       left = route_handler.getLeftLanelet(*left, true)) {
    if (auto match = find_primitive(left->id())) {
      sorted_primitives.push_front(*match);
    }
  }

  sorted_primitives.push_back(preferred_primitive);

  // Walk right lanes
  for (auto right = route_handler.getRightLanelet(current, true); right;
       right = route_handler.getRightLanelet(*right, true)) {
    if (auto match = find_primitive(right->id())) {
      sorted_primitives.push_back(*match);
    }
  }

  std::vector<autoware_planning_msgs::msg::LaneletPrimitive> result{
    sorted_primitives.begin(), sorted_primitives.end()};

  return result;
}

void ManualLaneChangeHandler::route_callback(const LaneletRoute::ConstSharedPtr msg)
{
  RCLCPP_INFO(logger_, "Received new route with %zu segments", msg->segments.size());
  auto route = *msg;
  planner_->updateRoute(*msg);

  const auto & route_handler = planner_->getRouteHandler();
  std::for_each(route.segments.begin(), route.segments.end(), [&](auto & segment) {
    segment.primitives =
      sort_primitives_left_to_right(route_handler, segment.preferred_primitive, segment.primitives);
  });

  if (!current_route_) {
    shift_number_ = 0;
    autoware_internal_debug_msgs::msg::Int32Stamped shift_msg;
    shift_msg.stamp = get_clock()->now();
    shift_msg.data = shift_number_;

    pub_shift_number_->publish(shift_msg);
  }

  if (current_route_ && msg->uuid != current_route_->uuid) {
    RCLCPP_INFO(logger_, "Resetting shift number due to route UUID change.");
    shift_number_ = 0;
    autoware_internal_debug_msgs::msg::Int32Stamped shift_msg;
    shift_msg.stamp = get_clock()->now();
    shift_msg.data = shift_number_;

    pub_shift_number_->publish(shift_msg);
  }

  current_route_ = std::make_shared<LaneletRoute>(route);
  planner_->updateRoute(*current_route_);
}

void ManualLaneChangeHandler::set_preferred_lane(
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

  const auto reroute_availability = sub_reroute_availability_.take_data();
  if (!reroute_availability || !reroute_availability->availability) {
    res->status.success = false;
    res->status.message = "Not in lane driving state. Wait for the current scenario to end.";
    return;
  }

  lanelet::ConstLanelet closest_lanelet =
    get_lanelet_by_id(current_route_->segments.front().preferred_primitive.id);
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
  set_preferred_primitive_req->reset =
    req->lane_change_direction == SetPreferredLane::Request::AUTO;
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

LaneChangeRequestResult ManualLaneChangeHandler::process_lane_change_request(
  const int64_t ego_lanelet_id, const SetPreferredLane::Request::SharedPtr req)
{
  const DIRECTION override_direction =
    req->lane_change_direction == SetPreferredLane::Request::LEFT    ? DIRECTION::MANUAL_LEFT
    : req->lane_change_direction == SetPreferredLane::Request::RIGHT ? DIRECTION::MANUAL_RIGHT
                                                                     : DIRECTION::AUTO;
  if (override_direction == DIRECTION::AUTO) {
    std::vector<autoware_planning_msgs::msg::LaneletPrimitive> preferred_primitives;
    shift_number_ = 0;

    autoware_internal_debug_msgs::msg::Int32Stamped shift_msg;
    shift_msg.stamp = get_clock()->now();
    shift_msg.data = shift_number_;

    pub_shift_number_->publish(shift_msg);

    return {{}, true, "Manual lane selection to AUTO is commanded and executed successfully."};
  }

  if (!current_route_) {
    return {
      {},
      false,
      "Manual lane selection to " +
        (override_direction == DIRECTION::MANUAL_LEFT    ? std::string("left")
         : override_direction == DIRECTION::MANUAL_RIGHT ? std::string("right")
                                                         : std::string("unknown")) +
        std::string(" is commanded but canceled due to no current route available.")};
  }

  LaneletRoute route = *current_route_;

  const auto final_iter = std::prev(route.segments.end());
  auto start_iter = final_iter;

  // Find the segment that contains the ego_lanelet_id
  for (auto iter = route.segments.begin(); iter != route.segments.end(); ++iter) {
    if (iter->primitives.empty()) {
      continue;
    }
    auto start_iter_primitive = std::find_if(
      iter->primitives.begin(), iter->primitives.end(),
      [&ego_lanelet_id](const autoware_planning_msgs::msg::LaneletPrimitive & p) {
        return p.id == ego_lanelet_id;
      });
    if (start_iter_primitive != iter->primitives.end()) {
      start_iter = iter;
      break;
    }
  }

  bool route_updated = false;
  for (auto iter = start_iter; iter != final_iter; ++iter) {
    auto & current_segment = *iter;

    RCLCPP_INFO_STREAM(
      logger_,
      "Current segment preferred primitive ID: " << current_segment.preferred_primitive.id);

    // Safely get next_segment iterator
    auto next_iter = std::next(iter);
    const auto & next_segment = *next_iter;

    // Find the index of the current preferred primitive
    auto current_it = std::find_if(
      current_segment.primitives.begin(), current_segment.primitives.end(),
      [&](const autoware_planning_msgs::msg::LaneletPrimitive & p) {
        return p.id == current_segment.preferred_primitive.id;
      });
    if (current_it == current_segment.primitives.end()) {
      throw std::runtime_error(
        "ManualLaneChangeHandler: Preferred primitive not found in current segment.");
    }

    // Find the index of the next preferred primitive
    auto next_it = std::find_if(
      next_segment.primitives.begin(), next_segment.primitives.end(),
      [&next_segment](const autoware_planning_msgs::msg::LaneletPrimitive & p) {
        return p.id == next_segment.preferred_primitive.id;
      });
    if (next_it == next_segment.primitives.end()) {
      throw std::runtime_error(
        "ManualLaneChangeHandler: Preferred primitive not found in next segment. next_it.id: " +
        std::to_string(next_segment.preferred_primitive.id));
    }

    std::size_t current_index = std::distance(current_segment.primitives.begin(), current_it);

    const auto current_lanelet = get_lanelet_by_id(current_it->id);
    std::string current_turning_dir = current_lanelet.attributeOr("turn_direction", "none");

    const auto next_lanelet = get_lanelet_by_id(next_it->id);
    std::string next_turning_dir = next_lanelet.attributeOr("turn_direction", "none");

    const bool left_shift_not_available =
      (override_direction == DIRECTION::MANUAL_LEFT && current_index == 0);
    const bool right_shift_not_available =
      (override_direction == DIRECTION::MANUAL_RIGHT &&
       current_index + 1 == current_segment.primitives.size());
    const bool next_segment_is_left_turn = (next_turning_dir == "left");
    const bool next_segment_is_right_turn = (next_turning_dir == "right");

    const bool current_segment_shift_not_available =
      left_shift_not_available || right_shift_not_available || next_segment_is_left_turn ||
      next_segment_is_right_turn;

    if (current_segment_shift_not_available) {
      std::string shift_unavailable_reason =
        left_shift_not_available    ? "left shift not available"
        : right_shift_not_available ? "right shift not available"
        : next_segment_is_left_turn ? "next segment is left turn"
                                    : "next segment is right turn";
      RCLCPP_INFO_STREAM(
        logger_, "Cannot shift on the current segment (ID: "
                   << current_segment.preferred_primitive.id << ")");
      break;
    }

    if (override_direction == DIRECTION::MANUAL_LEFT && current_index > 0) {
      // shift to the primitive on the left
      route_updated = true;
      current_segment.preferred_primitive = current_segment.primitives.at(current_index - 1);
      RCLCPP_INFO_STREAM(
        logger_, "Shifted left from "
                   << current_segment.primitives.at(current_index).id
                   << " to primitive ID: " << current_segment.preferred_primitive.id);
    } else if (
      override_direction == DIRECTION::MANUAL_RIGHT &&
      current_index + 1 < current_segment.primitives.size()) {
      // shift to the primitive on the right
      route_updated = true;
      current_segment.preferred_primitive = current_segment.primitives.at(current_index + 1);
      RCLCPP_INFO_STREAM(
        logger_, "Shifted right from "
                   << current_segment.primitives.at(current_index).id
                   << " to primitive ID: " << current_segment.preferred_primitive.id);
    }
  }

  if (!route_updated) {
    return {
      {},
      false,
      std::string("Manual lane selection to ") +
        (override_direction == DIRECTION::MANUAL_LEFT    ? std::string("left")
         : override_direction == DIRECTION::MANUAL_RIGHT ? std::string("right")
                                                         : std::string("unknown")) +
        " is not possible for the current preferred primitive configuration."};
  }

  if (override_direction == DIRECTION::MANUAL_LEFT) {
    shift_number_ -= 1;

    autoware_internal_debug_msgs::msg::Int32Stamped shift_msg;
    shift_msg.stamp = get_clock()->now();
    shift_msg.data = shift_number_;

    pub_shift_number_->publish(shift_msg);
  } else {
    shift_number_ += 1;

    autoware_internal_debug_msgs::msg::Int32Stamped shift_msg;
    shift_msg.stamp = get_clock()->now();
    shift_msg.data = shift_number_;

    pub_shift_number_->publish(shift_msg);
  }

  std::vector<LaneletPrimitive> preferred_primitives;
  preferred_primitives.reserve(route.segments.size());
  std::transform(
    route.segments.begin(), route.segments.end(), std::back_inserter(preferred_primitives),
    [](const auto & segment) { return segment.preferred_primitive; });

  return {
    preferred_primitives, true,
    std::string("Manual lane selection to ") +
      (override_direction == DIRECTION::MANUAL_LEFT    ? std::string("left")
       : override_direction == DIRECTION::MANUAL_RIGHT ? std::string("right")
                                                       : std::string("unknown")) +
      std::string(" is commanded and executed successfully.")};
}

}  // namespace autoware::manual_lane_change_handler

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::manual_lane_change_handler::ManualLaneChangeHandler)
