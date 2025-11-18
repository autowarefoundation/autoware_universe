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

#include "remaining_distance_time_calculator_node.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>

#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <cstddef>
#include <functional>
#include <iterator>
#include <memory>
#include <numeric>
#include <string>

namespace autoware::remaining_distance_time_calculator
{

RemainingDistanceTimeCalculatorNode::RemainingDistanceTimeCalculatorNode(
  const rclcpp::NodeOptions & options)
: Node("remaining_distance_time_calculator", options),
  is_graph_ready_{false},
  has_received_route_{false},
  has_received_scenario_{false},
  velocity_limit_{99.99},
  remaining_distance_{0.0},
  remaining_time_{0.0}
{
  using std::placeholders::_1;

  sub_odometry_ = create_subscription<Odometry>(
    "~/input/odometry", 1, std::bind(&RemainingDistanceTimeCalculatorNode::on_odometry, this, _1));

  const auto qos_transient_local = rclcpp::QoS{1}.transient_local();

  sub_map_ = create_subscription<HADMapBin>(
    "~/input/map", qos_transient_local,
    std::bind(&RemainingDistanceTimeCalculatorNode::on_map, this, _1));
  sub_route_ = create_subscription<LaneletRoute>(
    "~/input/route", qos_transient_local,
    std::bind(&RemainingDistanceTimeCalculatorNode::on_route, this, _1));
  sub_planning_velocity_ = create_subscription<autoware_internal_planning_msgs::msg::VelocityLimit>(
    "/planning/scenario_planning/current_max_velocity", qos_transient_local,
    std::bind(&RemainingDistanceTimeCalculatorNode::on_velocity_limit, this, _1));
  sub_scenario_ = this->create_subscription<autoware_internal_planning_msgs::msg::Scenario>(
    "~/input/scenario", 1, std::bind(&RemainingDistanceTimeCalculatorNode::on_scenario, this, _1));

  pub_mission_remaining_distance_time_ = create_publisher<MissionRemainingDistanceTime>(
    "~/output/mission_remaining_distance_time",
    rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().reliable());

  debug_processing_time_detail_ = create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms", 1);
  time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>(debug_processing_time_detail_);

  param_listener_ = std::make_shared<::remaining_distance_time_calculator::ParamListener>(
    this->get_node_parameters_interface());
  const auto param = param_listener_->get_params();

  const auto period_ns = rclcpp::Rate(param.update_rate).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&RemainingDistanceTimeCalculatorNode::on_timer, this));
}

void RemainingDistanceTimeCalculatorNode::on_map(const HADMapBin::ConstSharedPtr & msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  road_lanes_ = lanelet::utils::query::roadLanelets(all_lanelets);
  is_graph_ready_ = true;
}

void RemainingDistanceTimeCalculatorNode::compute_route()
{
  lanelet::ConstLanelet current_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(
        road_lanes_, current_vehicle_pose_, &current_lanelet)) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *get_clock(), 3000, "Failed to find current lanelet.");
    return;
  }

  if (!lanelet::utils::query::getClosestLanelet(road_lanes_, goal_pose_, &goal_lanelet_)) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *get_clock(), 3000, "Failed to find goal lanelet.");
    return;
  }

  const lanelet::Optional<lanelet::routing::Route> optional_route =
    routing_graph_ptr_->getRoute(current_lanelet, goal_lanelet_, 0);
  if (!optional_route) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *get_clock(), 3000, "Failed to find proper route.");
    return;
  }

  lanelet::routing::LaneletPath shortest_path;
  shortest_path = optional_route->shortestPath();

  current_lanes_.clear();
  current_lanes_.reserve(shortest_path.size());
  current_lanes_lengths_.clear();
  current_lanes_lengths_.reserve(shortest_path.size());
  for (const auto & llt : shortest_path) {
    current_lanes_lengths_.push_back(lanelet::geometry::length2d(llt));
    current_lanes_.push_back(llt);
  }
}

void RemainingDistanceTimeCalculatorNode::on_route(const LaneletRoute::ConstSharedPtr & msg)
{
  goal_pose_ = msg->goal_pose;
  has_received_route_ = true;
  compute_route();
}

void RemainingDistanceTimeCalculatorNode::on_odometry(const Odometry::ConstSharedPtr & msg)
{
  current_vehicle_pose_ = msg->pose.pose;
  current_vehicle_velocity_ = msg->twist.twist.linear;
}

void RemainingDistanceTimeCalculatorNode::on_velocity_limit(
  const VelocityLimit::ConstSharedPtr & msg)
{
  if (msg->max_velocity > 1e-5) {
    velocity_limit_ = msg->max_velocity;
  }
}

void RemainingDistanceTimeCalculatorNode::on_scenario(
  const autoware_internal_planning_msgs::msg::Scenario::ConstSharedPtr & msg)
{
  scenario_ = msg;
  has_received_scenario_ = true;
}

void RemainingDistanceTimeCalculatorNode::on_timer()
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  if (!has_received_scenario_) {
    return;
  }

  // check if the scenario is parking or not
  if (scenario_->current_scenario == autoware_internal_planning_msgs::msg::Scenario::PARKING) {
    remaining_distance_ = 0.0;
    remaining_time_ = 0.0;
    publish_mission_remaining_distance_time();
  } else if (is_graph_ready_ && has_received_route_) {
    calculate_remaining_distance();
    calculate_remaining_time();
    publish_mission_remaining_distance_time();
  }
}

void RemainingDistanceTimeCalculatorNode::calculate_remaining_distance()
{
  lanelet::ConstLanelet current_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(
        current_lanes_, current_vehicle_pose_, &current_lanelet)) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *get_clock(), 3000, "Failed to find current lanelet.");

    return;
  }

  if (
    current_lanes_.empty() || current_lanes_lengths_.empty() ||
    current_lanes_.size() != current_lanes_lengths_.size()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *get_clock(), 3000, "Current lanes are empty or misconfigured.");
    return;
  }

  auto current_lane_itr = std::find_if(
    current_lanes_.begin(), current_lanes_.end(),
    [&current_lanelet](const lanelet::ConstLanelet & llt) {
      return llt.id() == current_lanelet.id();
    });

  if (current_lane_itr == current_lanes_.end()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *get_clock(), 3000, "Failed to find current lanelet in current lanes.");
    return;
  }

  remaining_distance_ = std::invoke([&]() -> double {
    // remaining distance in current lanelet (if it is not the goal lanelet)
    lanelet::ArcCoordinates arc_coord =
      lanelet::utils::getArcCoordinates({current_lanelet}, current_vehicle_pose_);
    double this_lanelet_length = lanelet::geometry::length2d(current_lanelet);
    double dist_in_current_lanelet =
      (current_lanelet.id() != goal_lanelet_.id()) ? this_lanelet_length - arc_coord.length : 0.0;

    // distance from remaining lanelets between current lanelet and goal lanelet, if there are any
    const auto index = std::distance(current_lanes_.begin(), current_lane_itr);
    double middle_lanes_distance =
      (static_cast<std::size_t>(index + 1) < current_lanes_lengths_.size() - 1)
        ? std::accumulate(
            current_lanes_lengths_.begin() + index + 1, current_lanes_lengths_.end() - 1, 0.0)
        : 0.0;

    // remaining distance in goal lanelet
    double dist_in_goal_lanelet =
      (current_lanelet.id() != goal_lanelet_.id())
        ? lanelet::utils::getArcCoordinates({goal_lanelet_}, goal_pose_).length
        : lanelet::utils::getArcCoordinates({goal_lanelet_}, goal_pose_).length - arc_coord.length;
    return std::max(dist_in_current_lanelet + middle_lanes_distance + dist_in_goal_lanelet, 0.0);
  });
}

void RemainingDistanceTimeCalculatorNode::calculate_remaining_time()
{
  if (velocity_limit_ > 0.0) {
    remaining_time_ = remaining_distance_ / velocity_limit_;
  }
}

void RemainingDistanceTimeCalculatorNode::publish_mission_remaining_distance_time()
{
  MissionRemainingDistanceTime mission_remaining_distance_time;

  mission_remaining_distance_time.remaining_distance = remaining_distance_;
  mission_remaining_distance_time.remaining_time = remaining_time_;
  pub_mission_remaining_distance_time_->publish(mission_remaining_distance_time);
}

}  // namespace autoware::remaining_distance_time_calculator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::remaining_distance_time_calculator::RemainingDistanceTimeCalculatorNode)
