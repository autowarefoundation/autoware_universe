// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS",
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/default_scenario_selector/node.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::scenario_selector
{
namespace
{

std::shared_ptr<lanelet::ConstPolygon3d> findNearestParkinglot(
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr,
  const lanelet::BasicPoint2d & current_position)
{
  const auto linked_parking_lot = std::make_shared<lanelet::ConstPolygon3d>();
  const auto result = lanelet::utils::query::getLinkedParkingLot(
    current_position, lanelet_map_ptr, linked_parking_lot.get());

  if (result) return linked_parking_lot;
  return {};
}

bool isInLane(
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr,
  const geometry_msgs::msg::Point & current_pos)
{
  const lanelet::BasicPoint2d search_point(current_pos.x, current_pos.y);
  std::vector<std::pair<double, lanelet::Lanelet>> nearest_lanelets =
    lanelet::geometry::findNearest(lanelet_map_ptr->laneletLayer, search_point, 1);
  if (nearest_lanelets.empty()) return false;
  const auto dist_to_nearest_lanelet = nearest_lanelets.front().first;
  static constexpr double margin = 0.01;
  return dist_to_nearest_lanelet < margin;
}

bool isAlongLane(
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const geometry_msgs::msg::Pose & current_pose)
{
  lanelet::ConstLanelet closest_lanelet;
  if (!route_handler->getClosestLaneletWithConstrainsWithinRoute(
        current_pose, &closest_lanelet, 0.0, M_PI_4)) {
    return false;
  }
  const lanelet::BasicPoint2d src_point(current_pose.position.x, current_pose.position.y);
  const auto dist_to_centerline =
    lanelet::geometry::distanceToCenterline2d(closest_lanelet, src_point);
  static constexpr double margin = 1.0;
  return dist_to_centerline < margin;
}

bool isInParkingLot(
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr,
  const geometry_msgs::msg::Pose & current_pose)
{
  const auto & p = current_pose.position;
  const lanelet::Point3d search_point(lanelet::InvalId, p.x, p.y, p.z);
  const auto nearest_parking_lot =
    findNearestParkinglot(lanelet_map_ptr, search_point.basicPoint2d());
  if (!nearest_parking_lot) return false;
  return lanelet::geometry::within(search_point, nearest_parking_lot->basicPolygon());
}

bool isNearTrajectoryEnd(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr trajectory,
  const geometry_msgs::msg::Pose & current_pose, const double th_dist)
{
  if (!trajectory || trajectory->points.empty()) return false;
  const auto & p1 = current_pose.position;
  const auto & p2 = trajectory->points.back().pose.position;
  const auto dist = std::hypot(p1.x - p2.x, p1.y - p2.y);
  return dist < th_dist;
}

bool isStopped(
  const std::deque<geometry_msgs::msg::TwistStamped::ConstSharedPtr> & twist_buffer,
  const double th_stopped_velocity_mps)
{
  for (const auto & twist : twist_buffer) {
    if (std::abs(twist->twist.linear.x) > th_stopped_velocity_mps) return false;
  }
  return true;
}

}  // namespace

/* ============================= Plugin API ============================= */

void DefaultScenarioSelector::initialize(rclcpp::Node * node)
{
  node_ = node;

  update_rate_ = node_->declare_parameter<double>("update_rate", 10.0);
  th_max_message_delay_sec_ = node_->declare_parameter<double>("th_max_message_delay_sec", 1.0);
  th_arrived_distance_m_ = node_->declare_parameter<double>("th_arrived_distance_m", 1.0);
  th_stopped_time_sec_ = node_->declare_parameter<double>("th_stopped_time_sec", 1.0);
  th_stopped_velocity_mps_ = node_->declare_parameter<double>("th_stopped_velocity_mps", 0.01);
  enable_mode_switching_ = node_->declare_parameter<bool>("enable_mode_switching", true);

  if (update_rate_ <= 0.0) {
    RCLCPP_WARN(node_->get_logger(), "update_rate <= 0 (%.3f). Forcing to 10.0 Hz.", update_rate_);
    update_rate_ = 10.0;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Selector params: update_rate=%.3f, th_max_delay=%.3f, th_arrived=%.3f, th_stopped_time=%.3f, "
    "th_stopped_vel=%.4f, enable_mode_switching=%s",
    update_rate_, th_max_message_delay_sec_, th_arrived_distance_m_, th_stopped_time_sec_,
    th_stopped_velocity_mps_, enable_mode_switching_ ? "true" : "false");

  lane_driving_stop_time_ = {};
  empty_parking_trajectory_time_ = {};

  /* Inputs */
  sub_lane_driving_trajectory_ =
    node_->create_subscription<autoware_planning_msgs::msg::Trajectory>(
      "input/lane_driving/trajectory", rclcpp::QoS{1},
      std::bind(&DefaultScenarioSelector::onLaneDrivingTrajectory, this, std::placeholders::_1));

  sub_parking_trajectory_ = node_->create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "input/parking/trajectory", rclcpp::QoS{1},
    std::bind(&DefaultScenarioSelector::onParkingTrajectory, this, std::placeholders::_1));

  sub_lanelet_map_ = node_->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "input/lanelet_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&DefaultScenarioSelector::onMap, this, std::placeholders::_1));

  sub_route_ = node_->create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
    "input/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&DefaultScenarioSelector::onRoute, this, std::placeholders::_1));

  sub_odom_ = decltype(sub_odom_)::element_type::create_subscription(
    node_, "input/odometry", rclcpp::QoS{100});

  sub_parking_state_ = decltype(sub_parking_state_)::element_type::create_subscription(
    node_, "is_parking_completed", rclcpp::QoS{1});

  sub_operation_mode_state_ =
    decltype(sub_operation_mode_state_)::element_type::create_subscription(
      node_, "input/operation_mode_state", rclcpp::QoS{1}.transient_local());

  /* Outputs */
  pub_scenario_ = node_->create_publisher<autoware_internal_planning_msgs::msg::Scenario>(
    "output/scenario", rclcpp::QoS{1});
  pub_trajectory_ = node_->create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "output/trajectory", rclcpp::QoS{1});

  /* Timer */
  const auto hz = static_cast<double>(update_rate_);
  const auto period_ns = rclcpp::Rate(hz > 0.0 ? hz : 10.0).period();
  timer_ = rclcpp::create_timer(
    node_, node_->get_clock(), period_ns, std::bind(&DefaultScenarioSelector::onTimer, this));

  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(node_);

  pub_processing_time_ = node_->create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "~/debug/processing_time_ms", 1);
}

bool DefaultScenarioSelector::ready() const
{
  if (!current_pose_) return false;
  if (!route_handler_) return false;
  if (!route_) return false;
  if (!twist_) return false;
  if (!operation_mode_state_) return false;

  if (!route_handler_->isHandlerReady()) return false;

  return true;
}

/* ============================= Original Logic ============================= */

std::string DefaultScenarioSelector::select()
{
  return "default scenario selector";
}

autoware_planning_msgs::msg::Trajectory::ConstSharedPtr
DefaultScenarioSelector::getScenarioTrajectory(const std::string & scenario)
{
  if (scenario == autoware_internal_planning_msgs::msg::Scenario::LANEDRIVING) {
    return lane_driving_trajectory_;
  }
  if (scenario == autoware_internal_planning_msgs::msg::Scenario::PARKING) {
    return parking_trajectory_;
  }
  RCLCPP_ERROR_STREAM(node_->get_logger(), "invalid scenario argument: " << scenario);
  return lane_driving_trajectory_;
}

std::string DefaultScenarioSelector::selectScenarioByPosition()
{
  const auto is_in_lane =
    isInLane(route_handler_->getLaneletMapPtr(), current_pose_->pose.pose.position);
  const auto is_goal_in_lane =
    isInLane(route_handler_->getLaneletMapPtr(), route_->goal_pose.position);
  const auto is_in_parking_lot =
    isInParkingLot(route_handler_->getLaneletMapPtr(), current_pose_->pose.pose);

  if (current_scenario_ == autoware_internal_planning_msgs::msg::Scenario::EMPTY) {
    if (is_in_lane && is_goal_in_lane) {
      return autoware_internal_planning_msgs::msg::Scenario::LANEDRIVING;
    } else if (is_in_parking_lot) {
      return autoware_internal_planning_msgs::msg::Scenario::PARKING;
    }
    return autoware_internal_planning_msgs::msg::Scenario::LANEDRIVING;
  }

  if (current_scenario_ == autoware_internal_planning_msgs::msg::Scenario::LANEDRIVING) {
    if (is_in_parking_lot && !is_goal_in_lane) {
      return autoware_internal_planning_msgs::msg::Scenario::PARKING;
    }
  }

  if (current_scenario_ == autoware_internal_planning_msgs::msg::Scenario::PARKING) {
    if (is_parking_completed_ && is_in_lane) {
      is_parking_completed_ = false;
      return autoware_internal_planning_msgs::msg::Scenario::LANEDRIVING;
    }
  }

  return current_scenario_;
}

void DefaultScenarioSelector::updateCurrentScenario()
{
  const auto prev_scenario = current_scenario_;

  const auto scenario_trajectory = getScenarioTrajectory(current_scenario_);
  const auto is_near_trajectory_end =
    isNearTrajectoryEnd(scenario_trajectory, current_pose_->pose.pose, th_arrived_distance_m_);

  const auto stopped_now = isStopped(twist_buffer_, th_stopped_velocity_mps_);

  if (is_near_trajectory_end && stopped_now) {
    current_scenario_ = selectScenarioByPosition();
  }

  if (enable_mode_switching_) {
    if (isCurrentLaneDriving()) {
      current_scenario_ = isSwitchToParking(stopped_now)
                            ? autoware_internal_planning_msgs::msg::Scenario::PARKING
                            : current_scenario_;
    } else if (isCurrentParking()) {
      current_scenario_ = isSwitchToLaneDriving()
                            ? autoware_internal_planning_msgs::msg::Scenario::LANEDRIVING
                            : current_scenario_;
    }
  }

  if (current_scenario_ != prev_scenario) {
    lane_driving_stop_time_ = {};
    empty_parking_trajectory_time_ = {};
    RCLCPP_DEBUG_STREAM(
      node_->get_logger(), "scenario changed: " << prev_scenario << " -> " << current_scenario_);
  }
}

bool DefaultScenarioSelector::isSwitchToParking(const bool stopped_now)
{
  const auto is_in_parking_lot =
    isInParkingLot(route_handler_->getLaneletMapPtr(), current_pose_->pose.pose);
  const auto is_goal_in_lane =
    isInLane(route_handler_->getLaneletMapPtr(), route_->goal_pose.position);

  if (!stopped_now || !isAutonomous() || !is_in_parking_lot || is_goal_in_lane) {
    lane_driving_stop_time_ = {};
    return false;
  }

  if (!lane_driving_stop_time_) {
    lane_driving_stop_time_ = node_->now();
    return false;
  }

  return (node_->now() - lane_driving_stop_time_.get()).seconds() > lane_stopping_timeout_s;
}

bool DefaultScenarioSelector::isSwitchToLaneDriving()
{
  const auto is_along_lane = isAlongLane(route_handler_, current_pose_->pose.pose);

  if (!isEmptyParkingTrajectory() || !is_along_lane) {
    empty_parking_trajectory_time_ = {};
    return false;
  }

  if (!empty_parking_trajectory_time_) {
    empty_parking_trajectory_time_ = node_->now();
    return false;
  }

  const auto duration = (node_->now() - empty_parking_trajectory_time_.get()).seconds();
  return duration > empty_parking_trajectory_timeout_s;
}

bool DefaultScenarioSelector::isAutonomous() const
{
  return operation_mode_state_ &&
         operation_mode_state_->mode ==
           autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS &&
         operation_mode_state_->is_autoware_control_enabled;
}

bool DefaultScenarioSelector::isEmptyParkingTrajectory() const
{
  if (parking_trajectory_) return parking_trajectory_->points.size() <= 1;
  return false;
}

void DefaultScenarioSelector::onMap(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg)
{
  route_handler_ = std::make_shared<autoware::route_handler::RouteHandler>(*msg);
}

void DefaultScenarioSelector::onRoute(
  const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr msg)
{
  // When the route id is the same (e.g. rerouting with modified goal) keep the current scenario.
  // Otherwise, reset the scenario.
  if (!route_handler_ || route_handler_->getRouteUuid() != msg->uuid) {
    current_scenario_ = autoware_internal_planning_msgs::msg::Scenario::EMPTY;
  }
  route_ = msg;
}

void DefaultScenarioSelector::onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  current_pose_ = msg;
  auto twist = std::make_shared<geometry_msgs::msg::TwistStamped>();
  twist->header = msg->header;
  twist->twist = msg->twist.twist;

  twist_ = twist;
  twist_buffer_.push_back(twist);

  while (!twist_buffer_.empty()) {
    const auto time_diff =
      rclcpp::Time(msg->header.stamp) - rclcpp::Time(twist_buffer_.front()->header.stamp);
    if (time_diff.seconds() < th_stopped_time_sec_) break;
    twist_buffer_.pop_front();
  }
}

bool DefaultScenarioSelector::isDataReady()
{
  if (!current_pose_) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000, "Waiting for current pose.");
    return false;
  }
  if (!route_handler_) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000, "Waiting for route handler.");
    return false;
  }
  if (!route_) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "Waiting for route.");
    return false;
  }
  if (!twist_) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "Waiting for twist.");
    return false;
  }
  if (!operation_mode_state_) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000, "Waiting for operation mode state.");
    return false;
  }

  // Check route handler is ready
  route_handler_->setRoute(*route_);
  if (!route_handler_->isHandlerReady()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000, "Waiting for route handler.");
    return false;
  }
  return true;
}

void DefaultScenarioSelector::updateData()
{
  {
    stop_watch.tic();
  }
  {
    auto msg = sub_parking_state_->take_data();
    is_parking_completed_ = msg ? msg->data : is_parking_completed_;
  }
  {
    auto msgs = sub_odom_->take_data();
    for (const auto & m : msgs) {
      onOdom(m);
    }
  }
  {
    auto msg = sub_operation_mode_state_->take_data();
    if (msg) operation_mode_state_ = msg;
  }
}

void DefaultScenarioSelector::onTimer()
{
  updateData();

  if (!isDataReady()) return;

  // Initialize Scenario
  if (current_scenario_ == autoware_internal_planning_msgs::msg::Scenario::EMPTY) {
    current_scenario_ = selectScenarioByPosition();
  }

  updateCurrentScenario();

  autoware_internal_planning_msgs::msg::Scenario scenario;
  scenario.current_scenario = current_scenario_;
  if (current_scenario_ == autoware_internal_planning_msgs::msg::Scenario::PARKING) {
    scenario.activating_scenarios.push_back(current_scenario_);
  }
  pub_scenario_->publish(scenario);

  // Publish ProcessingTime
  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = node_->get_clock()->now();
  processing_time_msg.data = stop_watch.toc();
  pub_processing_time_->publish(processing_time_msg);
}

void DefaultScenarioSelector::onLaneDrivingTrajectory(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  lane_driving_trajectory_ = msg;
  if (current_scenario_ != autoware_internal_planning_msgs::msg::Scenario::LANEDRIVING) return;
  publishTrajectory(msg);
}

void DefaultScenarioSelector::onParkingTrajectory(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  parking_trajectory_ = msg;
  if (current_scenario_ != autoware_internal_planning_msgs::msg::Scenario::PARKING) return;
  publishTrajectory(msg);
}

void DefaultScenarioSelector::publishTrajectory(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  const auto now = node_->now();
  const auto delay_sec = (now - msg->header.stamp).seconds();
  if (delay_sec <= th_max_message_delay_sec_) {
    pub_trajectory_->publish(*msg);
    if (published_time_publisher_) {
      published_time_publisher_->publish_if_subscribed(pub_trajectory_, msg->header.stamp);
    }
  } else {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), std::chrono::milliseconds(1000).count(),
      "trajectory is delayed: scenario = %s, delay = %f, th_max_message_delay = %f",
      current_scenario_.c_str(), delay_sec, th_max_message_delay_sec_);
  }
}

}  // namespace autoware::scenario_selector

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::scenario_selector::DefaultScenarioSelector,
  autoware::scenario_selector::ScenarioSelectorPlugin)
