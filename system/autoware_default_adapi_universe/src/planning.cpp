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

#include "planning.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <autoware_adapi_v1_msgs/msg/planning_behavior.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::default_adapi
{

const std::map<std::uint16_t, std::uint16_t> direction_map = {
  {PlanningFactor::SHIFT_RIGHT, SteeringFactor::RIGHT},
  {PlanningFactor::SHIFT_LEFT, SteeringFactor::LEFT},
  {PlanningFactor::TURN_RIGHT, SteeringFactor::RIGHT},
  {PlanningFactor::TURN_LEFT, SteeringFactor::LEFT}};

const std::map<std::string, std::string> conversion_map = {
  {"behavior_path_planner", PlanningBehavior::INTERSECTION},
  {"static_obstacle_avoidance", PlanningBehavior::AVOIDANCE},
  {"crosswalk", PlanningBehavior::CROSSWALK},
  {"goal_planner", PlanningBehavior::GOAL_PLANNER},
  {"intersection", PlanningBehavior::INTERSECTION},
  {"lane_change_left", PlanningBehavior::LANE_CHANGE},
  {"lane_change_right", PlanningBehavior::LANE_CHANGE},
  {"merge_from_private", PlanningBehavior::MERGE},
  {"no_stopping_area", PlanningBehavior::NO_STOPPING_AREA},
  {"blind_spot", PlanningBehavior::REAR_CHECK},
  {"obstacle_cruise_planner", PlanningBehavior::ROUTE_OBSTACLE},
  {"obstacle_stop_planner", PlanningBehavior::ROUTE_OBSTACLE},
  {"obstacle_stop", PlanningBehavior::ROUTE_OBSTACLE},
  {"obstacle_slow_down", PlanningBehavior::ROUTE_OBSTACLE},
  {"obstacle_cruise", PlanningBehavior::ROUTE_OBSTACLE},
  {"motion_velocity_planner", PlanningBehavior::ROUTE_OBSTACLE},
  {"walkway", PlanningBehavior::SIDEWALK},
  {"start_planner", PlanningBehavior::START_PLANNER},
  {"stop_line", PlanningBehavior::STOP_SIGN},
  {"surround_obstacle_checker", PlanningBehavior::SURROUNDING_OBSTACLE},
  {"traffic_light", PlanningBehavior::TRAFFIC_SIGNAL},
  {"detection_area", PlanningBehavior::USER_DEFINED_DETECTION_AREA},
  {"virtual_traffic_light", PlanningBehavior::VIRTUAL_TRAFFIC_LIGHT},
  {"run_out", PlanningBehavior::RUN_OUT}};

template <class T>
void concat(std::vector<T> & v1, const std::vector<T> & v2)
{
  v1.insert(v1.end(), v2.begin(), v2.end());
}

template <class T>
std::vector<typename rclcpp::Subscription<T>::SharedPtr> init_factors(
  rclcpp::Node * node, std::vector<typename T::ConstSharedPtr> & factors,
  const std::vector<std::string> & topics)
{
  const auto callback = [&factors](const int index) {
    return [&factors, index](const typename T::ConstSharedPtr msg) { factors[index] = msg; };
  };

  std::vector<typename rclcpp::Subscription<T>::SharedPtr> subs;
  for (size_t index = 0; index < topics.size(); ++index) {
    subs.push_back(node->create_subscription<T>(topics[index], rclcpp::QoS(1), callback(index)));
  }
  factors.resize(topics.size());
  return subs;
}

template <class T>
std::vector<T> convert([[maybe_unused]] const std::vector<PlanningFactor> & factors)
{
  static_assert(sizeof(T) == 0, "Only specializations of convert can be used.");
  throw std::logic_error("Only specializations of convert can be used.");
}

template <>
std::vector<VelocityFactor> convert(const std::vector<PlanningFactor> & factors)
{
  std::vector<VelocityFactor> velocity_factors;

  for (const auto & factor : factors) {
    if (factor.behavior != PlanningFactor::SLOW_DOWN && factor.behavior != PlanningFactor::STOP) {
      continue;
    }

    if (factor.control_points.empty()) {
      continue;
    }

    if (conversion_map.count(factor.module) == 0) {
      continue;
    }

    VelocityFactor velocity_factor;
    velocity_factor.behavior = conversion_map.at(factor.module);
    velocity_factor.pose = factor.control_points.front().pose;
    velocity_factor.distance = factor.control_points.front().distance;

    velocity_factors.push_back(velocity_factor);
  }

  return velocity_factors;
}

template <>
std::vector<SteeringFactor> convert(const std::vector<PlanningFactor> & factors)
{
  std::vector<SteeringFactor> steering_factors;

  for (const auto & factor : factors) {
    if (
      factor.behavior != PlanningFactor::SHIFT_RIGHT &&
      factor.behavior != PlanningFactor::SHIFT_LEFT &&
      factor.behavior != PlanningFactor::TURN_RIGHT &&
      factor.behavior != PlanningFactor::TURN_LEFT) {
      continue;
    }

    if (factor.control_points.size() < 2) {
      continue;
    }

    if (conversion_map.count(factor.module) == 0) {
      continue;
    }

    if (direction_map.count(factor.behavior) == 0) {
      continue;
    }

    SteeringFactor steering_factor;
    steering_factor.behavior = conversion_map.at(factor.module);
    steering_factor.direction = direction_map.at(factor.behavior);
    steering_factor.pose = std::array<geometry_msgs::msg::Pose, 2>{
      factor.control_points.front().pose, factor.control_points.back().pose};
    steering_factor.distance = std::array<float, 2>{
      factor.control_points.front().distance, factor.control_points.back().distance};

    steering_factors.push_back(steering_factor);
  }

  return steering_factors;
}

template <class T>
T merge_factors(
  [[maybe_unused]] const rclcpp::Time stamp,
  [[maybe_unused]] const std::vector<PlanningFactorArray::ConstSharedPtr> & factors)
{
  static_assert(sizeof(T) == 0, "Only specializations of merge_factors can be used.");
  throw std::logic_error("Only specializations of merge_factors can be used.");
}

template <>
VelocityFactorArray merge_factors(
  const rclcpp::Time stamp, const std::vector<PlanningFactorArray::ConstSharedPtr> & factors)
{
  VelocityFactorArray message;
  message.header.stamp = stamp;
  message.header.frame_id = "map";

  for (const auto & factor : factors) {
    if (!factor) {
      continue;
    }

    concat<VelocityFactor>(message.factors, convert<VelocityFactor>(factor->factors));
  }
  return message;
}

template <>
SteeringFactorArray merge_factors(
  const rclcpp::Time stamp, const std::vector<PlanningFactorArray::ConstSharedPtr> & factors)
{
  SteeringFactorArray message;
  message.header.stamp = stamp;
  message.header.frame_id = "map";

  for (const auto & factor : factors) {
    if (!factor) {
      continue;
    }

    concat<SteeringFactor>(message.factors, convert<SteeringFactor>(factor->factors));
  }
  return message;
}

PlanningNode::PlanningNode(const rclcpp::NodeOptions & options) : Node("planning", options)
{
  // TODO(Takagi, Isamu): remove default value
  stop_distance_ = declare_parameter<double>("stop_distance", 1.0);
  stop_duration_ = declare_parameter<double>("stop_duration", 1.0);
  stop_checker_ = std::make_unique<VehicleStopChecker>(this, stop_duration_ + 1.0);

  std::vector<std::string> factor_topics = {
    "/planning/planning_factors/behavior_path_planner",
    "/planning/planning_factors/blind_spot",
    "/planning/planning_factors/crosswalk",
    "/planning/planning_factors/detection_area",
    "/planning/planning_factors/dynamic_obstacle_stop",
    "/planning/planning_factors/intersection",
    "/planning/planning_factors/merge_from_private",
    "/planning/planning_factors/no_stopping_area",
    "/planning/planning_factors/obstacle_cruise_planner",
    "/planning/planning_factors/obstacle_stop_planner",
    "/planning/planning_factors/obstacle_stop",
    "/planning/planning_factors/obstacle_cruise",
    "/planning/planning_factors/obstacle_slow_down",
    "/planning/planning_factors/occlusion_spot",
    "/planning/planning_factors/run_out",
    "/planning/planning_factors/stop_line",
    "/planning/planning_factors/surround_obstacle_checker",
    "/planning/planning_factors/traffic_light",
    "/planning/planning_factors/virtual_traffic_light",
    "/planning/planning_factors/walkway",
    "/planning/planning_factors/motion_velocity_planner",
    "/planning/planning_factors/static_obstacle_avoidance",
    "/planning/planning_factors/dynamic_obstacle_avoidance",
    "/planning/planning_factors/avoidance_by_lane_change",
    "/planning/planning_factors/lane_change_left",
    "/planning/planning_factors/lane_change_right",
    "/planning/planning_factors/start_planner",
    "/planning/planning_factors/goal_planner"};

  sub_factors_ = init_factors<PlanningFactorArray>(this, factors_, factor_topics);

  const auto adaptor = autoware::component_interface_utils::NodeAdaptor(this);
  adaptor.init_pub(pub_velocity_factors_);
  adaptor.init_pub(pub_steering_factors_);
  adaptor.init_sub(sub_kinematic_state_, this, &PlanningNode::on_kinematic_state);
  adaptor.init_sub(sub_trajectory_, nullptr);

  const auto rate = rclcpp::Rate(5);
  timer_ = rclcpp::create_timer(this, get_clock(), rate.period(), [this]() { on_timer(); });
}

void PlanningNode::on_kinematic_state(const KinematicState::ConstSharedPtr msg)
{
  kinematic_state_ = msg;

  geometry_msgs::msg::TwistStamped twist;
  twist.header = msg->header;
  twist.twist = msg->twist.twist;
  stop_checker_->addTwist(twist);
}

void PlanningNode::on_timer()
{
  using autoware_adapi_v1_msgs::msg::VelocityFactor;
  auto velocity = merge_factors<VelocityFactorArray>(now(), factors_);
  auto steering = merge_factors<SteeringFactorArray>(now(), factors_);

  sub_trajectory_->take_and_update(trajectory_);

  // Set the distance if it is nan.
  if (trajectory_ && kinematic_state_) {
    for (auto & factor : velocity.factors) {
      if (std::isnan(factor.distance)) {
        const auto & curr_point = kinematic_state_->pose.pose.position;
        const auto & stop_point = factor.pose.position;
        const auto & points = trajectory_->points;
        factor.distance =
          autoware::motion_utils::calcSignedArcLength(points, curr_point, stop_point);
      }
    }
  }

  // Set the status if it is unknown.
  const auto is_vehicle_stopped = stop_checker_->isVehicleStopped(stop_duration_);
  for (auto & factor : velocity.factors) {
    if ((factor.status == VelocityFactor::UNKNOWN) && (!std::isnan(factor.distance))) {
      const auto is_stopped = is_vehicle_stopped && (factor.distance < stop_distance_);
      factor.status = is_stopped ? VelocityFactor::STOPPED : VelocityFactor::APPROACHING;
    }
  }

  for (auto & factor : steering.factors) {
    if ((factor.status == SteeringFactor::UNKNOWN) && (!std::isnan(factor.distance.front()))) {
      const auto is_turning = factor.distance.front() < 0.0;
      factor.status = is_turning ? SteeringFactor::TURNING : SteeringFactor::APPROACHING;
    }
  }

  pub_velocity_factors_->publish(velocity);
  pub_steering_factors_->publish(steering);
}

}  // namespace autoware::default_adapi

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::default_adapi::PlanningNode)
