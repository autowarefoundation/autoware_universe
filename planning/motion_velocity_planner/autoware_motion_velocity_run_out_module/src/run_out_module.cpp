// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#include "run_out_module.hpp"

#include "collision.hpp"
#include "debug.hpp"
#include "decision.hpp"
#include "footprints.hpp"
#include "map_data.hpp"
#include "objects_filtering.hpp"
#include "parameters.hpp"
#include "slowdown.hpp"
#include "types.hpp"

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/objects_of_interest_marker_interface/marker_data.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/published_time_publisher.hpp>
#include <rclcpp/duration.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::motion_velocity_planner
{

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

void RunOutModule::init_parameters(rclcpp::Node & node)
{
  params_.initialize(node, ns_);
}

void RunOutModule::update_parameters(const std::vector<rclcpp::Parameter> & parameters)
{
  params_.update(parameters, ns_);
}

void RunOutModule::init(rclcpp::Node & node, const std::string & module_name)
{
  diagnostic_updater_.emplace(&node);
  module_name_ = module_name;
  logger_ = node.get_logger();
  clock_ = node.get_clock();

  debug_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/" + ns_ + "/debug_markers", 1);
  virtual_wall_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/" + ns_ + "/virtual_walls", 1);
  processing_diag_publisher_ = std::make_shared<autoware_utils::ProcessingTimePublisher>(
    &node, "~/debug/" + ns_ + "/processing_time_ms_diag");
  processing_time_publisher_ =
    node.create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "~/debug/" + ns_ + "/processing_time_ms", 1);
  debug_trajectory_publisher_ = node.create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "~/debug/" + ns_ + "/trajectory", 1);
  timekeeper_publisher_ = node.create_publisher<autoware::universe_utils::ProcessingTimeDetail>(
    "~/" + ns_ + "/processing_time", 1);
  time_keeper_ = std::make_shared<autoware::universe_utils::TimeKeeper>(timekeeper_publisher_);

  init_parameters(node);
  diagnostic_updater_->setHardwareID("mvp_run_out");
  diagnostic_updater_->add(
    "unavoidable_run_out_collision", this, &RunOutModule::update_unfeasible_stop_status);

  objects_of_interest_marker_interface_ = std::make_unique<
    autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface>(&node, ns_);
  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      &node, "mvp_run_out");
}

double calculate_keep_stop_distance_range(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const run_out::Parameters & params)
{
  std::vector<double> times = {0.0};
  std::vector<double> arc_lengths = {0.0};
  for (auto i = 1UL; i < trajectory.size(); ++i) {
    const auto t = rclcpp::Duration(trajectory[i].time_from_start).seconds();
    if (times.back() >= t) {
      // we skip trajectory points where the predicted time decreases to avoid interpolation errors
      continue;
    }
    const auto arc_length_delta = universe_utils::calcDistance2d(trajectory[i - 1], trajectory[i]);
    times.push_back(t);
    arc_lengths.push_back(arc_lengths.back() + arc_length_delta);
  }
  if (params.keep_stop_condition_time > times.back()) {
    return arc_lengths.back();
  }
  return autoware::interpolation::lerp(times, arc_lengths, params.keep_stop_condition_time) +
         params.keep_stop_condition_distance;
}

void RunOutModule::update_unfeasible_stop_status(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (unfeasible_stop_deceleration_.has_value()) {
    const std::string error_msg = "[RunOut]: Unfeasible stop";
    const auto diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    stat.summary(diag_level, error_msg);
  } else {
    const std::string error_msg = "[RunOut]: Nominal";
    const auto diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    stat.summary(diag_level, error_msg);
  }
  stat.addf("Unfeasible deceleration", "%2.2f", unfeasible_stop_deceleration_.value_or(0.0));
  unfeasible_stop_deceleration_.reset();
}

void RunOutModule::publish_debug_trajectory(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const VelocityPlanningResult & planning_result)
{
  autoware_planning_msgs::msg::Trajectory debug_trajectory;
  debug_trajectory.header.frame_id = "map";
  debug_trajectory.header.stamp = clock_->now();
  debug_trajectory.points = trajectory;
  for (const auto & stop_point : planning_result.stop_points) {
    const auto length = motion_utils::calcSignedArcLength(debug_trajectory.points, 0, stop_point);
    motion_utils::insertStopPoint(length, debug_trajectory.points);
  }
  for (const auto & slowdown_point : planning_result.slowdown_intervals) {
    const auto from_seg_idx =
      autoware::motion_utils::findNearestSegmentIndex(debug_trajectory.points, slowdown_point.from);
    const auto from_insert_idx = autoware::motion_utils::insertTargetPoint(
      from_seg_idx, slowdown_point.from, debug_trajectory.points);
    const auto to_seg_idx =
      autoware::motion_utils::findNearestSegmentIndex(debug_trajectory.points, slowdown_point.to);
    const auto to_insert_idx = autoware::motion_utils::insertTargetPoint(
      to_seg_idx, slowdown_point.to, debug_trajectory.points);
    if (from_insert_idx && to_insert_idx) {
      for (auto idx = *from_insert_idx; idx <= *to_insert_idx; ++idx) {
        debug_trajectory.points[idx].longitudinal_velocity_mps = std::min(
          debug_trajectory.points[idx].longitudinal_velocity_mps,
          static_cast<float>(slowdown_point.velocity));
      }
    }
  }
  debug_trajectory_publisher_->publish(debug_trajectory);
}

void RunOutModule::add_planning_factors(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const run_out::RunOutResult & result,
  const std::unordered_map<std::string, autoware_internal_planning_msgs::msg::SafetyFactor> &
    safety_factor_per_object)
{
  if (trajectory.empty()) {
    return;
  }
  geometry_msgs::msg::Pose p;
  for (auto i = 0UL; i < result.velocity_planning_result.slowdown_intervals.size(); ++i) {
    const auto & slowdown = result.velocity_planning_result.slowdown_intervals[i];
    p.position = slowdown.from;
    autoware_internal_planning_msgs::msg::SafetyFactorArray safety_array;
    safety_array.is_safe = false;
    if (safety_factor_per_object.count(result.slowdown_objects[i]) > 0UL) {
      safety_array.factors = {safety_factor_per_object.at(result.slowdown_objects[i])};
    }
    planning_factor_interface_->add(
      trajectory, trajectory.front().pose, p, PlanningFactor::SLOW_DOWN, safety_array, true,
      slowdown.velocity);
  }
  for (auto i = 0UL; i < result.velocity_planning_result.stop_points.size(); ++i) {
    p.position = result.velocity_planning_result.stop_points[i];
    autoware_internal_planning_msgs::msg::SafetyFactorArray safety_array;
    safety_array.is_safe = false;
    if (safety_factor_per_object.count(result.stop_objects[i]) > 0UL) {
      safety_array.factors = {safety_factor_per_object.at(result.stop_objects[i])};
    }
    planning_factor_interface_->add(
      trajectory, trajectory.front().pose, p, PlanningFactor::STOP, safety_array, true, 0.0);
  }
}

VelocityPlanningResult RunOutModule::plan(
  [[maybe_unused]] const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> &,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & smoothed_trajectory_points,
  const std::shared_ptr<const PlannerData> planner_data)
{
  const auto now = clock_->now();
  time_keeper_->start_track("plan()");
  time_keeper_->start_track("calc_ego_footprint()");
  const auto ego_footprint = calculate_trajectory_corner_footprint(
    smoothed_trajectory_points, planner_data->vehicle_info_, params_);
  time_keeper_->end_track("calc_ego_footprint()");
  time_keeper_->start_track("filter_objects()");
  const auto filtering_data = run_out::calculate_filtering_data(
    planner_data->route_handler->getLaneletMapPtr(), ego_footprint, planner_data->objects, params_);
  auto filtered_objects = run_out::prepare_dynamic_objects(
    planner_data->objects, ego_footprint, decisions_tracker_, filtering_data, params_);
  time_keeper_->end_track("filter_objects()");
  time_keeper_->start_track("calc_collisions()");
  params_.ignore_collision_conditions.if_ego_arrives_first_and_cannot_stop
    .calculated_stop_time_limit =
    planner_data->current_odometry.twist.twist.linear.x /
    params_.ignore_collision_conditions.if_ego_arrives_first_and_cannot_stop.deceleration_limit;
  run_out::calculate_collisions(
    filtered_objects, ego_footprint, filtering_data,
    planner_data->vehicle_info_.max_longitudinal_offset_m, params_);
  time_keeper_->end_track("calc_collisions()");
  time_keeper_->start_track("calc_decisions()");
  const auto keep_stop_distance_range =
    calculate_keep_stop_distance_range(smoothed_trajectory_points, params_);
  const auto safety_factor_per_object = run_out::calculate_decisions(
    decisions_tracker_, filtered_objects, now, keep_stop_distance_range, params_);
  for (const auto & obj : filtered_objects) {
    if (!decisions_tracker_.get(obj.uuid)) {
      continue;
    }
    auto color = objects_of_interest_marker_interface::ColorName::GREEN;
    if (decisions_tracker_.get(obj.uuid)->decisions.back().type == run_out::stop) {
      color = objects_of_interest_marker_interface::ColorName::RED;
    }
    if (decisions_tracker_.get(obj.uuid)->decisions.back().type == run_out::slowdown) {
      color = objects_of_interest_marker_interface::ColorName::AMBER;
    }
    objects_of_interest_marker_interface_->insertObjectData(
      obj.object->predicted_object.kinematics.initial_pose_with_covariance.pose,
      obj.object->predicted_object.shape, color);
  }
  time_keeper_->end_track("calc_decisions()");
  time_keeper_->start_track("calc_slowdowns()");
  const auto result = run_out::calculate_slowdowns(
    decisions_tracker_, smoothed_trajectory_points,
    planner_data->current_odometry.twist.twist.linear.x, unfeasible_stop_deceleration_, params_);
  diagnostic_updater_->force_update();
  time_keeper_->end_track("calc_slowdowns()");

  time_keeper_->start_track("publish_debug()");
  virtual_wall_marker_creator.add_virtual_walls(
    run_out::create_virtual_walls(
      result.velocity_planning_result, smoothed_trajectory_points,
      planner_data->vehicle_info_.max_longitudinal_offset_m));
  virtual_wall_publisher_->publish(virtual_wall_marker_creator.create_markers(now));
  add_planning_factors(smoothed_trajectory_points, result, safety_factor_per_object);
  if (debug_publisher_->get_subscription_count() > 0) {
    const auto & filtering_data_to_publish =
      filtering_data[run_out::Parameters::string_to_label(params_.debug.object_label)];
    debug_publisher_->publish(
      run_out::make_debug_markers(
        ego_footprint, filtered_objects, decisions_tracker_, smoothed_trajectory_points,
        filtering_data_to_publish, params_));
  }
  publish_debug_trajectory(smoothed_trajectory_points, result.velocity_planning_result);
  objects_of_interest_marker_interface_->publishMarkerArray();
  time_keeper_->end_track("publish_debug()");

  time_keeper_->end_track("plan()");
  return result.velocity_planning_result;
}

}  // namespace autoware::motion_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::motion_velocity_planner::RunOutModule,
  autoware::motion_velocity_planner::PluginModuleInterface)
