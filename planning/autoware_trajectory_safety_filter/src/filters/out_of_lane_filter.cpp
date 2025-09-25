// Copyright 2025 TIER IV, Inc.
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

#include "autoware/trajectory_safety_filter/filters/out_of_lane_filter.hpp"

#include <rclcpp/duration.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/geometry/Point.h>

#include <algorithm>
#include <any>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::trajectory_safety_filter::plugin
{

namespace
{
autoware_internal_planning_msgs::msg::PathWithLaneId convert_to_path_with_lane_id(
  const TrajectoryPoints & traj_points, double max_check_time)
{
  autoware_internal_planning_msgs::msg::PathWithLaneId path;
  path.header.stamp = rclcpp::Clock().now();
  path.header.frame_id = "map";

  for (const auto & traj_point : traj_points) {
    if (rclcpp::Duration(traj_point.time_from_start).seconds() > max_check_time) {
      break;
    }

    autoware_internal_planning_msgs::msg::PathPointWithLaneId path_point;
    path_point.point.pose = traj_point.pose;

    // Set velocity
    const double vel = std::sqrt(
      traj_point.longitudinal_velocity_mps * traj_point.longitudinal_velocity_mps +
      traj_point.lateral_velocity_mps * traj_point.lateral_velocity_mps);
    path_point.point.longitudinal_velocity_mps = vel;
    path_point.point.lateral_velocity_mps = 0.0;
    path_point.point.heading_rate_rps = traj_point.heading_rate_rps;

    // Lane IDs will be empty for this check
    path.points.push_back(path_point);
  }

  return path;
}
}  // namespace

OutOfLaneFilter::OutOfLaneFilter() : SafetyFilterInterface("OutOfLaneFilter")
{
  // BoundaryDepartureChecker will be initialized when vehicle_info is set
}

void OutOfLaneFilter::set_parameters(const std::unordered_map<std::string, std::any> & params)
{
  auto get_value = [&params](const std::string & key, auto & value) {
    auto it = params.find(key);
    if (it != params.end()) {
      try {
        value = std::any_cast<std::decay_t<decltype(value)>>(it->second);
      } catch (const std::bad_any_cast &) {
        // Keep default value if cast fails
      }
    }
  };

  get_value("max_check_time", params_.max_check_time);
  get_value("min_value", params_.min_value);

  // Initialize boundary departure checker if vehicle_info is available
  if (!boundary_departure_checker_ && vehicle_info_ptr_) {
    autoware::boundary_departure_checker::Param bdc_param{};
    boundary_departure_checker_ =
      std::make_unique<autoware::boundary_departure_checker::BoundaryDepartureChecker>(
        bdc_param, *vehicle_info_ptr_);
  }
}

bool OutOfLaneFilter::is_feasible(
  const TrajectoryPoints & traj_points, const FilterContext & context)
{
  // Check required context data
  if (
    !context.lanelet_map || !context.odometry || traj_points.empty() ||
    !boundary_departure_checker_) {
    return true;
  }

  const auto path = convert_to_path_with_lane_id(traj_points, params_.max_check_time);

  // Use boundary departure checker to verify if path will leave lane
  const bool will_leave_lane =
    boundary_departure_checker_->checkPathWillLeaveLane(context.lanelet_map, path);

  // Return false if the path will leave the lane
  return !will_leave_lane;
}
}  // namespace autoware::trajectory_safety_filter::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_safety_filter::plugin::OutOfLaneFilter,
  autoware::trajectory_safety_filter::plugin::SafetyFilterInterface)
