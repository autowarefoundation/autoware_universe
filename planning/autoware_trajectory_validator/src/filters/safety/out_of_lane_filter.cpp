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

#include "autoware/trajectory_validator/filters/safety/out_of_lane_filter.hpp"

#include <rclcpp/duration.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/geometry/Point.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety
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

OutOfLaneFilter::OutOfLaneFilter() : ValidatorInterface("OutOfLaneFilter")
{
  // BoundaryDepartureChecker will be initialized when vehicle_info is set
}

void OutOfLaneFilter::update_parameters(const validator::Params & params)
{
  params_.max_check_time = params.out_of_lane.time;
  params_.min_value = params.out_of_lane.min_value;
}

OutOfLaneFilter::result_t OutOfLaneFilter::is_feasible(
  const TrajectoryPoints & traj_points, const FilterContext & context)
{
  // Check required context data
  if (
    !context.lanelet_map || !context.odometry || traj_points.empty() ||
    !boundary_departure_checker_) {
    return tl::make_unexpected("Insufficient context data");
  }

  const auto path = convert_to_path_with_lane_id(traj_points, params_.max_check_time);

  // Use boundary departure checker to verify if path will leave lane
  const bool will_leave_lane =
    boundary_departure_checker_->checkPathWillLeaveLane(context.lanelet_map, path);

  // Set metrics as ERROR if the path will leave the lane
  std::vector<MetricReport> metrics{
    autoware_trajectory_validator::build<MetricReport>()
      .validator_name(get_name())
      .validator_category(category())
      .metric_name("check_out_of_lane")
      .metric_value(0.0)
      .level(will_leave_lane ? MetricReport::ERROR : MetricReport::OK)};

  const auto is_feasible = !will_leave_lane;

  return ValidationResult{is_feasible, std::move(metrics)};
}

}  // namespace autoware::trajectory_validator::plugin::safety

#include <pluginlib/class_list_macros.hpp>
namespace safety = autoware::trajectory_validator::plugin::safety;

PLUGINLIB_EXPORT_CLASS(
  safety::OutOfLaneFilter, autoware::trajectory_validator::plugin::ValidatorInterface)
