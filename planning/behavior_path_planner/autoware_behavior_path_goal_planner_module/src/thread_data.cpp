// Copyright 2024 TIER IV, Inc.
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

#include <autoware/behavior_path_goal_planner_module/thread_data.hpp>

#include <memory>

namespace autoware::behavior_path_planner
{

bool is_lane_change_context_expired(
  const std::optional<rclcpp::Time> & last_lane_change_trigger_time_saved,
  const std::optional<rclcpp::Time> & last_lane_change_trigger_time)
{
  if (last_lane_change_trigger_time_saved.has_value() ^ last_lane_change_trigger_time.has_value()) {
    // when "saved" lane change was not detected but now detected, so expired
    // which means between "saved" and now, lane change was triggered

    // when "saved" lane changes was detected but now not detected, so expired
    // which means between "saved" and now, lane change was cancelled
    return true;
  }
  if (!last_lane_change_trigger_time_saved) {
    // lane change not triggered up to now
    return false;
  }
  return (last_lane_change_trigger_time.value() - last_lane_change_trigger_time_saved.value())
           .seconds() > 0.0;
}

void LaneParkingRequest::update(
  const PlannerData & planner_data, const ModuleStatus & current_status,
  const BehaviorModuleOutput & upstream_module_output,
  const std::optional<PullOverPath> & pull_over_path, const PathDecisionState & prev_data,
  const bool trigger_thread_on_approach,
  const std::optional<rclcpp::Time> & last_lane_change_trigger_time)
{
  planner_data_ = std::make_shared<PlannerData>(planner_data);
  planner_data_->route_handler = std::make_shared<RouteHandler>(*(planner_data.route_handler));
  current_status_ = current_status;
  upstream_module_output_ = upstream_module_output;
  pull_over_path_ = pull_over_path;
  prev_data_ = prev_data;
  trigger_thread_on_approach_ = trigger_thread_on_approach;
  last_lane_change_trigger_time_ = last_lane_change_trigger_time;
}

void FreespaceParkingRequest::initializeOccupancyGridMap(
  const PlannerData & planner_data, const GoalPlannerParameters & parameters)
{
  OccupancyGridMapParam occupancy_grid_map_param{};
  const double margin = parameters.occupancy_grid_collision_check_margin;
  occupancy_grid_map_param.vehicle_shape.length =
    planner_data.parameters.vehicle_length + 2 * margin;
  occupancy_grid_map_param.vehicle_shape.width = planner_data.parameters.vehicle_width + 2 * margin;
  occupancy_grid_map_param.vehicle_shape.base2back =
    planner_data.parameters.base_link2rear + margin;
  occupancy_grid_map_param.theta_size = parameters.theta_size;
  occupancy_grid_map_param.obstacle_threshold = parameters.obstacle_threshold;
  occupancy_grid_map_ = std::make_shared<OccupancyGridBasedCollisionDetector>();
  occupancy_grid_map_->setParam(occupancy_grid_map_param);
}

void FreespaceParkingRequest::update(
  const PlannerData & planner_data, const ModuleStatus & current_status,
  const std::optional<PullOverPath> & pull_over_path,
  const std::optional<rclcpp::Time> & last_path_update_time, const bool is_stopped)
{
  planner_data_ = std::make_shared<PlannerData>(planner_data);
  planner_data_->route_handler = std::make_shared<RouteHandler>(*(planner_data.route_handler));
  current_status_ = current_status;
  occupancy_grid_map_->setMap(*(planner_data_->occupancy_grid));
  pull_over_path_ = pull_over_path;
  last_path_update_time_ = last_path_update_time;
  is_stopped_ = is_stopped;
}

}  // namespace autoware::behavior_path_planner
