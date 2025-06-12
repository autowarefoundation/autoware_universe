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

#include "autoware/behavior_path_goal_planner_module/pull_over_planner/geometric_pull_over.hpp"

#include "autoware/behavior_path_goal_planner_module/util.hpp"
#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"

#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <memory>
#include <vector>

namespace autoware::behavior_path_planner
{
GeometricPullOver::GeometricPullOver(
  rclcpp::Node & node, const GoalPlannerParameters & parameters, const bool is_forward)
: PullOverPlannerBase{node, parameters},
  parallel_parking_parameters_{parameters.parallel_parking_parameters},
  boundary_departure_checker_{[&]() {
    auto boundary_departure_checker_params = boundary_departure_checker::Param{};
    boundary_departure_checker_params.footprint_extra_margin =
      parameters.lane_departure_check_expansion_margin;
    return BoundaryDepartureChecker{boundary_departure_checker_params, vehicle_info_};
  }()},
  is_forward_{is_forward},
  left_side_parking_{parameters.parking_policy == ParkingPolicy::LEFT_SIDE}
{
  planner_.setParameters(parallel_parking_parameters_);
}

std::optional<PullOverPath> GeometricPullOver::plan(
  const GoalCandidate & modified_goal_pose, const size_t id,
  const std::shared_ptr<const PlannerData> planner_data,
  [[maybe_unused]] const BehaviorModuleOutput & upstream_module_output)
{
  const auto & route_handler = planner_data->route_handler;

  const auto & goal_pose = modified_goal_pose.goal_pose;
  // prepare road nad shoulder lanes
  const auto road_lanes = goal_planner_utils::get_reference_lanelets_for_pullover(
    upstream_module_output.path, planner_data, parameters_.backward_goal_search_length,
    parameters_.forward_goal_search_length);
  const auto pull_over_lanes = goal_planner_utils::getPullOverLanes(
    *route_handler, left_side_parking_, parameters_.backward_goal_search_length,
    parameters_.forward_goal_search_length);
  if (road_lanes.empty() || pull_over_lanes.empty()) {
    return {};
  }

  const auto & p = parallel_parking_parameters_;
  const double max_steer_angle = vehicle_info_.max_steer_angle_rad * p.max_steer_angle_margin_scale;
  planner_.setTurningRadius(planner_data->parameters, max_steer_angle);
  planner_.setPlannerData(planner_data);

  const bool found_valid_path = planner_.planPullOver(
    goal_pose, road_lanes, pull_over_lanes, max_steer_angle, is_forward_, left_side_parking_);
  if (!found_valid_path) {
    return {};
  }

  const auto departure_check_lane = goal_planner_utils::createDepartureCheckLanelet(
    pull_over_lanes, *planner_data->route_handler, left_side_parking_);
  const auto arc_path = planner_.getArcPath();

  // check lane departure with road and shoulder lanes
  // To improve the accuracy of lane departure detection, make the sampling interval finer
  // todo: Implement lane departure detection that does not depend on the footprint
  const auto resampled_arc_path =
    utils::resamplePathWithSpline(arc_path, parameters_.center_line_path_interval / 2);
  if (boundary_departure_checker_.checkPathWillLeaveLane({departure_check_lane}, arc_path))
    return {};

  auto pull_over_path_opt = PullOverPath::create(
    getPlannerType(), id, planner_.getPaths(), planner_.getStartPose(), modified_goal_pose,
    planner_.getPairsTerminalVelocityAndAccel());
  return pull_over_path_opt;
}
}  // namespace autoware::behavior_path_planner
