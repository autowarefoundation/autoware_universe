// Copyright 2020 Tier IV, Inc. All rights reserved.
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

#include "scene_occlusion_spot.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/trajectory_utils.hpp>
#include <autoware/trajectory/utils/crop.hpp>
#include <autoware/trajectory/utils/find_nearest.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

// turn on only when debugging.
#define DEBUG_PRINT(enable, n, x)                                  \
  if (enable) {                                                    \
    const std::string time_msg = (n) + std::to_string(x);          \
    RCLCPP_INFO_STREAM_THROTTLE(logger_, *clock_, 3000, time_msg); \
  }

namespace autoware::behavior_velocity_planner
{
namespace
{
using utils::PossibleCollisionInfo;

std::vector<PredictedObject> extractStuckVehicle(
  const std::vector<PredictedObject> & vehicles, const double stop_velocity)
{
  std::vector<PredictedObject> stuck_vehicles;
  for (const auto & obj : vehicles) {
    if (utils::isStuckVehicle(obj, stop_velocity)) {
      stuck_vehicles.emplace_back(obj);
    }
  }
  return stuck_vehicles;
}
}  // namespace

OcclusionSpotModule::OcclusionSpotModule(
  const lanelet::Id module_id, const PlannerData & planner_data, const PlannerParam & planner_param,
  const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface)
: SceneModuleInterface(module_id, logger, clock, time_keeper, planning_factor_interface),
  param_(planner_param)
{
  if (param_.detection_method == utils::DETECTION_METHOD::OCCUPANCY_GRID) {
    debug_data_.detection_type = "occupancy";
    //! occupancy grid limitation( 100 * 100 )
    const double max_length = 35.0;  // current available length
    param_.detection_area_length = std::min(max_length, param_.detection_area_length);
  } else if (param_.detection_method == utils::DETECTION_METHOD::PREDICTED_OBJECT) {
    debug_data_.detection_type = "object";
  }
  if (param_.use_partition_lanelet) {
    const lanelet::LaneletMapConstPtr & ll = planner_data.route_handler_->getLaneletMapPtr();
    planning_utils::getAllPartitionLanelets(ll, partition_lanelets_);
  }
}

bool OcclusionSpotModule::modifyPathVelocity(
  Trajectory & path, [[maybe_unused]] const std::vector<Point> & left_bound,
  [[maybe_unused]] const std::vector<Point> & right_bound, const PlannerData & planner_data)
{
  if (param_.is_show_processing_time) {
    stop_watch_.tic("total_processing_time");
  }

  auto new_path = path;

  debug_data_.resetData();

  // set planner data
  {
    param_.v.max_stop_jerk = planner_data.max_stop_jerk_threshold;
    param_.v.max_stop_accel = planner_data.max_stop_acceleration_threshold;
    param_.v.v_ego = planner_data.current_velocity->twist.linear.x;
    param_.v.a_ego = planner_data.current_acceleration->accel.accel.linear.x;
    param_.v.delay_time = planner_data.system_delay;
    param_.detection_area_max_length =
      planning_utils::calcJudgeLineDistWithJerkLimit(
        param_.v.v_ego, param_.v.a_ego, param_.v.non_effective_accel, param_.v.non_effective_jerk,
        planner_data.delay_response_time) +
      param_.detection_area_offset;  // To fill difference between planned and measured acc
  }

  const auto & ego_pose = planner_data.current_odometry->pose;

  new_path.crop(0., param_.detection_area_length);

  const auto s_ego = autoware::experimental::trajectory::find_first_nearest_index(
    new_path, ego_pose, param_.dist_thr, param_.angle_thr);
  if (!s_ego) {
    return true;
  }
  const auto offset_from_start_to_ego = -*s_ego;

  const bool show_time = param_.is_show_processing_time;
  if (show_time) stop_watch_.tic("processing_time");

  if (param_.pass_judge == utils::PASS_JUDGE::SMOOTH_VELOCITY) {
    const auto smoothed_path = smoothPath(new_path, planner_data);
    if (smoothed_path) {
      new_path = smoothed_path.value();
    } else {
      utils::applyVelocityToPath(new_path, param_.v.v_ego);
    }
  } else {
    utils::applyVelocityToPath(new_path, param_.v.v_ego);
  }
  DEBUG_PRINT(show_time, "apply velocity [ms]: ", stop_watch_.toc("processing_time", true));

  if (!utils::buildDetectionAreaPolygons(
        debug_data_.detection_area_polygons, new_path, *s_ego, param_)) {
    return true;  // path point is not enough
  }
  DEBUG_PRINT(show_time, "generate poly[ms]: ", stop_watch_.toc("processing_time", true));

  std::vector<PossibleCollisionInfo> possible_collisions;
  // extract only close lanelet
  if (param_.use_partition_lanelet) {
    planning_utils::extractClosePartition(
      ego_pose.position, partition_lanelets_, debug_data_.close_partition);
  }
  DEBUG_PRINT(show_time, "extract[ms]: ", stop_watch_.toc("processing_time", true));

  const auto objects_ptr = planner_data.predicted_objects;
  const auto vehicles =
    utils::extractVehicles(objects_ptr, ego_pose.position, param_.detection_area_length);
  const std::vector<PredictedObject> filtered_vehicles =
    utils::filterVehiclesByDetectionArea(vehicles, debug_data_.detection_area_polygons);
  DEBUG_PRINT(show_time, "filter obj[ms]: ", stop_watch_.toc("processing_time", true));

  if (param_.detection_method == utils::DETECTION_METHOD::OCCUPANCY_GRID) {
    const auto & occ_grid_ptr = planner_data.occupancy_grid;
    if (!occ_grid_ptr) {
      return true;  // no data
    }

    grid_map::GridMap grid_map;
    Polygons2d stuck_vehicle_foot_prints;
    Polygons2d moving_vehicle_foot_prints;
    utils::categorizeVehicles(
      filtered_vehicles, stuck_vehicle_foot_prints, moving_vehicle_foot_prints,
      param_.stuck_vehicle_vel);

    // occ -> image
    // find out occlusion from erode occlusion candidate num iter is strength of filter
    const int num_iter =
      param_.detection_area.min_occlusion_spot_size / occ_grid_ptr->info.resolution - 1;
    grid_utils::denoiseOccupancyGridCV(
      occ_grid_ptr, stuck_vehicle_foot_prints, moving_vehicle_foot_prints, grid_map, param_.grid,
      param_.is_show_cv_window, num_iter, param_.use_object_info,
      param_.use_moving_object_ray_cast);
    DEBUG_PRINT(show_time, "grid [ms]: ", stop_watch_.toc("processing_time", true));

    // Note: Don't consider offset from path start to ego here
    if (!utils::generatePossibleCollisionsFromGridMap(
          possible_collisions, grid_map, new_path, offset_from_start_to_ego, param_, debug_data_)) {
      // no occlusion spot
      return true;
    }
  } else if (param_.detection_method == utils::DETECTION_METHOD::PREDICTED_OBJECT) {
    const auto stuck_vehicles = extractStuckVehicle(filtered_vehicles, param_.stuck_vehicle_vel);
    // Note: Don't consider offset from path start to ego here
    if (!utils::generatePossibleCollisionsFromObjects(
          possible_collisions, new_path, param_, offset_from_start_to_ego, stuck_vehicles)) {
      // no occlusion spot
      return true;
    }
  }
  DEBUG_PRINT(show_time, "occlusion [ms]: ", stop_watch_.toc("processing_time", true));
  DEBUG_PRINT(show_time, "num collision:", possible_collisions.size());

  utils::calcSlowDownPointsForPossibleCollision(new_path, 0.0, possible_collisions);

  // Note: Consider offset from path start to ego here
  utils::handleCollisionOffset(possible_collisions, offset_from_start_to_ego);

  // apply safe velocity using ebs and pbs deceleration
  utils::applySafeVelocityConsideringPossibleCollision(
    new_path, possible_collisions, debug_data_.debug_poses, param_);
  debug_data_.baselink_to_front = param_.baselink_to_front;

  // these debug topics needs computation resource
  debug_data_.z = new_path.compute(0).point.pose.position.z;
  debug_data_.possible_collisions = possible_collisions;

  path = new_path;
  DEBUG_PRINT(show_time, "total [ms]: ", stop_watch_.toc("total_processing_time", true));

  return true;
}

}  // namespace autoware::behavior_velocity_planner
