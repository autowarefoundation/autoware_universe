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

#ifndef ROAD_USER_STOP_MODULE_HPP_
#define ROAD_USER_STOP_MODULE_HPP_

#include "path_length_buffer.hpp"
#include "type_alias.hpp"
#include "types.hpp"

#include <autoware/motion_velocity_planner_common/planner_data.hpp>
#include <autoware/motion_velocity_planner_common/plugin_module_interface.hpp>
#include <autoware/motion_velocity_planner_common/polygon_utils.hpp>
#include <autoware/motion_velocity_planner_common/utils.hpp>
#include <autoware/motion_velocity_planner_common/velocity_planning_result.hpp>
#include <autoware_motion_velocity_road_user_stop_module/road_user_stop_module_parameters.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner
{

class RoadUserStopModule : public PluginModuleInterface
{
public:
  RoadUserStopModule() = default;
  void init(rclcpp::Node & node, const std::string & module_name) override;
  void update_parameters(const std::vector<rclcpp::Parameter> & parameters) override;
  VelocityPlanningResult plan(
    const std::vector<TrajectoryPoint> & raw_trajectory_points,
    const std::vector<TrajectoryPoint> & smoothed_trajectory_points,
    const std::shared_ptr<const PlannerData> planner_data) override;
  std::string get_module_name() const override { return module_name_; }
  void publish_planning_factor() override;
  RequiredSubscriptionInfo getRequiredSubscriptions() const override
  {
    RequiredSubscriptionInfo required_subscription_info;
    required_subscription_info.predicted_objects = true;
    return required_subscription_info;
  }

private:
  // parameter listener
  std::shared_ptr<road_user_stop::ParamListener> param_listener_;
  CommonParam common_param_{};
  rclcpp::Clock::SharedPtr clock_{};

  std::string module_name_;
  rclcpp::Publisher<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr
    processing_time_detail_pub_{};
  mutable std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_{};

  mutable std::unordered_map<std::string, TrackedObject> tracked_objects_;
  mutable DebugData debug_data_;

  mutable std::unordered_map<double, std::vector<Polygon2d>> trajectory_polygon_for_inside_map_{};
  std::vector<StopObstacle> prev_stop_obstacles_{};

  // Previous stop distance info for holding stop position
  std::optional<std::pair<std::vector<TrajectoryPoint>, double>> prev_stop_distance_info_{
    std::nullopt};

  // Path length buffer for negative velocity obstacles
  PathLengthBuffer path_length_buffer_;

  // helper functions
  bool is_target_object(const uint8_t label) const;
  bool is_object_on_road(
    const PredictedObject & object, const std::vector<Polygon2d> & relevant_polygons) const;
  bool is_near_crosswalk(
    const PredictedObject & predicted_object,
    const std::shared_ptr<const PlannerData> & planner_data,
    const lanelet::ConstLanelets & ego_lanelets) const;
  bool is_opposite_traffic_user(
    const PredictedObject & object, const lanelet::ConstLanelet & lanelet) const;

  lanelet::ConstLanelets get_ego_lanelets(
    const std::vector<TrajectoryPoint> & smoothed_trajectory_points,
    const std::shared_ptr<const PlannerData> & planner_data) const;
  RelevantLaneletData get_relevant_lanelet_data(
    const lanelet::ConstLanelets & ego_lanelets,
    const std::shared_ptr<const PlannerData> planner_data) const;

  std::vector<Polygon2d> get_trajectory_polygons(
    const std::vector<TrajectoryPoint> & decimated_traj_points, const VehicleInfo & vehicle_info,
    const Pose & current_ego_pose, const double lat_margin,
    const bool enable_to_consider_current_pose, const double time_to_convergence,
    const double decimate_trajectory_step_length) const;

  std::optional<Point> plan_stop(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & trajectory_points,
    const std::vector<StopObstacle> & stop_obstacles, const double dist_to_bumper);

  std::vector<StopObstacle> filter_stop_obstacles(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<TrajectoryPoint> & decimated_traj_points,
    const std::vector<Polygon2d> & decimated_traj_polygons,
    const RelevantLaneletData & lanelet_data, const rclcpp::Time & current_time,
    const double dist_to_bumper);

  void update_tracked_objects(
    const std::vector<std::shared_ptr<PlannerData::Object>> & objects,
    const rclcpp::Time & current_time);

  bool has_minimum_detection_duration(
    const std::string & object_id, const rclcpp::Time & current_time) const;

  std::optional<StopObstacle> pick_stop_obstacle_from_predicted_object(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::shared_ptr<PlannerData::Object> object,
    const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<TrajectoryPoint> & decimated_traj_points,
    const std::vector<Polygon2d> & decimated_traj_polygons,
    const RelevantLaneletData & lanelet_data, const rclcpp::Time & current_time,
    const double dist_to_bumper);

  // Stop planning
  void hold_previous_stop_if_necessary(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & traj_points,
    std::optional<double> & determined_zero_vel_dist);

  std::optional<Point> calc_stop_point(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & traj_points,
    const std::optional<StopObstacle> & determined_stop_obstacle,
    const std::optional<double> & determined_zero_vel_dist);

  // Stop margin calculation
  double calc_desired_stop_margin(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & traj_points, const StopObstacle & stop_obstacle,
    const double dist_to_bumper, const size_t ego_segment_idx,
    const double dist_to_collide_on_ref_traj) const;

  std::optional<double> calc_candidate_zero_vel_dist(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & traj_points, const double dist_to_collide_on_ref_traj,
    const double desired_stop_margin, const double dist_to_bumper) const;

  double calc_margin_from_obstacle_on_curve(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & traj_points, const StopObstacle & stop_obstacle,
    const double dist_to_bumper, const double default_stop_margin) const;

  MarkerArray create_debug_marker_array() const;
  void publish_debug_info();
};

}  // namespace autoware::motion_velocity_planner

#endif  // ROAD_USER_STOP_MODULE_HPP_
