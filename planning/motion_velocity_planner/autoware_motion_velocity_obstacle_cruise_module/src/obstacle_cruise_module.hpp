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

#ifndef OBSTACLE_CRUISE_MODULE_HPP_
#define OBSTACLE_CRUISE_MODULE_HPP_

#include "autoware/motion_velocity_planner_common/polygon_utils.hpp"
#include "autoware/motion_velocity_planner_common/utils.hpp"
#include "autoware_utils/system/stop_watch.hpp"
#include "autoware_utils/system/time_keeper.hpp"
#include "cruise_planner_interface.hpp"
#include "optimization_based_planner/optimization_based_planner.hpp"
#include "parameters.hpp"
#include "pid_based_planner/pid_based_planner.hpp"
#include "type_alias.hpp"
#include "types.hpp"

#include <autoware/motion_velocity_planner_common/plugin_module_interface.hpp>
#include <autoware/motion_velocity_planner_common/velocity_planning_result.hpp>
#include <autoware/objects_of_interest_marker_interface/objects_of_interest_marker_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::motion_velocity_planner
{
class ObstacleCruiseModule : public PluginModuleInterface
{
public:
  void init(rclcpp::Node & node, const std::string & module_name) override;
  void update_parameters(const std::vector<rclcpp::Parameter> & parameters) override;
  VelocityPlanningResult plan(
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & raw_trajectory_points,
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & smoothed_trajectory_points,
    const std::shared_ptr<const PlannerData> planner_data) override;
  std::string get_module_name() const override;
  RequiredSubscriptionInfo getRequiredSubscriptions() const override
  {
    RequiredSubscriptionInfo required_subscription_info;
    required_subscription_info.predicted_objects = true;
    return required_subscription_info;
  }

private:
  std::string module_name_;
  rclcpp::Clock::SharedPtr clock_{};

  // ros parameters
  std::string planning_algorithm_;
  CommonParam common_param_;
  CruisePlanningParam cruise_planning_param_;
  ObstacleFilteringParam obstacle_filtering_param_;

  // common publisher
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr debug_cruise_planning_info_pub_;

  // module publisher
  std::unique_ptr<autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface>
    objects_of_interest_marker_interface_;
  rclcpp::Publisher<autoware_utils::ProcessingTimeDetail>::SharedPtr processing_time_detail_pub_;

  // cruise planner
  std::unique_ptr<CruisePlannerInterface> cruise_planner_;

  // internal variables
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch_;
  std::vector<CruiseObstacle> prev_cruise_object_obstacles_;
  mutable std::shared_ptr<DebugData> debug_data_ptr_;
  bool need_to_clear_velocity_limit_{false};
  mutable std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_;

  std::unique_ptr<CruisePlannerInterface> create_cruise_planner(rclcpp::Node & node) const;
  std::vector<CruiseObstacle> filter_cruise_obstacle_for_predicted_object(
    const Odometry & odometry, const double ego_nearest_dist_threshold,
    const double ego_nearest_yaw_threshold, const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<std::shared_ptr<PlannerData::Object>> & objects,
    const rclcpp::Time & predicted_objects_stamp, const bool is_driving_forward,
    const VehicleInfo & vehicle_info,
    const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check);
  void publish_debug_info();
  std::optional<CruiseObstacle> create_cruise_obstacle(
    const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<TrajectoryPoint> & decimated_traj_points,
    const std::vector<Polygon2d> & decimated_traj_polys,
    const std::shared_ptr<PlannerData::Object> object, const rclcpp::Time & predicted_objects_stamp,
    const double dist_from_obj_poly_to_traj_poly, const bool is_driving_forward,
    const VehicleInfo & vehicle_info,
    const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check) const;
  std::optional<std::vector<CruiseObstacle>> find_yield_cruise_obstacles(
    const Odometry & odometry, const std::vector<std::shared_ptr<PlannerData::Object>> & objects,
    const rclcpp::Time & predicted_objects_stamp, const std::vector<TrajectoryPoint> & traj_points,
    const VehicleInfo & vehicle_info);
  std::optional<std::vector<polygon_utils::PointWithStamp>>
  create_collision_points_for_inside_cruise_obstacle(
    const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<TrajectoryPoint> & decimated_traj_points,
    const std::vector<Polygon2d> & decimated_traj_polys,
    const std::shared_ptr<PlannerData::Object> object, const rclcpp::Time & predicted_objects_stamp,
    const bool is_driving_forward, const VehicleInfo & vehicle_info,
    const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check) const;
  std::optional<std::vector<polygon_utils::PointWithStamp>>
  create_collision_points_for_outside_cruise_obstacle(
    const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<TrajectoryPoint> & decimated_traj_points,
    const std::vector<Polygon2d> & decimated_traj_polys,
    const std::shared_ptr<PlannerData::Object> object, const rclcpp::Time & predicted_objects_stamp,
    const bool is_driving_forward, const VehicleInfo & vehicle_info,
    const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check) const;
  std::optional<CruiseObstacle> create_yield_cruise_obstacle(
    const std::shared_ptr<PlannerData::Object> object,
    const std::shared_ptr<PlannerData::Object> stopped_object,
    const rclcpp::Time & predicted_objects_stamp, const std::vector<TrajectoryPoint> & traj_points);
  bool is_inside_cruise_obstacle(const uint8_t label) const;
  bool is_outside_cruise_obstacle(const uint8_t label) const;
  bool is_side_stopped_obstacle(const uint8_t label) const;
  bool is_cruise_obstacle(const uint8_t label) const;
  bool is_front_collide_obstacle(
    const std::vector<TrajectoryPoint> & traj_points,
    const std::shared_ptr<PlannerData::Object> object, const size_t first_collision_idx) const;
  bool is_obstacle_crossing(
    const std::vector<TrajectoryPoint> & traj_points,
    const std::shared_ptr<PlannerData::Object> object) const;
};
}  // namespace autoware::motion_velocity_planner

#endif  // OBSTACLE_CRUISE_MODULE_HPP_
