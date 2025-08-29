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

#ifndef OBSTACLE_SLOW_DOWN_MODULE_HPP_
#define OBSTACLE_SLOW_DOWN_MODULE_HPP_

#include "autoware/motion_velocity_planner_common/polygon_utils.hpp"
#include "autoware/motion_velocity_planner_common/utils.hpp"
#include "autoware_utils/system/stop_watch.hpp"
#include "autoware_utils/system/time_keeper.hpp"
#include "parameters.hpp"
#include "type_alias.hpp"
#include "types.hpp"

#include <autoware/motion_velocity_planner_common/plugin_module_interface.hpp>
#include <autoware/motion_velocity_planner_common/velocity_planning_result.hpp>
#include <autoware/objects_of_interest_marker_interface/objects_of_interest_marker_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner
{
class ObstacleSlowDownModule : public PluginModuleInterface
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
    required_subscription_info.no_ground_pointcloud = true;
    return required_subscription_info;
  }

private:
  std::string module_name_;
  rclcpp::Clock::SharedPtr clock_{};

  // ros parameters
  CommonParam common_param_;
  SlowDownPlanningParam slow_down_planning_param_;
  ObstacleFilteringParam obstacle_filtering_param_;

  // module publisher
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr debug_slow_down_planning_info_pub_;
  rclcpp::Publisher<autoware_utils::ProcessingTimeDetail>::SharedPtr processing_time_detail_pub_;

  // interface publisher
  std::unique_ptr<autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface>
    objects_of_interest_marker_interface_;

  // internal variables
  std::vector<SlowDownObstacle> prev_slow_down_object_obstacles_;
  std::vector<SlowDownOutput> prev_slow_down_output_;
  SlowDownConditionCounter slow_down_condition_counter_;
  Float32MultiArrayStamped slow_down_debug_multi_array_;
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch_;
  mutable std::shared_ptr<DebugData> debug_data_ptr_;
  bool need_to_clear_velocity_limit_{false};
  mutable std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_;
  mutable std::optional<std::vector<Polygon2d>> decimated_traj_polys_{std::nullopt};

  std::vector<autoware::motion_velocity_planner::SlowDownPointData>
  convert_point_cloud_to_slow_down_points(
    const PlannerData::Pointcloud & pointcloud, const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<Polygon2d> & decimated_traj_polys_with_lat_margin,
    const VehicleInfo & vehicle_info, const size_t ego_idx);
  std::vector<SlowDownObstacle> filter_slow_down_obstacle_for_predicted_object(
    const Odometry & odometry, const double ego_nearest_dist_threshold,
    const double ego_nearest_yaw_threshold,
    const std::vector<Polygon2d> & decimated_traj_polys_with_lat_margin,
    const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<std::shared_ptr<PlannerData::Object>> & objects,
    const rclcpp::Time & predicted_objects_stamp, const VehicleInfo & vehicle_info,
    const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check);
  std::vector<SlowDownObstacle> filter_slow_down_obstacle_for_point_cloud(
    const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<Polygon2d> & decimated_traj_polys_with_lat_margin,
    const PlannerData::Pointcloud & point_cloud, const VehicleInfo & vehicle_info, size_t ego_idx);
  std::optional<SlowDownObstacle> create_slow_down_obstacle_for_predicted_object(
    const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<Polygon2d> & decimated_traj_polys_with_lat_margin,
    const std::shared_ptr<PlannerData::Object> object, const rclcpp::Time & predicted_objects_stamp,
    const double dist_from_obj_poly_to_traj_poly);
  SlowDownObstacle create_slow_down_obstacle_for_point_cloud(
    const rclcpp::Time & stamp, const geometry_msgs::msg::Point & front_collision_point,
    const geometry_msgs::msg::Point & back_collision_point, const double lat_dist_to_traj,
    const Side side);
  std::vector<SlowdownInterval> plan_slow_down(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<SlowDownObstacle> & obstacles,
    [[maybe_unused]] std::optional<VelocityLimit> & velocity_limit,
    const VehicleInfo & vehicle_info);
  Float32MultiArrayStamped get_slow_down_planning_debug_message(const rclcpp::Time & current_time);
  void publish_debug_info();
  bool is_slow_down_obstacle(const uint8_t label) const;
  std::optional<std::tuple<double, double, double>>
  calculate_distance_to_slow_down_with_constraints(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & traj_points, const SlowDownObstacle & obstacle,
    const std::optional<SlowDownOutput> & prev_output, const double dist_to_ego,
    const VehicleInfo & vehicle_info, const Motion obstacle_motion) const;
  double calculate_slow_down_velocity(
    const SlowDownObstacle & obstacle, const std::optional<SlowDownOutput> & prev_output,
    const Motion obstacle_motion) const;
  std::vector<Polygon2d> get_decimated_traj_polys(
    const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & current_pose,
    const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
    const double ego_nearest_dist_threshold, const double ego_nearest_yaw_threshold,
    const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check) const;
};
}  // namespace autoware::motion_velocity_planner

#endif  // OBSTACLE_SLOW_DOWN_MODULE_HPP_
