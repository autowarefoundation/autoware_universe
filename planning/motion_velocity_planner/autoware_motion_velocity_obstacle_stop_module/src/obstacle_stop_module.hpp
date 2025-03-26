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

#ifndef OBSTACLE_STOP_MODULE_HPP_
#define OBSTACLE_STOP_MODULE_HPP_

#include "autoware/motion_velocity_planner_common_universe/polygon_utils.hpp"
#include "autoware/motion_velocity_planner_common_universe/utils.hpp"
#include "autoware/object_recognition_utils/predicted_path_utils.hpp"
#include "autoware_utils/system/stop_watch.hpp"
#include "autoware_utils/system/time_keeper.hpp"
#include "metrics_manager.hpp"
#include "parameters.hpp"
#include "stop_planning_debug_info.hpp"
#include "type_alias.hpp"
#include "types.hpp"

#include <autoware/motion_velocity_planner_common_universe/plugin_module_interface.hpp>
#include <autoware/motion_velocity_planner_common_universe/velocity_planning_result.hpp>
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
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner
{
class ObstacleStopModule : public PluginModuleInterface
{
public:
  void init(rclcpp::Node & node, const std::string & module_name) override;
  void update_parameters(const std::vector<rclcpp::Parameter> & parameters) override;
  std::string get_module_name() const override { return module_name_; }

  VelocityPlanningResult plan(
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & raw_trajectory_points,
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & smoothed_trajectory_points,
    const std::shared_ptr<const PlannerData> planner_data) override;

private:
  std::string module_name_{};
  rclcpp::Clock::SharedPtr clock_{};

  // ros parameters
  bool ignore_crossing_obstacle_{};
  bool suppress_sudden_stop_{};
  CommonParam common_param_{};
  StopPlanningParam stop_planning_param_{};
  ObstacleFilteringParam obstacle_filtering_param_{};

  // module publisher
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr debug_stop_planning_info_pub_{};
  rclcpp::Publisher<MetricArray>::SharedPtr metrics_pub_{};
  rclcpp::Publisher<autoware_utils::ProcessingTimeDetail>::SharedPtr processing_time_detail_pub_{};

  // interface publisher
  std::unique_ptr<autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface>
    objects_of_interest_marker_interface_{};

  // internal variables
  mutable StopPlanningDebugInfo stop_planning_debug_info_{};
  mutable std::shared_ptr<DebugData> debug_data_ptr_{};
  std::vector<StopObstacle> prev_closest_stop_obstacles_{};
  std::vector<StopObstacle> prev_stop_obstacles_{};

  // PointCloud-based stop obstacle history
  std::vector<StopObstacle> stop_pointcloud_obstacle_history_;

  MetricsManager metrics_manager_{};
  // previous trajectory and distance to stop
  // NOTE: Previous trajectory is memorized to deal with nearest index search for overlapping or
  // crossing lanes.
  std::optional<std::pair<std::vector<TrajectoryPoint>, double>> prev_stop_distance_info_{
    std::nullopt};
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch_{};
  mutable std::unordered_map<double, std::vector<Polygon2d>> trajectory_polygon_for_inside_map_{};
  mutable std::optional<std::vector<Polygon2d>> trajectory_polygon_for_outside_{std::nullopt};
  mutable std::optional<std::vector<Polygon2d>> decimated_traj_polys_{std::nullopt};
  mutable std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_{};

  std::vector<geometry_msgs::msg::Point> convert_point_cloud_to_stop_points(
    const PlannerData::Pointcloud & pointcloud, const std::vector<TrajectoryPoint> & traj_points,
    const VehicleInfo & vehicle_info, size_t ego_idx);

  std::vector<Polygon2d> get_trajectory_polygon_for_inside(
    const std::vector<TrajectoryPoint> & decimated_traj_points, const VehicleInfo & vehicle_info,
    const geometry_msgs::msg::Pose & current_ego_pose, const double lat_margin,
    const bool enable_to_consider_current_pose, const double time_to_convergence,
    const double decimate_trajectory_step_length) const;

  std::vector<Polygon2d> get_trajectory_polygon_for_outside(
    const std::vector<TrajectoryPoint> & decimated_traj_points, const VehicleInfo & vehicle_info,
    const geometry_msgs::msg::Pose & current_ego_pose, const double lat_margin,
    const bool enable_to_consider_current_pose, const double time_to_convergence,
    const double decimate_trajectory_step_length) const;

  std::vector<StopObstacle> filter_stop_obstacle_for_predicted_object(
    const Odometry & odometry, const double ego_nearest_dist_threshold,
    const double ego_nearest_yaw_threshold, const rclcpp::Time & predicted_objects_stamp,
    const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<TrajectoryPoint> & decimated_traj_points,
    const std::vector<std::shared_ptr<PlannerData::Object>> & objects,
    const bool is_driving_forward, const VehicleInfo & vehicle_info, const double dist_to_bumper,
    const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check);

  std::vector<StopObstacle> filter_stop_obstacle_for_point_cloud(
    const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<TrajectoryPoint> & decimated_traj_points,
    const PlannerData::Pointcloud & point_cloud, const VehicleInfo & vehicle_info,
    const double dist_to_bumper,
    const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check, size_t ego_idx);

  std::optional<geometry_msgs::msg::Point> plan_stop(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<StopObstacle> & stop_obstacles, const double dist_to_bumper);
  double calc_desired_stop_margin(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & traj_points, const StopObstacle & stop_obstacle,
    const double dist_to_bumper, const size_t ego_segment_idx,
    const double dist_to_collide_on_ref_traj);
  std::optional<double> calc_candidate_zero_vel_dist(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & traj_points, const StopObstacle & stop_obstacle,
    const double dist_to_collide_on_ref_traj, const double desired_stop_margin);
  void hold_previous_stop_if_necessary(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & traj_points,
    std::optional<double> & determined_zero_vel_dist);
  std::optional<geometry_msgs::msg::Point> calc_stop_point(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & traj_points, const double dist_to_bumper,
    const std::optional<StopObstacle> & determined_stop_obstacle,
    const std::optional<double> & determined_zero_vel_dist);
  void set_stop_planning_debug_info(
    const std::optional<StopObstacle> & determined_stop_obstacle,
    const std::optional<double> & determined_desired_stop_margin) const;
  void publish_debug_info();

  std::optional<StopObstacle> filter_inside_stop_obstacle_for_predicted_object(
    const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<TrajectoryPoint> & decimated_traj_points,
    const std::shared_ptr<PlannerData::Object> object, const rclcpp::Time & predicted_objects_stamp,
    const double dist_from_obj_poly_to_traj_poly, const VehicleInfo & vehicle_info,
    const double dist_to_bumper,
    const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check) const;
  bool is_inside_stop_obstacle_velocity(
    const std::shared_ptr<PlannerData::Object> object,
    const std::vector<TrajectoryPoint> & traj_points) const;
  bool is_crossing_transient_obstacle(
    const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<TrajectoryPoint> & decimated_traj_points,
    const std::shared_ptr<PlannerData::Object> object, const double dist_to_bumper,
    const std::vector<Polygon2d> & decimated_traj_polys_with_lat_margin,
    const std::optional<std::pair<geometry_msgs::msg::Point, double>> & collision_point) const;

  std::optional<StopObstacle> filter_outside_stop_obstacle_for_predicted_object(
    const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<TrajectoryPoint> & decimated_traj_points,
    const rclcpp::Time & predicted_objects_stamp, const std::shared_ptr<PlannerData::Object> object,
    const double dist_from_obj_poly_to_traj_poly, const bool is_driving_forward,
    const VehicleInfo & vehicle_info, const double dist_to_bumper,
    const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check) const;

  StopObstacle create_stop_obstacle_for_point_cloud(
    const std::vector<TrajectoryPoint> & traj_points, const rclcpp::Time & stamp,
    const geometry_msgs::msg::Point & stop_point, const double dist_to_bumper) const;

  std::optional<std::pair<geometry_msgs::msg::Point, double>>
  create_collision_point_for_outside_stop_obstacle(
    const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<TrajectoryPoint> & decimated_traj_points,
    const std::vector<Polygon2d> & decimated_traj_polys,
    const std::shared_ptr<PlannerData::Object> object, const rclcpp::Time & predicted_objects_stamp,
    const PredictedPath & resampled_predicted_path, double max_lat_margin,
    const bool is_driving_forward, const VehicleInfo & vehicle_info, const double dist_to_bumper,
    const double decimate_trajectory_step_length) const;
  double calc_collision_time_margin(
    const Odometry & odometry, const std::vector<polygon_utils::PointWithStamp> & collision_points,
    const std::vector<TrajectoryPoint> & traj_points, const double dist_to_bumper) const;
  void check_consistency(
    const rclcpp::Time & current_time,
    const std::vector<std::shared_ptr<PlannerData::Object>> & objects,
    std::vector<StopObstacle> & stop_obstacles);
  double calc_margin_from_obstacle_on_curve(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & traj_points, const StopObstacle & stop_obstacle,
    const double dist_to_bumper, const double default_stop_margin) const;
  std::vector<StopObstacle> get_closest_stop_obstacles(
    const std::vector<StopObstacle> & stop_obstacles);
  double get_max_lat_margin(const uint8_t obj_label) const;
  std::vector<Polygon2d> get_decimated_traj_polys(
    const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & current_pose,
    const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
    const double ego_nearest_dist_threshold, const double ego_nearest_yaw_threshold,
    const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check) const;
};
}  // namespace autoware::motion_velocity_planner

#endif  // OBSTACLE_STOP_MODULE_HPP_
