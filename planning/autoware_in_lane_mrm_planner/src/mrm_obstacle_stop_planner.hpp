// Copyright 2026 TIER IV, Inc.
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

#ifndef MRM_OBSTACLE_STOP_PLANNER_HPP_
#define MRM_OBSTACLE_STOP_PLANNER_HPP_

#include "obstacle_stop_utils.hpp"
#include "type_alias.hpp"

#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::in_lane_mrm_planner
{
using autoware_internal_debug_msgs::msg::StringStamped;
using autoware_internal_planning_msgs::msg::PlanningFactor;
using autoware_internal_planning_msgs::msg::SafetyFactor;
using autoware_internal_planning_msgs::msg::SafetyFactorArray;
using autoware_utils_geometry::MultiPolygon2d;
using autoware_utils_geometry::Polygon2d;
using obstacle_stop::CollisionPoint;
using obstacle_stop::DebugData;
using obstacle_stop::ObjectDecelMap;
using obstacle_stop::ObjectType;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

class MrmObstacleStopPlanner
{
public:
  void initialize(rclcpp::Node * node, const VehicleInfo & vehicle_info, const Params & params);
  void set_input(
    const Odometry & odom, const AccelWithCovarianceStamped & accel,
    const PredictedObjects & objects);
  void set_obstacle_pointcloud(const PointCloud2 & pointcloud);
  void update_params(const Params & params);
  void apply(TrajectoryPoints & traj_points);
  void publish_planning_factor();

private:
  using ObstacleStopParams = Params::ObstacleStop;

  void update_object_decel_map();

  bool is_obstacle_detected(const TrajectoryPoints & traj_points);

  std::optional<CollisionPoint> check_predicted_objects(const TrajectoryPoints & traj_points);
  std::optional<CollisionPoint> check_pointcloud(const TrajectoryPoints & traj_points);

  void update_collision_points_buffer(
    std::vector<CollisionPoint> & collision_points_buffer, const TrajectoryPoints & traj_points,
    const std::optional<CollisionPoint> & collision_point);

  std::optional<CollisionPoint> get_nearest_collision_point() const;

  void set_stop_point(TrajectoryPoints & traj_points);

  void publish_debug_string(bool is_safe) const;
  void publish_debug_data(const std::string & ns) const;

  rclcpp::Node * node_{nullptr};
  VehicleInfo vehicle_info_;
  ObstacleStopParams params_;

  Odometry odom_;
  AccelWithCovarianceStamped accel_;
  PredictedObjects objects_;
  std::optional<PointCloud2> obstacle_pointcloud_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  struct CollisionPointBuffer
  {
    std::vector<CollisionPoint> pcd;
    std::vector<CollisionPoint> objects;

    bool empty() const { return pcd.empty() && objects.empty(); }
  } collision_points_buffer_;

  std::optional<CollisionPoint> nearest_collision_point_;

  SafetyFactorArray safety_factors_;

  DebugData debug_data_;

  std::unique_ptr<obstacle_stop::PointCloudFilter> pointcloud_filter_;

  std::unique_ptr<obstacle_stop::ObjectFilter> object_filter_;

  ObjectDecelMap object_decel_map_;

  std::unique_ptr<autoware::planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface_;

  rclcpp::Publisher<MarkerArray>::SharedPtr debug_viz_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_clustered_pointcloud_;
  rclcpp::Publisher<StringStamped>::SharedPtr pub_debug_text_;
};

}  // namespace autoware::in_lane_mrm_planner

#endif  // MRM_OBSTACLE_STOP_PLANNER_HPP_
