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

#ifndef PLANNING__AUTOWARE_MINIMUM_RULE_BASED_PLANNER__PLUGINS__OBSTACLE_STOP_HPP_
#define PLANNING__AUTOWARE_MINIMUM_RULE_BASED_PLANNER__PLUGINS__OBSTACLE_STOP_HPP_

#include "autoware/trajectory_modifier/trajectory_modifier_utils/obstacle_stop_utils.hpp"
#include "plugin_interface.hpp"

#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::minimum_rule_based_planner::plugin
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_utils_geometry::MultiPolygon2d;
using autoware_utils_geometry::Polygon2d;
using trajectory_modifier::utils::obstacle_stop::CollisionPoint;
using trajectory_modifier::utils::obstacle_stop::DebugData;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using trajectory_modifier::utils::obstacle_stop::PointCloud2;

class ObstacleStop : public PluginInterface
{
public:
  ObstacleStop() = default;

  void run(TrajectoryPoints & traj_points) override;

  void update_params(const MinimumRuleBasedPlannerParams & params) override
  {
    params_ = params.obstacle_stop;
  }
  const MinimumRuleBasedPlannerParams::ObstacleStop & get_params() const { return params_; }

protected:
  void on_initialize(const MinimumRuleBasedPlannerParams & params) override;

private:
  MinimumRuleBasedPlannerParams::ObstacleStop params_;

  struct CollisionPointBuffer
  {
    std::vector<CollisionPoint> pcd;
    std::vector<CollisionPoint> objects;

    bool empty() const { return pcd.empty() && objects.empty(); }
  } collision_points_buffer_;

  std::optional<CollisionPoint> nearest_collision_point_;

  DebugData debug_data_;

  std::unique_ptr<trajectory_modifier::utils::obstacle_stop::PointCloudFilter> pointcloud_filter_;

  bool is_obstacle_detected(const TrajectoryPoints & traj_points);

  std::optional<CollisionPoint> check_predicted_objects(const TrajectoryPoints & traj_points);
  std::optional<CollisionPoint> check_pointcloud(const TrajectoryPoints & traj_points);

  void update_collision_points_buffer(
    std::vector<CollisionPoint> & collision_points_buffer, const TrajectoryPoints & traj_points,
    const std::optional<CollisionPoint> & collision_point);

  std::optional<CollisionPoint> get_nearest_collision_point() const;

  void set_stop_point(TrajectoryPoints & traj_points);
};

}  // namespace autoware::minimum_rule_based_planner::plugin

#endif  // PLANNING__AUTOWARE_MINIMUM_RULE_BASED_PLANNER__PLUGINS__OBSTACLE_STOP_HPP_
