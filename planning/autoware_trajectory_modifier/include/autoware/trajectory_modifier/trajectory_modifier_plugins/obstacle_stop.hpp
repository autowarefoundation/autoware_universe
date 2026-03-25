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

#ifndef AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__OBSTACLE_STOP_HPP_
#define AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__OBSTACLE_STOP_HPP_

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/trajectory_modifier_plugin_base.hpp"
#include "autoware/trajectory_modifier/trajectory_modifier_utils/obstacle_stop_utils.hpp"
#include "autoware/trajectory_modifier/trajectory_modifier_utils/utils.hpp"

#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_modifier::plugin
{
using autoware_internal_planning_msgs::msg::SafetyFactor;
using autoware_internal_planning_msgs::msg::SafetyFactorArray;
using autoware_utils_geometry::MultiPolygon2d;
using autoware_utils_geometry::Polygon2d;
using utils::obstacle_stop::CollisionPoint;
using utils::obstacle_stop::DebugData;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

class ObstacleStop : public TrajectoryModifierPluginBase
{
public:
  ObstacleStop() = default;

  bool modify_trajectory(TrajectoryPoints & traj_points) override;

  [[nodiscard]] bool is_trajectory_modification_required(
    const TrajectoryPoints & traj_points) override;

  void update_params(const TrajectoryModifierParams & params) override
  {
    params_ = params.obstacle_stop;
    enabled_ = params.use_obstacle_stop;
    trajectory_time_step_ = params.trajectory_time_step;
  }
  const TrajectoryModifierParams::ObstacleStop & get_params() const { return params_; }

  void publish_debug_data([[maybe_unused]] const std::string & ns) const override;

protected:
  void on_initialize(const TrajectoryModifierParams & params) override;

private:
  TrajectoryModifierParams::ObstacleStop params_;

  std::optional<CollisionPoint> nearest_collision_point_;

  DebugData debug_data_;

  std::unique_ptr<utils::obstacle_stop::PointCloudFilter> pointcloud_filter_;

  SafetyFactorArray safety_factors_;

  MarkerArray marker_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_viz_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_clustered_pointcloud_;

  std::optional<CollisionPoint> check_predicted_objects(const TrajectoryPoints & traj_points);
  std::optional<CollisionPoint> check_pointcloud(const TrajectoryPoints & traj_points);

  bool set_stop_point(TrajectoryPoints & traj_points);

  bool apply_stopping(
    TrajectoryPoints & traj_points, const double target_stop_point_arc_length) const;
};

}  // namespace autoware::trajectory_modifier::plugin

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__OBSTACLE_STOP_HPP_
