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

#ifndef AUTOWARE__PLANNING_VALIDATOR_INTERSECTION_COLLISION_CHECKER__INTERSECTION_COLLISION_CHECKER_HPP_  // NOLINT
#define AUTOWARE__PLANNING_VALIDATOR_INTERSECTION_COLLISION_CHECKER__INTERSECTION_COLLISION_CHECKER_HPP_  // NOLINT

#include "autoware/planning_validator_intersection_collision_checker/types.hpp"

#include <autoware/planning_validator/plugin_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <pcl/common/transforms.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

namespace autoware::planning_validator
{
using sensor_msgs::msg::PointCloud2;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

class IntersectionCollisionChecker : public PluginInterface
{
public:
  void init(
    rclcpp::Node & node, const std::string & name,
    const std::shared_ptr<PlanningValidatorContext> & context) override;
  void validate(bool & is_critical) override;
  void setup_diag() override;
  std::string get_module_name() const override { return module_name_; };

private:
  void setup_parameters(rclcpp::Node & node);

  [[nodiscard]] EgoTrajectory get_ego_trajectory() const;

  [[nodiscard]] Direction get_turn_direction(
    const lanelet::ConstLanelets & trajectory_lanelets) const;

  [[nodiscard]] Direction get_lanelets(
    CollisionCheckerLanelets & lanelets, const EgoTrajectory & ego_trajectory) const;

  void filter_pointcloud(
    PointCloud2::ConstSharedPtr & input, PointCloud::Ptr & filtered_point_cloud) const;

  void get_points_within(
    const PointCloud::Ptr & input, const BasicPolygon2d & polygon,
    const PointCloud::Ptr & output) const;

  void cluster_pointcloud(const PointCloud::Ptr & input, PointCloud::Ptr & output) const;

  void set_lanelets_debug_marker(const CollisionCheckerLanelets & lanelets) const;

  bool check_collision(
    const TargetLanelets & target_lanelets, const PointCloud::Ptr & filtered_point_cloud,
    const rclcpp::Time & time_stamp);

  std::optional<PCDObject> get_pcd_object(
    const rclcpp::Time & time_stamp, const PointCloud::Ptr & filtered_point_cloud,
    const TargetLanelet & target_lanelet) const;

  CollisionCheckerParams params_;
  PCDObjectsMap history_;
  std::optional<rclcpp::Time> last_invalid_time_;
};

}  // namespace autoware::planning_validator

// clang-format off
#endif  // AUTOWARE__PLANNING_VALIDATOR_INTERSECTION_COLLISION_CHECKER__INTERSECTION_COLLISION_CHECKER_HPP_  // NOLINT
// clang-format on
