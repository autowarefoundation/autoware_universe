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

#ifndef REAR_COLLISION_CHECKER_HPP_
#define REAR_COLLISION_CHECKER_HPP_

#include "structs.hpp"

#include <autoware/planning_validator/plugin_interface.hpp>
#include <autoware_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <map>
#include <memory>
#include <string>

namespace autoware::planning_validator
{

class RearCollisionChecker : public PluginInterface
{
public:
  void init(
    rclcpp::Node & node, const std::string & name,
    const std::shared_ptr<PlanningValidatorContext> & context) override;
  void validate(bool & is_critical) override;
  void setup_diag() override;
  std::string get_module_name() const override { return module_name_; };

private:
  void fill_rss_distance(PointCloudObjects & objects) const;

  void fill_velocity(PointCloudObject & pointcloud_object);

  auto filter_pointcloud(DebugData & debug) const -> PointCloud::Ptr;

  auto get_clustered_pointcloud(const PointCloud::Ptr in, DebugData & debug) const
    -> PointCloud::Ptr;

  auto get_pointcloud_object(
    const rclcpp::Time & now, const PointCloud::Ptr & pointcloud_ptr,
    const DetectionAreas & detection_areas, DebugData & debug) -> std::optional<PointCloudObject>;

  auto get_pointcloud_objects(
    const lanelet::ConstLanelets & current_lanes, const Behavior & shift_behavior,
    const Behavior & turn_behavior, DebugData & debug) -> PointCloudObjects;

  auto get_pointcloud_objects_on_adjacent_lane(
    const lanelet::ConstLanelets & current_lanes, const Behavior & shift_behavior,
    const double forward_distance, const double backward_distance,
    const PointCloud::Ptr & obstacle_pointcloud, DebugData & debug) -> PointCloudObjects;

  auto get_pointcloud_objects_at_blind_spot(
    const lanelet::ConstLanelets & current_lanes, const Behavior & turn_behavior,
    const double forward_distance, const double backward_distance,
    const PointCloud::Ptr & obstacle_pointcloud, DebugData & debug) -> PointCloudObjects;

  bool is_safe(const PointCloudObjects & objects, DebugData & debug) const;

  bool is_safe(DebugData & debug);

  void post_process();

  void publish_marker(const DebugData & debug) const;

  void publish_planning_factor(const DebugData & debug) const;

  rclcpp::Publisher<PointCloud2>::SharedPtr pub_voxel_pointcloud_;

  rclcpp::Publisher<PointCloud2>::SharedPtr pub_cluster_pointcloud_;

  rclcpp::Publisher<StringStamped>::SharedPtr pub_string_;

  rclcpp::Publisher<autoware_utils::ProcessingTimeDetail>::SharedPtr
    pub_debug_processing_time_detail_;

  rclcpp::Time last_safe_time_;

  rclcpp::Time last_unsafe_time_;

  std::unique_ptr<rear_collision_checker_node::ParamListener> param_listener_;

  std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_;

  std::map<lanelet::Id, PointCloudObject> history_;
};

}  // namespace autoware::planning_validator

#endif  // REAR_COLLISION_CHECKER_HPP_
