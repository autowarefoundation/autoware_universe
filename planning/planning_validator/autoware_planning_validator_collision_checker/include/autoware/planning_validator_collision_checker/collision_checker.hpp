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

#ifndef AUTOWARE__PLANNING_VALIDATOR_COLLISION_CHECKER__COLLISION_CHECKER_HPP_
#define AUTOWARE__PLANNING_VALIDATOR_COLLISION_CHECKER__COLLISION_CHECKER_HPP_

#include "autoware/planning_validator_collision_checker/parameters.hpp"

#include <autoware/planning_validator/plugin_interface.hpp>

#include <pcl/filters/crop_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace autoware::planning_validator
{
using sensor_msgs::msg::PointCloud2;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

class CollisionChecker : public PluginInterface
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

  void filter_pointcloud(
    PointCloud2::ConstSharedPtr & input,
    PointCloud::Ptr & filtered_point_cloud) const;
  
  CollisionCheckerParams params_;
};

}  // namespace autoware::planning_validator

#endif  // AUTOWARE__PLANNING_VALIDATOR_COLLISION_CHECKER__COLLISION_CHECKER_HPP_
