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

#include "autoware/cuda_pointcloud_preprocessor/cuda_pointcloud2costmap_filter/cuda_pointcloud2costmap_filter_node.hpp"

#include "autoware/cuda_pointcloud_preprocessor/cuda_pointcloud2costmap_filter/cuda_pointcloud2costmap_filter.hpp"
#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"
#include "autoware/cuda_pointcloud_preprocessor/types.hpp"

#include <grid_map_ros/GridMapRosConverter.hpp>

namespace autoware::cuda_pointcloud_preprocessor
{
CudaPointcloud2CostmapFilterNode::CudaPointcloud2CostmapFilterNode(
  const rclcpp::NodeOptions & node_options)
: Node("cuda_pointcloud2costmap_filter", node_options)
{
  // Get costmap parameters
  costmap_frame_ = declare_parameter<std::string>("costmap_frame", "map");
  costmap_params_.grid_length_x = declare_parameter<double>("grid_length_x", 100.0);
  costmap_params_.grid_length_y = declare_parameter<double>("grid_length_y", 100.0);
  costmap_params_.grid_resolution = declare_parameter<double>("grid_resolution", 0.5);
  costmap_params_.grid_position_x = declare_parameter<double>("grid_position_x", 0.0);
  costmap_params_.grid_position_y = declare_parameter<double>("grid_position_y", 0.0);
  costmap_params_.maximum_height_thres = declare_parameter<double>("maximum_height_thres", 2.0);
  costmap_params_.minimum_height_thres = declare_parameter<double>("minimum_height_thres", -2.0);
  costmap_params_.grid_min_value = declare_parameter<double>("grid_min_value", 0.0);
  costmap_params_.grid_max_value = declare_parameter<double>("grid_max_value", 1.0);

  int64_t max_mem_pool_size_in_byte = declare_parameter<int64_t>("max_mem_pool_size_in_byte", 1e9);

  if (max_mem_pool_size_in_byte < 0) {
    RCLCPP_ERROR(
      this->get_logger(), "Invalid pool size was specified. The value should be positive");
    return;
  }

  // Create subscriber
  sub_ =
    std::make_shared<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/input/pointcloud",
      std::bind(
        &CudaPointcloud2CostmapFilterNode::cudaPointcloudCallback, this, std::placeholders::_1));

  // Create publisher
  pub_costmap_ = this->create_publisher<grid_map_msgs::msg::GridMap>("~/output/grid_map", 1);

  // Create filter
  cuda_pointcloud2costmap_filter_ =
    std::make_unique<CudaPointcloud2CostmapFilter>(costmap_params_, max_mem_pool_size_in_byte);
}

void CudaPointcloud2CostmapFilterNode::cudaPointcloudCallback(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr msg)
{
  // Validate point_step matches one of the supported formats
  constexpr size_t point_xyzirc_size = sizeof(OutputPointType);
  constexpr size_t point_xyzircaedt_size = sizeof(InputPointType);
  const bool is_point_xyzirc = (msg->point_step == point_xyzirc_size);
  const bool is_point_xyzircaedt = (msg->point_step == point_xyzircaedt_size);

  if (!is_point_xyzirc && !is_point_xyzircaedt) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Point cloud point_step mismatch! Expected PointXYZIRC (%zu bytes) or "
      "PointXYZIRCAEDT (%zu bytes), but got %u bytes. Point cloud will be skipped.",
      point_xyzirc_size, point_xyzircaedt_size, msg->point_step);
    return;
  }

  // Validate data pointer
  if (!msg->data) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Point cloud data pointer is null! Point cloud will be skipped.");
    return;
  }

  // Process the point cloud with error handling
  try {
    // Create grid map
    grid_map::GridMap grid_map;
    grid_map.setFrameId(costmap_frame_);
    grid_map.setGeometry(
      grid_map::Length(costmap_params_.grid_length_x, costmap_params_.grid_length_y),
      costmap_params_.grid_resolution,
      grid_map::Position(costmap_params_.grid_position_x, costmap_params_.grid_position_y));
    grid_map.add("points", costmap_params_.grid_min_value);

    // Generate costmap
    grid_map::Matrix costmap_matrix;
    cuda_pointcloud2costmap_filter_->generateCostmap(msg, costmap_matrix);
    grid_map["points"] = costmap_matrix;

    // Set timestamp
    grid_map.setTimestamp(rclcpp::Time(msg->header.stamp).nanoseconds());

    // Convert to message and publish
    auto grid_map_msg = grid_map::GridMapRosConverter::toMessage(grid_map);
    pub_costmap_->publish(*grid_map_msg);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Exception in costmap generation: %s. Costmap will not be published.", e.what());
  } catch (...) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Unknown exception in costmap generation. Costmap will not be published.");
  }
}
}  // namespace autoware::cuda_pointcloud_preprocessor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::cuda_pointcloud_preprocessor::CudaPointcloud2CostmapFilterNode)
