// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/cuda_scan_ground_segmentation/cuda_scan_ground_segmentation_filter_node.hpp"

#include "autoware/pointcloud_preprocessor/utility/memory.hpp"

#include <autoware_utils/math/unit_conversion.hpp>

namespace autoware::cuda_ground_segmentation
{

using autoware_utils::deg2rad;
CudaScanGroundSegmentationFilterNode::CudaScanGroundSegmentationFilterNode(
  const rclcpp::NodeOptions & options)
: Node("cuda_scan_ground_segmentation_filter_node", options)
{
  // Delare processing time debug
  {
    using autoware_utils::DebugPublisher;
    using autoware_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ptr_ = std::make_unique<DebugPublisher>(this, "cuda_scan_ground_filter");
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }
  // Declare parameters
  FilterParameters filter_parameters;
  // global parameters
  filter_parameters.max_radius = static_cast<float>(declare_parameter<double>("max_radius"));

  // common parameters
  filter_parameters.sector_angle_rad =
    static_cast<float>(deg2rad(declare_parameter<double>("sector_angle_deg")));
  filter_parameters.inv_sector_angle_rad = 1.0f / filter_parameters.sector_angle_rad;
  filter_parameters.num_sectors =
    static_cast<uint32_t>(std::ceil(2.0 * M_PI * filter_parameters.inv_sector_angle_rad));

  // common thresholds
  filter_parameters.global_slope_max_angle_rad =
    static_cast<float>(deg2rad(declare_parameter<double>("global_slope_max_angle_deg")));
  filter_parameters.local_slope_max_angle_rad =
    static_cast<float>(deg2rad(declare_parameter<double>("local_slope_max_angle_deg")));
  filter_parameters.global_slope_max_ratio = std::tan(filter_parameters.global_slope_max_angle_rad);
  filter_parameters.local_slope_max_ratio = std::tan(filter_parameters.local_slope_max_angle_rad);
  filter_parameters.split_points_distance_tolerance =
    static_cast<float>(declare_parameter<double>("split_points_distance_tolerance"));

  // cell mode parameters
  filter_parameters.use_recheck_ground_cluster =
    static_cast<uint32_t>(declare_parameter<bool>("use_recheck_ground_cluster"));
  filter_parameters.recheck_start_distance =
    static_cast<float>(declare_parameter<double>("recheck_start_distance"));
  filter_parameters.detection_range_z_max =
    static_cast<float>(declare_parameter<double>("detection_range_z_max"));
  filter_parameters.non_ground_height_threshold =
    static_cast<float>(declare_parameter<double>("non_ground_height_threshold"));

  // cell parameters
  filter_parameters.cell_divider_size_m =
    static_cast<float>(declare_parameter<double>("grid_size_m"));
  filter_parameters.max_num_cells_per_sector =
    static_cast<uint32_t>(filter_parameters.max_radius / filter_parameters.cell_divider_size_m);
  filter_parameters.max_num_cells = static_cast<uint32_t>(
    filter_parameters.max_num_cells_per_sector * filter_parameters.num_sectors);
  // filter_parameters.gnd_cell_buffer_size =
  //   static_cast<uint32_t>(declare_parameter<int>("gnd_cell_buffer_size"));

  int64_t max_mem_pool_size_in_byte =
    declare_parameter<int64_t>("max_mem_pool_size_in_byte", 1e9);  // 1 GB
  // Initialize CUDA blackboard subscriber
  sub_ =
    std::make_shared<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/input/pointcloud",
      std::bind(
        &CudaScanGroundSegmentationFilterNode::cudaPointCloudCallback, this,
        std::placeholders::_1));

  // Initialize CUDA blackboard publisher
  pub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/output/pointcloud");

  pub_gnd_ =
    std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/output/ground_pointcloud");

  cuda_ground_segmentation_filter_ = std::make_unique<CudaScanGroundSegmentationFilter>(
    filter_parameters, max_mem_pool_size_in_byte);
}

void CudaScanGroundSegmentationFilterNode::cudaPointCloudCallback(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & msg)
{
  // start time measurement
  if (stop_watch_ptr_) {
    stop_watch_ptr_->tic("processing_time");
  }
  // Create unique_ptr first
  auto non_ground_unique = std::make_unique<cuda_blackboard::CudaPointCloud2>();
  auto ground_unique = std::make_unique<cuda_blackboard::CudaPointCloud2>();

  // Create shared_ptr from raw pointers for the function call
  auto non_ground_shared = std::shared_ptr<cuda_blackboard::CudaPointCloud2>(
    non_ground_unique.get(), [](auto *) {});  // no-op deleter
  auto ground_shared =
    std::shared_ptr<cuda_blackboard::CudaPointCloud2>(ground_unique.get(), [](auto *) {});

  cuda_ground_segmentation_filter_->classifyPointcloud(msg, non_ground_shared, ground_shared);

  // Publish using the original unique_ptr
  pub_->publish(std::move(non_ground_unique));
  pub_gnd_->publish(std::move(ground_unique));

  // end time measurement
  if (debug_publisher_ptr_ && stop_watch_ptr_) {
    stop_watch_ptr_->toc("processing_time");
    stop_watch_ptr_->toc("cyclic_time");
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
  }
}

}  // namespace autoware::cuda_ground_segmentation

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::cuda_ground_segmentation::CudaScanGroundSegmentationFilterNode)
