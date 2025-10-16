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

#include "autoware/pointcloud_preprocessor/downsample_filter/pickup_based_voxel_grid_downsample_filter_node.hpp"

#include "robin_hood.h"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <memory>
#include <string>
#include <vector>

namespace
{
/**
 * @brief Hash function for voxel keys.
 * Utilizes prime numbers to calculate a unique hash for each voxel key.
 */
struct VoxelKeyHash
{
  std::size_t operator()(const std::array<int, 3> & k) const
  {
    // Primes based on the following paper: 'Investigating the Use of Primes in Hashing for
    // Volumetric Data'.
    return (k[0] * 73856093 ^ k[1] * 19349663 ^ k[2] * 83492791);
    // In general, the performance of the search may be improved by restricting the hashkey to the
    // following However, the risk of key collisions also increases, so the value must be
    // appropriate. Enable the following code depending on the situation. return ((1 << 16) - 1) &
    // (k[0] * 73856093 ^ k[1] * 19349663 ^ k[2] * 83492791);
  }
};

/**
 * @brief Equality function for voxel keys.
 * Checks if two voxel keys are equal.
 */
struct VoxelKeyEqual
{
  bool operator()(const std::array<int, 3> & a, const std::array<int, 3> & b) const
  {
    return a == b;
  }
};
}  // namespace

namespace autoware::pointcloud_preprocessor
{
PickupBasedVoxelGridDownsampleFilterComponent::PickupBasedVoxelGridDownsampleFilterComponent(
  const rclcpp::NodeOptions & options)
: Filter("PickupBasedVoxelGridDownsampleFilterComponent", options)
{
  // initialize debug tool
  {
    using autoware_utils::DebugPublisher;
    using autoware_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<DebugPublisher>(this, this->get_name());
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

  // Initialization of voxel sizes from parameters
  voxel_size_.x = declare_parameter<float>("voxel_size_x");
  voxel_size_.y = declare_parameter<float>("voxel_size_y");
  voxel_size_.z = declare_parameter<float>("voxel_size_z");

  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&PickupBasedVoxelGridDownsampleFilterComponent::param_callback, this, _1));
}

using VoxelKey = std::array<int, 3>;
using PointIndexHashMap = robin_hood::unordered_map<VoxelKey, size_t, VoxelKeyHash, VoxelKeyEqual>;

void extract_unique_voxel_point_indices(
  const sensor_msgs::msg::PointCloud2 & input, const VoxelSize & voxel_size,
  PointIndexHashMap & index_map)
{
  constexpr float large_num_offset = 100000.0;
  const float inverse_voxel_size_x = 1.0 / voxel_size.x;
  const float inverse_voxel_size_y = 1.0 / voxel_size.y;
  const float inverse_voxel_size_z = 1.0 / voxel_size.z;

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(input, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(input, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(input, "z");

  // Process each point in the point cloud
  size_t point_index = 0;
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++point_index) {
    const float x = *iter_x;
    const float y = *iter_y;
    const float z = *iter_z;

    // The reason for adding a large value is that when converting from float to int, values around
    // -1 to 1 are all rounded down to 0. Therefore, to prevent the numbers from becoming negative,
    // a large value is added. It has been tuned to reduce computational costs, and deliberately
    // avoids using round or floor functions.
    VoxelKey key = {
      static_cast<int>((x + large_num_offset) * inverse_voxel_size_x),
      static_cast<int>((y + large_num_offset) * inverse_voxel_size_y),
      static_cast<int>((z + large_num_offset) * inverse_voxel_size_z)};
    index_map.emplace(key, point_index);
  }
}

void copy_filtered_points(
  const sensor_msgs::msg::PointCloud2 & input, const PointIndexHashMap & index_map,
  sensor_msgs::msg::PointCloud2 & output)
{
  size_t output_global_offset = 0;
  output.data.resize(index_map.size() * input.point_step);
  for (const auto & kv : index_map) {
    const size_t byte_offset = kv.second * input.point_step;
    std::memcpy(&output.data[output_global_offset], &input.data[byte_offset], input.point_step);
    output_global_offset += input.point_step;
  }

  output.header.frame_id = input.header.frame_id;
  output.height = 1;
  output.fields = input.fields;
  output.is_bigendian = input.is_bigendian;
  output.point_step = input.point_step;
  output.is_dense = input.is_dense;
  output.width = static_cast<uint32_t>(output.data.size() / output.height / output.point_step);
  output.row_step = static_cast<uint32_t>(output.data.size() / output.height);
}

void downsample_with_voxel_grid(
  const sensor_msgs::msg::PointCloud2 & input, const VoxelSize & voxel_size,
  sensor_msgs::msg::PointCloud2 & output)
{
  // Extract unique voxel point indices
  PointIndexHashMap index_map;
  index_map.reserve(input.data.size() / input.point_step);
  extract_unique_voxel_point_indices(input, voxel_size, index_map);

  // Copy the filtered points to the output
  copy_filtered_points(input, index_map, output);
}

void PickupBasedVoxelGridDownsampleFilterComponent::filter(
  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output)
{
  std::scoped_lock lock(mutex_);

  stop_watch_ptr_->toc("processing_time", true);

  // process downsample filter
  downsample_with_voxel_grid(*input, voxel_size_, output);

  // add processing time for debug
  if (debug_publisher_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);

    auto pipeline_latency_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds((this->get_clock()->now() - input->header.stamp).nanoseconds()))
        .count();

    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/pipeline_latency_ms", pipeline_latency_ms);
  }
}

rcl_interfaces::msg::SetParametersResult
PickupBasedVoxelGridDownsampleFilterComponent::param_callback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);

  // Handling dynamic updates for the voxel sizes
  if (get_param(p, "voxel_size_x", voxel_size_.x)) {
    RCLCPP_DEBUG(get_logger(), "Setting new voxel_size_x to: %f.", voxel_size_.x);
  }
  if (get_param(p, "voxel_size_y", voxel_size_.y)) {
    RCLCPP_DEBUG(get_logger(), "Setting new voxel_size_y to: %f.", voxel_size_.y);
  }
  if (get_param(p, "voxel_size_z", voxel_size_.z)) {
    RCLCPP_DEBUG(get_logger(), "Setting new voxel_size_z to: %f.", voxel_size_.z);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::pointcloud_preprocessor::PickupBasedVoxelGridDownsampleFilterComponent)
