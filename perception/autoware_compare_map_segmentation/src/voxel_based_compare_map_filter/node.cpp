// Copyright 2020 Tier IV, Inc.
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

#include "node.hpp"

#include "autoware_utils/ros/debug_publisher.hpp"
#include "autoware_utils/system/stop_watch.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/segment_differences.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::compare_map_segmentation
{

VoxelBasedCompareMapFilterComponent::VoxelBasedCompareMapFilterComponent(
  const rclcpp::NodeOptions & options)
: Filter("VoxelBasedCompareMapFilter", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  // initialize debug tool
  {
    using autoware_utils::DebugPublisher;
    using autoware_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<DebugPublisher>(this, "voxel_based_compare_map_filter");
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

  distance_threshold_ = declare_parameter<double>("distance_threshold");
  bool use_dynamic_map_loading = declare_parameter<bool>("use_dynamic_map_loading");
  double downsize_ratio_z_axis = declare_parameter<double>("downsize_ratio_z_axis");
  if (downsize_ratio_z_axis <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "downsize_ratio_z_axis should be positive");
    return;
  }
  set_map_in_voxel_grid_ = false;
  if (use_dynamic_map_loading) {
    rclcpp::CallbackGroup::SharedPtr main_callback_group;
    main_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    voxel_grid_map_loader_ = std::make_unique<VoxelGridDynamicMapLoader>(
      this, distance_threshold_, downsize_ratio_z_axis, &tf_input_frame_, main_callback_group);
  } else {
    voxel_grid_map_loader_ = std::make_unique<VoxelGridStaticMapLoader>(
      this, distance_threshold_, downsize_ratio_z_axis, &tf_input_frame_);
  }
  tf_input_frame_ = *(voxel_grid_map_loader_->tf_map_input_frame_);
  RCLCPP_INFO(this->get_logger(), "tf_map_input_frame: %s", tf_input_frame_.c_str());
}

// TODO(badai-nguyen): Temporary Implementation of input_indices_callback and  convert_output_costly
// functions; Delete this override function when autoware_utils refactor
// (https://github.com/autowarefoundation/autoware_utils/pull/50) or new ManagedTransformBuffer lib
// is deployed for autoware
void VoxelBasedCompareMapFilterComponent::input_indices_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud, const PointIndicesConstPtr indices)
{
  // If cloud is given, check if it's valid
  if (!isValid(cloud)) {
    RCLCPP_ERROR(this->get_logger(), "[input_indices_callback] Invalid input!");
    return;
  }
  // If indices are given, check if they are valid
  if (indices && !isValid(indices)) {
    RCLCPP_ERROR(this->get_logger(), "[input_indices_callback] Invalid indices!");
    return;
  }

  // Check whether the user has given a different input TF frame
  tf_input_orig_frame_ = cloud->header.frame_id;
  PointCloud2ConstPtr cloud_tf;
  if (cloud->header.frame_id != tf_input_frame_) {
    RCLCPP_DEBUG(
      this->get_logger(), "[input_indices_callback] Transforming input dataset from %s to %s.",
      cloud->header.frame_id.c_str(), tf_input_frame_.c_str());
    // Save the original frame ID
    // Convert the cloud into the different frame
    PointCloud2 cloud_transformed;

    try {
      // Lookup the transform from input frame to "map"
      geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_.lookupTransform(
        tf_input_frame_, tf_input_orig_frame_, rclcpp::Time(cloud->header.stamp),
        rclcpp::Duration::from_seconds(0.0));

      // Transform the point cloud
      tf2::doTransform(*cloud, cloud_transformed, transform_stamped);
      cloud_tf = std::make_shared<PointCloud2>(cloud_transformed);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, "Could not transform pointcloud: %s",
        ex.what());
      cloud_tf = cloud;  // Fallback to original data
    }
  } else {
    cloud_tf = cloud;
  }
  // Need setInputCloud () here because we have to extract x/y/z
  IndicesPtr vindices;
  if (indices) {
    vindices.reset(new std::vector<int>(indices->indices));
  }

  computePublish(cloud_tf, vindices);
}

bool VoxelBasedCompareMapFilterComponent::convert_output_costly(
  std::unique_ptr<PointCloud2> & output)
{
  if (!output || output->data.empty() || output->fields.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Invalid output point cloud!");
    return false;
  }
  if (
    pcl::getFieldIndex(*output, "x") == -1 || pcl::getFieldIndex(*output, "y") == -1 ||
    pcl::getFieldIndex(*output, "z") == -1) {
    RCLCPP_ERROR(this->get_logger(), "Input pointcloud does not have xyz fields");
    return false;
  }
  if (!tf_output_frame_.empty() && output->header.frame_id != tf_output_frame_) {
    auto cloud_transformed = std::make_unique<PointCloud2>();
    try {
      geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_.lookupTransform(
        tf_output_frame_, output->header.frame_id, rclcpp::Time(output->header.stamp),
        rclcpp::Duration::from_seconds(0.0));
      tf2::doTransform(*output, *cloud_transformed, transform_stamped);
      cloud_transformed->header.frame_id = tf_output_frame_;
      output = std::move(cloud_transformed);
    } catch (tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, "Could not transform pointcloud: %s",
        e.what());
      return false;
    }
  }

  if (tf_output_frame_.empty() && output->header.frame_id != tf_input_orig_frame_) {
    auto cloud_transformed = std::make_unique<PointCloud2>();
    try {
      geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_.lookupTransform(
        tf_input_orig_frame_, output->header.frame_id, rclcpp::Time(output->header.stamp),
        rclcpp::Duration::from_seconds(0.0));
      tf2::doTransform(*output, *cloud_transformed, transform_stamped);
      cloud_transformed->header.frame_id = tf_input_orig_frame_;
      output = std::move(cloud_transformed);
    } catch (tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, "Could not transform pointcloud: %s",
        e.what());
      return false;
    }
  }
  return true;
}

void VoxelBasedCompareMapFilterComponent::filter(
  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output)
{
  std::scoped_lock lock(mutex_);
  stop_watch_ptr_->toc("processing_time", true);
  int point_step = input->point_step;
  int offset_x = input->fields[pcl::getFieldIndex(*input, "x")].offset;
  int offset_y = input->fields[pcl::getFieldIndex(*input, "y")].offset;
  int offset_z = input->fields[pcl::getFieldIndex(*input, "z")].offset;

  output.data.resize(input->data.size());
  output.point_step = point_step;
  size_t output_size = 0;
  for (size_t global_offset = 0; global_offset < input->data.size(); global_offset += point_step) {
    pcl::PointXYZ point{};
    std::memcpy(&point.x, &input->data[global_offset + offset_x], sizeof(float));
    std::memcpy(&point.y, &input->data[global_offset + offset_y], sizeof(float));
    std::memcpy(&point.z, &input->data[global_offset + offset_z], sizeof(float));
    if (voxel_grid_map_loader_->is_close_to_map(point, distance_threshold_)) {
      continue;
    }
    std::memcpy(&output.data[output_size], &input->data[global_offset], point_step);
    output_size += point_step;
  }
  output.header = input->header;
  output.fields = input->fields;
  output.data.resize(output_size);
  output.height = input->height;
  output.width = output_size / point_step / output.height;
  output.row_step = output_size / output.height;
  output.is_bigendian = input->is_bigendian;
  output.is_dense = input->is_dense;

  // add processing time for debug
  if (debug_publisher_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
  }
}

}  // namespace autoware::compare_map_segmentation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::compare_map_segmentation::VoxelBasedCompareMapFilterComponent)
