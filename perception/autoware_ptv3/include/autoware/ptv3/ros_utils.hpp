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

#ifndef AUTOWARE__PTV3__ROS_UTILS_HPP_
#define AUTOWARE__PTV3__ROS_UTILS_HPP_

#include "autoware/ptv3/preprocess/point_type.hpp"

#include <autoware/point_types/memory.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>

#include <sensor_msgs/msg/point_field.hpp>

#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::ptv3
{

/**
 * @brief Build a single PointField descriptor.
 */
inline sensor_msgs::msg::PointField make_point_field(
  const std::string & name, uint32_t offset, uint8_t datatype, uint32_t count)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = count;
  return field;
}

/**
 * @brief Return point fields for the segmented point cloud output (x, y, z, class_id,
 * probability, entropy).
 */
inline std::vector<sensor_msgs::msg::PointField> create_segmented_pointcloud_fields()
{
  return {
    make_point_field("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1),
    make_point_field("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1),
    make_point_field("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1),
    make_point_field("class_id", 12, sensor_msgs::msg::PointField::UINT8, 1),
    make_point_field("probability", 13, sensor_msgs::msg::PointField::FLOAT32, 1),
    make_point_field("entropy", 17, sensor_msgs::msg::PointField::FLOAT32, 1),
  };
}

/**
 * @brief Return point fields for the visualization point cloud output (x, y, z, rgb).
 */
inline std::vector<sensor_msgs::msg::PointField> create_visualization_pointcloud_fields()
{
  return {
    make_point_field("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1),
    make_point_field("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1),
    make_point_field("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1),
    make_point_field("rgb", 12, sensor_msgs::msg::PointField::FLOAT32, 1),
  };
}

/**
 * @brief Return point fields for the filtered point cloud output matching the given format.
 *
 * @param format Output cloud format. Must not be CloudFormat::UNKNOWN.
 * @throws std::runtime_error if the format is not supported.
 */
inline std::vector<sensor_msgs::msg::PointField> create_filtered_pointcloud_fields(
  CloudFormat format)
{
  switch (format) {
    case CloudFormat::XYZIRCAEDT:
      return {
        make_point_field("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1),
        make_point_field("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1),
        make_point_field("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1),
        make_point_field("intensity", 12, sensor_msgs::msg::PointField::UINT8, 1),
        make_point_field("return_type", 13, sensor_msgs::msg::PointField::UINT8, 1),
        make_point_field("channel", 14, sensor_msgs::msg::PointField::UINT16, 1),
        make_point_field("azimuth", 16, sensor_msgs::msg::PointField::FLOAT32, 1),
        make_point_field("elevation", 20, sensor_msgs::msg::PointField::FLOAT32, 1),
        make_point_field("distance", 24, sensor_msgs::msg::PointField::FLOAT32, 1),
        make_point_field("time_stamp", 28, sensor_msgs::msg::PointField::UINT32, 1),
      };
    case CloudFormat::XYZIRADRT:
      return {
        make_point_field("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1),
        make_point_field("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1),
        make_point_field("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1),
        make_point_field("intensity", 12, sensor_msgs::msg::PointField::FLOAT32, 1),
        make_point_field("ring", 16, sensor_msgs::msg::PointField::UINT16, 1),
        make_point_field("azimuth", 18, sensor_msgs::msg::PointField::FLOAT32, 1),
        make_point_field("distance", 22, sensor_msgs::msg::PointField::FLOAT32, 1),
        make_point_field("return_type", 26, sensor_msgs::msg::PointField::UINT8, 1),
        make_point_field("time_stamp", 27, sensor_msgs::msg::PointField::FLOAT64, 1),
      };
    case CloudFormat::XYZIRC:
      return {
        make_point_field("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1),
        make_point_field("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1),
        make_point_field("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1),
        make_point_field("intensity", 12, sensor_msgs::msg::PointField::UINT8, 1),
        make_point_field("return_type", 13, sensor_msgs::msg::PointField::UINT8, 1),
        make_point_field("channel", 14, sensor_msgs::msg::PointField::UINT16, 1),
      };
    case CloudFormat::XYZI:
      return {
        make_point_field("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1),
        make_point_field("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1),
        make_point_field("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1),
        make_point_field("intensity", 12, sensor_msgs::msg::PointField::FLOAT32, 1),
      };
    default:
      throw std::runtime_error("Unsupported filtered point cloud format.");
  }
}

/**
 * @brief Determine the CloudFormat of a point cloud from its PointField descriptors.
 *
 * @param cloud Input point cloud whose fields are inspected.
 * @return Detected CloudFormat, or CloudFormat::UNKNOWN if the layout is unrecognised.
 */
inline CloudFormat detect_cloud_format(const cuda_blackboard::CudaPointCloud2 & cloud)
{
  const auto & fields = cloud.fields;
  const auto num_fields = fields.size();

  if (num_fields == 10 && point_types::is_data_layout_compatible_with_point_xyzircaedt(fields)) {
    return CloudFormat::XYZIRCAEDT;
  }
  if (num_fields == 9 && point_types::is_data_layout_compatible_with_point_xyziradrt(fields)) {
    return CloudFormat::XYZIRADRT;
  }
  if (num_fields == 6 && point_types::is_data_layout_compatible_with_point_xyzirc(fields)) {
    return CloudFormat::XYZIRC;
  }
  if (num_fields == 4 && point_types::is_data_layout_compatible_with_point_xyzi(fields)) {
    return CloudFormat::XYZI;
  }

  return CloudFormat::UNKNOWN;
}

/**
 * @brief Allocate a CudaPointCloud2 for segmented output if not already allocated.
 *
 * @param output_capacity Maximum number of points the buffer must hold.
 * @param msg_ptr Message to allocate; no-op if already non-null.
 */
inline void allocate_seg3d_segmentation_msg(
  const std::int64_t output_capacity, std::unique_ptr<cuda_blackboard::CudaPointCloud2> & msg_ptr)
{
  if (msg_ptr == nullptr) {
    msg_ptr = std::make_unique<cuda_blackboard::CudaPointCloud2>();
    msg_ptr->height = 1;
    msg_ptr->width = output_capacity;
    msg_ptr->fields = create_segmented_pointcloud_fields();
    msg_ptr->is_bigendian = false;
    msg_ptr->is_dense = true;
    msg_ptr->point_step = 21U;
    msg_ptr->data =
      cuda_blackboard::make_unique<std::uint8_t[]>(output_capacity * msg_ptr->point_step);
  }
}

/**
 * @brief Allocate a CudaPointCloud2 for visualization output if not already allocated.
 *
 * @param output_capacity Maximum number of points the buffer must hold.
 * @param msg_ptr Message to allocate; no-op if already non-null.
 */
inline void allocate_seg3d_visualization_msg(
  const std::int64_t output_capacity, std::unique_ptr<cuda_blackboard::CudaPointCloud2> & msg_ptr)
{
  if (msg_ptr == nullptr) {
    msg_ptr = std::make_unique<cuda_blackboard::CudaPointCloud2>();
    msg_ptr->height = 1;
    msg_ptr->width = output_capacity;
    msg_ptr->fields = create_visualization_pointcloud_fields();
    msg_ptr->is_bigendian = false;
    msg_ptr->is_dense = true;
    msg_ptr->point_step = 16U;
    msg_ptr->data =
      cuda_blackboard::make_unique<std::uint8_t[]>(output_capacity * msg_ptr->point_step);
  }
}

/**
 * @brief Allocate a CudaPointCloud2 for filtered output if not already allocated.
 *
 * @param output_capacity Maximum number of points the buffer must hold.
 * @param filtered_output_format Output format; must not be CloudFormat::UNKNOWN.
 * @param msg_ptr Message to allocate; no-op if already non-null.
 * @throws std::runtime_error if filtered_output_format is CloudFormat::UNKNOWN.
 */
inline void allocate_seg3d_filtered_msg(
  const std::int64_t output_capacity, const CloudFormat filtered_output_format,
  std::unique_ptr<cuda_blackboard::CudaPointCloud2> & msg_ptr)
{
  if (filtered_output_format == CloudFormat::UNKNOWN) {
    throw std::runtime_error("Cannot allocate filtered point cloud message with unknown format.");
  }

  if (msg_ptr == nullptr) {
    msg_ptr = std::make_unique<cuda_blackboard::CudaPointCloud2>();
    msg_ptr->height = 1;
    msg_ptr->width = output_capacity;
    msg_ptr->fields = create_filtered_pointcloud_fields(filtered_output_format);
    msg_ptr->is_bigendian = false;
    msg_ptr->is_dense = true;
    msg_ptr->point_step = static_cast<std::uint32_t>(get_point_step(filtered_output_format));
    msg_ptr->data =
      cuda_blackboard::make_unique<std::uint8_t[]>(output_capacity * msg_ptr->point_step);
  }
}

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__ROS_UTILS_HPP_
