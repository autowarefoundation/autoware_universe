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

#ifndef AUTOWARE__LIDAR_FRNET__ROS_UTILS_HPP_
#define AUTOWARE__LIDAR_FRNET__ROS_UTILS_HPP_

#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <cuda_blackboard/cuda_unique_ptr.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <memory>
#include <vector>

namespace autoware::lidar_frnet::ros_utils
{
struct PointCloudLayout
{
  PointCloudLayout(
    std::vector<sensor_msgs::msg::PointField> & layout_fields, size_t layout_point_step)
  : fields(layout_fields), point_step(layout_point_step)
  {
  }
  std::vector<sensor_msgs::msg::PointField> fields;
  size_t point_step;
};

/**
 * @brief Generate and allocate point cloud message from input message with layout
 *
 * This function combines memory allocation and metadata setup from input message.
 *
 * @param msg_in Input message
 * @param layout Layout for output message
 * @return std::unique_ptr<cuda_blackboard::CudaPointCloud2> Initialized message
 */
inline std::unique_ptr<cuda_blackboard::CudaPointCloud2> generatePointCloudMessageFromInput(
  const cuda_blackboard::CudaPointCloud2 & msg_in, const PointCloudLayout & layout)
{
  auto cloud_msg_ptr = std::make_unique<cuda_blackboard::CudaPointCloud2>();

  // Allocate memory based on input message size
  const auto num_points = msg_in.width * msg_in.height;
  cloud_msg_ptr->data =
    cuda_blackboard::make_unique<std::uint8_t[]>(num_points * layout.point_step);

  // Set metadata from layout
  cloud_msg_ptr->fields = layout.fields;
  cloud_msg_ptr->point_step = layout.point_step;

  // Set metadata from input message
  cloud_msg_ptr->header = msg_in.header;
  cloud_msg_ptr->height = msg_in.height;
  cloud_msg_ptr->width = msg_in.width;
  cloud_msg_ptr->row_step = layout.point_step * num_points;

  cloud_msg_ptr->is_bigendian = msg_in.is_bigendian;
  cloud_msg_ptr->is_dense = msg_in.is_dense;

  return cloud_msg_ptr;
}

inline PointCloudLayout generateSegmentationPointCloudLayout()
{
  sensor_msgs::msg::PointCloud2 msg;
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(
    4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32, "class_id", 1,
    sensor_msgs::msg::PointField::UINT8);
  PointCloudLayout layout(msg.fields, msg.point_step);
  return layout;
}

inline PointCloudLayout generateVisualizationPointCloudLayout()
{
  sensor_msgs::msg::PointCloud2 msg;
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(
    4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32, "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
  PointCloudLayout layout(msg.fields, msg.point_step);
  return layout;
}

inline PointCloudLayout generateFilteredPointCloudLayout()
{
  sensor_msgs::msg::PointCloud2 msg;
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(
    6, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
    sensor_msgs::msg::PointField::UINT8, "return_type", 1, sensor_msgs::msg::PointField::UINT8,
    "channel", 1, sensor_msgs::msg::PointField::UINT16);
  PointCloudLayout layout(msg.fields, msg.point_step);
  return layout;
}

}  // namespace autoware::lidar_frnet::ros_utils

#endif  // AUTOWARE__LIDAR_FRNET__ROS_UTILS_HPP_
