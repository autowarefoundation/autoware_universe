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

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

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

sensor_msgs::msg::PointCloud2 getMsgFromLayout(
  const sensor_msgs::msg::PointCloud2 & msg_in, const PointCloudLayout & pointcloud_layout)
{
  auto new_msg = sensor_msgs::msg::PointCloud2();
  new_msg.fields = pointcloud_layout.fields;
  const auto msg_size = pointcloud_layout.point_step * msg_in.width * msg_in.height;
  new_msg.point_step = pointcloud_layout.point_step;
  new_msg.header = msg_in.header;
  new_msg.height = msg_in.height;
  new_msg.width = msg_in.width;
  new_msg.row_step = msg_size;
  new_msg.data.resize(msg_size);
  return new_msg;
}

PointCloudLayout generateSegmentationPointCloudLayout()
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

PointCloudLayout generateVisualizationPointCloudLayout()
{
  sensor_msgs::msg::PointCloud2 msg;
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(
    4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32, "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
  PointCloudLayout layout(msg.fields, msg.point_step);
  return layout;
}

PointCloudLayout generateFilteredPointCloudLayout()
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
