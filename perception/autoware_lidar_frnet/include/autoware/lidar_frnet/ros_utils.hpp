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

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <array>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::lidar_frnet::ros_utils
{
struct PointCloudLayout
{
  PointCloudLayout(
    std::vector<sensor_msgs::msg::PointField> layout_fields, size_t layout_point_step)
  : fields(std::move(layout_fields)), point_step(layout_point_step)
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
 * @param max_num_points Optional capacity (if > 0, allocates this many points; actual size set by
 *        pipeline); when 0, uses msg_in.width * msg_in.height
 * @return std::unique_ptr<cuda_blackboard::CudaPointCloud2> Initialized message
 */
inline std::unique_ptr<cuda_blackboard::CudaPointCloud2> generatePointCloudMessageFromInput(
  const cuda_blackboard::CudaPointCloud2 & msg_in, const PointCloudLayout & layout,
  size_t max_num_points = 0)
{
  auto cloud_msg_ptr = std::make_unique<cuda_blackboard::CudaPointCloud2>();

  const auto input_num_points = msg_in.width * msg_in.height;
  const auto num_points = max_num_points > 0 ? max_num_points : input_num_points;
  cloud_msg_ptr->data =
    cuda_blackboard::make_unique<std::uint8_t[]>(num_points * layout.point_step);

  cloud_msg_ptr->fields = layout.fields;
  cloud_msg_ptr->point_step = layout.point_step;
  cloud_msg_ptr->header = msg_in.header;
  cloud_msg_ptr->height = max_num_points > 0 ? 1 : msg_in.height;
  cloud_msg_ptr->width = max_num_points > 0 ? static_cast<uint32_t>(max_num_points) : msg_in.width;
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

/// @brief Generate filtered point cloud layout from input message fields (preserves input format)
inline PointCloudLayout generateFilteredPointCloudLayoutFromInput(
  const cuda_blackboard::CudaPointCloud2 & msg_in)
{
  return PointCloudLayout(msg_in.fields, msg_in.point_step);
}

/// @brief Generate filtered point cloud layout for default XYZIRC format (legacy fallback).
///        The node uses generateFilteredPointCloudLayoutFromInput() to preserve input format;
///        use this only if a fixed XYZIRC layout is required.
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

// Ego crop box debug visualization constants
constexpr double EGO_CROP_BOX_LINE_SCALE = 0.05;
constexpr float EGO_CROP_BOX_COLOR_R = 0.0f;
constexpr float EGO_CROP_BOX_COLOR_G = 1.0f;
constexpr float EGO_CROP_BOX_COLOR_B = 0.0f;
constexpr float EGO_CROP_BOX_COLOR_A = 1.0f;

/// @brief Compute 8 corners of an AABB from bounds [min_x, min_y, min_z, max_x, max_y, max_z].
///        Order: bottom face 0-3, top face 4-7.
inline void getEgoCropBoxCorners(
  const std::array<float, 6> & b, std::array<geometry_msgs::msg::Point, 8> & corners)
{
  const double min_x = b[0];
  const double min_y = b[1];
  const double min_z = b[2];
  const double max_x = b[3];
  const double max_y = b[4];
  const double max_z = b[5];
  corners[0].x = max_x;
  corners[0].y = max_y;
  corners[0].z = min_z;
  corners[1].x = min_x;
  corners[1].y = max_y;
  corners[1].z = min_z;
  corners[2].x = min_x;
  corners[2].y = min_y;
  corners[2].z = min_z;
  corners[3].x = max_x;
  corners[3].y = min_y;
  corners[3].z = min_z;
  corners[4].x = max_x;
  corners[4].y = max_y;
  corners[4].z = max_z;
  corners[5].x = min_x;
  corners[5].y = max_y;
  corners[5].z = max_z;
  corners[6].x = min_x;
  corners[6].y = min_y;
  corners[6].z = max_z;
  corners[7].x = max_x;
  corners[7].y = min_y;
  corners[7].z = max_z;
}

inline geometry_msgs::msg::Point32 toPoint32(const geometry_msgs::msg::Point & p)
{
  geometry_msgs::msg::Point32 q;
  q.x = static_cast<float>(p.x);
  q.y = static_cast<float>(p.y);
  q.z = static_cast<float>(p.z);
  return q;
}

/// @brief Fill polygon message for ego crop box wireframe. Bounds: [min_x, min_y, min_z, max_x,
/// max_y, max_z].
inline void setPolygonMsg(
  const std::array<float, 6> & bounds, const std::string & frame_id,
  geometry_msgs::msg::PolygonStamped & msg)
{
  std::array<geometry_msgs::msg::Point, 8> corners;
  getEgoCropBoxCorners(bounds, corners);
  msg.header.frame_id = frame_id;
  msg.polygon.points.clear();
  msg.polygon.points.reserve(16);
  msg.polygon.points.push_back(toPoint32(corners[0]));
  msg.polygon.points.push_back(toPoint32(corners[1]));
  msg.polygon.points.push_back(toPoint32(corners[2]));
  msg.polygon.points.push_back(toPoint32(corners[3]));
  msg.polygon.points.push_back(toPoint32(corners[0]));
  msg.polygon.points.push_back(toPoint32(corners[4]));
  msg.polygon.points.push_back(toPoint32(corners[5]));
  msg.polygon.points.push_back(toPoint32(corners[1]));
  msg.polygon.points.push_back(toPoint32(corners[5]));
  msg.polygon.points.push_back(toPoint32(corners[6]));
  msg.polygon.points.push_back(toPoint32(corners[2]));
  msg.polygon.points.push_back(toPoint32(corners[6]));
  msg.polygon.points.push_back(toPoint32(corners[7]));
  msg.polygon.points.push_back(toPoint32(corners[3]));
  msg.polygon.points.push_back(toPoint32(corners[7]));
  msg.polygon.points.push_back(toPoint32(corners[4]));
}

/// @brief Fill marker message for ego crop box LINE_LIST. Bounds: [min_x, min_y, min_z, max_x,
/// max_y, max_z].
inline void setMarkerMsg(
  const std::array<float, 6> & bounds, const std::string & frame_id,
  visualization_msgs::msg::Marker & msg)
{
  std::array<geometry_msgs::msg::Point, 8> corners;
  getEgoCropBoxCorners(bounds, corners);
  msg.header.frame_id = frame_id;
  msg.ns = "ego_crop_box";
  msg.id = 0;
  msg.type = visualization_msgs::msg::Marker::LINE_LIST;
  msg.action = visualization_msgs::msg::Marker::ADD;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = EGO_CROP_BOX_LINE_SCALE;
  msg.color.a = EGO_CROP_BOX_COLOR_A;
  msg.color.r = EGO_CROP_BOX_COLOR_R;
  msg.color.g = EGO_CROP_BOX_COLOR_G;
  msg.color.b = EGO_CROP_BOX_COLOR_B;
  msg.points.clear();
  msg.points.reserve(24);
  msg.points.push_back(corners[0]);
  msg.points.push_back(corners[1]);
  msg.points.push_back(corners[1]);
  msg.points.push_back(corners[2]);
  msg.points.push_back(corners[2]);
  msg.points.push_back(corners[3]);
  msg.points.push_back(corners[3]);
  msg.points.push_back(corners[0]);
  msg.points.push_back(corners[4]);
  msg.points.push_back(corners[5]);
  msg.points.push_back(corners[5]);
  msg.points.push_back(corners[6]);
  msg.points.push_back(corners[6]);
  msg.points.push_back(corners[7]);
  msg.points.push_back(corners[7]);
  msg.points.push_back(corners[4]);
  msg.points.push_back(corners[0]);
  msg.points.push_back(corners[4]);
  msg.points.push_back(corners[1]);
  msg.points.push_back(corners[5]);
  msg.points.push_back(corners[2]);
  msg.points.push_back(corners[6]);
  msg.points.push_back(corners[3]);
  msg.points.push_back(corners[7]);
}

}  // namespace autoware::lidar_frnet::ros_utils

#endif  // AUTOWARE__LIDAR_FRNET__ROS_UTILS_HPP_
