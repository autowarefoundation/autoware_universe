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

#include "autoware/lidar_frnet/point_type.hpp"

#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <cuda_blackboard/cuda_unique_ptr.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <array>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::lidar_frnet::ros_utils
{

/**
 * @brief Layout descriptor for a point cloud (fields and point step size).
 */
struct PointCloudLayout
{
  /**
   * @brief Construct layout from field list and point step.
   * @param layout_fields Point field list (e.g. from sensor_msgs PointCloud2)
   * @param layout_point_step Byte stride per point
   */
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
 * @param max_num_points Capacity (allocates this many points; actual size set by pipeline)
 * @return Initialized message
 */
inline std::unique_ptr<cuda_blackboard::CudaPointCloud2> generatePointCloudMessageFromInput(
  const cuda_blackboard::CudaPointCloud2 & msg_in, const PointCloudLayout & layout,
  const size_t max_num_points)
{
  auto cloud_msg_ptr = std::make_unique<cuda_blackboard::CudaPointCloud2>();

  cloud_msg_ptr->data =
    cuda_blackboard::make_unique<std::uint8_t[]>(max_num_points * layout.point_step);

  cloud_msg_ptr->fields = layout.fields;
  cloud_msg_ptr->point_step = layout.point_step;
  cloud_msg_ptr->header = msg_in.header;
  cloud_msg_ptr->height = 1;
  cloud_msg_ptr->width = static_cast<uint32_t>(max_num_points);
  cloud_msg_ptr->row_step = layout.point_step * max_num_points;
  cloud_msg_ptr->is_bigendian = msg_in.is_bigendian;
  cloud_msg_ptr->is_dense = msg_in.is_dense;

  return cloud_msg_ptr;
}

/**
 * @brief Build point cloud layout for segmentation output (x, y, z, class_id, probability).
 * @return Layout with FLOAT32 x,y,z, FLOAT32 probability and UINT8 class_id
 */
inline PointCloudLayout generateSegmentationPointCloudLayout()
{
  sensor_msgs::msg::PointCloud2 msg;
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(
    5, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32, "class_id", 1,
    sensor_msgs::msg::PointField::UINT8, "probability", 1, sensor_msgs::msg::PointField::FLOAT32);
  PointCloudLayout layout(msg.fields, msg.point_step);
  return layout;
}

/**
 * @brief Build point cloud layout for visualization output (x, y, z, rgb).
 * @return Layout with FLOAT32 x, y, z, rgb
 */
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

/**
 * @brief Generate filtered point cloud layout from input message fields.
 *
 * Preserves the input point format so filtered output matches input (e.g. XYZIRCAEDT).
 *
 * @param msg_in Input point cloud message
 * @return Layout with same fields and point_step as msg_in
 */
inline CloudFormat detectCloudFormat(const std::vector<sensor_msgs::msg::PointField> & fields)
{
  const auto num_fields = fields.size();
  if (num_fields == 10) {
    return CloudFormat::XYZIRCAEDT;
  }
  if (num_fields == 9) {
    return CloudFormat::XYZIRADRT;
  }
  if (num_fields == 6) {
    return CloudFormat::XYZIRC;
  }
  if (num_fields == 4) {
    return CloudFormat::XYZI;
  }
  return CloudFormat::UNKNOWN;
}

/**
 * @brief Generate filtered point cloud layout for a supported cloud format.
 */
inline PointCloudLayout generateFilteredPointCloudLayout(CloudFormat format)
{
  sensor_msgs::msg::PointCloud2 msg;
  sensor_msgs::PointCloud2Modifier modifier(msg);
  switch (format) {
    case CloudFormat::XYZIRCAEDT:
      modifier.setPointCloud2Fields(
        10, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
        sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::UINT8, "return_type", 1,
        sensor_msgs::msg::PointField::UINT8, "channel", 1, sensor_msgs::msg::PointField::UINT16,
        "azimuth", 1, sensor_msgs::msg::PointField::FLOAT32, "elevation", 1,
        sensor_msgs::msg::PointField::FLOAT32, "distance", 1, sensor_msgs::msg::PointField::FLOAT32,
        "time_stamp", 1, sensor_msgs::msg::PointField::UINT32);
      break;
    case CloudFormat::XYZIRADRT:
      modifier.setPointCloud2Fields(
        9, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
        sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32, "ring", 1,
        sensor_msgs::msg::PointField::UINT16, "azimuth", 1, sensor_msgs::msg::PointField::FLOAT32,
        "distance", 1, sensor_msgs::msg::PointField::FLOAT32, "return_type", 1,
        sensor_msgs::msg::PointField::UINT8, "time_stamp", 1,
        sensor_msgs::msg::PointField::FLOAT64);
      break;
    case CloudFormat::XYZIRC:
      modifier.setPointCloud2Fields(
        6, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
        sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::UINT8, "return_type", 1,
        sensor_msgs::msg::PointField::UINT8, "channel", 1, sensor_msgs::msg::PointField::UINT16);
      break;
    case CloudFormat::XYZI:
      modifier.setPointCloud2Fields(
        4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
        sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
      break;
    default:
      throw std::runtime_error("Unsupported filtered point cloud format.");
  }
  PointCloudLayout layout(msg.fields, msg.point_step);
  return layout;
}

/** Line scale for ego crop box wireframe (Marker scale.x). */
constexpr double EGO_CROP_BOX_LINE_SCALE = 0.05;
/** Red component for ego crop box color. */
constexpr float EGO_CROP_BOX_COLOR_R = 0.0f;
/** Green component for ego crop box color. */
constexpr float EGO_CROP_BOX_COLOR_G = 1.0f;
/** Blue component for ego crop box color. */
constexpr float EGO_CROP_BOX_COLOR_B = 0.0f;
/** Alpha component for ego crop box color. */
constexpr float EGO_CROP_BOX_COLOR_A = 1.0f;

/**
 * @brief Compute 8 corners of an AABB from bounds.
 *
 * Bounds order: [min_x, min_y, min_z, max_x, max_y, max_z].
 * Corner order: bottom face 0-3, top face 4-7.
 *
 * @param b Axis-aligned bounds (6 floats)
 * @param corners Output array of 8 geometry_msgs::Point corners
 */
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

/**
 * @brief Convert geometry_msgs::Point to Point32 (float coordinates).
 * @param p Input point (double x, y, z)
 * @return Point32 with same coordinates as float
 */
inline geometry_msgs::msg::Point32 toPoint32(const geometry_msgs::msg::Point & p)
{
  geometry_msgs::msg::Point32 q;
  q.x = static_cast<float>(p.x);
  q.y = static_cast<float>(p.y);
  q.z = static_cast<float>(p.z);
  return q;
}

/**
 * @brief Fill marker message for ego crop box as LINE_LIST.
 *
 * Bounds order: [min_x, min_y, min_z, max_x, max_y, max_z].
 * Sets ns, id, type, action, pose, scale, color and points.
 *
 * @param bounds Axis-aligned box bounds (6 floats)
 * @param frame_id Frame ID for the message header
 * @param msg Output marker message (header, ns, type, scale, color, points are set)
 */
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
