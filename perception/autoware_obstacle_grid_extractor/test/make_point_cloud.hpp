// Copyright 2026 The Autoware Contributors
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
#ifndef MAKE_POINT_CLOUD_HPP_
#define MAKE_POINT_CLOUD_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <string>
#include <vector>

namespace autoware::obstacle_grid_extractor::test
{
struct TestPoint
{
  float x;
  float y;
  float z;
};

namespace detail
{
inline void fillXyz(sensor_msgs::msg::PointCloud2 & cloud, const std::vector<TestPoint> & points)
{
  sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
  for (const auto & point : points) {
    *it_x = point.x;
    *it_y = point.y;
    *it_z = point.z;
    ++it_x;
    ++it_y;
    ++it_z;
  }
}
}  // namespace detail

/// A minimal xyz cloud (12-byte points) in `frame_id`.
inline sensor_msgs::msg::PointCloud2 makePointCloud(
  const std::string & frame_id, const std::vector<TestPoint> & points)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame_id;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());
  detail::fillXyz(cloud, points);
  return cloud;
}

/// The same points in Autoware's production `PointXYZIRC` layout (16-byte points with mixed field
/// types), so the extractor's iteration is exercised against a stride wider than packed xyz.
inline sensor_msgs::msg::PointCloud2 makePointXyzircCloud(
  const std::string & frame_id, const std::vector<TestPoint> & points)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame_id;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
    6, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
    sensor_msgs::msg::PointField::UINT8, "return_type", 1, sensor_msgs::msg::PointField::UINT8,
    "channel", 1, sensor_msgs::msg::PointField::UINT16);
  modifier.resize(points.size());
  detail::fillXyz(cloud, points);
  return cloud;
}
}  // namespace autoware::obstacle_grid_extractor::test

#endif  // MAKE_POINT_CLOUD_HPP_
