// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__MAP_LOADER__PLUGINS__VECTOR_MAP_TF_GENERATOR_PLUGIN_HPP_
#define AUTOWARE__MAP_LOADER__PLUGINS__VECTOR_MAP_TF_GENERATOR_PLUGIN_HPP_

#include <autoware/lanelet2_utils/conversion.hpp>
#include <rclcpp/time.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Point.h>

#include <numeric>
#include <string>
#include <vector>

namespace autoware::map_tf_generator
{

geometry_msgs::msg::TransformStamped create_viewer_transform(
  const autoware_map_msgs::msg::LaneletMapBin & map_bin_msg, const std::string & map_frame,
  const std::string & viewer_frame, const rclcpp::Time & stamp)
{
  const auto lanelet_map_ptr = autoware::experimental::lanelet2_utils::remove_const(
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(map_bin_msg));

  std::vector<double> points_x;
  std::vector<double> points_y;
  std::vector<double> points_z;

  for (const lanelet::Point3d & point : lanelet_map_ptr->pointLayer) {
    points_x.push_back(point.x());
    points_y.push_back(point.y());
    points_z.push_back(point.z());
  }

  const double coordinate_x =
    std::accumulate(points_x.begin(), points_x.end(), 0.0) / static_cast<double>(points_x.size());
  const double coordinate_y =
    std::accumulate(points_y.begin(), points_y.end(), 0.0) / static_cast<double>(points_y.size());
  const double coordinate_z =
    std::accumulate(points_z.begin(), points_z.end(), 0.0) / static_cast<double>(points_z.size());

  geometry_msgs::msg::TransformStamped static_transform_stamped;
  static_transform_stamped.header.stamp = stamp;
  static_transform_stamped.header.frame_id = map_frame;
  static_transform_stamped.child_frame_id = viewer_frame;
  static_transform_stamped.transform.translation.x = coordinate_x;
  static_transform_stamped.transform.translation.y = coordinate_y;
  static_transform_stamped.transform.translation.z = coordinate_z;
  tf2::Quaternion quat;
  quat.setRPY(0, 0, 0);
  static_transform_stamped.transform.rotation.x = quat.x();
  static_transform_stamped.transform.rotation.y = quat.y();
  static_transform_stamped.transform.rotation.z = quat.z();
  static_transform_stamped.transform.rotation.w = quat.w();

  return static_transform_stamped;
}

}  // namespace autoware::map_tf_generator

#endif  // AUTOWARE__MAP_LOADER__PLUGINS__VECTOR_MAP_TF_GENERATOR_PLUGIN_HPP_
