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

#ifndef STRUCTS_HPP_
#define STRUCTS_HPP_

#include <autoware_utils/geometry/boost_geometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{
using autoware_internal_planning_msgs::msg::PathWithLaneId;

using sensor_msgs::msg::PointCloud2;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

struct DetectionAreaInfo
{
  DetectionAreaInfo(
    const lanelet::ConstLanelets & _lanelets, const lanelet::BasicPolygon3d & _polygon,
    const double _conflict_point_coordinate)
  : lanelets{_lanelets}, polygon{_polygon}, conflict_point_coordinate{_conflict_point_coordinate}
  {
  }
  lanelet::ConstLanelets lanelets;

  lanelet::BasicPolygon3d polygon;

  double conflict_point_coordinate;
};

using DetectionAreasInfo = std::vector<DetectionAreaInfo>;

struct PointCloudObject
{
  rclcpp::Time last_update_time;

  geometry_msgs::msg::Pose pose;

  lanelet::ConstLanelet furthest_lane;

  double tracking_duration;

  double distance;

  double distance_with_delay_compensation;

  double velocity;

  double ttc;

  bool safe;

  bool ignore;
};

using PointCloudObjects = std::vector<PointCloudObject>;

struct DebugData
{
  PointCloudObjects pointcloud_objects;

  std::vector<geometry_msgs::msg::Pose> stop_poses;

  std::vector<geometry_msgs::msg::Pose> dead_line_poses;

  std::vector<autoware_utils::Polygon3d> hull_polygons;

  std::vector<size_t> pointcloud_nums{};

  sensor_msgs::msg::PointCloud2::SharedPtr obstacle_pointcloud;
};
}  // namespace autoware::behavior_velocity_planner

#endif  // STRUCTS_HPP_
