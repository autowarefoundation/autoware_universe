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

#include <autoware_planning_validator_rear_collision_checker/rear_collision_checker_node_parameters.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Point.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::planning_validator
{

using autoware::vehicle_info_utils::VehicleInfo;
using autoware_internal_debug_msgs::msg::StringStamped;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::Shape;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::PointCloud2;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

using DetectionArea = std::pair<lanelet::BasicPolygon3d, lanelet::ConstLanelets>;

using DetectionAreas = std::vector<DetectionArea>;

enum class Behavior {
  NONE = 0,
  SHIFT_LEFT,
  SHIFT_RIGHT,
  TURN_LEFT,
  TURN_RIGHT,
};

struct PointCloudObject
{
  rclcpp::Time last_update_time;

  rclcpp::Time last_stop_time;

  geometry_msgs::msg::Pose pose;

  lanelet::ConstLanelet furthest_lane;

  double tracking_duration;

  double absolute_distance;

  double relative_distance;

  double relative_distance_with_delay_compensation;

  double rss_distance;

  double velocity;

  double moving_time;

  bool safe;

  bool ignore;

  bool is_vru;

  std::string detail{""};
};

using PointCloudObjects = std::vector<PointCloudObject>;

struct DebugData
{
  autoware_utils::LineString3d reachable_line;

  autoware_utils::LineString3d stoppable_line;

  lanelet::ConstLanelets current_lanes;

  PointCloudObjects pointcloud_objects;

  DetectionAreas detection_areas;

  sensor_msgs::msg::PointCloud2::SharedPtr cluster_points;

  sensor_msgs::msg::PointCloud2::SharedPtr voxel_points;

  Behavior turn_behavior{Behavior::NONE};

  Behavior shift_behavior{Behavior::NONE};

  std::vector<autoware_utils::Polygon3d> hull_polygons;

  std::vector<size_t> pointcloud_nums{};

  std::string text{"-"};

  double processing_time_detail_ms;

  bool is_active{false};

  bool is_safe{true};

  auto get_detection_polygons() const -> lanelet::BasicPolygons3d
  {
    lanelet::BasicPolygons3d ret{};
    for (const auto & [polygon, lanes] : detection_areas) {
      ret.push_back(polygon);
    }
    return ret;
  }

  auto get_detection_lanes() const -> lanelet::ConstLanelets
  {
    lanelet::ConstLanelets ret{};
    for (const auto & [polygon, lanes] : detection_areas) {
      ret.insert(ret.end(), lanes.begin(), lanes.end());
    }
    return ret;
  }
};
}  // namespace autoware::planning_validator

#endif  // STRUCTS_HPP_
