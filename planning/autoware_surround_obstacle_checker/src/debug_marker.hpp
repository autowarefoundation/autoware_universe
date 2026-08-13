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

#ifndef DEBUG_MARKER_HPP_
#define DEBUG_MARKER_HPP_

#include "type_alias.hpp"
#include "types.hpp"

#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/planning_factor_array.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/geometry.hpp>

#include <memory>
#include <string>

namespace autoware::surround_obstacle_checker
{

namespace bg = boost::geometry;
using Point2d = bg::model::d2::point_xy<double>;
using Polygon2d = bg::model::polygon<Point2d>;

enum class PoseType : int8_t { NoStart = 0 };
enum class PointType : int8_t { NoStart = 0 };

class SurroundObstacleCheckerDebugNode
{
public:
  explicit SurroundObstacleCheckerDebugNode(
    const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
    const std::string & object_label, const double & surround_check_front_distance,
    const double & surround_check_side_distance, const double & surround_check_back_distance,
    const double & surround_check_hysteresis_distance, const geometry_msgs::msg::Pose & self_pose,
    const rclcpp::Clock::SharedPtr clock, rclcpp::Node & node);

  bool pushPose(const geometry_msgs::msg::Pose & pose, const PoseType & type);
  void pushStopObstacle(const std::optional<StopObstacle> & stop_obstacle);
  void publish();
  void publishFootprints();

private:
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_viz_pub_;

  rclcpp::Publisher<PolygonStamped>::SharedPtr vehicle_footprint_pub_;
  rclcpp::Publisher<PolygonStamped>::SharedPtr vehicle_footprint_offset_pub_;
  rclcpp::Publisher<PolygonStamped>::SharedPtr vehicle_footprint_recover_offset_pub_;

  std::unique_ptr<autoware::planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface_;

  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  std::string object_label_;
  double surround_check_front_distance_;
  double surround_check_side_distance_;
  double surround_check_back_distance_;
  double surround_check_hysteresis_distance_;
  geometry_msgs::msg::Pose self_pose_;

  MarkerArray makeVisualizationMarker();

  PolygonStamped boostPolygonToPolygonStamped(const Polygon2d & boost_polygon, const double & z);

  std::optional<StopObstacle> stop_obstacle_;
  std::shared_ptr<geometry_msgs::msg::Pose> stop_pose_ptr_;
  rclcpp::Clock::SharedPtr clock_;
};
}  // namespace autoware::surround_obstacle_checker
#endif  // DEBUG_MARKER_HPP_
