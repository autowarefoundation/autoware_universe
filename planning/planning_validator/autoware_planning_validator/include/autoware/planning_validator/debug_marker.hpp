// Copyright 2022 Tier IV, Inc.
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

#ifndef AUTOWARE__PLANNING_VALIDATOR__DEBUG_MARKER_HPP_
#define AUTOWARE__PLANNING_VALIDATOR__DEBUG_MARKER_HPP_

#include "autoware_planning_validator/msg/planning_validator_status.hpp"

#include <autoware_utils/geometry/boost_geometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::planning_validator
{
using autoware_planning_validator::msg::PlanningValidatorStatus;

class PlanningValidatorDebugMarkerPublisher
{
public:
  explicit PlanningValidatorDebugMarkerPublisher(rclcpp::Node * node);

  void pushPoseMarker(
    const autoware_planning_msgs::msg::TrajectoryPoint & p, const std::string & ns, int id = 0);
  void pushPoseMarker(const geometry_msgs::msg::Pose & pose, const std::string & ns, int id = 0);
  void pushPointMarker(
    const geometry_msgs::msg::Point & point, const std::string & ns, int id = 0, double scale = 0.3,
    bool is_cube = false);
  void pushVirtualWall(const geometry_msgs::msg::Pose & pose);
  void pushWarningMsg(const geometry_msgs::msg::Pose & pose, const std::string & msg);
  void pushLineSegmentMarker(
    const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2,
    const std::string & ns, int id = 0);

  void pushLaneletPolygonsMarker(
    const lanelet::BasicPolygons2d & polygon, const std::string & ns, int id = 0);

  void pushMarker(const visualization_msgs::msg::Marker & marker);

  void pushMarkers(const visualization_msgs::msg::MarkerArray & marker);

  std::string getStatusDebugString(const PlanningValidatorStatus & status) const
  {
    std::stringstream ss;

    auto append_string = [&ss](const std::string & str) {
      if (ss.str().empty())
        ss << "[";
      else
        ss << " | ";
      ss << str;
    };

    if (!status.is_valid_latency) append_string("latency");
    if (!status.is_valid_size) append_string("traj_size");
    if (!status.is_valid_finite_value) append_string("infinite_value");
    if (!status.is_valid_interval) append_string("interval");
    if (!status.is_valid_relative_angle) append_string("relative_angle");
    if (!status.is_valid_distance_deviation) append_string("distance_deviation");
    if (!status.is_valid_longitudinal_distance_deviation) append_string("lon_distance_deviation");
    if (!status.is_valid_velocity_deviation) append_string("velocity_deviation");
    if (!status.is_valid_longitudinal_max_acc) append_string("lon_max_acc");
    if (!status.is_valid_longitudinal_min_acc) append_string("lon_min_acc");
    if (!status.is_valid_lateral_acc) append_string("lat_acc");
    if (!status.is_valid_lateral_jerk) append_string("lat_jerk");
    if (!status.is_valid_yaw_deviation) append_string("yaw_deviation");
    if (!status.is_valid_curvature) append_string("curvature");
    if (!status.is_valid_steering) append_string("steering");
    if (!status.is_valid_steering_rate) append_string("steering_rate");
    if (!status.is_valid_forward_trajectory_length) append_string("forward_traj_length");
    if (!status.is_valid_trajectory_shift) append_string("traj_shift");
    if (!status.is_valid_intersection_collision_check) append_string("collision");

    if (ss.str().empty()) {
      return "";
    }

    ss << "]";
    return ss.str();
  }

  void publish();

  void clearMarkers();

private:
  rclcpp::Node * node_;
  visualization_msgs::msg::MarkerArray marker_array_;
  visualization_msgs::msg::MarkerArray marker_array_virtual_wall_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_viz_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr virtual_wall_pub_;
  std::map<std::string, int> marker_id_;

  int getMarkerId(const std::string & ns)
  {
    if (marker_id_.count(ns) == 0) {
      marker_id_[ns] = 0.0;
    }
    return marker_id_[ns]++;
  }
};

}  // namespace autoware::planning_validator

#endif  // AUTOWARE__PLANNING_VALIDATOR__DEBUG_MARKER_HPP_
