// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__TYPE_ALIAS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__TYPE_ALIAS_HPP_

#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/trajectory/path_with_lane_id.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/geometry/pose_deviation.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg_array.hpp>
#include <tier4_rtc_msgs/msg/state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace autoware::behavior_path_planner
{
// auto msgs
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedPath;
using autoware_perception_msgs::msg::Shape;

// ROS 2 general msgs
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseWithCovariance;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::Vector3;
using std_msgs::msg::ColorRGBA;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

// tier4 msgs
using tier4_planning_msgs::msg::AvoidanceDebugMsg;
using tier4_planning_msgs::msg::AvoidanceDebugMsgArray;
using tier4_rtc_msgs::msg::State;

// tier4 utils functions
using autoware_utils::append_marker_array;
using autoware_utils::calc_distance2d;
using autoware_utils::calc_lateral_deviation;
using autoware_utils::calc_offset_pose;
using autoware_utils::calc_yaw_deviation;
using autoware_utils::create_default_marker;
using autoware_utils::create_marker_color;
using autoware_utils::create_marker_scale;
using autoware_utils::create_point;
using autoware_utils::create_quaternion_from_rpy;
using autoware_utils::get_point;
using autoware_utils::get_pose;
using autoware_utils::Point2d;
using autoware_utils::Polygon2d;
using autoware_utils::pose2transform;
using autoware_utils::to_hex_string;
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__TYPE_ALIAS_HPP_
