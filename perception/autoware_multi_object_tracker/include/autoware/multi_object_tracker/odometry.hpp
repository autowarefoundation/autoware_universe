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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__ODOMETRY_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__ODOMETRY_HPP_

#include "autoware/multi_object_tracker/object_model/types.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <map>
#include <optional>
#include <string>

namespace autoware::multi_object_tracker
{

class Odometry
{
public:
  Odometry(
    rclcpp::Node & node, const std::string & world_frame_id, const std::string & ego_frame_id,
    bool enable_odometry_uncertainty = false);

  std::optional<geometry_msgs::msg::Transform> getTransform(
    const std::string & source_frame_id, const rclcpp::Time & time) const;
  std::optional<geometry_msgs::msg::Transform> getTransform(const rclcpp::Time & time) const;

  std::optional<nav_msgs::msg::Odometry> getOdometryFromTf(const rclcpp::Time & time) const;

  std::optional<types::DynamicObjectList> transformObjects(
    const types::DynamicObjectList & input_objects) const;

private:
  rclcpp::Node & node_;
  // frame id
  std::string ego_frame_id_;    // ego vehicle frame
  std::string world_frame_id_;  // absolute/relative ground frame
  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

public:
  bool enable_odometry_uncertainty_ = false;

private:
  void updateTfCache(
    const rclcpp::Time & time, const geometry_msgs::msg::Transform & transform) const;

  // cache of tf
  mutable std::map<rclcpp::Time, geometry_msgs::msg::Transform> tf_cache_;
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__ODOMETRY_HPP_
