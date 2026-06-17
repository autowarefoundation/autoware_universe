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

#include "autoware/multi_object_tracker/types.hpp"

#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/agnocast_wrapper/tf2.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <map>
#include <memory>
#include <optional>
#include <string>

namespace autoware::multi_object_tracker
{

/// Backend used to obtain the time-varying ego pose (world <- ego).
enum class EgoSource {
  TF,        ///< look up the ego pose from the TF tree (default, legacy behavior)
  ODOMETRY,  ///< interpolate the ego pose from a subscribed odometry buffer
};

/// Reference time the exported objects are predicted/published to (object-export side).
enum class DelayReference {
  DETECTION,        ///< the latest measurement/detection stamp (publish on measurement, no timer)
  NOW,              ///< the current wall-clock time
  LATEST_ODOMETRY,  ///< the stamp of the newest buffered odometry sample
};

/// Parse helpers (strict string match; throw std::invalid_argument on mismatch).
EgoSource toEgoSource(const std::string & name);
DelayReference toDelayReference(const std::string & name);

class Odometry
{
public:
  Odometry(
    rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock,
    std::shared_ptr<autoware::agnocast_wrapper::Buffer> tf_buffer,
    const std::string & world_frame_id, const std::string & ego_frame_id,
    bool enable_odometry_uncertainty = false, EgoSource ego_source = EgoSource::TF);

  /// Feed a new odometry sample (world <- ego) into the interpolation buffer.
  void updateOdometryBuffer(const nav_msgs::msg::Odometry & odometry);

  /// Stamp of the newest buffered odometry sample (nullopt if the buffer is empty).
  std::optional<rclcpp::Time> getLatestOdometryTime() const;

  std::optional<geometry_msgs::msg::Transform> getTransform(
    const std::string & source_frame_id, const rclcpp::Time & time) const;
  std::optional<geometry_msgs::msg::Transform> getTransform(const rclcpp::Time & time) const;

  std::optional<nav_msgs::msg::Odometry> getOdometryFromTf(const rclcpp::Time & time) const;

  std::optional<types::DynamicObjectList> transformObjects(
    const types::DynamicObjectList & input_objects) const;

private:
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  // frame id
  std::string ego_frame_id_;    // ego vehicle frame
  std::string world_frame_id_;  // absolute/relative ground frame
  // tf
  std::shared_ptr<autoware::agnocast_wrapper::Buffer> tf_buffer_;
  autoware::agnocast_wrapper::TransformListener tf_listener_;
  // ego source
  EgoSource ego_source_;

public:
  bool enable_odometry_uncertainty_ = false;

private:
  void updateTfCache(
    const rclcpp::Time & time, const geometry_msgs::msg::Transform & transform) const;

  // --- odometry-source helpers ---
  /// Interpolate (or bounded-extrapolate) the buffered odometry to `time`.
  std::optional<nav_msgs::msg::Odometry> interpolateOdometry(const rclcpp::Time & time) const;
  /// world <- ego transform from the odometry buffer at `time` (nullopt on miss).
  std::optional<geometry_msgs::msg::Transform> getEgoTransformFromOdometry(
    const rclcpp::Time & time) const;

  // cache of tf (TF backend)
  mutable std::map<rclcpp::Time, geometry_msgs::msg::Transform> tf_cache_;
  // buffer of odometry samples (ODOMETRY backend)
  mutable std::map<rclcpp::Time, nav_msgs::msg::Odometry> odom_buffer_;
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__ODOMETRY_HPP_
