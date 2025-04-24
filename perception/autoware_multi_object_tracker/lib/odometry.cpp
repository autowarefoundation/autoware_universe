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

#include "autoware/multi_object_tracker/odometry.hpp"

#include "autoware/multi_object_tracker/uncertainty/uncertainty_processor.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2_ros/create_timer_ros.h>

#include <memory>
#include <string>

namespace autoware::multi_object_tracker
{

Odometry::Odometry(
  rclcpp::Node & node, const std::string & world_frame_id, const std::string & ego_frame_id,
  bool enable_odometry_uncertainty)
: node_(node),
  ego_frame_id_(ego_frame_id),
  world_frame_id_(world_frame_id),
  tf_buffer_(node_.get_clock()),
  tf_listener_(tf_buffer_),
  enable_odometry_uncertainty_(enable_odometry_uncertainty)
{
  // Create tf timer
  auto cti = std::make_shared<tf2_ros::CreateTimerROS>(
    node_.get_node_base_interface(), node_.get_node_timers_interface());
  tf_buffer_.setCreateTimerInterface(cti);
}

void Odometry::updateTfCache(
  const rclcpp::Time & time, const geometry_msgs::msg::Transform & tf) const
{
  // update the tf buffer
  tf_cache_.emplace(time, tf);

  // remove too old tf
  const auto max_tf_age = rclcpp::Duration::from_seconds(1.0);
  for (auto it = tf_cache_.begin(); it != tf_cache_.end();) {
    if (time - it->first > max_tf_age) {
      it = tf_cache_.erase(it);
    } else {
      ++it;
    }
  }
}

std::optional<geometry_msgs::msg::Transform> Odometry::getTransform(const rclcpp::Time & time) const
{
  // check buffer and return if the transform is found
  for (const auto & tf : tf_cache_) {
    if (tf.first == time) {
      return std::optional<geometry_msgs::msg::Transform>(tf.second);
    }
  }
  // if not found, get the transform from tf
  return getTransform(ego_frame_id_, time);
}

std::optional<geometry_msgs::msg::Transform> Odometry::getTransform(
  const std::string & source_frame_id, const rclcpp::Time & time) const
{
  try {
    // Check if the frames are ready
    std::string errstr;  // This argument prevents error msg from being displayed in the terminal.
    if (!tf_buffer_.canTransform(
          world_frame_id_, source_frame_id, tf2::TimePointZero, tf2::Duration::zero(), &errstr)) {
      return std::nullopt;
    }

    // Lookup the transform
    geometry_msgs::msg::TransformStamped self_transform_stamped;
    self_transform_stamped = tf_buffer_.lookupTransform(
      world_frame_id_, source_frame_id, time, rclcpp::Duration::from_seconds(0.5));

    // update the cache
    if (source_frame_id == ego_frame_id_) {
      updateTfCache(time, self_transform_stamped.transform);
    }

    return std::optional<geometry_msgs::msg::Transform>(self_transform_stamped.transform);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("multi_object_tracker"), ex.what());
    return std::nullopt;
  }
}

std::optional<nav_msgs::msg::Odometry> Odometry::getOdometryFromTf(const rclcpp::Time & time) const
{
  const auto self_transform = getTransform(time);
  if (!self_transform) {
    return std::nullopt;
  }
  const auto current_transform = self_transform.value();

  nav_msgs::msg::Odometry odometry;
  {
    odometry.header.stamp = time + rclcpp::Duration::from_seconds(0.00001);

    // set the odometry pose
    auto & odom_pose = odometry.pose.pose;
    odom_pose.position.x = current_transform.translation.x;
    odom_pose.position.y = current_transform.translation.y;
    odom_pose.position.z = current_transform.translation.z;
    odom_pose.orientation = current_transform.rotation;

    // set odometry twist
    auto & odom_twist = odometry.twist.twist;
    odom_twist.linear.x = 10.0;  // m/s
    odom_twist.linear.y = 0.1;   // m/s
    odom_twist.angular.z = 0.1;  // rad/s

    // model the uncertainty
    auto & odom_pose_cov = odometry.pose.covariance;
    odom_pose_cov[0] = 0.1;      // x-x
    odom_pose_cov[7] = 0.1;      // y-y
    odom_pose_cov[35] = 0.0001;  // yaw-yaw

    auto & odom_twist_cov = odometry.twist.covariance;
    odom_twist_cov[0] = 2.0;     // x-x [m^2/s^2]
    odom_twist_cov[7] = 0.2;     // y-y [m^2/s^2]
    odom_twist_cov[35] = 0.001;  // yaw-yaw [rad^2/s^2]
  }

  return std::optional<nav_msgs::msg::Odometry>(odometry);
}

std::optional<types::DynamicObjectList> Odometry::transformObjects(
  const types::DynamicObjectList & input_objects) const
{
  types::DynamicObjectList output_objects = input_objects;

  // transform to world coordinate
  if (input_objects.header.frame_id != world_frame_id_) {
    output_objects.header.frame_id = world_frame_id_;
    tf2::Transform tf_target2objects_world;
    tf2::Transform tf_target2objects;
    tf2::Transform tf_objects_world2objects;
    {
      const auto ros_target2objects_world =
        getTransform(input_objects.header.frame_id, input_objects.header.stamp);
      if (!ros_target2objects_world) {
        return std::nullopt;
      }
      tf2::fromMsg(*ros_target2objects_world, tf_target2objects_world);
    }
    for (auto & object : output_objects.objects) {
      auto & pose = object.pose;
      auto & pose_cov = object.pose_covariance;
      tf2::fromMsg(pose, tf_objects_world2objects);
      tf_target2objects = tf_target2objects_world * tf_objects_world2objects;
      // transform pose, frame difference and object pose
      tf2::toMsg(tf_target2objects, pose);
      // transform covariance, only the frame difference
      pose_cov = tf2::transformCovariance(pose_cov, tf_target2objects_world);
    }
  }
  // Add the odometry uncertainty to the object uncertainty
  if (enable_odometry_uncertainty_) {
    // Create a modeled odometry message
    const auto odometry = getOdometryFromTf(input_objects.header.stamp);
    if (!odometry) {
      return std::nullopt;
    }
    // Add the odometry uncertainty to the object uncertainty
    uncertainty::addOdometryUncertainty(odometry.value(), output_objects);
  }

  return std::optional<types::DynamicObjectList>(output_objects);
}

}  // namespace autoware::multi_object_tracker
