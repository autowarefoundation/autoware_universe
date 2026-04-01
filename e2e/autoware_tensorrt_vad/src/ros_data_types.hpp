// Copyright 2025 TIER IV.
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

#ifndef ROS_DATA_TYPES_HPP_
#define ROS_DATA_TYPES_HPP_

#include "data_types.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace autoware::tensorrt_vad
{

class VadInputTopicData
{
public:
  explicit VadInputTopicData(const int32_t num_cameras);

  bool is_complete() const;
  void reset();

  void set_image(const std::size_t camera_id, const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void set_camera_info(
    const std::size_t camera_id, const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);
  void set_kinematic_state(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void set_acceleration(const geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr & msg);

  rclcpp::Time stamp;

  std::vector<sensor_msgs::msg::Image::ConstSharedPtr> images;
  std::vector<sensor_msgs::msg::CameraInfo::ConstSharedPtr> camera_infos;
  nav_msgs::msg::Odometry::ConstSharedPtr kinematic_state;
  geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr acceleration;

private:
  int32_t num_cameras_;
  bool frame_started_ = false;

  void ensure_frame_started(const rclcpp::Time & msg_stamp);
};

struct VadOutputTopicData
{
  autoware_internal_planning_msgs::msg::CandidateTrajectories candidate_trajectories;
  autoware_planning_msgs::msg::Trajectory trajectory;
  visualization_msgs::msg::MarkerArray map_points;
  autoware_perception_msgs::msg::PredictedObjects objects;
};

}  // namespace autoware::tensorrt_vad

#endif  // ROS_DATA_TYPES_HPP_
