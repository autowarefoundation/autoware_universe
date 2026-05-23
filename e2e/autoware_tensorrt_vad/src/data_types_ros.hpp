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

#ifndef DATA_TYPES_ROS_HPP_
#define DATA_TYPES_ROS_HPP_

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

/**
 * @class VadInputTopicData
 * @brief Class for managing ROS topic data for VAD input
 */
class VadInputTopicData
{
public:
  // Constructor: Initialize vectors with specified number of cameras
  explicit VadInputTopicData(const int32_t num_cameras);

  // Check if frame is complete
  bool is_complete() const;

  // Reset frame data
  void reset();

  // Setter methods with frame initialization
  void set_image(const std::size_t camera_id, const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void set_camera_info(
    const std::size_t camera_id, const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);
  void set_kinematic_state(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void set_acceleration(const geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr & msg);

  // Reference timestamp for current frame
  rclcpp::Time stamp;

  // Image data from multiple cameras.
  // Corresponds to ~/input/image0, ~/input/image1, ... remapped in launch file.
  // Vector index corresponds to autoware_camera_id.
  std::vector<sensor_msgs::msg::Image::ConstSharedPtr> images;

  // Camera calibration information corresponding to each image above
  std::vector<sensor_msgs::msg::CameraInfo::ConstSharedPtr> camera_infos;

  // Vehicle kinematic state data (from /localization/kinematic_state etc.)
  nav_msgs::msg::Odometry::ConstSharedPtr kinematic_state;

  // Acceleration data (from /localization/acceleration)
  geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr acceleration;

private:
  int32_t num_cameras_;
  bool frame_started_ = false;

  /**
   * @brief Ensure frame is started with the specified timestamp if not already started
   *
   * This function checks the frame state and starts the frame with the specified timestamp
   * only if it has not been started yet. If the frame is already started, it does nothing.
   *
   * @note This function assumes that proper locking has been acquired by the caller.
   *       Thread safety is the responsibility of the caller.
   *
   * @param msg_stamp The timestamp to set when starting the frame
   */
  void ensure_frame_started(const rclcpp::Time & msg_stamp);
};

struct VadOutputTopicData
{
  autoware_internal_planning_msgs::msg::CandidateTrajectories candidate_trajectories;
  autoware_planning_msgs::msg::Trajectory trajectory;
  visualization_msgs::msg::MarkerArray
    map_points;  // Transformed map points in Autoware coordinate system
  autoware_perception_msgs::msg::PredictedObjects objects;
};

}  // namespace autoware::tensorrt_vad

#endif  // DATA_TYPES_ROS_HPP_
