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

#ifndef AUTOWARE__TENSORRT_VAD__DATA_TYPES_HPP_
#define AUTOWARE__TENSORRT_VAD__DATA_TYPES_HPP_

#include <opencv2/core/mat.hpp>

#include <array>
#include <map>
#include <string>
#include <vector>

// ROS2 message includes
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace autoware::tensorrt_vad
{

/**
 * @brief Structure representing predicted trajectory
 */
struct PredictedTrajectory
{
  std::vector<std::array<float, 2>> trajectory;  // Dynamic timesteps × 2 coordinates (x, y)
  float confidence;                              // Confidence of this trajectory

  // Default constructor
  PredictedTrajectory() : confidence(0.0f)
  {
    // Default to empty trajectory
  }

  // Constructor with specific number of timesteps
  explicit PredictedTrajectory(size_t timesteps) : confidence(0.0f)
  {
    trajectory.resize(timesteps);
    // Initialize trajectory coordinates to 0
    for (size_t i = 0; i < timesteps; ++i) {
      trajectory[i][0] = 0.0f;
      trajectory[i][1] = 0.0f;
    }
  }
};

/**
 * @brief each map polyline type and its points
 */
struct MapPolyline
{
  std::string type;                        // polyline type ("divider", "ped_crossing", "boundary")
  std::vector<std::vector<float>> points;  // polyline points (each point has [x, y])

  MapPolyline() = default;

  MapPolyline(const std::string & map_type, const std::vector<std::vector<float>> & map_points)
  : type(map_type), points(map_points)
  {
  }
};

/**
 * @brief Structure representing bounding box and its predicted trajectory
 */
struct BBox
{
  std::array<float, 10> bbox;  // [c_x, c_y, w, l, c_z, h, sin(theta), cos(theta), v_x, v_y]
  float confidence;            // Object confidence
  int32_t object_class;        // Object class
  std::vector<PredictedTrajectory> trajectories;  // Dynamic number of predicted trajectories

  // Default constructor
  BBox() : confidence(0.0f), object_class(-1)
  {
    // Initialize bbox coordinates to 0
    for (int32_t i = 0; i < 10; ++i) {
      bbox[i] = 0.0f;
    }
  }

  // Constructor with specific number of trajectory modes and timesteps
  BBox(size_t trajectory_modes, size_t timesteps) : confidence(0.0f), object_class(-1)
  {
    // Initialize bbox coordinates to 0
    for (int32_t i = 0; i < 10; ++i) {
      bbox[i] = 0.0f;
    }

    // Initialize trajectories with specified modes and timesteps
    trajectories.reserve(trajectory_modes);
    for (size_t i = 0; i < trajectory_modes; ++i) {
      trajectories.emplace_back(timesteps);
    }
  }
};

// VAD inference input data structure
struct VadInputData
{
  // Camera image data (multi-camera support)
  std::vector<cv::Mat> camera_images;

  // Shift information (img_metas.0[shift])
  std::vector<float> shift;

  // Transform matrix from coordinate used in VAD inference to camera image coordinate system
  // (img_metas.0[lidar2img])
  std::vector<float> vad_base2img;

  // CAN-BUS data (vehicle state information: velocity, angular velocity, etc.)
  // (img_metas.0[can_bus])
  std::vector<float> can_bus;

  // Command index (for trajectory selection)
  int32_t command{2};
};

// VAD inference output data structure
struct VadOutputData
{
  // Predicted trajectory (6 2D coordinate points, expressed as cumulative coordinates)
  // planning[0,1] = 1st point (x,y), planning[2,3] = 2nd point (x,y), ...
  std::vector<float> predicted_trajectory{};  // size: 12 (6 points * 2 coordinates)

  // Map of predicted trajectories for multiple commands
  // key: command index (int32_t), value: trajectory (std::vector<float>)
  std::map<int32_t, std::vector<float>> predicted_trajectories{};

  // map polylines (each polyline has map_type and points)
  std::vector<MapPolyline> map_polylines{};

  // Predicted objects
  std::vector<BBox> predicted_objects{};
};

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

#endif  // AUTOWARE__TENSORRT_VAD__DATA_TYPES_HPP_
