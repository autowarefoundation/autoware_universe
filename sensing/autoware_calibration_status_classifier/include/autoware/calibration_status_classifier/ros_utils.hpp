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

#ifndef AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__ROS_UTILS_HPP_
#define AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__ROS_UTILS_HPP_

#include "autoware/calibration_status_classifier/data_type.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::calibration_status_classifier
{

/// @brief ROS message type for the linear velocity subscriber
enum class LinearVelocitySource : int {
  TWIST_STAMPED = 0,
  TWIST_WITH_COV_STAMPED = 1,
  ODOMETRY = 2
};

inline LinearVelocitySource string_to_linear_velocity_source(const std::string & source_str)
{
  if (source_str == "twist_stamped") return LinearVelocitySource::TWIST_STAMPED;
  if (source_str == "twist_with_cov_stamped") return LinearVelocitySource::TWIST_WITH_COV_STAMPED;
  if (source_str == "odometry") return LinearVelocitySource::ODOMETRY;
  throw std::invalid_argument("Invalid linear velocity source: " + source_str);
}

inline std::string linear_velocity_source_to_string(LinearVelocitySource source)
{
  switch (source) {
    case LinearVelocitySource::TWIST_STAMPED:
      return "twist_stamped";
    case LinearVelocitySource::TWIST_WITH_COV_STAMPED:
      return "twist_with_cov_stamped";
    case LinearVelocitySource::ODOMETRY:
      return "odometry";
    default:
      throw std::invalid_argument("Unknown linear velocity source");
  }
}

/// @brief ROS message type for the angular velocity subscriber
enum class AngularVelocitySource : int {
  TWIST_STAMPED = 0,
  TWIST_WITH_COV_STAMPED = 1,
  ODOMETRY = 2
};

inline AngularVelocitySource string_to_angular_velocity_source(const std::string & source_str)
{
  if (source_str == "twist_stamped") return AngularVelocitySource::TWIST_STAMPED;
  if (source_str == "twist_with_cov_stamped") return AngularVelocitySource::TWIST_WITH_COV_STAMPED;
  if (source_str == "odometry") return AngularVelocitySource::ODOMETRY;
  throw std::invalid_argument("Invalid angular velocity source: " + source_str);
}

inline std::string angular_velocity_source_to_string(AngularVelocitySource source)
{
  switch (source) {
    case AngularVelocitySource::TWIST_STAMPED:
      return "twist_stamped";
    case AngularVelocitySource::TWIST_WITH_COV_STAMPED:
      return "twist_with_cov_stamped";
    case AngularVelocitySource::ODOMETRY:
      return "odometry";
    default:
      throw std::invalid_argument("Unknown angular velocity source");
  }
}

/// @brief ROS message type for the object detection subscriber
enum class ObjectsSource : int { PREDICTED_OBJECTS = 0, TRACKED_OBJECTS = 1, DETECTED_OBJECTS = 2 };

inline ObjectsSource string_to_objects_source(const std::string & source_str)
{
  if (source_str == "predicted_objects") return ObjectsSource::PREDICTED_OBJECTS;
  if (source_str == "tracked_objects") return ObjectsSource::TRACKED_OBJECTS;
  if (source_str == "detected_objects") return ObjectsSource::DETECTED_OBJECTS;
  throw std::invalid_argument("Invalid objects source: " + source_str);
}

inline std::string objects_source_to_string(ObjectsSource source)
{
  switch (source) {
    case ObjectsSource::PREDICTED_OBJECTS:
      return "predicted_objects";
    case ObjectsSource::TRACKED_OBJECTS:
      return "tracked_objects";
    case ObjectsSource::DETECTED_OBJECTS:
      return "detected_objects";
    default:
      throw std::invalid_argument("Unknown objects source");
  }
}

/**
 * @brief Runtime operation modes for calibration status monitoring
 */
enum class RuntimeMode { MANUAL, PERIODIC, ACTIVE };

/**
 * @brief Convert string to RuntimeMode enum
 * @param mode_str String representation ("manual", "periodic", "active")
 * @return RuntimeMode enum value
 * @throws std::invalid_argument for invalid mode strings
 */
inline RuntimeMode string_to_runtime_mode(const std::string & mode_str)
{
  if (mode_str == "manual") {
    return RuntimeMode::MANUAL;
  }
  if (mode_str == "periodic") {
    return RuntimeMode::PERIODIC;
  }
  if (mode_str == "active") {
    return RuntimeMode::ACTIVE;
  }
  throw std::invalid_argument("Invalid calibration mode: " + mode_str);
}

/**
 * @brief Convert RuntimeMode enum to string representation
 * @param mode RuntimeMode enum value
 * @return String representation of the mode
 * @throws std::invalid_argument for unknown mode values
 */
inline std::string runtime_mode_to_string(RuntimeMode mode)
{
  switch (mode) {
    case RuntimeMode::MANUAL:
      return "manual";
    case RuntimeMode::PERIODIC:
      return "periodic";
    case RuntimeMode::ACTIVE:
      return "active";
    default:
      throw std::invalid_argument("Unknown runtime mode");
  }
}

/**
 * @brief Compose camera and LiDAR topic information for sensor pairs
 *
 * Creates a list of CameraLidarTopicsInfo structures based on input topic lists.
 * Supports 1:1, 1:N, and N:1 pairing configurations between LiDAR and camera topics.
 * Automatically derives camera_info topics and projected points topics from input names.
 *
 * @param lidar_topics List of LiDAR point cloud topic names
 * @param camera_topics List of camera image topic names
 * @param approx_deltas Approximate time synchronization deltas for each pair
 * @param miscalibration_confidence_thresholds Decision thresholds for each pair
 * @param already_rectified Flags indicating whether camera images are already rectified
 * @return Vector of CameraLidarTopicsInfo for each sensor pair
 * @throws std::invalid_argument if topic configurations are invalid
 */
inline std::vector<CameraLidarTopicsInfo> compose_in_out_topics(
  const std::vector<std::string> & lidar_topics, const std::vector<std::string> & camera_topics,
  const std::vector<double> & approx_deltas,
  const std::vector<double> & miscalibration_confidence_thresholds,
  const std::vector<bool> & already_rectified)
{
  std::vector<CameraLidarTopicsInfo> inputs;
  const auto num_lidars = lidar_topics.size();
  const auto num_cameras = camera_topics.size();
  const auto num_deltas = approx_deltas.size();
  const auto num_thresholds = miscalibration_confidence_thresholds.size();
  const auto num_pairs = std::max(num_cameras, num_lidars);
  bool use_lidar_ns = num_lidars > 1 && num_cameras == 1;

  if (
    (lidar_topics.size() != camera_topics.size()) &&           // Not 1:1
    (lidar_topics.size() > 1 && camera_topics.size() == 1) &&  // Not N:1
    (lidar_topics.size() == 1 && camera_topics.size() > 1)) {  // Not 1:N
    throw std::invalid_argument(
      "Invalid topic configuration: only 1:N, N:1, and 1:1 pairing supported");
  }

  if (num_deltas != 1 && num_deltas != num_pairs) {
    throw std::invalid_argument(
      "Invalid approx_delta configuration: must be 1 or match number of sensor pairs");
  }

  if (num_thresholds != 1 && num_thresholds != num_pairs) {
    throw std::invalid_argument(
      "Invalid miscalibration_confidence_thresholds configuration: "
      "must be 1 or match number of sensor pairs");
  }

  if (already_rectified.size() != 1 && already_rectified.size() != num_cameras) {
    throw std::invalid_argument(
      "Invalid already_rectified configuration: must be 1 or match number of camera topics");
  }

  // Setup camera info topics
  auto camera_info_topics = std::vector<std::string>{};
  for (auto camera_info_topic : camera_topics) {
    // Remove topic suffix and add "camera_info" suffix
    if (auto pos = camera_info_topic.rfind('/'); pos != std::string::npos) {
      camera_info_topic.replace(pos + 1, std::string::npos, "camera_info");
    } else {
      camera_info_topic = "camera_info";
    }
    camera_info_topics.push_back(camera_info_topic);
  }

  // Determine namespace for projected points topics
  if (num_lidars > 1 && num_cameras == 1) {
    use_lidar_ns = true;
  }

  for (size_t i = 0; i < num_pairs; ++i) {
    // Determine which topics to use for this pair
    size_t lidar_idx = (lidar_topics.size() == 1) ? 0 : i;
    size_t camera_idx = (camera_topics.size() == 1) ? 0 : i;
    auto preview_image_topic = use_lidar_ns ? lidar_topics.at(i) : camera_topics.at(i);
    if (auto pos = preview_image_topic.rfind('/'); pos != std::string::npos) {
      preview_image_topic.replace(pos + 1, std::string::npos, "points_projected");
    } else {
      preview_image_topic = "points_projected";
    }

    CameraLidarTopicsInfo input;
    input.camera_topic = camera_topics.at(camera_idx);
    input.camera_info_topic = camera_info_topics.at(camera_idx);
    input.lidar_topic = lidar_topics.at(lidar_idx);
    input.projected_points_topic = preview_image_topic;
    input.approx_delta = (num_deltas == 1) ? approx_deltas.at(0) : approx_deltas.at(i);
    input.miscalibration_confidence_threshold = (num_thresholds == 1)
                                                  ? miscalibration_confidence_thresholds.at(0)
                                                  : miscalibration_confidence_thresholds.at(i);
    input.already_rectified =
      (already_rectified.size() == 1) ? already_rectified.at(0) : already_rectified.at(camera_idx);

    inputs.push_back(input);
  }

  return inputs;
};

}  // namespace autoware::calibration_status_classifier
#endif  // AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__ROS_UTILS_HPP_
