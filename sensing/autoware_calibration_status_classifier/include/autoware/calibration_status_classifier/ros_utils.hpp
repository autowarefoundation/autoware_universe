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

#include <rclcpp/time.hpp>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::calibration_status_classifier
{
/**
 * @brief Runtime operation modes for calibration status monitoring
 */
enum class RuntimeMode { MANUAL, PERIODIC, ACTIVE };

/**
 * @brief Supported velocity message source types
 */
enum class VelocitySource {
  TWIST,
  TWIST_WITH_COV,
  TWIST_STAMPED,
  TWIST_WITH_COV_STAMPED,
  ODOMETRY
};

/**
 * @brief Check status for calibration prerequisites
 */
template <typename T>
struct FilterStatus
{
  bool is_activated;
  T current_state;
  bool is_threshold_met;
  double state_age;
};

/**
 * @brief Camera and LiDAR topic information for sensor pairs
 *
 * Contains the necessary topic names and configuration for a specific
 * camera-LiDAR pair used in calibration status monitoring.
 */
struct InputMetadata
{
  FilterStatus<double> velocity_filter_status;
  FilterStatus<size_t> objects_filter_status;
  rclcpp::Time cloud_stamp;
  rclcpp::Time image_stamp;
  rclcpp::Time common_stamp;
};

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
 * @brief Convert string to VelocitySource enum
 * @param source_str String representation of velocity source
 * @return VelocitySource enum value
 * @throws std::invalid_argument for invalid source strings
 */
inline VelocitySource string_to_velocity_source(const std::string & source_str)
{
  if (source_str == "twist") {
    return VelocitySource::TWIST;
  }
  if (source_str == "twist_with_cov") {
    return VelocitySource::TWIST_WITH_COV;
  }
  if (source_str == "twist_stamped") {
    return VelocitySource::TWIST_STAMPED;
  }
  if (source_str == "twist_with_cov_stamped") {
    return VelocitySource::TWIST_WITH_COV_STAMPED;
  }
  if (source_str == "odometry") {
    return VelocitySource::ODOMETRY;
  }
  throw std::invalid_argument("Invalid velocity source: " + source_str);
}

/**
 * @brief Convert VelocitySource enum to string representation
 * @param source VelocitySource enum value
 * @return String representation of the velocity source
 * @throws std::invalid_argument for unknown source values
 */
inline std::string velocity_source_to_string(VelocitySource source)
{
  switch (source) {
    case VelocitySource::TWIST:
      return "twist";
    case VelocitySource::TWIST_WITH_COV:
      return "twist_with_cov";
    case VelocitySource::TWIST_STAMPED:
      return "twist_stamped";
    case VelocitySource::TWIST_WITH_COV_STAMPED:
      return "twist_with_cov_stamped";
    case VelocitySource::ODOMETRY:
      return "odometry";
    default:
      throw std::invalid_argument("Unknown velocity source");
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
 * @param already_rectified Flags indicating whether camera images are already rectified
 * @return Vector of CameraLidarTopicsInfo for each sensor pair
 * @throws std::invalid_argument if topic configurations are invalid
 */
inline std::vector<CameraLidarTopicsInfo> compose_in_out_topics(
  const std::vector<std::string> & lidar_topics, const std::vector<std::string> & camera_topics,
  const std::vector<double> & approx_deltas, const std::vector<bool> & already_rectified)
{
  std::vector<CameraLidarTopicsInfo> inputs;
  const auto num_lidars = lidar_topics.size();
  const auto num_cameras = camera_topics.size();
  const auto num_deltas = approx_deltas.size();
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
    input.already_rectified =
      (already_rectified.size() == 1) ? already_rectified.at(0) : already_rectified.at(camera_idx);

    inputs.push_back(input);
  }

  return inputs;
};

}  // namespace autoware::calibration_status_classifier
#endif  // AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__ROS_UTILS_HPP_
