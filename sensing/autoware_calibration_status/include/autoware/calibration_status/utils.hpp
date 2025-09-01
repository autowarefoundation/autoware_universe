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

#ifndef AUTOWARE__CALIBRATION_STATUS__UTILS_HPP_
#define AUTOWARE__CALIBRATION_STATUS__UTILS_HPP_

#include <stdexcept>
#include <string>

namespace autoware::calibration_status
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
 * @brief Vehicle velocity check status for calibration prerequisites
 */
struct VelocityCheckStatus
{
  bool is_activated;
  double current_velocity;
  bool is_vehicle_moving;
  double velocity_age;
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
 * @brief Result structure for calibration status detection
 *
 * Contains the complete output from calibration validation including
 * classification results, confidence scores, and performance metrics.
 */
struct CalibrationStatusResult
{
  float calibration_confidence;
  float miscalibration_confidence;
  double preprocessing_time_ms;
  double inference_time_ms;
  uint32_t num_points_projected;
};

}  // namespace autoware::calibration_status
#endif  // AUTOWARE__CALIBRATION_STATUS__UTILS_HPP_
