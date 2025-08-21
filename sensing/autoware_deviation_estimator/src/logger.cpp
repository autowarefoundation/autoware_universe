// Copyright 2018-2019 Autoware Foundation
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

#include "autoware/deviation_estimator/logger.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace autoware::deviation_estimator
{

/**
 * @brief constructor for Logger class
 */
Logger::Logger(const std::string & output_dir)
: output_log_path_(output_dir + "/output.txt"),
  output_imu_param_path_(output_dir + "/imu_corrector.param.yaml"),
  output_velocity_param_path_(output_dir + "/vehicle_velocity_converter.param.yaml")
{
  std::ofstream file_log(output_log_path_);
  file_log.close();

  std::ofstream file_imu_param(output_imu_param_path_);
  file_imu_param.close();

  std::ofstream file_velocity_param(output_velocity_param_path_);
  file_velocity_param.close();
}

/**
 * @brief log estimated results (IMU and velocity parameters)
 */
void Logger::log_estimated_result_section(
  const double stddev_vx, const double coef_vx,
  const geometry_msgs::msg::Vector3 & angular_velocity_stddev,
  const geometry_msgs::msg::Vector3 & angular_velocity_offset) const
{
  std::ofstream file_velocity_param(output_velocity_param_path_);
  file_velocity_param << "# Estimated by deviation_estimator\n";
  file_velocity_param << "/**:\n";
  file_velocity_param << "  ros__parameters:\n";
  file_velocity_param << fmt::format("    speed_scale_factor: {:.5f}\n", coef_vx);
  file_velocity_param << fmt::format("    velocity_stddev_xx: {:.5f}\n", stddev_vx);
  file_velocity_param << "    angular_velocity_stddev_zz: 0.1 # Default value\n";
  file_velocity_param << "    frame_id: base_link # Default value\n";
  file_velocity_param.close();

  std::ofstream file_imu_param(output_imu_param_path_);
  file_imu_param << "# Estimated by deviation_estimator\n";
  file_imu_param << "/**:\n";
  file_imu_param << "  ros__parameters:\n";
  file_imu_param << fmt::format(
    "    angular_velocity_offset_x: {:.5f}\n", angular_velocity_offset.x);
  file_imu_param << fmt::format(
    "    angular_velocity_offset_y: {:.5f}\n", angular_velocity_offset.y);
  file_imu_param << fmt::format(
    "    angular_velocity_offset_z: {:.5f}\n", angular_velocity_offset.z);
  file_imu_param << fmt::format(
    "    angular_velocity_stddev_xx: {:.5f}\n", angular_velocity_stddev.x);
  file_imu_param << fmt::format(
    "    angular_velocity_stddev_yy: {:.5f}\n", angular_velocity_stddev.y);
  file_imu_param << fmt::format(
    "    angular_velocity_stddev_zz: {:.5f}\n", angular_velocity_stddev.z);
}

/**
 * @brief log validation results
 */
void Logger::log_validation_result_section(const ValidationModule & validation_module) const
{
  std::ofstream file(output_log_path_);
  file << "# Validation results\n";
  file << "# value: [min, max]\n";

  std::vector<std::string> keys{
    "coef_vx",
    "stddev_vx",
    "angular_velocity_offset_x",
    "angular_velocity_offset_y",
    "angular_velocity_offset_z",
    "angular_velocity_stddev_xx",
    "angular_velocity_stddev_yy",
    "angular_velocity_stddev_zz"};

  for (const std::string & key : keys) {
    try {
      const auto min_max = validation_module.get_min_max(key);
      if (validation_module.is_valid(key)) {
        file << "[OK] ";
      } else {
        file << "[NG] ";
      }
      file << fmt::format(
        "{}: [{}, {}]\n", key, double_round(min_max.first, 5), double_round(min_max.second, 5));
    } catch (std::domain_error & e) {  // if the data is not enough
      file << fmt::format("[NG] {}: Not enough data provided yet\n", key);
    }
  }
  file.close();
}

}  // namespace autoware::deviation_estimator
