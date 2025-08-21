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

#ifndef AUTOWARE__DEVIATION_ESTIMATOR__LOGGER_HPP_
#define AUTOWARE__DEVIATION_ESTIMATOR__LOGGER_HPP_

#include "autoware/deviation_estimator/utils.hpp"
#include "autoware/deviation_estimator/validation_module.hpp"

#include "geometry_msgs/msg/vector3.hpp"

#include <fmt/core.h>

#include <fstream>
#include <string>

namespace autoware::deviation_estimator
{

class Logger
{
public:
  explicit Logger(const std::string & output_dir);
  void log_estimated_result_section(
    const double stddev_vx, const double coef_vx,
    const geometry_msgs::msg::Vector3 & angular_velocity_stddev,
    const geometry_msgs::msg::Vector3 & angular_velocity_offset) const;
  void log_validation_result_section(const ValidationModule & validation_module) const;

private:
  const std::string output_log_path_;
  const std::string output_imu_param_path_;
  const std::string output_velocity_param_path_;
};

}  // namespace autoware::deviation_estimator

#endif  // AUTOWARE__DEVIATION_ESTIMATOR__LOGGER_HPP_
