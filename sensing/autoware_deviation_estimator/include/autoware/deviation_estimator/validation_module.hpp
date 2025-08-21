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

#ifndef AUTOWARE__DEVIATION_ESTIMATOR__VALIDATION_MODULE_HPP_
#define AUTOWARE__DEVIATION_ESTIMATOR__VALIDATION_MODULE_HPP_

#include "autoware/deviation_estimator/utils.hpp"

#include "geometry_msgs/msg/vector3_stamped.hpp"

#include <fmt/core.h>

#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace autoware::deviation_estimator
{

class ValidationModule
{
public:
  ValidationModule(
    const double threshold_coef_vx, const double threshold_stddev_vx,
    const double threshold_bias_gyro, const double threshold_stddev_gyro, const size_t num_history);
  void set_velocity_data(const double coef_vx, const double stddev_vx);
  void set_gyro_data(
    const geometry_msgs::msg::Vector3 & bias_gyro, const geometry_msgs::msg::Vector3 & stddev_gyro);

  std::pair<double, double> get_min_max(const std::string key) const;
  bool is_valid(const std::string key) const;

private:
  std::map<std::string, std::vector<double>> data_list_dict_;

  std::map<std::string, double> threshold_dict_;
  const size_t num_history_;
};

}  // namespace autoware::deviation_estimator

#endif  // AUTOWARE__DEVIATION_ESTIMATOR__VALIDATION_MODULE_HPP_
