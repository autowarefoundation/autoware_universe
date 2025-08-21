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

#include "autoware/deviation_estimator/validation_module.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace autoware::deviation_estimator
{

/**
 * @brief ValidationModule validates if estimated parameters are properly converged, given a
 * predefined threshold in this constructor arguments
 */
ValidationModule::ValidationModule(
  const double threshold_coef_vx, const double threshold_stddev_vx,
  const double threshold_bias_gyro, const double threshold_stddev_gyro, const size_t num_history)
: num_history_(num_history)
{
  threshold_dict_["coef_vx"] = threshold_coef_vx;
  threshold_dict_["stddev_vx"] = threshold_stddev_vx;
  threshold_dict_["angular_velocity_offset_x"] = threshold_bias_gyro;
  threshold_dict_["angular_velocity_offset_y"] = threshold_bias_gyro;
  threshold_dict_["angular_velocity_offset_z"] = threshold_bias_gyro;
  threshold_dict_["angular_velocity_stddev_xx"] = threshold_stddev_gyro;
  threshold_dict_["angular_velocity_stddev_yy"] = threshold_stddev_gyro;
  threshold_dict_["angular_velocity_stddev_zz"] = threshold_stddev_gyro;
}

/**
 * @brief get a min and max of a given vector
 */
std::pair<double, double> get_min_max_from_vector(const std::vector<double> & vec, const int num)
{
  double min_val = std::numeric_limits<double>::max();
  double max_val = -std::numeric_limits<double>::max();
  for (int i = 0; i < num; ++i) {
    min_val = std::min(min_val, vec[vec.size() - i - 1]);
    max_val = std::max(max_val, vec[vec.size() - i - 1]);
  }
  return std::pair<double, double>(min_val, max_val);
}

/**
 * @brief add a newly-estimated velocity related parameters
 */
void ValidationModule::set_velocity_data(const double coef_vx, const double stddev_vx)
{
  if (data_list_dict_.count("coef_vx")) {
    if (coef_vx == data_list_dict_["coef_vx"].back()) {
      return;
    }
  }

  data_list_dict_["coef_vx"].push_back(coef_vx);
  data_list_dict_["stddev_vx"].push_back(stddev_vx);
}

/**
 * @brief add a newly-estimated gyroscope related parameters
 */
void ValidationModule::set_gyro_data(
  const geometry_msgs::msg::Vector3 & bias_gyro, const geometry_msgs::msg::Vector3 & stddev_gyro)
{
  if (data_list_dict_.count("angular_velocity_offset_x")) {
    if (bias_gyro.x == data_list_dict_["angular_velocity_offset_x"].back()) {
      return;
    }
  }

  data_list_dict_["angular_velocity_offset_x"].push_back(bias_gyro.x);
  data_list_dict_["angular_velocity_offset_y"].push_back(bias_gyro.y);
  data_list_dict_["angular_velocity_offset_z"].push_back(bias_gyro.z);
  data_list_dict_["angular_velocity_stddev_xx"].push_back(stddev_gyro.x);
  data_list_dict_["angular_velocity_stddev_yy"].push_back(stddev_gyro.y);
  data_list_dict_["angular_velocity_stddev_zz"].push_back(stddev_gyro.z);
}

/**
 * @brief get a min and max of a certain vector (designated by a key)
 */
std::pair<double, double> ValidationModule::get_min_max(const std::string key) const
{
  if (data_list_dict_.count(key)) {
    if (data_list_dict_.at(key).size() < num_history_) {
      throw std::domain_error("The data is not enough. Provide more data for valid results.");
    }
    return get_min_max_from_vector(data_list_dict_.at(key), num_history_);
  } else {
    throw std::runtime_error("Invalid key in ValidationModule::get_min_max");
  }
}

/**
 * @brief check if the given item (="key") is valid
 */
bool ValidationModule::is_valid(const std::string key) const
{
  const auto min_max = this->get_min_max(key);
  return min_max.second - min_max.first < threshold_dict_.at(key);
}

}  // namespace autoware::deviation_estimator
