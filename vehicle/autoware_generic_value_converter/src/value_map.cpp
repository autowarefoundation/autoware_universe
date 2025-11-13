//
// Copyright 2025 TIER IV, Inc. All rights reserved.
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
//

#include "autoware_generic_value_converter/value_map.hpp"

#include "autoware/interpolation/linear_interpolation.hpp"

#include <algorithm>
#include <string>
#include <vector>

namespace autoware::generic_value_converter
{

bool ValueMap::readValueMapFromCSV(
  const std::string & csv_path, [[maybe_unused]] const bool validation)
{
  CSVLoader csv(csv_path);
  std::vector<std::vector<std::string>> table;

  if (!csv.readCSV(table)) {
    return false;
  }

  vehicle_name_ = table[0][0];
  vel_index_ = CSVLoader::getColumnIndex(table);
  value_index_ = CSVLoader::getRowIndex(table);
  value_map_ = CSVLoader::getMap(table);

  // Always validate the map to ensure integrity
  if (!CSVLoader::validateMap(value_map_, true)) {
    RCLCPP_ERROR(logger_, "Value map validation failed for: %s", csv_path.c_str());
    return false;
  }

  return true;
}

bool ValueMap::getValue(const double acc, double vel, double & value) const
{
  if (value_map_.empty() || vel_index_.empty() || value_index_.empty()) {
    RCLCPP_WARN_THROTTLE(logger_, clock_, 5000, "Value map is empty");
    return false;
  }

  RCLCPP_DEBUG_THROTTLE(logger_, clock_, 5000, "getValue input: acc=%.3f, vel=%.3f", acc, vel);

  std::vector<double> interpolated_acc_vec;
  const double clamped_vel = CSVLoader::clampValue(vel, vel_index_, "value: vel");

  RCLCPP_DEBUG_THROTTLE(
    logger_, clock_, 5000, "Clamped velocity: %.3f (original: %.3f)", clamped_vel, vel);

  // (value, vel, acc) map => (value, acc) map by fixing vel
  interpolated_acc_vec.reserve(value_map_.size());
  for (const std::vector<double> & accelerations : value_map_) {
    // Log the inputs to the first lerp function
    if (interpolated_acc_vec.empty()) {
      RCLCPP_DEBUG_THROTTLE(
        logger_, clock_, 5000,
        "First lerp call - vel_index size: %zu, accelerations size: %zu, clamped_vel: %.3f",
        vel_index_.size(), accelerations.size(), clamped_vel);

      // Log some sample values from the vectors
      std::string vel_samples = "vel_index samples: ";
      std::string acc_samples = "accelerations samples: ";
      for (size_t i = 0; i < std::min(size_t(3), vel_index_.size()); ++i) {
        vel_samples += std::to_string(vel_index_[i]) + " ";
      }
      for (size_t i = 0; i < std::min(size_t(3), accelerations.size()); ++i) {
        acc_samples += std::to_string(accelerations[i]) + " ";
      }
      RCLCPP_DEBUG_THROTTLE(logger_, clock_, 5000, "%s", vel_samples.c_str());
      RCLCPP_DEBUG_THROTTLE(logger_, clock_, 5000, "%s", acc_samples.c_str());
    }

    interpolated_acc_vec.push_back(
      autoware::interpolation::lerp(vel_index_, accelerations, clamped_vel));
  }

  RCLCPP_DEBUG_THROTTLE(
    logger_, clock_, 5000, "Interpolated acceleration vector size: %zu",
    interpolated_acc_vec.size());

  // Log some sample values from the interpolated vector
  if (!interpolated_acc_vec.empty()) {
    std::string interp_samples = "interpolated_acc_vec samples: ";
    for (size_t i = 0; i < std::min(size_t(3), interpolated_acc_vec.size()); ++i) {
      interp_samples += std::to_string(interpolated_acc_vec[i]) + " ";
    }
    RCLCPP_DEBUG_THROTTLE(logger_, clock_, 5000, "%s", interp_samples.c_str());
  }

  // calculate value from acceleration
  // When the desired acceleration is smaller than the value area, return min value
  // When the desired acceleration is greater than the value area, return max value
  if (acc < interpolated_acc_vec.front()) {
    value = value_index_.front();
    RCLCPP_DEBUG_THROTTLE(
      logger_, clock_, 5000, "Acceleration %.3f is below min (%.3f), returning min value: %.3f",
      acc, interpolated_acc_vec.front(), value);
    return true;
  }
  if (interpolated_acc_vec.back() < acc) {
    value = value_index_.back();
    RCLCPP_DEBUG_THROTTLE(
      logger_, clock_, 5000, "Acceleration %.3f is above max (%.3f), returning max value: %.3f",
      acc, interpolated_acc_vec.back(), value);
    return true;
  }

  // Log the inputs to the second lerp function
  RCLCPP_DEBUG_THROTTLE(
    logger_, clock_, 5000,
    "Second lerp call - interpolated_acc_vec size: %zu, value_index size: %zu, acc: %.3f",
    interpolated_acc_vec.size(), value_index_.size(), acc);

  // Log some sample values from the vectors
  std::string value_samples = "value_index_ samples: ";
  std::string interpolated_acc_vec_samples = "interpolated_acc_vec samples: ";
  for (size_t i = 0; i < value_index_.size(); ++i) {
    value_samples += std::to_string(value_index_[i]) + " ";
  }
  for (size_t i = 0; i < interpolated_acc_vec.size(); ++i) {
    interpolated_acc_vec_samples += std::to_string(interpolated_acc_vec[i]) + " ";
  }
  RCLCPP_DEBUG_THROTTLE(logger_, clock_, 5000, "%s", value_samples.c_str());
  RCLCPP_DEBUG_THROTTLE(logger_, clock_, 5000, "%s", interpolated_acc_vec_samples.c_str());

  value = autoware::interpolation::lerp(interpolated_acc_vec, value_index_, acc);

  RCLCPP_DEBUG_THROTTLE(
    logger_, clock_, 5000, "Final interpolated value: %.3f for acc=%.3f, vel=%.3f", value, acc,
    vel);

  return true;
}

bool ValueMap::getAcceleration(const double value, const double vel, double & acc) const
{
  if (value_map_.empty() || vel_index_.empty() || value_index_.empty()) {
    RCLCPP_WARN_THROTTLE(logger_, clock_, 5000, "Value map is empty");
    return false;
  }

  std::vector<double> interpolated_acc_vec;
  const double clamped_vel = CSVLoader::clampValue(vel, vel_index_, "value: vel");

  // (value, vel, acc) map => (value, acc) map by fixing vel
  interpolated_acc_vec.reserve(value_map_.size());
  for (const auto & acc_vec : value_map_) {
    interpolated_acc_vec.push_back(autoware::interpolation::lerp(vel_index_, acc_vec, clamped_vel));
  }

  // calculate acceleration from value
  const double clamped_value = CSVLoader::clampValue(value, value_index_, "value: acc");
  acc = autoware::interpolation::lerp(value_index_, interpolated_acc_vec, clamped_value);

  return true;
}

}  // namespace autoware::generic_value_converter
