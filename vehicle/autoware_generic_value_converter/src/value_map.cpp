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

  std::vector<double> interpolated_acc_vec;
  const double clamped_vel = CSVLoader::clampValue(vel, vel_index_, "value: vel");

  // (value, vel, acc) map => (value, acc) map by fixing vel
  interpolated_acc_vec.reserve(value_map_.size());
  for (const std::vector<double> & accelerations : value_map_) {
    interpolated_acc_vec.push_back(
      autoware::interpolation::lerp(vel_index_, accelerations, clamped_vel));
  }

  // Make sure we have data to work with
  if (interpolated_acc_vec.empty()) {
    RCLCPP_WARN_THROTTLE(logger_, clock_, 5000, "Interpolated acceleration vector is empty");
    return false;
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

  value = autoware::interpolation::lerp(interpolated_acc_vec, value_index_, acc);
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
