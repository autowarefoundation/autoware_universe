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

#ifndef AUTOWARE_GENERIC_VALUE_CONVERTER__VALUE_MAP_HPP_
#define AUTOWARE_GENERIC_VALUE_CONVERTER__VALUE_MAP_HPP_

#include "autoware_generic_value_converter/csv_loader.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

namespace autoware::generic_value_converter
{
class ValueMap
{
public:
  bool readValueMapFromCSV(const std::string & csv_path, const bool validation = false);
  bool getValue(const double acc, const double vel, double & value) const;
  bool getAcceleration(const double value, const double vel, double & acc) const;
  std::vector<double> getVelIdx() const { return vel_index_; }
  std::vector<double> getValueIdx() const { return value_index_; }
  std::vector<std::vector<double>> getValueMap() const { return value_map_; }

private:
  rclcpp::Logger logger_{
    rclcpp::get_logger("autoware_generic_value_converter").get_child("value_map")};
  mutable rclcpp::Clock clock_{RCL_ROS_TIME};
  std::string vehicle_name_;
  std::vector<double> vel_index_;
  std::vector<double> value_index_;
  std::vector<std::vector<double>> value_map_;
};
}  // namespace autoware::generic_value_converter

#endif  // AUTOWARE_GENERIC_VALUE_CONVERTER__VALUE_MAP_HPP_
