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

#ifndef AUTOWARE_GENERIC_VALUE_CONVERTER__CSV_LOADER_HPP_
#define AUTOWARE_GENERIC_VALUE_CONVERTER__CSV_LOADER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace autoware::generic_value_converter
{
class CSVLoader
{
private:
  std::string csv_path_;
  rclcpp::Logger logger_{rclcpp::get_logger("CSVLoader")};
  rclcpp::Clock clock_{RCL_ROS_TIME};

public:
  explicit CSVLoader(const std::string & csv_path);
  bool readCSV(std::vector<std::vector<std::string>> & result);
  static std::vector<double> getRowIndex(const std::vector<std::vector<std::string>> & table);
  static std::vector<double> getColumnIndex(const std::vector<std::vector<std::string>> & table);
  static std::vector<std::vector<double>> getMap(
    const std::vector<std::vector<std::string>> & table);
  static bool validateMap(const std::vector<std::vector<double>> & map, bool is_col_decent = false);
  static double clampValue(const double val, const std::vector<double> & ranges, std::string name);
};
}  // namespace autoware::generic_value_converter

#endif  // AUTOWARE_GENERIC_VALUE_CONVERTER__CSV_LOADER_HPP_
