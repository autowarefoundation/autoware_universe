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

#include "autoware_generic_value_converter/csv_loader.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace autoware::generic_value_converter
{

CSVLoader::CSVLoader(const std::string & csv_path) : csv_path_(csv_path)
{
}

bool CSVLoader::readCSV(std::vector<std::vector<std::string>> & result)
{
  std::ifstream ifs(csv_path_);
  if (!ifs.is_open()) {
    RCLCPP_ERROR_STREAM_THROTTLE(logger_, clock_, 5000, "Cannot open " << csv_path_);
    return false;
  }

  std::string line;
  while (std::getline(ifs, line)) {
    std::istringstream iss(line);
    std::string cell;
    std::vector<std::string> row;
    while (std::getline(iss, cell, ',')) {
      row.push_back(cell);
    }
    result.push_back(row);
  }
  return true;
}

std::vector<double> CSVLoader::getRowIndex(const std::vector<std::vector<std::string>> & table)
{
  // Get row index from first column (skip header)
  std::vector<double> index;
  for (std::size_t i = 1; i < table.size(); ++i) {
    index.push_back(std::stod(table[i][0]));
  }
  return index;
}

std::vector<double> CSVLoader::getColumnIndex(const std::vector<std::vector<std::string>> & table)
{
  // Get column index from first row (skip first cell)
  std::vector<double> index;
  for (std::size_t i = 1; i < table[0].size(); ++i) {
    index.push_back(std::stod(table[0][i]));
  }
  return index;
}

std::vector<std::vector<double>> CSVLoader::getMap(
  const std::vector<std::vector<std::string>> & table)
{
  // Get map data (skip first row and first column)
  std::vector<std::vector<double>> map;
  for (std::size_t i = 1; i < table.size(); ++i) {
    std::vector<double> row;
    for (std::size_t j = 1; j < table[i].size(); ++j) {
      row.push_back(std::stod(table[i][j]));
    }
    map.push_back(row);
  }
  return map;
}

bool CSVLoader::validateMap(
  const std::vector<std::vector<double>> & map, [[maybe_unused]] bool is_col_decent)
{
  // Check if map is empty
  if (map.empty()) {
    return false;
  }

  // Check if map is rectangular (all rows have same number of columns)
  const std::size_t cols = map[0].size();
  if (cols == 0) {
    return false;
  }

  for (const auto & row : map) {
    if (row.size() != cols) {
      return false;
    }

    // Check for NaN or Inf values
    for (const auto & val : row) {
      if (std::isnan(val) || std::isinf(val)) {
        return false;
      }
    }
  }

  return true;
}

double CSVLoader::clampValue(
  const double val, const std::vector<double> & ranges, [[maybe_unused]] std::string name)
{
  if (ranges.empty()) {
    return val;
  }
  return std::max(ranges.front(), std::min(ranges.back(), val));
}

}  // namespace autoware::generic_value_converter
