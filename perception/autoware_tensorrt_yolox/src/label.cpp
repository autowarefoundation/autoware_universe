// Copyright 2026 TIER IV, Inc.
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

#include "autoware/tensorrt_yolox/label.hpp"

#include <experimental/filesystem>

#include <assert.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace autoware::tensorrt_yolox
{
void trim_left(std::string & s)
{
  s.erase(s.begin(), find_if(s.begin(), s.end(), [](int ch) { return !isspace(ch); }));
}

void trim_right(std::string & s)
{
  s.erase(find_if(s.rbegin(), s.rend(), [](int ch) { return !isspace(ch); }).base(), s.end());
}

std::string trim(std::string & s)
{
  trim_left(s);
  trim_right(s);
  return s;
}

/**
 * @brief Reads a CSV file and returns a vector of rows, where each row is a vector of trimmed
 * strings.
 * @param filename Path to the file.
 * @param skip_header_lines Number of lines to skip at the top (default 0).
 * @return Parsed data that contains parsed strings of each line.
 */
std::optional<std::vector<std::vector<std::string>>> read_csv(
  const std::string & filename, uint32_t skip_header_lines = 0)
{
  std::ifstream file(filename);
  if (!file.is_open()) {
    // return nullopt when it fails to open
    return std::nullopt;
  }

  std::vector<std::vector<std::string>> parsed_strings;
  std::string line;
  uint32_t current_line = 0;

  while (std::getline(file, line)) {
    // skip header lines
    if (current_line < skip_header_lines) {
      current_line++;
      continue;
    }

    // remove comments
    // '#' is widely used as starting symbol of comment in csv, but it is not an official rule
    // i.e. RFC 4180 does not mention this feature
    size_t comment_pos = line.find('#');
    if (comment_pos != std::string::npos) {
      line = line.substr(0, comment_pos);
    }

    // skip empty lines
    // create a temp copy to trim so we don't modify 'line' before splitting
    std::string temp_line = line;
    if (trim(temp_line).empty()) {
      continue;
    }

    // parse tokens
    std::stringstream ss(line);
    std::string token;
    std::vector<std::string> row;

    while (std::getline(ss, token, ',')) {
      trim(token);
      row.push_back(token);
    }

    if (!row.empty()) {
      parsed_strings.push_back(row);
    }

    current_line++;
  }

  return parsed_strings;
}

bool file_exists(const std::string & file_name, bool verbose)
{
  if (!std::experimental::filesystem::exists(std::experimental::filesystem::path(file_name))) {
    if (verbose) {
      std::cout << "File does not exist : " << file_name << std::endl;
    }
    return false;
  }
  return true;
}

std::vector<std::string> load_list_from_text_file(const std::string & filename)
{
  assert(file_exists(filename, true));
  std::vector<std::string> list;

  std::ifstream f(filename);
  if (!f) {
    std::cout << "failed to open " << filename << std::endl;
    assert(0);
  }

  std::string line;
  while (std::getline(f, line)) {
    if (line.empty()) {
      continue;
    } else {
      list.push_back(trim(line));
    }
  }

  return list;
}

std::vector<std::string> load_image_list(const std::string & filename, const std::string & prefix)
{
  std::vector<std::string> fileList = load_list_from_text_file(filename);
  for (auto & file : fileList) {
    if (file_exists(file, false)) {
      continue;
    } else {
      std::string prefixed = prefix + file;
      if (file_exists(prefixed, false))
        file = prefixed;
      else
        std::cerr << "WARNING: couldn't find: " << prefixed << " while loading: " << filename
                  << std::endl;
    }
  }

  return fileList;
}

// read label names of the model's outputs
void read_label_file(
  const std::string & label_path, std::vector<std::string> & roi_id_to_name_map,
  std::unordered_map<std::string, int> & roi_name_to_id_map)
{
  std::ifstream label_file(label_path);
  if (!label_file.is_open()) {
    std::stringstream error_msg;
    error_msg << "Could not open label file: " << label_path;
    throw std::runtime_error{error_msg.str()};
  }

  int label_index = 0;
  std::string label_name;
  while (getline(label_file, label_name)) {
    roi_id_to_name_map.push_back(label_name);
    roi_name_to_id_map.insert({label_name, label_index});

    ++label_index;
  }
}

void load_segmentation_colormap(
  const std::string & file_name, std::vector<autoware::tensorrt_yolox::Colormap> & semseg_color_map,
  std::unordered_map<std::string, int> & semseg_name_to_id_map, uint32_t skip_header_lines = 1)
{
  auto rows = read_csv(file_name, skip_header_lines);
  // check loaded status
  if (!rows) {
    std::stringstream error_msg;
    error_msg << "Could not open the segmentation color map file: " << file_name;
    throw std::runtime_error{error_msg.str()};
  }

  constexpr size_t expected_column_num = 5;
  for (const auto & row : rows.value()) {
    // ensure we have expected columns (id, name, r, g, b)
    if (row.size() != expected_column_num) {
      std::stringstream error_msg;
      error_msg << "Invalid row: " << expected_column_num << " columns was expected.";
      throw std::runtime_error{error_msg.str()};
    }

    Colormap cmap;

    try {
      // col 0: ID
      const int label = std::stoi(row[0]);
      cmap.id = label;
      // col 1: name
      std::string label_name = row[1];
      cmap.name = label_name;
      // col 2~4: colors
      for (size_t i = 2; i < expected_column_num; ++i) {
        // assuming color is provided with 0~255 range in integer
        cmap.color.push_back(static_cast<unsigned char>(std::stoi(row[i])));
      }

      semseg_color_map.push_back(cmap);
      semseg_name_to_id_map.insert({label_name, label});
    } catch (const std::exception & e) {
      std::stringstream error_msg;
      error_msg << "Invalid row: " << e.what();
      throw std::runtime_error{error_msg.str()};
    }
  }
}

void load_label_remap_file(
  const std::string & file_name, std::unordered_map<std::string, std::string> & label_name_remap,
  uint32_t skip_header_lines = 1)
{
  auto rows = read_csv(file_name, skip_header_lines);
  // check loaded status
  if (!rows) {
    std::stringstream error_msg;
    error_msg << "Could not open the label map file: " << file_name;
    throw std::runtime_error{error_msg.str()};
  }

  // expecting 2 columns (label names of from and to)
  constexpr size_t expected_column_num = 2;
  for (const auto & row : rows.value()) {
    // ensure we have expected columns numbers
    if (row.size() != expected_column_num) {
      std::stringstream error_msg;
      error_msg << "Invalid row: " << expected_column_num << " columns was expected.";
      throw std::runtime_error{error_msg.str()};
    }

    const std::string label_name_from = row[0];
    const std::string label_name_to = row[1];
    label_name_remap[label_name_from] = label_name_to;
  }
}

void load_label_id_remap_file(
  const std::string & file_name, std::unordered_map<std::string, int> & label_name_to_id_remap,
  uint32_t skip_header_lines = 1)
{
  auto rows = read_csv(file_name, skip_header_lines);
  if (!rows) {
    std::stringstream error_msg;
    error_msg << "Could not open the label map file: " << file_name;
    throw std::runtime_error{error_msg.str()};
  }

  // expecting 2 columns (label_name, label_id)
  constexpr size_t expected_column_num = 2;
  for (const auto & row : rows.value()) {
    if (row.size() != expected_column_num) {
      std::stringstream error_msg;
      error_msg << "Invalid row: " << expected_column_num << " columns were expected.";
      throw std::runtime_error{error_msg.str()};
    }

    const std::string label_name = row[0];

    try {
      const int label_id = std::stoi(row[1]);
      label_name_to_id_remap[label_name] = label_id;
    } catch (const std::exception & e) {
      std::stringstream error_msg;
      error_msg << "Failed to parse label ID as integer for " << label_name << ": " << e.what();
      throw std::runtime_error{error_msg.str()};
    }
  }
}

}  // namespace autoware::tensorrt_yolox
