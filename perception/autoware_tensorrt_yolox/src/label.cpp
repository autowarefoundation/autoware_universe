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
#include <unordered_map>
#include <utility>
#include <vector>

// cspell: ignore semseg

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
 * strings. The first line is always treated as a header and skipped.
 * @param filename Path to the file.
 * @return Parsed data that contains parsed strings of each line.
 */
std::optional<std::vector<std::vector<std::string>>> read_csv(const std::string & filename)
{
  std::ifstream file(filename);
  if (!file.is_open()) {
    // return nullopt when it fails to open
    return std::nullopt;
  }

  std::vector<std::vector<std::string>> parsed_strings;
  std::string line;

  // skip the header line
  std::getline(file, line);

  while (std::getline(file, line)) {
    // remove comments
    // '#' is widely used as starting symbol of comment in csv, but it is not an official rule
    // i.e. RFC 4180 does not mention this feature
    size_t comment_pos = line.find('#');
    if (comment_pos != std::string::npos) {
      line.resize(comment_pos);
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

std::vector<std::string> load_image_list(const std::string & filepath)
{
  std::vector<std::string> file_list = load_list_from_text_file(filepath);
  for (const auto & file : file_list) {
    if (!file_exists(file, false)) {
      std::cerr << "WARNING: couldn't find: " << file << " while loading: " << filepath
                << std::endl;
    }
  }

  return file_list;
}

// read label names of the model's outputs, indexed by the model's output class ID
std::vector<std::string> read_label_file(const std::string & label_path)
{
  std::ifstream label_file(label_path);
  if (!label_file.is_open()) {
    std::stringstream error_msg;
    error_msg << "Could not open label file: " << label_path;
    throw std::runtime_error{error_msg.str()};
  }

  std::vector<std::string> roi_class_name_list;
  std::string label_name;
  while (getline(label_file, label_name)) {
    trim(label_name);
    roi_class_name_list.push_back(label_name);
  }

  return roi_class_name_list;
}

std::vector<Colormap> load_segmentation_colormap(const std::string & file_name)
{
  if (file_name.empty()) {
    return {};
  }

  auto rows = read_csv(file_name);
  // check loaded status
  if (!rows) {
    std::stringstream error_msg;
    error_msg << "Could not open the segmentation color map file: " << file_name;
    throw std::runtime_error{error_msg.str()};
  }

  std::vector<Colormap> parsed;
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
      cmap.id = std::stoi(row[0]);
      // col 1: name
      cmap.name = row[1];
      // col 2~4: colors
      for (size_t i = 2; i < expected_column_num; ++i) {
        // assuming color is provided with 0~255 range in integer
        cmap.color.push_back(static_cast<unsigned char>(std::stoi(row[i])));
      }

      parsed.push_back(cmap);
    } catch (const std::exception & e) {
      std::stringstream error_msg;
      error_msg << "Invalid row: " << e.what();
      throw std::runtime_error{error_msg.str()};
    }
  }

  const int max_id =
    std::max_element(parsed.begin(), parsed.end(), [](const Colormap & a, const Colormap & b) {
      return a.id < b.id;
    })->id;
  std::vector<Colormap> semseg_color_map(max_id + 1);
  for (auto & cmap : parsed) {
    semseg_color_map[cmap.id] = std::move(cmap);
  }

  return semseg_color_map;
}

std::unordered_map<std::string, int> load_label_id_remap_file(const std::string & file_name)
{
  auto rows = read_csv(file_name);
  if (!rows) {
    std::stringstream error_msg;
    error_msg << "Could not open the label map file: " << file_name;
    throw std::runtime_error{error_msg.str()};
  }

  std::unordered_map<std::string, int> label_name_to_id_remap;
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
      label_name_to_id_remap[label_name] = std::stoi(row[1]);
    } catch (const std::exception & e) {
      std::stringstream error_msg;
      error_msg << "Failed to parse label ID as integer for " << label_name << ": " << e.what();
      throw std::runtime_error{error_msg.str()};
    }
  }

  return label_name_to_id_remap;
}

std::vector<int> build_roi_id_to_target_id_map(
  const std::vector<std::string> & roi_class_name_list,
  const std::unordered_map<std::string, int> & label_name_to_target_id, int unmapped_id)
{
  std::vector<int> roi_id_to_target_id_map(roi_class_name_list.size(), unmapped_id);

  // an empty remap means remapping is disabled, so every entry stays unmapped
  if (label_name_to_target_id.empty()) {
    return roi_id_to_target_id_map;
  }

  for (size_t roi_class_id = 0; roi_class_id < roi_class_name_list.size(); ++roi_class_id) {
    const std::string & class_name = roi_class_name_list[roi_class_id];
    const auto remap_iter = label_name_to_target_id.find(class_name);
    if (remap_iter == label_name_to_target_id.end()) {
      std::stringstream error_msg;
      error_msg << "ROI label " << class_name << " not found in remap file.";
      throw std::runtime_error{error_msg.str()};
    }
    roi_id_to_target_id_map[roi_class_id] = remap_iter->second;
  }

  return roi_id_to_target_id_map;
}

std::vector<RoiLabel> load_label_maps(
  const std::string & label_path, const std::string & roi_remap_path,
  const std::string & roi_to_semseg_remap_path)
{
  const auto roi_class_name_list = read_label_file(label_path);

  // resolve ROI -> Autoware interface class IDs (e.g. MOTORBIKE -> 5). An empty remap leaves every
  // entry unmapped, but the table is always sized so it can be indexed for every detection.
  std::unordered_map<std::string, int> roi_label_to_new_id_remap;
  if (!roi_remap_path.empty()) {
    roi_label_to_new_id_remap = load_label_id_remap_file(roi_remap_path);
    if (roi_label_to_new_id_remap.empty()) {
      throw std::runtime_error{"ROI remap file is empty: " + roi_remap_path};
    }
  }
  const auto class_id_map = build_roi_id_to_target_id_map(
    roi_class_name_list, roi_label_to_new_id_remap, g_unmapped_label_id);

  // resolve ROI -> semantic segmentation IDs (e.g. PEDESTRIAN -> 6). Stays all-unmapped when no
  // remap file is given.
  std::vector<int> semseg_id_map(roi_class_name_list.size(), g_unmapped_label_id);
  if (!roi_to_semseg_remap_path.empty()) {
    const auto roi_name_to_semseg_id_remap = load_label_id_remap_file(roi_to_semseg_remap_path);
    if (roi_name_to_semseg_id_remap.empty()) {
      throw std::runtime_error{"ROI-to-semseg remap file is empty: " + roi_to_semseg_remap_path};
    }
    semseg_id_map = build_roi_id_to_target_id_map(
      roi_class_name_list, roi_name_to_semseg_id_remap, g_unmapped_label_id);
  }

  std::vector<RoiLabel> roi_labels;
  roi_labels.reserve(roi_class_name_list.size());
  for (size_t i = 0; i < roi_class_name_list.size(); ++i) {
    roi_labels.push_back({roi_class_name_list[i], class_id_map[i], semseg_id_map[i]});
  }
  return roi_labels;
}

}  // namespace autoware::tensorrt_yolox
