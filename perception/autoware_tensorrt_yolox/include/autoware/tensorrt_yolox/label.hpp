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

#ifndef AUTOWARE__TENSORRT_YOLOX__LABEL_HPP_
#define AUTOWARE__TENSORRT_YOLOX__LABEL_HPP_

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::tensorrt_yolox
{
typedef struct Colormap_
{
  int id;
  std::string name;
  std::vector<unsigned char> color;
} Colormap;

void trim_left(std::string & s);
void trim_right(std::string & s);
std::string trim(std::string & s);
std::optional<std::vector<std::vector<std::string>>> read_csv(
  const std::string & filename, uint32_t skip_header_lines);
bool fileExists(const std::string & file_name, bool verbose);
std::vector<std::string> loadListFromTextFile(const std::string & filename);
std::vector<std::string> loadImageList(const std::string & filename, const std::string & prefix);
void read_label_file(
  const std::string & label_path, std::vector<std::string> & roi_class_id_to_class_name_map,
  std::unordered_map<std::string, int> & roi_class_name_to_class_id_map);
void load_segmentation_colormap(
  const std::string & file_name, std::vector<autoware::tensorrt_yolox::Colormap> & semseg_color_map,
  std::unordered_map<std::string, int> & semseg_name_to_semseg_id_map, uint32_t skip_header_lines);
void load_label_remap_file(
  const std::string & file_name, std::unordered_map<std::string, std::string> & label_name_remap,
  uint32_t skip_header_lines);
void replace_roi_label_map(
  std::vector<std::string> & roi_id_to_name_map,
  const std::unordered_map<std::string, std::string> & roi_class_name_remap);
void replace_semseg_label_map(
  std::vector<int> & roi_id_to_semseg_id_map,
  const std::unordered_map<std::string, std::string> & roi_name_to_semseg_name_map,
  const std::unordered_map<std::string, int> & roi_name_to_id_map,
  const std::unordered_map<std::string, int> & semseg_name_to_id_map);
void load_label_id_remap_file(
  const std::string & file_name, std::unordered_map<std::string, int> & label_name_to_id_remap,
  uint32_t skip_header_lines);

}  // namespace autoware::tensorrt_yolox

#endif  // AUTOWARE__TENSORRT_YOLOX__LABEL_HPP_
