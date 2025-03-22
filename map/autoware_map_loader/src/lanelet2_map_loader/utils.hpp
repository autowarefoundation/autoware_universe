// Copyright 2024 The Autoware Contributors
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

#ifndef LANELET2_MAP_LOADER__UTILS_HPP_
#define LANELET2_MAP_LOADER__UTILS_HPP_

#include <rclcpp/time.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_map_msgs/msg/map_projector_info.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <map>
#include <string>
#include <vector>
namespace autoware::map_loader
{
struct Lanelet2FileMetaData
{
  std::string id;
  double min_x;
  double min_y;
};

namespace utils
{
std::map<std::string, Lanelet2FileMetaData> loadLanelet2Metadata(
  const std::string & lanelet2_metadata_path, double & x_resolution, double & y_resolution);

std::map<std::string, Lanelet2FileMetaData> replaceWithAbsolutePath(
  const std::map<std::string, Lanelet2FileMetaData> & lanelet2_metadata_path,
  const std::vector<std::string> & lanelet2_paths);

void merge_lanelet2_maps(lanelet::LaneletMap & merge_target, lanelet::LaneletMap & merge_source);

lanelet::LaneletMapPtr load_map(
  const std::string & lanelet2_filename,
  const autoware_map_msgs::msg::MapProjectorInfo & projector_info);

autoware_map_msgs::msg::LaneletMapBin create_map_bin_msg(
  const lanelet::LaneletMapPtr map, const std::string & lanelet2_filename,
  const rclcpp::Time & now);

}  // namespace utils

}  // namespace autoware::map_loader

#endif  // LANELET2_MAP_LOADER__UTILS_HPP_
