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

#include "lanelet2_local_projector.hpp"

#include <autoware/geography_utils/lanelet2_projector.hpp>
#include <autoware/map_loader/lanelet2_map_loader_utils.hpp>
#include <autoware_lanelet2_extension/io/autoware_osm_parser.hpp>
#include <autoware_lanelet2_extension/projection/mgrs_projector.hpp>
#include <autoware_lanelet2_extension/projection/transverse_mercator_projector.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <yaml-cpp/yaml.h>

#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace autoware::map_loader::utils
{

std::map<std::string, Lanelet2FileMetaData> load_lanelet2_metadata(
  const std::string & lanelet2_metadata_path, double & x_resolution, double & y_resolution)
{
  YAML::Node config = YAML::LoadFile(lanelet2_metadata_path);

  std::map<std::string, Lanelet2FileMetaData> lanelet2_metadata;

  x_resolution = config["x_resolution"].as<double>();
  y_resolution = config["y_resolution"].as<double>();

  for (const auto & node : config) {
    if (
      node.first.as<std::string>() == "x_resolution" ||
      node.first.as<std::string>() == "y_resolution") {
      continue;
    }

    auto key = node.first.as<std::string>();

    Lanelet2FileMetaData metadata;
    std::stringstream(
      node.first.as<std::string>().substr(0, node.first.as<std::string>().find('.'))) >>
      metadata.id;
    metadata.min_x = node.second.as<std::vector<double>>()[0];
    metadata.min_y = node.second.as<std::vector<double>>()[1];

    lanelet2_metadata[key] = metadata;
  }

  return lanelet2_metadata;
}

std::map<std::string, Lanelet2FileMetaData> replace_with_absolute_path(
  const std::map<std::string, Lanelet2FileMetaData> & lanelet2_metadata_path,
  const std::vector<std::string> & lanelet2_paths)
{
  std::map<std::string, Lanelet2FileMetaData> absolute_path_map;
  for (const auto & path : lanelet2_paths) {
    std::string filename = path.substr(path.find_last_of("/\\") + 1);
    auto it = lanelet2_metadata_path.find(filename);
    if (it != lanelet2_metadata_path.end()) {
      absolute_path_map[path] = it->second;
    }
  }

  return absolute_path_map;
}

void merge_lanelet2_maps(lanelet::LaneletMap & merge_target, lanelet::LaneletMap & merge_source)
{
  for (lanelet::Lanelet & lanelet : merge_source.laneletLayer) {
    merge_target.add(lanelet);
  }

  for (lanelet::Area & area : merge_source.areaLayer) {
    merge_target.add(area);
  }

  for (lanelet::RegulatoryElementPtr & regulatory_element : merge_source.regulatoryElementLayer) {
    merge_target.add(regulatory_element);
  }

  for (lanelet::LineString3d & line_string : merge_source.lineStringLayer) {
    // we need a special handling for line_string to make sure that the points are not duplicated
    // Otherwise, we cannot calculate the relationship of succeeding lanelets correctly
    for (lanelet::Point3d & point : line_string) {
      if (merge_target.pointLayer.find(point.id()) != merge_target.pointLayer.end()) {
        point = merge_target.pointLayer.get(point.id());
      }
    }
    merge_target.add(line_string);
  }

  for (lanelet::Polygon3d & polygon : merge_source.polygonLayer) {
    merge_target.add(polygon);
  }

  for (lanelet::Point3d & point : merge_source.pointLayer) {
    merge_target.add(point);
  }
}

}  // namespace autoware::map_loader::utils
