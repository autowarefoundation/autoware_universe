// Copyright 2021 TierIV
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

/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Simon Thompson, Ryohsuke Mitsudome
 *
 */

#include "lanelet2_map_loader_node.hpp"

#include "lanelet2_local_projector.hpp"
#include "utils.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <autoware/geography_utils/lanelet2_projector.hpp>
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

#include <filesystem>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::map_loader
{

namespace
{
bool isOsmFile(const std::string & f)
{
  if (std::filesystem::is_directory(f)) {
    return false;
  }

  const std::string ext = std::filesystem::path(f).extension();

  if (ext != ".osm" && ext != ".OSM") {
    return false;
  }

  return true;
}
}  // namespace

using autoware_map_msgs::msg::LaneletMapBin;
using autoware_map_msgs::msg::MapProjectorInfo;

Lanelet2MapLoaderNode::Lanelet2MapLoaderNode(const rclcpp::NodeOptions & options)
: Node("lanelet2_map_loader", options)
{
  const auto adaptor = autoware::component_interface_utils::NodeAdaptor(this);

  // subscription
  adaptor.init_sub(
    sub_map_projector_info_,
    [this](const MapProjectorInfo::Message::ConstSharedPtr msg) { on_map_projector_info(msg); });

  declare_parameter<bool>("allow_unsupported_version");
  declare_parameter<std::string>("lanelet2_map_path");
  declare_parameter<double>("center_line_resolution");
  declare_parameter<bool>("use_waypoints");
  declare_parameter<bool>("enable_differential_map_loading");

  if (get_parameter("enable_differential_map_loading").as_bool()) {
    differential_loader_module_ = std::make_unique<Lanelet2DifferentialLoaderModule>(
      this, get_parameter("center_line_resolution").as_double(),
      get_parameter("use_waypoints").as_bool());
  }
}

void Lanelet2MapLoaderNode::on_map_projector_info(
  const MapProjectorInfo::Message::ConstSharedPtr msg)
{
  const auto allow_unsupported_version = get_parameter("allow_unsupported_version").as_bool();
  const auto lanelet2_paths_or_directory = std::vector<std::string>{
    get_parameter("lanelet2_map_path").as_string()};
  const auto center_line_resolution = get_parameter("center_line_resolution").as_double();
  const auto use_waypoints = get_parameter("use_waypoints").as_bool();
  const auto enable_differential_map_loading =
    get_parameter("enable_differential_map_loading").as_bool();

  // validate lanelet2_paths_or_directory
  if (lanelet2_paths_or_directory.empty()) {
    RCLCPP_ERROR(get_logger(), "No lanelet2 map files given to the node");
    return;
  }

  // get lanelet2 paths
  const std::vector<std::string> lanelet2_paths = get_lanelet2_paths(lanelet2_paths_or_directory);
  if (lanelet2_paths.empty()) {
    RCLCPP_ERROR(
      get_logger(), "No lanelet2 map files found from %s", lanelet2_paths_or_directory[0].c_str());
    return;
  }

  // setup differential map loader module
  if (enable_differential_map_loading) {
    RCLCPP_INFO(get_logger(), "Differential lanelet2 map loading is enabled.");

    // generate metadata
    const auto lanelet2_metadata_path =
      declare_parameter<std::string>("lanelet2_map_metadata_path");
    double x_resolution, y_resolution;
    std::map<std::string, Lanelet2FileMetaData> lanelet2_metadata_dict;
    if (std::filesystem::exists(lanelet2_metadata_path)) {
      lanelet2_metadata_dict =
        get_lanelet2_metadata(lanelet2_metadata_path, lanelet2_paths, x_resolution, y_resolution);
    } else {
      throw std::runtime_error("Lanelet2 metadata file not found: " + lanelet2_metadata_path);
    }

    // set metadata and projection info to differential loader module
    differential_loader_module_->setLaneletMapMetadata(
      lanelet2_metadata_dict, x_resolution, y_resolution);
    differential_loader_module_->setProjectionInfo(*msg);
  }

  // load lanelet2 map
  lanelet::LaneletMapPtr map = std::make_shared<lanelet::LaneletMap>();
  for (const auto & path : lanelet2_paths) {
    auto map_tmp = utils::load_map(path, *msg);
    if (!map_tmp) {
      RCLCPP_ERROR(get_logger(), "Failed to load lanelet2_map. Not published.");
      return;
    }
    utils::merge_lanelet2_maps(*map, *map_tmp);
  }

  // we use first lanelet2 path to get format_version and map_version
  std::string format_version{"null"}, map_version{""};
  lanelet::io_handlers::AutowareOsmParser::parseVersions(
    lanelet2_paths[0], &format_version, &map_version);
  if (format_version == "null" || format_version.empty() || !isdigit(format_version[0])) {
    RCLCPP_WARN(
      get_logger(),
      "%s has no format_version(null) or non semver-style format_version(%s) information",
      lanelet2_paths[0].c_str(), format_version.c_str());
    if (!allow_unsupported_version) {
      throw std::invalid_argument(
        "allow_unsupported_version is false, so stop loading lanelet map");
    }
  } else if (const auto map_major_ver_opt = lanelet::io_handlers::parseMajorVersion(format_version);
             map_major_ver_opt.has_value()) {
    const auto map_major_ver = map_major_ver_opt.value();
    if (map_major_ver > static_cast<uint64_t>(lanelet::autoware::version)) {
      RCLCPP_WARN(
        get_logger(),
        "format_version(%ld) of the provided map(%s) is larger than the supported version(%ld)",
        map_major_ver, lanelet2_paths[0].c_str(),
        static_cast<uint64_t>(lanelet::autoware::version));
      if (!allow_unsupported_version) {
        throw std::invalid_argument(
          "allow_unsupported_version is false, so stop loading lanelet map");
      }
    }
  }
  RCLCPP_INFO(get_logger(), "Loaded map format_version: %s", format_version.c_str());

  // overwrite centerline
  if (use_waypoints) {
    lanelet::utils::overwriteLaneletsCenterlineWithWaypoints(map, center_line_resolution, false);
  } else {
    lanelet::utils::overwriteLaneletsCenterline(map, center_line_resolution, false);
  }

  // create map bin msg
  const auto map_bin_msg = utils::create_map_bin_msg(map, lanelet2_paths[0], now());

  // create publisher and publish
  pub_map_bin_ =
    create_publisher<LaneletMapBin>("output/lanelet2_map", rclcpp::QoS{1}.transient_local());
  pub_map_bin_->publish(map_bin_msg);
  RCLCPP_INFO(get_logger(), "Succeeded to load lanelet2_map. Map is published.");
}

/**
 * @brief Get list of lanelet2 map file paths from input paths/directories
 * @param lanelet2_paths_or_directory Vector of paths that can be either lanelet2 map files or
 * directories containing them
 * @return Vector of absolute paths to lanelet2 map files
 */
std::vector<std::string> Lanelet2MapLoaderNode::get_lanelet2_paths(
  const std::vector<std::string> & lanelet2_paths_or_directory) const
{
  std::vector<std::string> lanelet2_paths;
  for (const auto & path : lanelet2_paths_or_directory) {
    if (!std::filesystem::exists(path)) {
      RCLCPP_ERROR_STREAM(get_logger(), "No such file or directory: " << path);
      continue;
    }

    // If the given path is already a lanelet2 map file, add it to the list
    if (isOsmFile(path)) {
      lanelet2_paths.push_back(path);
    }

    // If the given path is a directory, add all the lanelet2 map files in the directory
    if (std::filesystem::is_directory(path)) {
      for (const auto & file : std::filesystem::directory_iterator(path)) {
        const auto filename = file.path().string();
        if (isOsmFile(filename)) {
          lanelet2_paths.push_back(filename);
        }
      }
    }
  }
  return lanelet2_paths;
}

std::map<std::string, Lanelet2FileMetaData> Lanelet2MapLoaderNode::get_lanelet2_metadata(
  const std::string & lanelet2_metadata_path, const std::vector<std::string> & lanelet2_paths,
  double & x_resolution, double & y_resolution) const
{
  std::map<std::string, Lanelet2FileMetaData> lanelet2_metadata_dict;
  lanelet2_metadata_dict =
    utils::loadLanelet2Metadata(lanelet2_metadata_path, x_resolution, y_resolution);
  lanelet2_metadata_dict = utils::replaceWithAbsolutePath(lanelet2_metadata_dict, lanelet2_paths);
  RCLCPP_INFO_STREAM(get_logger(), "Loaded Lanelet2 metadata: " << lanelet2_metadata_path);

  return lanelet2_metadata_dict;
}

}  // namespace autoware::map_loader

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::map_loader::Lanelet2MapLoaderNode)
