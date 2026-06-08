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

#include "autoware/map_loader/plugins/map_projection_loader_plugin.hpp"

#include "autoware/map_loader/map_loader_plugin_base.hpp"

#include <autoware/component_interface_specs/map.hpp>
#include <autoware_lanelet2_extension/io/autoware_osm_parser.hpp>
#include <autoware_lanelet2_extension/projection/mgrs_projector.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/ros/parameter.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <autoware_map_msgs/msg/map_projector_info.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace autoware::map_projection_loader
{
autoware_map_msgs::msg::MapProjectorInfo load_info_from_yaml(const std::string & filename)
{
  YAML::Node data = YAML::LoadFile(filename);

  autoware_map_msgs::msg::MapProjectorInfo msg;
  msg.projector_type = data["projector_type"].as<std::string>();
  if (msg.projector_type == autoware_map_msgs::msg::MapProjectorInfo::MGRS) {
    msg.vertical_datum = data["vertical_datum"].as<std::string>();
    msg.mgrs_grid = data["mgrs_grid"].as<std::string>();

  } else if (
    msg.projector_type == autoware_map_msgs::msg::MapProjectorInfo::LOCAL_CARTESIAN_UTM ||
    msg.projector_type == autoware_map_msgs::msg::MapProjectorInfo::LOCAL_CARTESIAN ||
    msg.projector_type == autoware_map_msgs::msg::MapProjectorInfo::TRANSVERSE_MERCATOR) {
    msg.vertical_datum = data["vertical_datum"].as<std::string>();
    msg.map_origin.latitude = data["map_origin"]["latitude"].as<double>();
    msg.map_origin.longitude = data["map_origin"]["longitude"].as<double>();
    msg.map_origin.altitude = 0.0;

  } else if (msg.projector_type == autoware_map_msgs::msg::MapProjectorInfo::LOCAL) {
    ;  // do nothing

  } else if (msg.projector_type == "local") {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("MapProjectionLoader"),
      "Load " << filename << "\n"
              << "DEPRECATED WARNING: projector type \"local\" is deprecated."
                 "Please use \"Local\" instead. For more info, visit "
                 "https://github.com/autowarefoundation/autoware.universe/blob/main/map/"
                 "map_projection_loader README.md");
    msg.projector_type = autoware_map_msgs::msg::MapProjectorInfo::LOCAL;
  } else {
    throw std::runtime_error(
      "Invalid map projector type. Currently supported types: MGRS, LocalCartesian, "
      "LocalCartesianUTM, "
      "TransverseMercator, and local");
  }

  // set scale factor
  static constexpr float scale_factor_for_utm = 0.9996;
  static constexpr float scale_factor_for_local = 1.0;
  if (msg.projector_type == autoware_map_msgs::msg::MapProjectorInfo::TRANSVERSE_MERCATOR) {
    if (data["scale_factor"]) {
      msg.scale_factor = data["scale_factor"].as<float>();
    } else {
      msg.scale_factor = scale_factor_for_utm;
    }
  } else if (
    msg.projector_type == autoware_map_msgs::msg::MapProjectorInfo::MGRS ||
    msg.projector_type == autoware_map_msgs::msg::MapProjectorInfo::LOCAL_CARTESIAN_UTM) {
    msg.scale_factor = scale_factor_for_utm;
  } else if (
    msg.projector_type == autoware_map_msgs::msg::MapProjectorInfo::LOCAL ||
    msg.projector_type == autoware_map_msgs::msg::MapProjectorInfo::LOCAL_CARTESIAN) {
    msg.scale_factor = scale_factor_for_local;
  }

  if (msg.scale_factor <= 0.0) {
    throw std::runtime_error(
      "Invalid scale factor. The scale factor must be a value greater than 0.");
  }
  return msg;
}

autoware_map_msgs::msg::MapProjectorInfo load_map_projector_info(
  const std::string & yaml_filename, const std::string & lanelet2_map_filename)
{
  autoware_map_msgs::msg::MapProjectorInfo msg;

  if (std::filesystem::exists(yaml_filename)) {
    std::cout << "Load " << yaml_filename << std::endl;
    msg = load_info_from_yaml(yaml_filename);
  } else if (std::filesystem::exists(lanelet2_map_filename)) {
    std::cout << "Load " << lanelet2_map_filename << std::endl;
    std::cout
      << "DEPRECATED WARNING: Loading map projection info from lanelet2 map may soon be deleted. "
         "Please use map_projector_info.yaml instead. For more info, visit "
         "https://github.com/autowarefoundation/autoware.universe/blob/main/map/"
         "map_projection_loader/"
         "README.md"
      << std::endl;
    msg = load_info_from_lanelet2_map(lanelet2_map_filename);
  } else {
    throw std::runtime_error(
      "No map projector info files found. Please provide either "
      "map_projector_info.yaml or lanelet2_map.osm");
  }
  return msg;
}

autoware_map_msgs::msg::MapProjectorInfo load_info_from_lanelet2_map(const std::string & filename)
{
  lanelet::ErrorMessages errors{};
  lanelet::projection::MGRSProjector projector{};
  const lanelet::LaneletMapPtr map = lanelet::load(filename, projector, &errors);
  if (!errors.empty()) {
    std::stringstream ss;
    ss << "Error occurred while loading lanelet2 map:\n";
    for (const auto & err : errors) {
      ss << "- " << err << "\n";
    }
    throw std::runtime_error(ss.str());
  }

  // If the lat & lon values in all the points of lanelet2 map are all zeros,
  // it will be interpreted as a local map.
  // If any single point exists with non-zero lat or lon values, it will be interpreted as MGRS.
  bool is_local = true;
  for (const auto & point : map->pointLayer) {
    const auto gps_point = projector.reverse(point);
    if (gps_point.lat != 0.0 || gps_point.lon != 0.0) {
      is_local = false;
      break;
    }
  }

  autoware_map_msgs::msg::MapProjectorInfo msg;
  if (is_local) {
    msg.projector_type = autoware_map_msgs::msg::MapProjectorInfo::LOCAL;
  } else {
    msg.projector_type = autoware_map_msgs::msg::MapProjectorInfo::MGRS;
    msg.mgrs_grid = projector.getProjectedMGRSGrid();
  }

  // We assume that the vertical datum of the map is WGS84 when using lanelet2 map.
  // However, do note that this is not always true, and may cause problems in the future.
  // Thus, please consider using the map_projector_info.yaml instead of this deprecated function.
  msg.vertical_datum = autoware_map_msgs::msg::MapProjectorInfo::WGS84;
  return msg;
}
}  // namespace autoware::map_projection_loader

namespace autoware::map_loader::plugin
{

class MapProjectionLoaderPlugin : public MapLoaderPluginBase
{
public:
  void initialize(
    const std::string & /*name*/, rclcpp::Node * node, MapLoaderData & /*data*/) override
  {
    using autoware_utils_rclcpp::get_or_declare_parameter;
    const std::string yaml_filename =
      get_or_declare_parameter<std::string>(*node, "map_projector_info_path");
    const std::string lanelet2_map_filename =
      get_or_declare_parameter<std::string>(*node, "lanelet2_map_path");

    projector_info_ = autoware::map_projection_loader::load_map_projector_info(
      yaml_filename, lanelet2_map_filename);

    using MapProjectorInfo = autoware::component_interface_specs::map::MapProjectorInfo;
    publisher_ = node->create_publisher<MapProjectorInfo::Message>(
      MapProjectorInfo::name, autoware::component_interface_specs::get_qos<MapProjectorInfo>());
  }

  void on_startup(MapLoaderData & data) override
  {
    publisher_->publish(projector_info_);
    data.projector_info = projector_info_;
    RCLCPP_INFO(
      get_node_ptr()->get_logger(), "Published map projector info (type: %s)",
      projector_info_.projector_type.c_str());
  }

private:
  autoware_map_msgs::msg::MapProjectorInfo projector_info_;
  rclcpp::Publisher<autoware_map_msgs::msg::MapProjectorInfo>::SharedPtr publisher_;
};

}  // namespace autoware::map_loader::plugin

PLUGINLIB_EXPORT_CLASS(
  autoware::map_loader::plugin::MapProjectionLoaderPlugin,
  autoware::map_loader::plugin::MapLoaderPluginBase)
