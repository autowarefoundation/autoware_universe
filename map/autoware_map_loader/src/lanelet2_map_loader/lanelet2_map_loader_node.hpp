// Copyright 2021 Tier IV, Inc.
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

#ifndef AUTOWARE__MAP_LOADER__LANELET2_MAP_LOADER_NODE_HPP_
#define AUTOWARE__MAP_LOADER__LANELET2_MAP_LOADER_NODE_HPP_

#include "lanelet2_differential_loader_module.hpp"
#include "utils.hpp"

#include <autoware/component_interface_specs_universe/map.hpp>
#include <autoware/component_interface_utils/rclcpp.hpp>
#include <autoware_lanelet2_extension/version.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_map_msgs/msg/map_projector_info.hpp>

#include <lanelet2_projection/UTM.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::map_loader
{
class Lanelet2MapLoaderNode : public rclcpp::Node
{
public:
  static constexpr lanelet::autoware::Version version = lanelet::autoware::version;

public:
  explicit Lanelet2MapLoaderNode(const rclcpp::NodeOptions & options);

private:
  using MapProjectorInfo = autoware::component_interface_specs_universe::map::MapProjectorInfo;

  void on_map_projector_info(const MapProjectorInfo::Message::ConstSharedPtr msg);

  autoware::component_interface_utils::Subscription<MapProjectorInfo>::SharedPtr
    sub_map_projector_info_;
  std::unique_ptr<Lanelet2DifferentialLoaderModule> differential_loader_module_;

  rclcpp::Publisher<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr pub_map_bin_;

  std::vector<std::string> get_lanelet2_paths(
    const std::vector<std::string> & lanelet2_paths_or_directory) const;
  std::map<std::string, Lanelet2FileMetaData> get_lanelet2_metadata(
    const std::string & lanelet2_metadata_path, const std::vector<std::string> & lanelet2_paths,
    double & x_resolution, double & y_resolution) const;
  std::map<std::string, Lanelet2FileMetaData> get_dummy_lanelet2_metadata(
    const std::string & lanelet2_path,
    const MapProjectorInfo::Message::ConstSharedPtr projection_info, double & x_resolution,
    double & y_resolution);
};
}  // namespace autoware::map_loader

#endif  // AUTOWARE__MAP_LOADER__LANELET2_MAP_LOADER_NODE_HPP_
