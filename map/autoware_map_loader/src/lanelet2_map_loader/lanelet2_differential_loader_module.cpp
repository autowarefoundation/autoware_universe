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

#include "lanelet2_differential_loader_module.hpp"

#include "lanelet2_local_projector.hpp"
#include "lanelet2_map_loader_node.hpp"

#include <autoware/map_loader/lanelet2_map_loader_utils.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::map_loader
{

Lanelet2DifferentialLoaderModule::Lanelet2DifferentialLoaderModule(
  rclcpp::Node * node, const double & center_line_resolution, const bool & use_waypoints)
: logger_(node->get_logger()),
  clock_(node->get_clock()),
  center_line_resolution_(center_line_resolution),
  use_waypoints_(use_waypoints)
{
  const auto metadata_adaptor = autoware::component_interface_utils::NodeAdaptor(node);
  metadata_adaptor.init_pub(pub_lanelet_map_meta_data_);

  get_differential_lanelet2_maps_service_ = node->create_service<GetDifferentialLanelet2Map>(
    "service/get_differential_lanelet_map",
    std::bind(
      &Lanelet2DifferentialLoaderModule::onServiceGetDifferentialLanelet2Map, this,
      std::placeholders::_1, std::placeholders::_2));
}

bool Lanelet2DifferentialLoaderModule::onServiceGetDifferentialLanelet2Map(
  GetDifferentialLanelet2Map::Request::SharedPtr req,
  GetDifferentialLanelet2Map::Response::SharedPtr res)
{
  if (!projector_info_) {
    RCLCPP_ERROR(logger_, "Projector info is not set");
    return false;
  }

  // get the list of lanelet2 map paths from the requested cell_ids
  std::vector<std::string> lanelet2_paths;
  for (const auto & id : req->cell_ids) {
    auto it = std::find_if(
      lanelet2_file_metadata_dict_.begin(), lanelet2_file_metadata_dict_.end(),
      [&id](const auto & file) { return file.second.id == id; });
    if (it == lanelet2_file_metadata_dict_.end()) {
      continue;
    }
    if (!std::filesystem::exists(it->first)) {
      continue;
    }
    lanelet2_paths.push_back(it->first);
  }
  if (lanelet2_paths.empty()) {
    RCLCPP_ERROR(logger_, "Failed to load differential lanelet2 map");
    return false;
  }

  // load lanelet2 map
  // We have to keep all loaded maps until publishing the map bin msg
  // because the loaded lanelets will be expired when map is destructed
  std::vector<lanelet::LaneletMapPtr> maps;
  for (const auto & path : lanelet2_paths) {
    auto map_tmp = utils::load_map(path, *projector_info_);
    if (!map_tmp) {
      RCLCPP_ERROR(logger_, "Failed to load lanelet2_map, %s", path.c_str());
      return false;
    }
    maps.push_back(map_tmp);
  }

  // merge all maps
  lanelet::LaneletMapPtr map = std::make_shared<lanelet::LaneletMap>();
  for (const auto & map_i : maps) {
    utils::merge_lanelet2_maps(*map, *map_i);
  }

  // overwrite centerline
  if (use_waypoints_) {
    lanelet::utils::overwriteLaneletsCenterlineWithWaypoints(map, center_line_resolution_, false);
  } else {
    lanelet::utils::overwriteLaneletsCenterline(map, center_line_resolution_, false);
  }

  // create the map bin message
  const auto map_bin_msg = utils::create_map_bin_msg(map, lanelet2_paths[0], rclcpp::Clock().now());

  res->lanelet2_cells = map_bin_msg;
  res->header.frame_id = "map";

  return true;
}

void Lanelet2DifferentialLoaderModule::setLaneletMapMetadata(
  const std::map<std::string, Lanelet2FileMetaData> & lanelet2_metadata_dict, const double x_res,
  const double y_res)
{
  lanelet2_file_metadata_dict_ = lanelet2_metadata_dict;

  const auto msg = getLaneletMapMetaDataMsg(x_res, y_res);
  pub_lanelet_map_meta_data_->publish(msg);
}

void Lanelet2DifferentialLoaderModule::setProjectionInfo(
  const autoware_map_msgs::msg::MapProjectorInfo & projector_info)
{
  projector_info_ = projector_info;
}

autoware_map_msgs::msg::LaneletMapMetaData
Lanelet2DifferentialLoaderModule::getLaneletMapMetaDataMsg(
  const double & x_res, const double & y_res) const
{
  autoware_map_msgs::msg::LaneletMapMetaData metadata;
  for (const auto & file : lanelet2_file_metadata_dict_) {
    autoware_map_msgs::msg::LaneletMapCellMetaData cell_msg;
    cell_msg.cell_id = file.second.id;
    cell_msg.min_x = file.second.min_x;
    cell_msg.min_y = file.second.min_y;
    cell_msg.max_x = file.second.min_x + x_res;
    cell_msg.max_y = file.second.min_y + y_res;

    metadata.metadata_list.push_back(cell_msg);
  }
  metadata.header.frame_id = "map";
  metadata.header.stamp = clock_->now();

  return metadata;
}

}  // namespace autoware::map_loader
