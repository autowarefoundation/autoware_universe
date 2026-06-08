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

#include "autoware/map_loader/map_loader_plugin_base.hpp"
#include "differential_map_loader_module.hpp"
#include "partial_map_loader_module.hpp"
#include "pointcloud_map_loader_module.hpp"
#include "selected_map_loader_module.hpp"
#include "utils.hpp"

#include <autoware_utils/ros/parameter.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace autoware::map_loader::plugin
{
namespace fs = std::filesystem;

class PointCloudMapLoaderPlugin : public MapLoaderPluginBase
{
public:
  void initialize(
    const std::string & /*name*/, rclcpp::Node * node, MapLoaderData & /*data*/) override
  {
    node_ = node;
    using autoware_utils_rclcpp::get_or_declare_parameter;
    const auto pcd_paths = get_pcd_paths(
      get_or_declare_parameter<std::vector<std::string>>(*node, "pcd_paths_or_directory"));
    const std::string pcd_metadata_path =
      get_or_declare_parameter<std::string>(*node, "pcd_metadata_path");
    const bool enable_whole_load = get_or_declare_parameter<bool>(*node, "enable_whole_load");
    const bool enable_downsample_whole_load =
      get_or_declare_parameter<bool>(*node, "enable_downsampled_whole_load");
    const bool enable_partial_load = get_or_declare_parameter<bool>(*node, "enable_partial_load");
    const bool enable_selected_load = get_or_declare_parameter<bool>(*node, "enable_selected_load");

    if (enable_whole_load) {
      pcd_map_loader_ =
        std::make_unique<PointcloudMapLoaderModule>(node, pcd_paths, "pointcloud_map", false);
    }

    if (enable_downsample_whole_load) {
      downsampled_pcd_map_loader_ = std::make_unique<PointcloudMapLoaderModule>(
        node, pcd_paths, "debug/downsampled_pointcloud_map", true);
    }

    const auto pcd_metadata_dict = get_pcd_metadata(pcd_metadata_path, pcd_paths);

    if (enable_partial_load) {
      partial_map_loader_ = std::make_unique<PartialMapLoaderModule>(node, pcd_metadata_dict);
    }

    differential_map_loader_ =
      std::make_unique<DifferentialMapLoaderModule>(node, pcd_metadata_dict);

    if (enable_selected_load) {
      selected_map_loader_ = std::make_unique<SelectedMapLoaderModule>(node, pcd_metadata_dict);
    }
  }

  void on_startup(MapLoaderData & /*data*/) override
  {
    RCLCPP_INFO(node_->get_logger(), "Pointcloud map loader plugin initialized.");
  }

private:
  rclcpp::Node * node_{nullptr};
  std::unique_ptr<PointcloudMapLoaderModule> pcd_map_loader_;
  std::unique_ptr<PointcloudMapLoaderModule> downsampled_pcd_map_loader_;
  std::unique_ptr<PartialMapLoaderModule> partial_map_loader_;
  std::unique_ptr<DifferentialMapLoaderModule> differential_map_loader_;
  std::unique_ptr<SelectedMapLoaderModule> selected_map_loader_;

  static bool is_pcd_file(const std::string & p)
  {
    if (fs::is_directory(p)) {
      return false;
    }
    const std::string ext = fs::path(p).extension();
    return ext == ".pcd" || ext == ".PCD";
  }

  static std::vector<std::string> get_pcd_paths(
    const std::vector<std::string> & pcd_paths_or_directory)
  {
    std::vector<std::string> pcd_paths;
    for (const auto & p : pcd_paths_or_directory) {
      if (!fs::exists(p)) {
        RCLCPP_ERROR(rclcpp::get_logger("map_loader"), "invalid path: %s", p.c_str());
      }
      if (is_pcd_file(p)) {
        pcd_paths.push_back(p);
      }
      if (fs::is_directory(p)) {
        for (const auto & file : fs::directory_iterator(p)) {
          const auto filename = file.path().string();
          if (is_pcd_file(filename)) {
            pcd_paths.push_back(filename);
          }
        }
      }
    }
    return pcd_paths;
  }

  static std::map<std::string, PCDFileMetadata> get_pcd_metadata(
    const std::string & pcd_metadata_path, const std::vector<std::string> & pcd_paths)
  {
    if (fs::exists(pcd_metadata_path)) {
      std::set<std::string> missing_pcd_names;
      auto pcd_metadata_dict = load_pcd_metadata(pcd_metadata_path);
      pcd_metadata_dict =
        replace_with_absolute_path(pcd_metadata_dict, pcd_paths, missing_pcd_names);
      if (!missing_pcd_names.empty()) {
        std::ostringstream oss;
        oss << "The following segment(s) are missing from the input PCDs: ";
        for (const auto & fname : missing_pcd_names) {
          oss << std::endl << fname;
        }
        RCLCPP_ERROR(rclcpp::get_logger("map_loader"), "%s", oss.str().c_str());
        throw std::runtime_error("Missing PCD segments. Exiting map loader...");
      }
      return pcd_metadata_dict;
    }

    if (pcd_paths.size() == 1) {
      pcl::PointCloud<pcl::PointXYZ> single_pcd;
      const auto & pcd_path = pcd_paths.front();
      if (pcl::io::loadPCDFile(pcd_path, single_pcd) == -1) {
        throw std::runtime_error("PCD load failed: " + pcd_path);
      }
      PCDFileMetadata metadata = {};
      pcl::getMinMax3D(single_pcd, metadata.min, metadata.max);
      return std::map<std::string, PCDFileMetadata>{{pcd_path, metadata}};
    }
    throw std::runtime_error("PCD metadata file not found: " + pcd_metadata_path);
  }
};

}  // namespace autoware::map_loader::plugin

PLUGINLIB_EXPORT_CLASS(
  autoware::map_loader::plugin::PointCloudMapLoaderPlugin,
  autoware::map_loader::plugin::MapLoaderPluginBase)
