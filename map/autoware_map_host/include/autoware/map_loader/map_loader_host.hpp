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

#ifndef AUTOWARE__MAP_LOADER__MAP_LOADER_HOST_HPP_
#define AUTOWARE__MAP_LOADER__MAP_LOADER_HOST_HPP_

#include "autoware/map_loader/map_loader_data.hpp"
#include "autoware/map_loader/map_loader_plugin_base.hpp"

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::map_loader
{

class MapLoaderHost : public rclcpp::Node
{
public:
  explicit MapLoaderHost(const rclcpp::NodeOptions & options);

private:
  void initialize_plugins();
  void load_plugin(const std::string & plugin_name);

  std::unique_ptr<pluginlib::ClassLoader<plugin::MapLoaderPluginBase>> plugin_loader_;
  std::vector<std::shared_ptr<plugin::MapLoaderPluginBase>> plugins_;
  MapLoaderData data_;
  bool initialized_plugins_{false};
};

}  // namespace autoware::map_loader

#endif  // AUTOWARE__MAP_LOADER__MAP_LOADER_HOST_HPP_
