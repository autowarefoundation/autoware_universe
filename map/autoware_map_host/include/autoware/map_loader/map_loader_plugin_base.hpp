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

#ifndef AUTOWARE__MAP_LOADER__MAP_LOADER_PLUGIN_BASE_HPP_
#define AUTOWARE__MAP_LOADER__MAP_LOADER_PLUGIN_BASE_HPP_

#include "autoware/map_loader/map_loader_data.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace autoware::map_loader::plugin
{

class MapLoaderPluginBase
{
public:
  MapLoaderPluginBase() = default;
  virtual ~MapLoaderPluginBase() = default;

  virtual void initialize(
    const std::string & name, rclcpp::Node * node_ptr, MapLoaderData & data) = 0;

  virtual void on_startup(MapLoaderData & data) = 0;

  std::string get_name() const { return name_; }

  void bind(const std::string & name, rclcpp::Node * node_ptr)
  {
    name_ = name;
    node_ptr_ = node_ptr;
  }

protected:
  rclcpp::Node * get_node_ptr() const { return node_ptr_; }

private:
  std::string name_{"unnamed_map_loader_plugin"};
  rclcpp::Node * node_ptr_{nullptr};
};

}  // namespace autoware::map_loader::plugin

#endif  // AUTOWARE__MAP_LOADER__MAP_LOADER_PLUGIN_BASE_HPP_
